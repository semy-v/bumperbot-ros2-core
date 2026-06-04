/**
 * @file diff_drive_controller.ino
 *
 * SERIAL COMMUNICATION PROTOCOL SEQUENCE (ARDUINO PERSPECTIVE)
 * ------------------------------------------------------------
 * This microcontroller acts as the "Slave" in the communication protocol. It responds 
 * to commands from the ROS 2 hardware interface and handles the real-time physical 
 * constraints (PID loops, encoder hardware interrupts, failsafes).
 *
 * 1. STARTUP BARRIER (setup)
 * - Upon boot, the MCU enters a blocking loop, strictly awaiting a `Config` message.
 * - It ignores all other incoming data (Velocity, Deactivate) until configuration succeeds.
 * - Once `Config` is received, it writes the PID/deadband settings to local memory,
 * then echoes the exact `Config` message back to ROS 2 to acknowledge readiness.
 *
 * 2. REAL-TIME LOOP (loop / processNextSerialInputMessage)
 * - Polls the serial buffer for incoming messages from ROS 2.
 * - If a `Velocity` message is received:
 * a) It applies the target wheel velocities to the PID controllers.
 * b) It refreshes the Emergency Stop watchdog timer (`g_last_valid_msg_time_ms`).
 * c) It sets an internal flag (`g_received_velocity_msg = true`).
 * - Runs the PID loop calculation (`update()`) to adjust physical motor PWM based on encoders.
 * - ASYNC REPLY: At the end of the loop, if the velocity flag is true, it transmits the 
 * *actual* real-world velocities back to ROS 2.
 *
 * 3. FAILSAFE WATCHDOG (processEmergencyDeactivation)
 * - Continuously evaluates the elapsed time since the last valid `Velocity` command.
 * - If no message arrives for `kEmergencyStopTimeoutMs` (2000ms), it safely de-energizes 
 * the motors. This prevents the robot from driving out of control if the USB cable drops 
 * or the ROS 2 host crashes.
 *
 * 4. DEACTIVATION (MsgId::Deactivate)
 * - If a `Deactivate` message is parsed, it instantly drops motor torque and echoes 
 * the `Deactivate` message back to ROS 2.
 */

#include "quadrature_encoder.hpp"
#include "l298n_motor.hpp"
#include "wheel_controller.hpp"
#include "protocol/diff_drive_messages.hpp"
#include "protocol/diff_drive_serialize.hpp"
#include "protocol/diff_drive_deserialize.hpp"

// right wheel control pins
#define L298N_EN_A 9
#define L298N_IN_1 12
#define L298N_IN_2 10
#define RW_ENCODER_PHASE_A 3
#define RW_ENCODER_PHASE_B 5

// left wheel control pins
#define L298N_EN_B 11
#define L298N_IN_3 7
#define L298N_IN_4 8
#define LW_ENCODER_PHASE_A 2
#define LW_ENCODER_PHASE_B 4

// unconfigurable constants
constexpr unsigned long kEmergencyStopTimeoutMs{2000};
constexpr double kPulsePerRevolution{1280.0};

// configurable constants (overriden by Config message)
constexpr double kDefaultPidControlRate{25.0}; // Hz

constexpr WheelConfig kDefaultRightMotorConfig{
  .kp = 15.5,
  .ki = 39.0,
  .kd = 0.0,
  .pwm_deadband = 17
};

constexpr WheelConfig kDefaultLeftMotorConfig{
  .kp = 14.2,
  .ki = 43.0,
  .kd = 0.0,
  .pwm_deadband = 18
};

// Initialize wheel controllers with defaults
WheelController g_right_wheel{
  .motor = L298NMotor{L298N_EN_A, L298N_IN_1, L298N_IN_2},
  .encoder = QuadratureEncoder{RW_ENCODER_PHASE_A, RW_ENCODER_PHASE_B},
  .pid_control_rate = kDefaultPidControlRate,
  .wheel_config = kDefaultRightMotorConfig,
  .ticks_per_rev = kPulsePerRevolution,
  .invert_logic = false
};

WheelController g_left_wheel{
  .motor = L298NMotor{L298N_EN_B, L298N_IN_3, L298N_IN_4},
  .encoder = QuadratureEncoder{LW_ENCODER_PHASE_A, LW_ENCODER_PHASE_B},
  .pid_control_rate = kDefaultPidControlRate,
  .wheel_config = kDefaultLeftMotorConfig,
  .ticks_per_rev = kPulsePerRevolution,
  .invert_logic = true
};

// Serial input message process
MessageStreamDeserializer<DiffDriveMessageRegistry> g_message_deserializer;
bool g_received_velocity_msg{false};

enum class InputProcessResult {
  Success = 0,
  MessageUnavailable = 1,
  MessageInvalid = 2
};

constexpr MsgId AnyMsgId = MsgId::End;

// Process next expected serial input message
InputProcessResult processNextSerialInputMessage(const MsgId expected_msg_id = AnyMsgId);

// Process all expected serial input messages
InputProcessResult processAllSerialInputMessages(const MsgId expected_msg_id = AnyMsgId);

// Serial output message process
using DiffDriveMessageSerializer = MessageSerializer<DiffDriveMessageRegistry>;

template<typename TData>
void sendSerialMessage(const TData& message_data);

template<MsgId Id>
void sendSerialMessage();

// Activate/Deactivate control
unsigned long g_last_valid_msg_time_ms{};
void processEmergencyDeactivation(const unsigned long current_time_ms = millis());
void activate(const unsigned long current_time_ms = millis());
void deactivate();


void setup() {
  Serial.begin(115200);
  while(!Serial);

  bool init_successful{false};

  while(!init_successful) {
    const auto init_msg_process_result = processAllSerialInputMessages(MsgId::Config);
    if (InputProcessResult::Success != init_msg_process_result) {
      // Wait until Init message received
      continue;
    }

    g_right_wheel.begin();
    g_left_wheel.begin();

    // right wheel encoder interrupt setup
    attachInterrupt(digitalPinToInterrupt(RW_ENCODER_PHASE_A), rightWheelEncoderCallback, CHANGE);
    // left wheel encoder interrupt setup
    attachInterrupt(digitalPinToInterrupt(LW_ENCODER_PHASE_A), leftWheelEncoderCallback, CHANGE);

    init_successful = true;
  }
}

void loop() {
  const auto inputResult = processAllSerialInputMessages();

  if (InputProcessResult::Success != inputResult) {
    processEmergencyDeactivation();
  }

  g_right_wheel.update();
  g_left_wheel.update();

  if (InputProcessResult::MessageUnavailable != inputResult && g_received_velocity_msg) {
    sendSerialMessage<MsgId::Velocity>();
    g_received_velocity_msg = false;
  }
}

InputProcessResult processNextSerialInputMessage(const MsgId expected_msg_id) {
  InputProcessResult result{InputProcessResult::MessageUnavailable};

  while(Serial.available()) {
    const auto state = g_message_deserializer.processByte(Serial.read());

    if (ProcessResult::SUCCESS == state) {
      const MsgId next_msg_id = g_message_deserializer.getReceivedMessageId();
      if (expected_msg_id != AnyMsgId && expected_msg_id != next_msg_id) {
        // treat unexpected message as invalid
        return InputProcessResult::MessageInvalid;
      }

      switch (next_msg_id) {
        case MsgId::Config:
        {
          ConfigData config_data;
          if (!g_message_deserializer.getPayload(config_data)) {
            return InputProcessResult::MessageInvalid;
          }

          g_right_wheel.configure(config_data.pid_rate, config_data.r_wheel);
          g_left_wheel.configure(config_data.pid_rate, config_data.l_wheel);

          // Deactivate the wheels torque until next velocity command
          deactivate();

          // send same config data in response
          sendSerialMessage(config_data);

          return InputProcessResult::Success;
        }
        case MsgId::Velocity:
        {
          VelocityData vel_data;
          if (!g_message_deserializer.getPayload(vel_data)) {
            return InputProcessResult::MessageInvalid;
          }

          g_right_wheel.setTargetVelocity(vel_data.right_wheel_velocity);
          g_left_wheel.setTargetVelocity(vel_data.left_wheel_velocity);

          // keep wheels torque active for velocity command
          activate();

          // set flag to send velocity message response
          g_received_velocity_msg = true;

          return InputProcessResult::Success;
        }
        case MsgId::Deactivate:
          // deactivate wheels torque
          deactivate();
          // send Deactivate message in response
          sendSerialMessage<MsgId::Deactivate>();
          return InputProcessResult::Success;

        default:
          return InputProcessResult::MessageInvalid;
      }
    } else if (ProcessResult::INCOMPLETE != state) {
      // treat all error states as invalid message
      return InputProcessResult::MessageInvalid;
    }
  }

  return result;
}

InputProcessResult processAllSerialInputMessages(const MsgId expected_msg_id) {
  auto next_msg_result = processNextSerialInputMessage(expected_msg_id);
  bool valid_message_found = (InputProcessResult::Success == next_msg_result);

  // MessageUnavailable is returned only if no serial input left
  // meaning all possible input messages are processed
  while(InputProcessResult::MessageUnavailable != next_msg_result) {
    next_msg_result = processNextSerialInputMessage(expected_msg_id);
    // accumulate valid message success result
    valid_message_found |= (InputProcessResult::Success == next_msg_result);
  }

  return valid_message_found ? InputProcessResult::Success : next_msg_result;
}

template<typename TData>
void sendSerialMessage(const TData& message_data) {
  uint8_t message[DiffDriveMessageSerializer::getFrameSize<TData>()];
  DiffDriveMessageSerializer::serialize(message_data, message);
  Serial.write(message, sizeof(message));
}

template<MsgId TargetId>
void sendSerialMessage() {
  uint8_t message[DiffDriveMessageSerializer::template getFrameSize<TargetId>()];
  DiffDriveMessageSerializer::template serialize<TargetId>(message);
  Serial.write(message, sizeof(message));
}

template<>
void sendSerialMessage<MsgId::Velocity>() {
  const VelocityData data{
    .right_wheel_velocity = g_right_wheel.getCurrentVelocity(),
    .left_wheel_velocity = g_left_wheel.getCurrentVelocity()
  };
  sendSerialMessage(data);
}

void processEmergencyDeactivation(const unsigned long current_time_ms) {
  if (current_time_ms - g_last_valid_msg_time_ms >= kEmergencyStopTimeoutMs) {
    deactivate();
  }
}

void activate(const unsigned long current_time_ms) {
  g_right_wheel.setActive();
  g_left_wheel.setActive();
  g_last_valid_msg_time_ms = current_time_ms;
}

void deactivate() {
  g_right_wheel.setActive(false);
  g_left_wheel.setActive(false);
}

void rightWheelEncoderCallback() {
  g_right_wheel.encoder().update();
}

void leftWheelEncoderCallback() {
  g_left_wheel.encoder().update();
}
