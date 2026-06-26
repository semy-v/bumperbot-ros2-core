/**
 * @file main.cpp
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

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "quadrature_encoder.hpp"
#include "l298n_motor.hpp"
#include "wheel_controller.hpp"
#include "task_shared_data.hpp"
#include "serial_message_processor.hpp"


void setup() {
  Serial.begin(115200);
  while(!Serial);

  static TaskSharedData task_shared_data{
    .config_message_queue = xQueueCreate(1, sizeof(ConfigData)),
    .current_velocity_queue = xQueueCreate(1, sizeof(VelocityData)),
    .target_velocity_message_queue = xQueueCreate(1, sizeof(VelocityData)),
    .diff_drive_control_task_handle = nullptr
  };

  {
    SerialInputProcessor input_processor(task_shared_data);
    bool init_successful{false};
    while(!init_successful) {
      const auto result = input_processor.processNextSerialInputMessage(MsgId::Config);
      init_successful = (SerialInputProcessor::Result::Success == result);
    }
  }

  // CPU core 0 tasks
  xTaskCreatePinnedToCore(serialProcessTask, "SerialProcessTask", 4096, &task_shared_data, 2, NULL, 0);

  // CPU core 1 tasks
  xTaskCreatePinnedToCore(diffDriveControlTask, "DiffDriveControlTask", 4096, &task_shared_data, 2,
    &task_shared_data.diff_drive_control_task_handle, 1);
}

void loop() {}
