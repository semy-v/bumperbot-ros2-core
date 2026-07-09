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

#include "l298n_motor.hpp"
#include "quadrature_encoder.hpp"
#include "serial_message_processor.hpp"
#include "task_shared_data.hpp"
#include "wheel_controller.hpp"

namespace {
// CPU core 0 tasks
constexpr UBaseType_t kSerialProcessTaskPriority{1};
constexpr BaseType_t kSerialProcessTaskCpuCore{0};

constexpr UBaseType_t kSensorReadTaskPriority{2};
constexpr BaseType_t kSensorReadTaskCpuCore{0};

// CPU core 1 tasks
constexpr UBaseType_t kDiffDriveControlTaskPriority{2};
constexpr BaseType_t kDiffDriveControlTaskCpuCore{1};
}  // namespace

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    static TaskSharedData task_shared_data{
        .diff_drive_config_queue = xQueueCreate(1, sizeof(DiffDriveConfigData)),
        .diff_drive_command_queue = xQueueCreate(1, sizeof(DiffDriveVelocityData)),
        .diff_drive_state_queue = xQueueCreate(1, sizeof(DiffDriveVelocityData)),
        .diff_drive_control_task_handle = nullptr,
        .sensor_read_task_handle = nullptr};

    xTaskCreatePinnedToCore(diffDriveControlTask, "DiffDriveControlTask", 4096/*bytes*/, &task_shared_data,
                            kDiffDriveControlTaskPriority,
                            &task_shared_data.diff_drive_control_task_handle,
                            kDiffDriveControlTaskCpuCore);
    xTaskCreatePinnedToCore(sensorReadTask, "SensorReadTask", 2048/*bytes*/, &task_shared_data,
                            kSensorReadTaskPriority, &task_shared_data.sensor_read_task_handle,
                            kSensorReadTaskCpuCore);
    xTaskCreatePinnedToCore(serialProcessTask, "SerialProcessTask", 2048/*bytes*/, &task_shared_data,
                            kSerialProcessTaskPriority, nullptr, kSerialProcessTaskCpuCore);

    // Terminate the setup/loop task to free up CPU cycles
    vTaskDelete(nullptr);
}

void loop() {}
