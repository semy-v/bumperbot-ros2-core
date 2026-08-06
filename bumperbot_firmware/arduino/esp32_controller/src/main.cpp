/**
 * @file main.cpp
 *
 * SERIAL COMMUNICATION PROTOCOL SEQUENCE (ARDUINO PERSPECTIVE)
 * ------------------------------------------------------------
 * This microcontroller acts as the "Slave" in the communication protocol. It responds
 * to commands from the ROS 2 hardware interface and handles the real-time physical
 * constraints (PID loops, encoder hardware interrupts, failsafes) utilizing a
 * multi-core FreeRTOS architecture.
 *
 * 1. STARTUP & INITIALIZATION (setup)
 * - The system initializes three FreeRTOS tasks pinned to specific CPU cores
 * (`DiffDriveControlTask`, `SensorReadTask`, `SerialProcessTask`) and terminates the main setup
 * loop to free CPU cycles.
 * - `DiffDriveControlTask` enters a blocking loop, strictly awaiting `DiffDriveConfigData` via a
 * FreeRTOS queue before configuring the wheel controllers.
 * - `SensorReadTask` awaits an `ImuConfig` notification event to initialize and calibrate the
 * MPU6050 IMU, subsequently echoing the configuration result back over serial.
 *
 * 2. ASYNCHRONOUS REAL-TIME LOOP
 * - `SerialProcessTask` (Core 0) continuously polls the serial buffer for incoming payloads using
 * the `SerialInputProcessor`.
 * - When a `DiffDriveCommand` message arrives, the target velocities are pushed to the
 * `diff_drive_command_queue`.
 * - Concurrently, a task notification is dispatched to `SensorReadTask` containing the
 * `response_delay_ms` parameter as its payload.
 * - `SensorReadTask` delays for the commanded time, reads IMU telemetry, peeks the latest physical
 * wheel velocities from the `diff_drive_state_queue`, and transmits the unified `SystemStateData`
 * back to ROS 2.
 * - On Core 1, `DiffDriveControlTask` continuously reads commands from the queue, runs the PID loop
 * updates for both wheels, and overwrites the state queue with the current physical
 * velocities.
 *
 * 3. FAILSAFE WATCHDOG
 * - `DiffDriveControlTask` tracks the FreeRTOS tick time elapsed since the last valid velocity
 * command was received.
 * - If no command is received within `kEmergencyStopTimeoutMs` (2000ms), it safely de-energizes
 * both motors by calling `deactivateWheels()` to prevent runaway behavior.
 *
 * 4. DEACTIVATION
 * - If a `MsgId::Deactivate` message is parsed, the serial processor sends a notification index to
 * `DiffDriveControlTask`.
 * - The control task immediately drops motor torque in response to the notification.
 * - The serial processor instantly echoes the `Deactivate` message back to ROS 2 as an
 * acknowledgement.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "serial_message_processor.hpp"
#include "task_shared_data.hpp"

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

    xTaskCreatePinnedToCore(diffDriveControlTask, "DiffDriveControlTask", 4096 /*bytes*/,
                            &task_shared_data, kDiffDriveControlTaskPriority,
                            &task_shared_data.diff_drive_control_task_handle,
                            kDiffDriveControlTaskCpuCore);
    xTaskCreatePinnedToCore(sensorReadTask, "SensorReadTask", 2048 /*bytes*/, &task_shared_data,
                            kSensorReadTaskPriority, &task_shared_data.sensor_read_task_handle,
                            kSensorReadTaskCpuCore);
    xTaskCreatePinnedToCore(serialProcessTask, "SerialProcessTask", 2048 /*bytes*/,
                            &task_shared_data, kSerialProcessTaskPriority, nullptr,
                            kSerialProcessTaskCpuCore);

    // Terminate the setup/loop task to free up CPU cycles
    vTaskDelete(nullptr);
}

void loop() {}
