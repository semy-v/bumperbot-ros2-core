#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include "protocol/system_data.hpp"
#include "serial_message_processor.hpp"
#include "task_shared_data.hpp"

namespace {

// MPU6050 produces samples at 100 Hz. Treat data as unavailable after five
// missed periods rather than sending a stale valid-looking measurement forever.
constexpr TickType_t kImuMaxAgeTicks{pdMS_TO_TICKS(50U)};
constexpr TickType_t kSerialLoopDelayTicks{pdMS_TO_TICKS(1U)};

void sendImuConfigResponseIfAvailable(TaskSharedData& shared_data) {
    ImuConfigData response{};
    if (xQueueReceive(shared_data.imu_config_response_queue, &response, 0) == pdPASS) {
        sendSerialMessage(response);
    }
}

void fillLatestImuState(TaskSharedData& shared_data, SystemStateData& state) {
    ImuTaskState imu_state{};
    if (xQueuePeek(shared_data.imu_state_queue, &imu_state, 0) != pdPASS || !imu_state.valid) {
        state.status = SystemStateFlags::ImuUnavailable;
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t age = now - imu_state.sample_time_ticks;  // wrap-safe unsigned subtraction
    if (age > kImuMaxAgeTicks) {
        state.status = SystemStateFlags::ImuUnavailable;
        return;
    }

    state.imu = imu_state.data;
}

void sendSystemStateResponseIfDue(TaskSharedData& shared_data) {
    SystemStateResponseRequest request{};
    if (xQueuePeek(shared_data.system_state_response_queue, &request, 0) != pdPASS) {
        return;
    }

    if (esp_timer_get_time() < request.due_time_us) {
        return;
    }

    // Remove the reservation only when its response is actually being sent.
    // This keeps overlapping DiffDriveCommand messages rejected until now.
    if (xQueueReceive(shared_data.system_state_response_queue, &request, 0) != pdPASS) {
        return;
    }

    SystemStateData state{
        .status = SystemStateFlags::None,
        .imu = {},
        .diff_drive = {},
    };

    fillLatestImuState(shared_data, state);

    // Non-blocking snapshot of the latest wheel-control state. Queue length is
    // one and DiffDriveControlTask overwrites it every control period.
    (void)xQueuePeek(shared_data.diff_drive_state_queue, &state.diff_drive.velocity, 0);

    sendSerialMessage(state);
}

}  // namespace

void serialProcessTask(void* pvParameters) {
    auto& shared_data = *static_cast<TaskSharedData*>(pvParameters);
    SerialInputProcessor serial_processor(shared_data);

    for (;;) {
        // Check before parsing so an already-due response is not delayed by a
        // newly arriving frame.
        sendSystemStateResponseIfDue(shared_data);

        // Process at most one complete frame per iteration. This gives a newly
        // parsed zero-delay command an immediate response check below instead
        // of draining an arbitrarily long serial backlog first.
        (void)serial_processor.processNextSerialInputMessage();

        // SensorReadTask places calibration results in a queue; SerialProcessTask
        // remains the sole Serial TX owner.
        sendImuConfigResponseIfAvailable(shared_data);

        // Handles response_delay_ms == 0 in the same iteration that parsed the
        // command, and positive delays with ~1 ms scheduling resolution.
        sendSystemStateResponseIfDue(shared_data);

        vTaskDelay(kSerialLoopDelayTicks);
    }
}