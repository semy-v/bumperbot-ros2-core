#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "serial_message_processor.hpp"

void serialProcessTask(void *pvParameters) {
    SerialInputProcessor serial_processor(
        *static_cast<TaskSharedData*>(pvParameters));

    for (;;) {
        serial_processor.processAllSerialInputMessages();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}