#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include <array>
#include <atomic>

#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

class WirelessConsole : public Print {
 public:
    struct Config {
        const char* ssid;
        const char* password;

        uint16_t port{23};

        // Bounded so logging can never consume unlimited RAM.
        size_t log_buffer_size{8192};

        UBaseType_t task_priority{1};
        BaseType_t cpu_core{0};
    };

    explicit WirelessConsole(const Config& config) : config_{config}, server_{config.port} {}

    WirelessConsole(const WirelessConsole&) = delete;
    WirelessConsole& operator=(const WirelessConsole&) = delete;

    bool begin() {
        if (log_stream_ != nullptr) {
            return true;
        }

        log_stream_ = xStreamBufferCreate(config_.log_buffer_size, 1);

        command_queue_ = xQueueCreate(8, sizeof(char));

        if (log_stream_ == nullptr || command_queue_ == nullptr) {
            return false;
        }

        const BaseType_t result =
            xTaskCreatePinnedToCore(&WirelessConsole::networkTaskEntry, "WirelessConsole", 4096,
                                    this, config_.task_priority, &network_task_, config_.cpu_core);

        return result == pdPASS;
    }

    // -------------------------------------------------------------------------
    // Arduino Print interface
    // -------------------------------------------------------------------------

    size_t write(uint8_t value) override { return write(&value, 1); }

    size_t write(const uint8_t* buffer, size_t size) override {
        if (buffer == nullptr || size == 0) {
            return 0;
        }

        // USB remains a useful development fallback, but don't depend on it.
        if (Serial) {
            Serial.write(buffer, size);
        }

        if (log_stream_ != nullptr) {
            const size_t queued = xStreamBufferSend(log_stream_, buffer, size,
                                                    0);  // NEVER BLOCK

            if (queued < size) {
                dropped_bytes_.fetch_add(size - queued, std::memory_order_relaxed);
            }
        }

        // From the application's perspective logging is best-effort.
        return size;
    }

    // -------------------------------------------------------------------------
    // Input
    // -------------------------------------------------------------------------

    bool readCommand(char& command) {
        // Preserve USB control during development.
        if (Serial.available() > 0) {
            command = static_cast<char>(Serial.read());
            return true;
        }

        if (command_queue_ == nullptr) {
            return false;
        }

        return xQueueReceive(command_queue_, &command, 0) == pdTRUE;
    }

    uint32_t droppedBytes() const noexcept {
        return dropped_bytes_.load(std::memory_order_relaxed);
    }

    // set board reset reason
    void setBootResetReason(esp_reset_reason_t reason) noexcept { boot_reset_reason_ = reason; }

 private:
    static void networkTaskEntry(void* context) {
        static_cast<WirelessConsole*>(context)->networkTask();
    }

    void networkTask() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(config_.ssid, config_.password);

        bool server_started{false};

        for (;;) {
            if (WiFi.status() != WL_CONNECTED) {
                if (client_) {
                    client_.stop();
                }

                server_started = false;

                // begin() already enables automatic reconnect behavior;
                // occasionally request reconnect as an additional recovery.
                WiFi.reconnect();

                // Discard stale log data while offline.
                drainWithoutClient();

                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (!server_started) {
                server_.begin();
                server_started = true;
            }

            acceptClient();

            processClientInput();

            if (client_ && client_.connected()) {
                sendPendingLogs();
            } else {
                drainWithoutClient();
            }

            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    void acceptClient() {
        WiFiClient candidate = server_.available();

        if (!candidate) {
            return;
        }

        // One diagnostic terminal at a time.
        if (client_) {
            client_.stop();
        }

        client_ = candidate;

        client_.println();
        client_.println("Bumperbot wireless diagnostic console");

        client_.printf("Previous reset: %s (%d)\r\n", resetReasonToString(boot_reset_reason_),
                       static_cast<int>(boot_reset_reason_));
        client_.printf("Free heap: %lu | minimum free heap: %lu\r\n",
                       static_cast<unsigned long>(esp_get_free_heap_size()),
                       static_cast<unsigned long>(esp_get_minimum_free_heap_size()));

        client_.printf("IP: %s | port: %u\r\n", WiFi.localIP().toString().c_str(), config_.port);
        client_.println("Commands: s=start, x=stop, h=help");
        client_.println();
    }

    void processClientInput() {
        if (!client_ || !client_.connected()) {
            return;
        }

        while (client_.available() > 0) {
            const int value = client_.read();

            if (value < 0) {
                break;
            }

            const char command = static_cast<char>(value);

            // Deliberately accept only known printable commands.
            //
            // This also ignores Telnet negotiation/control bytes.
            switch (command) {
                case 's':
                case 'S':
                case 'x':
                case 'X':
                case 'h':
                case 'H':
                    (void)xQueueSend(command_queue_, &command, 0);
                    break;

                default:
                    break;
            }
        }
    }

    void sendPendingLogs() {
        std::array<uint8_t, 256> buffer{};

        while (client_ && client_.connected()) {
            const size_t count = xStreamBufferReceive(log_stream_, buffer.data(), buffer.size(), 0);

            if (count == 0) {
                break;
            }

            // This may block/retry internally, but ONLY this low-priority
            // network task is affected.
            const size_t sent = client_.write(buffer.data(), count);

            if (sent < count) {
                dropped_bytes_.fetch_add(count - sent, std::memory_order_relaxed);

                break;
            }
        }
    }

    void drainWithoutClient() {
        std::array<uint8_t, 256> buffer{};

        while (xStreamBufferReceive(log_stream_, buffer.data(), buffer.size(), 0) != 0) {
        }
    }

    static const char* resetReasonToString(const esp_reset_reason_t reason) noexcept {
        switch (reason) {
            case ESP_RST_UNKNOWN:
                return "UNKNOWN";

            case ESP_RST_POWERON:
                return "POWERON";

            case ESP_RST_EXT:
                return "EXTERNAL";

            case ESP_RST_SW:
                return "SOFTWARE";

            case ESP_RST_PANIC:
                return "PANIC";

            case ESP_RST_INT_WDT:
                return "INTERRUPT_WATCHDOG";

            case ESP_RST_TASK_WDT:
                return "TASK_WATCHDOG";

            case ESP_RST_WDT:
                return "WATCHDOG";

            case ESP_RST_DEEPSLEEP:
                return "DEEPSLEEP";

            case ESP_RST_BROWNOUT:
                return "BROWNOUT";

            case ESP_RST_SDIO:
                return "SDIO";

            default:
                return "OTHER";
        }
    }

    Config config_;

    WiFiServer server_;
    WiFiClient client_;

    StreamBufferHandle_t log_stream_{nullptr};
    QueueHandle_t command_queue_{nullptr};
    TaskHandle_t network_task_{nullptr};
    esp_reset_reason_t boot_reset_reason_{ESP_RST_UNKNOWN};

    std::atomic<uint32_t> dropped_bytes_{0};
};