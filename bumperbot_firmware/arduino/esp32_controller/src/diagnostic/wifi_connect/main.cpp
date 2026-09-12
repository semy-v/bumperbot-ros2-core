#include <Arduino.h>
#include <WiFi.h>
#include "wifi_credentials.hpp"

constexpr uint32_t kSerialBaudRate{115200U};
constexpr const char* configured_ssid = wifi_credentials::kSsid;
constexpr const char* configured_password = wifi_credentials::kPassword;

constexpr const char* toStr(const wl_status_t status) {
    switch (status) {
        case WL_IDLE_STATUS:
            return "WL_IDLE_STATUS";
        case WL_NO_SSID_AVAIL:
            return "WL_NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:
            return "WL_SCAN_COMPLETED";
        case WL_CONNECTED:
            return "WL_CONNECTED";
        case WL_CONNECT_FAILED:
            return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
            return "WL_CONNECTION_LOST";
        case WL_DISCONNECTED:
            return "WL_DISCONNECTED";
        case WL_NO_SHIELD:
            return "WL_NO_SHIELD";
        default:
            return "UNDEFINED";
    }
}

void setup() {
    Serial.begin(kSerialBaudRate);

    while (!Serial) {
        delay(100);
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(configured_ssid, configured_password);
}

void loop() {
    Serial.printf("wifi connection status: %s\n", toStr(WiFi.status()));
    delay(1000);
}