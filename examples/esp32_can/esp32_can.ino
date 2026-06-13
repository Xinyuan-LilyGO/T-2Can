/*
 * @Description: ESP32 TWAI CAN transmit/receive test
 * @Author: LILYGO_L
 * @Date: 2026-06-12 16:30:00
 * @LastEditTime: 2026-06-12 16:58:56
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_err.h"
#include "pin_config.h"

namespace {

constexpr int kCanDataLength = 8;
constexpr TickType_t kCanTxWaitTicks = 0;
constexpr int kCanTxIntervalMs = 20;
constexpr int kCanBusOffDelayMs = 500;
constexpr int kCanRecoverRetryIntervalMs = 1000;
constexpr int kCanStatusPrintIntervalMs = 5000;
constexpr int kCanPollMs = 100;
constexpr uint32_t kCanTestId = 0x0F1;
constexpr char kCanTestChar = 'C';
constexpr int kCanTxQueueDepth = 16;
constexpr int kCanRxQueueDepth = 32;

constexpr int kPrintIntervalMs = 3000;
constexpr int kTaskStackSize = 6 * 1024;
constexpr UBaseType_t kTaskPriority = 5;

TaskHandle_t g_can_task_handle = nullptr;
volatile bool g_can_exit_test = false;
String g_console_buffer;
bool g_can_driver_installed = false;
bool g_can_driver_started = false;
bool g_can_recovering = false;
uint32_t g_can_pending_alerts = 0;
uint32_t g_can_last_status_print_ms = 0;
uint32_t g_can_last_recover_ms = 0;

void PrintCanFrame(const twai_message_t& frame)
{
  Serial.printf("[can receive] raw id=0x%lX dlc=%u ext=%d rtr=%d data:",
                static_cast<unsigned long>(frame.identifier),
                frame.data_length_code, frame.extd ? 1 : 0,
                frame.rtr ? 1 : 0);
  for (uint8_t i = 0; i < frame.data_length_code; ++i) {
    Serial.printf(" 0x%02X", frame.data[i]);
  }
  Serial.println();
}

uint32_t GetCanBusErrorCount()
{
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    return 0;
  }
  return status.bus_error_count;
}

void PrintCanTransferStatus(const char* tag, size_t total_size)
{
  Serial.printf("[%s] total %u B | bus error %lu\n", tag,
                static_cast<unsigned>(total_size),
                static_cast<unsigned long>(GetCanBusErrorCount()));
}

bool InitCan()
{
  g_can_recovering = false;
  g_can_pending_alerts = 0;
  g_can_last_status_print_ms = 0;
  g_can_last_recover_ms = 0;

  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(CAN_TX), static_cast<gpio_num_t>(CAN_RX),
      TWAI_MODE_NORMAL);
  general_config.tx_queue_len = kCanTxQueueDepth;
  general_config.rx_queue_len = kCanRxQueueDepth;

  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err =
      twai_driver_install(&general_config, &timing_config, &filter_config);
  if (err != ESP_OK) {
    Serial.printf("Install TWAI driver failed: %s\n", esp_err_to_name(err));
    return false;
  }
  g_can_driver_installed = true;

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("Start TWAI driver failed: %s\n", esp_err_to_name(err));
    twai_driver_uninstall();
    g_can_driver_installed = false;
    return false;
  }
  g_can_driver_started = true;

  const uint32_t alerts_to_enable =
      TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_BUS_RECOVERED;
  err = twai_reconfigure_alerts(alerts_to_enable, nullptr);
  if (err != ESP_OK) {
    Serial.printf("Configure TWAI alerts failed: %s\n", esp_err_to_name(err));
    twai_stop();
    twai_driver_uninstall();
    g_can_driver_started = false;
    g_can_driver_installed = false;
    return false;
  }

  Serial.println("[can] speed: 1000kbps");
  return true;
}

void PrintCanStatus(uint32_t alerts, bool force_print = false)
{
  g_can_pending_alerts |= alerts;
  const uint32_t now = millis();
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }

  const uint32_t pending_alerts = g_can_pending_alerts;
  if ((pending_alerts & TWAI_ALERT_BUS_RECOVERED) != 0) {
    g_can_recovering = false;
    if (g_can_driver_started) {
      const esp_err_t err = twai_start();
      if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        Serial.println("[can] bus recovered");
      }
    }
  }

  if (status.state == TWAI_STATE_BUS_OFF && !g_can_recovering &&
      now - g_can_last_recover_ms >= kCanRecoverRetryIntervalMs) {
    const esp_err_t err = twai_initiate_recovery();
    g_can_last_recover_ms = now;
    if (err == ESP_OK) {
      g_can_recovering = true;
      Serial.println("[can] bus off, start recovery");
    }
  }

  if (!force_print &&
      now - g_can_last_status_print_ms < kCanStatusPrintIntervalMs) {
    return;
  }

  g_can_pending_alerts = 0;
  const bool need_print = force_print || pending_alerts != 0 ||
                          status.state != TWAI_STATE_RUNNING;
  if (need_print) {
    Serial.printf("[can] state=%d tx_err=%lu rx_err=%lu tx_buffered=%lu "
                  "rx_buffered=%lu bus_err=%lu alerts=0x%08lX\n",
                  status.state,
                  static_cast<unsigned long>(status.tx_error_counter),
                  static_cast<unsigned long>(status.rx_error_counter),
                  static_cast<unsigned long>(status.msgs_to_tx),
                  static_cast<unsigned long>(status.msgs_to_rx),
                  static_cast<unsigned long>(status.bus_error_count),
                  static_cast<unsigned long>(pending_alerts));
    g_can_last_status_print_ms = now;
  }
}

void DeinitCan()
{
  if (g_can_driver_started) {
    twai_stop();
    g_can_driver_started = false;
  }
  if (g_can_driver_installed) {
    twai_driver_uninstall();
    g_can_driver_installed = false;
  }
}

void CanTask(void* param)
{
  const bool is_send = (param != nullptr);
  Serial.printf("can_test_task (%s) start\n", is_send ? "send" : "receive");

  if (!InitCan()) {
    g_can_task_handle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  size_t total_size = 0;
  uint32_t last_print_time = millis();

  if (is_send) {
    twai_message_t message = {};
    message.identifier = kCanTestId;
    message.data_length_code = kCanDataLength;
    for (int i = 0; i < kCanDataLength; ++i) {
      message.data[i] = kCanTestChar;
    }

    while (!g_can_exit_test) {
      uint32_t alerts = 0;
      twai_read_alerts(&alerts, 0);
      PrintCanStatus(alerts);

      twai_status_info_t status = {};
      if (twai_get_status_info(&status) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(kCanTxIntervalMs));
        continue;
      }
      if (status.state == TWAI_STATE_BUS_OFF ||
          status.state == TWAI_STATE_RECOVERING) {
        vTaskDelay(pdMS_TO_TICKS(kCanBusOffDelayMs));
        continue;
      }
      if (status.msgs_to_tx < kCanTxQueueDepth) {
        const esp_err_t err = twai_transmit(&message, kCanTxWaitTicks);
        if (err == ESP_OK) {
          total_size += kCanDataLength;
        }
      }

      const uint32_t now = millis();
      if (now - last_print_time >= kPrintIntervalMs) {
        PrintCanTransferStatus("can send", total_size);
        last_print_time = now;
      }
      vTaskDelay(pdMS_TO_TICKS(kCanTxIntervalMs));
    }
    PrintCanTransferStatus("can send", total_size);
  } else {
    while (!g_can_exit_test) {
      uint32_t alerts = 0;
      twai_read_alerts(&alerts, pdMS_TO_TICKS(kCanPollMs));
      PrintCanStatus(alerts);

      twai_message_t rx_message = {};
      while (twai_receive(&rx_message, 0) == ESP_OK) {
        if (!rx_message.rtr && !rx_message.extd &&
            rx_message.identifier == kCanTestId &&
            rx_message.data_length_code == kCanDataLength) {
          total_size += rx_message.data_length_code;
        } else {
          PrintCanFrame(rx_message);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      const uint32_t now = millis();
      if (now - last_print_time >= kPrintIntervalMs) {
        PrintCanTransferStatus("can receive", total_size);
        last_print_time = now;
      }
    }
    PrintCanTransferStatus("can receive", total_size);
  }

  DeinitCan();
  Serial.println("[can] task completed");
  g_can_task_handle = nullptr;
  vTaskDelete(nullptr);
}

String TrimCommandToken(String token)
{
  token.trim();
  return token;
}

void StopCanTask()
{
  if (g_can_task_handle == nullptr) {
    g_can_exit_test = false;
    return;
  }

  g_can_exit_test = true;
  for (int i = 0; i < 10; ++i) {
    if (g_can_task_handle == nullptr) {
      break;
    }
    delay(100);
  }
  if (g_can_task_handle != nullptr) {
    Serial.println("Stop CAN task timeout");
  }
  g_can_exit_test = false;
}

bool StartCanTask(bool is_send)
{
  Serial.printf("Start CAN %s command\n", is_send ? "send" : "receive");
  StopCanTask();
  if (g_can_task_handle != nullptr) {
    Serial.println("Start CAN failed: previous CAN task is still running");
    return false;
  }

  g_can_exit_test = false;
  if (xTaskCreate(CanTask, "can_task", kTaskStackSize,
                  is_send ? reinterpret_cast<void*>(1) : nullptr,
                  kTaskPriority, &g_can_task_handle) != pdPASS) {
    g_can_task_handle = nullptr;
    Serial.println("Create CAN task failed");
    return false;
  }
  return true;
}

void PrintCommandHelp()
{
  Serial.println("operation command:");
  Serial.println("[can:send:]");
  Serial.println("[can:receive:]");
  Serial.println("[all_exit:]");
}

bool ParseCommand(const String& command)
{
  if (command == "all_exit") {
    StopCanTask();
    Serial.println("all exit test");
    return true;
  }
  if (command == "can:send") {
    return StartCanTask(true);
  }
  if (command == "can:receive") {
    return StartCanTask(false);
  }

  Serial.println("parse_cmd fail (unknown command)");
  PrintCommandHelp();
  return false;
}

bool IsCompleteCommand(const String& content)
{
  const String command = TrimCommandToken(content);
  return command == "all_exit:" || command == "can:send:" ||
         command == "can:receive:";
}

void ParseCommandLine(String content)
{
  content = TrimCommandToken(content);
  if (content.endsWith(":")) {
    content.remove(content.length() - 1);
  }
  ParseCommand(content);
}

void AppendConsoleInput(const String& content)
{
  g_console_buffer += content;

  int line_end = -1;
  while ((line_end = g_console_buffer.indexOf('\n')) >= 0 ||
         (line_end = g_console_buffer.indexOf('\r')) >= 0) {
    String line = g_console_buffer.substring(0, line_end);
    g_console_buffer.remove(0, line_end + 1);
    line = TrimCommandToken(line);
    if (line.length() > 0) {
      ParseCommandLine(line);
    }
  }

  if (IsCompleteCommand(g_console_buffer)) {
    ParseCommandLine(g_console_buffer);
    g_console_buffer = "";
  }
}

}  // namespace

void setup()
{
  Serial.begin(115200);
  Serial.println("Ciallo");
  Serial.println("ESP32 CAN speed: 1000kbps");
  PrintCommandHelp();
}

void loop()
{
  while (Serial.available() > 0) {
    AppendConsoleInput(String(static_cast<char>(Serial.read())));
  }
  delay(10);
}
