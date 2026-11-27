/*
 * @Description: original_test_http (Control Center UI + Dual CAN V2.0B 500kbps)
 */

#include <Arduino.h>
#include <SPI.h>
#include <WebServer.h>
#include <esp_mac.h>
#include <stdarg.h>
#include <time.h>

#include "WiFi.h"
#include "driver/twai.h"
#include "index_html.h"
#include "pin_config.h"

#if defined T_2Can
#include "mcp2515.h"
#elif defined T_2Can_Fd
#include "mcp2518fd_can.h"
#else
#error "no macro definition is set"
#endif

// Timing & Wi-Fi configuration
#define POLLING_RATE_MS 20
#define CAN_AUTO_INTERVAL_MS 1500UL
#define CAN_TX_INTERVAL_MS 500UL
#define STATUS_UPDATE_INTERVAL_MS 500UL
#define WIFI_RETRY_INTERVAL_MS 10000UL
#define NTP_SYNC_INTERVAL_MS 20000UL

#define WIFI_SSID_1 "LilyGo-AABB"
#define WIFI_PASSWORD_1 "xinyuandianzi"
#define WIFI_SSID_2 "xinyuandianzi"
#define WIFI_PASSWORD_2 "AA15994823428"

#define WIFI_TIME_ZONE_OFFSET_SEC (8 * 60 * 60)
#define WIFI_DAYLIGHT_OFFSET_SEC 0
#define WIFI_AP_SSID_PREFIX "T-2Can_"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONNECTIONS 4
#define WEB_LOG_MAX_LENGTH 12000

#define MAX_DATA_SIZE 8

#if defined T_2Can
#define BOARD_NAME "T-2Can"
#define CAN_A_MODULE_NAME "MCP2515"
#define CAN_A_PROTOCOL_NAME "CAN V2.0B"
#define CAN_A_SPEED_DESC "500kbps"
#elif defined T_2Can_Fd
#define BOARD_NAME "T-2Can-Fd"
#define CAN_A_MODULE_NAME "MCP2518FD"
#define CAN_A_PROTOCOL_NAME "CAN V2.0B"
#define CAN_A_SPEED_DESC "500kbps"
#endif

#define CAN_B_MODULE_NAME "ESP32-S3 TWAI"
#define CAN_B_PROTOCOL_NAME "CAN V2.0B"
#define CAN_B_SPEED_DESC "500kbps"

#define SOFTWARE_NAME "Original_Test_Http"
#define SOFTWARE_LASTEDITTIME "202611271500"
#define BOARD_VERSION "V1.0"

enum class CanMode
{
    kAuto,
    kSend,
    kReceive,
    kStop,
};

struct WifiNetworkCredential
{
    const char *ssid;
    const char *password;
};

static const WifiNetworkCredential Wifi_Networks[] = {
    {WIFI_SSID_1, WIFI_PASSWORD_1},
    {WIFI_SSID_2, WIFI_PASSWORD_2},
};
static const size_t WIFI_NETWORK_COUNT = sizeof(Wifi_Networks) / sizeof(Wifi_Networks[0]);

static CanMode g_can_a_mode = CanMode::kAuto;
static CanMode g_can_b_mode = CanMode::kAuto;
static bool g_auto_send_a_turn = true;
static uint32_t g_last_auto_tick = 0;
static uint32_t g_last_can_a_tx_tick = 0;
static uint32_t g_last_can_b_tx_tick = 0;

static bool g_can_a_init_ok = false;
static bool g_can_b_init_ok = false;
static uint32_t g_can_a_tx_count = 0;
static uint32_t g_can_a_rx_count = 0;
static uint32_t g_can_a_rx_bytes = 0;
static uint32_t g_can_a_err_count = 0;
static uint32_t g_can_a_last_rx_id = 0;
static char g_can_a_last_rx_data[48] = "-";

static uint32_t g_can_b_tx_count = 0;
static uint32_t g_can_b_rx_count = 0;
static uint32_t g_can_b_rx_bytes = 0;
static uint32_t g_can_b_bus_errors = 0;
static uint32_t g_can_b_last_rx_id = 0;
static char g_can_b_last_rx_data[48] = "-";
static char g_can_b_state_text[24] = "off";

static bool Wifi_AP_Flag = false;
static char Wifi_AP_SSID[sizeof(WIFI_AP_SSID_PREFIX) + 12] = "T-2Can";
static char g_ap_ip[16] = "192.168.4.1";
static bool g_wifi_sta_connected = false;
static char g_sta_ssid[33] = "-";
static char g_sta_ip[16] = "0.0.0.0";
static int8_t g_sta_rssi = 0;
static size_t g_wifi_network_index = 0;
static uint32_t g_last_wifi_attempt_ms = 0;

static bool g_ntp_configured = false;
static char g_time_text[32] = "syncing";
static uint32_t g_time_age_seconds = 0;
static uint32_t g_last_time_check_ms = 0;
static uint32_t g_last_second_ms = 0;

static String Web_Log_Buffer;
static char g_status_json[2048] = {};
WebServer Http_Server(80);

#if defined T_2Can
struct can_frame Can_Receive_Package;
struct can_frame Can_Send_Package;
MCP2515 Can_A(MCP2515_CS, 10000000, &SPI);
#elif defined T_2Can_Fd
uint8_t Can_Receive_Package[MAX_DATA_SIZE];
uint8_t Can_Send_Package[MAX_DATA_SIZE];
mcp2518fd Can_A(MCP2518_CS);
#endif

void AppendWebLog(const String &message)
{
    Web_Log_Buffer += message;
    if (Web_Log_Buffer.length() > WEB_LOG_MAX_LENGTH)
    {
        Web_Log_Buffer.remove(0, Web_Log_Buffer.length() - WEB_LOG_MAX_LENGTH);
    }
}

void AppLogPrint(const String &message)
{
    Serial.print(message);
    AppendWebLog(message);
}

void AppLogPrintln(const String &message = "")
{
    Serial.println(message);
    AppendWebLog(message + "\n");
}

void AppLogPrintf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0)
    {
        return;
    }
    if ((size_t)length < sizeof(buffer))
    {
        AppLogPrint(String(buffer));
        return;
    }
    char *dynamic_buffer = (char *)malloc(length + 1);
    if (dynamic_buffer == NULL)
    {
        AppLogPrint(String(buffer));
        return;
    }
    va_start(args, format);
    vsnprintf(dynamic_buffer, length + 1, format, args);
    va_end(args);
    AppLogPrint(String(dynamic_buffer));
    free(dynamic_buffer);
}

const char *ModeName(CanMode mode)
{
    switch (mode)
    {
    case CanMode::kAuto:
        return "auto";
    case CanMode::kSend:
        return "send";
    case CanMode::kReceive:
        return "receive";
    default:
        return "stop";
    }
}

CanMode ParseMode(const String &mode)
{
    if (mode == "auto")
    {
        return CanMode::kAuto;
    }
    if (mode == "send")
    {
        return CanMode::kSend;
    }
    if (mode == "receive")
    {
        return CanMode::kReceive;
    }
    return CanMode::kStop;
}

const char *TwaiStateName(twai_state_t state)
{
    switch (state)
    {
    case TWAI_STATE_RUNNING:
        return "active";
    case TWAI_STATE_BUS_OFF:
        return "bus_off";
    case TWAI_STATE_STOPPED:
        return "stopped";
    case TWAI_STATE_RECOVERING:
        return "recovering";
    default:
        return "unknown";
    }
}

void FormatDataPreview(char *out, size_t out_size, const uint8_t *data, uint8_t len)
{
    if (len == 0)
    {
        snprintf(out, out_size, "-");
        return;
    }
    size_t pos = 0;
    for (uint8_t i = 0; i < len && pos + 5 < out_size; ++i)
    {
        int written = snprintf(out + pos, out_size - pos, "%s%02X", i == 0 ? "" : " ", data[i]);
        if (written <= 0)
        {
            break;
        }
        pos += (size_t)written;
    }
}

#if defined T_2Can
uint8_t MCP2515_Read_Register(uint8_t address)
{
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(MCP2515_CS, LOW);
    SPI.transfer(0x03);
    SPI.transfer(address);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(MCP2515_CS, HIGH);
    SPI.endTransaction();
    return value;
}
#elif defined T_2Can_Fd
uint32_t MCP2518_Read_Register(uint16_t address)
{
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(MCP2518_CS, LOW);
    SPI.transfer((cINSTRUCTION_READ << 4) | ((address >> 8) & 0x0F));
    SPI.transfer(address & 0xFF);
    uint32_t value = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        value |= (uint32_t)SPI.transfer(0x00) << (8 * i);
    }
    digitalWrite(MCP2518_CS, HIGH);
    SPI.endTransaction();
    return value;
}
#endif

void Can_A_Drive_Initialization()
{
#if defined T_2Can
    Can_Send_Package.can_id = 0xAA;
    Can_Send_Package.can_dlc = 8;
    Can_Send_Package.data[0] = 8;
    Can_Send_Package.data[1] = 7;
    Can_Send_Package.data[2] = 6;
    Can_Send_Package.data[3] = 5;
    Can_Send_Package.data[4] = 4;
    Can_Send_Package.data[5] = 3;
    Can_Send_Package.data[6] = 2;
    Can_Send_Package.data[7] = 1;

    pinMode(MCP2515_RST, OUTPUT);
    digitalWrite(MCP2515_RST, HIGH);
    delay(50);
    digitalWrite(MCP2515_RST, LOW);
    delay(50);
    digitalWrite(MCP2515_RST, HIGH);
    delay(50);

    SPI.begin(MCP2515_SCLK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS);

    MCP2515::ERROR reset_result = Can_A.reset();
    uint8_t config_status = MCP2515_Read_Register(0x0E);
    bool chip_ok = reset_result == MCP2515::ERROR_OK && (config_status & 0xE0) == 0x80;
    MCP2515::ERROR bitrate_result = Can_A.setBitrate(CAN_500KBPS);
    MCP2515::ERROR mode_result = Can_A.setNormalMode();
    g_can_a_init_ok = chip_ok && bitrate_result == MCP2515::ERROR_OK && mode_result == MCP2515::ERROR_OK;

    AppLogPrintf("[CAN A][MCP2515] Init %s (CANSTAT=0x%02X, protocol=%s, speed=%s)\n",
                 g_can_a_init_ok ? "OK" : "FAIL", (unsigned int)config_status,
                 CAN_A_PROTOCOL_NAME, CAN_A_SPEED_DESC);
#elif defined T_2Can_Fd
    for (uint8_t i = 0; i < MAX_DATA_SIZE; ++i)
    {
        Can_Send_Package[i] = 0xA0 + i;
    }

    Can_A.setMode(CAN_CLASSIC_MODE);
    SPI.begin(MCP2518_SCLK, MCP2518_MISO, MCP2518_MOSI, MCP2518_CS);

    uint8_t begin_result = Can_A.begin(CAN20_500KBPS);
    Can_A.__flgFDF = 0; // Force Classic CAN 2.0B frames

    uint32_t control = MCP2518_Read_Register(cREGADDR_CiCON);
    uint32_t nominal_timing = MCP2518_Read_Register(cREGADDR_CiNBTCFG);
    uint32_t device_id = MCP2518_Read_Register(cREGADDR_DEVID);
    bool device_id_ok = device_id != 0 && device_id != 0xFFFFFFFFUL;
    bool chip_ok = device_id_ok && control != 0 && control != 0xFFFFFFFFUL &&
                   nominal_timing != 0 && nominal_timing != 0xFFFFFFFFUL;
    uint8_t actual_mode = (control >> 21) & 0x07;
    g_can_a_init_ok = (begin_result == CAN_OK) && chip_ok && (actual_mode == CAN_CLASSIC_MODE);

    AppLogPrintf("[CAN A][MCP2518FD] Init %s (CiCON=0x%08lX, DEVID=0x%08lX, OPMOD=%u, protocol=%s, speed=%s)\n",
                 g_can_a_init_ok ? "OK" : "FAIL", (unsigned long)control,
                 (unsigned long)device_id, (unsigned int)actual_mode,
                 CAN_A_PROTOCOL_NAME, CAN_A_SPEED_DESC);
#endif
}

void Can_B_Drive_Initialization()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    bool installed = (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK);
    bool started = installed && (twai_start() == ESP_OK);

    uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS |
                                TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_DATA |
                                TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF;
    bool alerts_ok = started && (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK);
    g_can_b_init_ok = installed && started && alerts_ok;
    snprintf(g_can_b_state_text, sizeof(g_can_b_state_text), "%s", g_can_b_init_ok ? "active" : "error");

    AppLogPrintf("[CAN B][TWAI] Init %s (protocol=%s, speed=%s)\n",
                 g_can_b_init_ok ? "OK" : "FAIL", CAN_B_PROTOCOL_NAME, CAN_B_SPEED_DESC);
}

void Can_A_Send_Message()
{
    if (!g_can_a_init_ok)
    {
        return;
    }
#if defined T_2Can
    if (Can_A.sendMessage(&Can_Send_Package) == MCP2515::ERROR_OK)
    {
        ++g_can_a_tx_count;
        AppLogPrintf("[CAN A] TX id=0x%X len=%d\n", Can_Send_Package.can_id, Can_Send_Package.can_dlc);
    }
    else
    {
        ++g_can_a_err_count;
        AppLogPrintln("[CAN A] TX failed");
    }
#elif defined T_2Can_Fd
    if (Can_A.sendMsgBuf(0xAA, 0, MAX_DATA_SIZE, Can_Send_Package) == CAN_OK)
    {
        ++g_can_a_tx_count;
        AppLogPrintf("[CAN A] TX id=0xAA len=%d\n", MAX_DATA_SIZE);
    }
    else
    {
        ++g_can_a_err_count;
        AppLogPrintln("[CAN A] TX failed");
    }
#endif
}

void Can_B_Twai_Send_Message()
{
    if (!g_can_b_init_ok)
    {
        return;
    }
    twai_message_t message = {};
    message.identifier = 0xBB;
    message.data_length_code = 8;
    for (uint8_t i = 0; i < 8; ++i)
    {
        message.data[i] = i + 1;
    }

    if (twai_transmit(&message, pdMS_TO_TICKS(50)) == ESP_OK)
    {
        ++g_can_b_tx_count;
        AppLogPrintf("[CAN B] TX id=0x%lX len=%d\n", (unsigned long)message.identifier, message.data_length_code);
    }
    else
    {
        AppLogPrintln("[CAN B] TX queue failed");
    }
}

void Poll_Can_A_Receive()
{
    if (!g_can_a_init_ok)
    {
        return;
    }
#if defined T_2Can
    uint8_t irq = Can_A.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF)
    {
        if (Can_A.readMessage(MCP2515::RXB0, &Can_Receive_Package) == MCP2515::ERROR_OK)
        {
            ++g_can_a_rx_count;
            g_can_a_rx_bytes += Can_Receive_Package.can_dlc;
            g_can_a_last_rx_id = Can_Receive_Package.can_id;
            FormatDataPreview(g_can_a_last_rx_data, sizeof(g_can_a_last_rx_data),
                              Can_Receive_Package.data, Can_Receive_Package.can_dlc);
            AppLogPrintf("[CAN A] RX id=0x%lX len=%u data=[%s]\n",
                         (unsigned long)g_can_a_last_rx_id, Can_Receive_Package.can_dlc, g_can_a_last_rx_data);
        }
    }
    else if (irq & MCP2515::CANINTF_RX1IF)
    {
        if (Can_A.readMessage(MCP2515::RXB1, &Can_Receive_Package) == MCP2515::ERROR_OK)
        {
            ++g_can_a_rx_count;
            g_can_a_rx_bytes += Can_Receive_Package.can_dlc;
            g_can_a_last_rx_id = Can_Receive_Package.can_id;
            FormatDataPreview(g_can_a_last_rx_data, sizeof(g_can_a_last_rx_data),
                              Can_Receive_Package.data, Can_Receive_Package.can_dlc);
            AppLogPrintf("[CAN A] RX id=0x%lX len=%u data=[%s]\n",
                         (unsigned long)g_can_a_last_rx_id, Can_Receive_Package.can_dlc, g_can_a_last_rx_data);
        }
    }
#elif defined T_2Can_Fd
    while (CAN_MSGAVAIL == Can_A.checkReceive())
    {
        uint8_t len = 0;
        Can_A.readMsgBuf(&len, Can_Receive_Package);
        unsigned long id = Can_A.getCanId();
        if (len > MAX_DATA_SIZE)
        {
            len = MAX_DATA_SIZE;
        }
        ++g_can_a_rx_count;
        g_can_a_rx_bytes += len;
        g_can_a_last_rx_id = (uint32_t)id;
        FormatDataPreview(g_can_a_last_rx_data, sizeof(g_can_a_last_rx_data), Can_Receive_Package, len);
        AppLogPrintf("[CAN A] RX id=0x%lX len=%u data=[%s]\n",
                     (unsigned long)g_can_a_last_rx_id, len, g_can_a_last_rx_data);
    }
#endif
}

void Poll_Can_B_Twai()
{
    if (!g_can_b_init_ok)
    {
        return;
    }

    uint32_t alerts_triggered = 0;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

    twai_status_info_t status_info = {};
    if (twai_get_status_info(&status_info) == ESP_OK)
    {
        g_can_b_bus_errors = status_info.bus_error_count;
        snprintf(g_can_b_state_text, sizeof(g_can_b_state_text), "%s", TwaiStateName(status_info.state));
        if (status_info.state == TWAI_STATE_BUS_OFF)
        {
            twai_initiate_recovery();
        }
        else if (status_info.state == TWAI_STATE_STOPPED)
        {
            twai_start();
        }
    }

    if (alerts_triggered & TWAI_ALERT_RX_DATA)
    {
        twai_message_t rx_buf = {};
        while (twai_receive(&rx_buf, 0) == ESP_OK)
        {
            if (g_can_b_mode == CanMode::kStop || g_can_b_mode == CanMode::kSend)
            {
                continue;
            }
            ++g_can_b_rx_count;
            g_can_b_rx_bytes += rx_buf.data_length_code;
            g_can_b_last_rx_id = rx_buf.identifier;
            FormatDataPreview(g_can_b_last_rx_data, sizeof(g_can_b_last_rx_data),
                              rx_buf.data, rx_buf.data_length_code);
            AppLogPrintf("[CAN B] RX id=0x%lX len=%u data=[%s]\n",
                         (unsigned long)g_can_b_last_rx_id, rx_buf.data_length_code, g_can_b_last_rx_data);
        }
    }
}

void BuildWifiAPSSID(void)
{
    uint8_t factory_mac[6] = {0};
    esp_efuse_mac_get_default(factory_mac);
    snprintf(Wifi_AP_SSID, sizeof(Wifi_AP_SSID),
             WIFI_AP_SSID_PREFIX "%02X%02X%02X%02X%02X%02X",
             factory_mac[0], factory_mac[1], factory_mac[2],
             factory_mac[3], factory_mac[4], factory_mac[5]);
}

const char *StatusJson()
{
    snprintf(
        g_status_json, sizeof(g_status_json),
        "{"
        "\"board\":\"%s\",\"software\":\"%s\",\"version\":\"%s\","
        "\"time\":\"%s\",\"time_age_seconds\":%lu,"
        "\"wifi\":{\"sta\":%s,\"sta_ssid\":\"%s\",\"sta_ip\":\"%s\",\"sta_rssi\":%d,"
        "\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\"},"
        "\"can_a\":{\"module\":\"%s\",\"protocol\":\"%s\",\"speed\":\"%s\",\"init\":%s,"
        "\"mode\":\"%s\",\"tx\":%lu,\"rx\":%lu,\"rx_bytes\":%lu,\"errors\":%lu,"
        "\"last_id\":\"0x%lX\",\"last_data\":\"%s\"},"
        "\"can_b\":{\"module\":\"%s\",\"protocol\":\"%s\",\"speed\":\"%s\",\"init\":%s,"
        "\"mode\":\"%s\",\"state\":\"%s\",\"tx\":%lu,\"rx\":%lu,\"rx_bytes\":%lu,\"errors\":%lu,"
        "\"last_id\":\"0x%lX\",\"last_data\":\"%s\"}"
        "}",
        BOARD_NAME, SOFTWARE_NAME, BOARD_VERSION,
        g_time_text, (unsigned long)g_time_age_seconds,
        g_wifi_sta_connected ? "true" : "false", g_sta_ssid, g_sta_ip, (int)g_sta_rssi,
        Wifi_AP_SSID, g_ap_ip,
        CAN_A_MODULE_NAME, CAN_A_PROTOCOL_NAME, CAN_A_SPEED_DESC, g_can_a_init_ok ? "true" : "false",
        ModeName(g_can_a_mode), (unsigned long)g_can_a_tx_count, (unsigned long)g_can_a_rx_count,
        (unsigned long)g_can_a_rx_bytes, (unsigned long)g_can_a_err_count,
        (unsigned long)g_can_a_last_rx_id, g_can_a_last_rx_data,
        CAN_B_MODULE_NAME, CAN_B_PROTOCOL_NAME, CAN_B_SPEED_DESC, g_can_b_init_ok ? "true" : "false",
        ModeName(g_can_b_mode), g_can_b_state_text, (unsigned long)g_can_b_tx_count,
        (unsigned long)g_can_b_rx_count, (unsigned long)g_can_b_rx_bytes,
        (unsigned long)g_can_b_bus_errors, (unsigned long)g_can_b_last_rx_id, g_can_b_last_rx_data);
    return g_status_json;
}

void HandleRoot()
{
    Http_Server.send_P(200, "text/html; charset=utf-8", kIndexHtml);
}

void HandleStatus()
{
    Http_Server.send(200, "application/json", StatusJson());
}

void HandleSet()
{
    String target = Http_Server.arg("target");
    String mode = Http_Server.arg("mode");
    if (target == "can_a")
    {
        g_can_a_mode = ParseMode(mode);
        AppLogPrintf("[UI] CAN A mode -> %s\n", ModeName(g_can_a_mode));
    }
    else if (target == "can_b")
    {
        g_can_b_mode = ParseMode(mode);
        AppLogPrintf("[UI] CAN B mode -> %s\n", ModeName(g_can_b_mode));
    }
    Http_Server.send(200, "application/json", "{\"ok\":true}");
}

void HandleLogText()
{
    Http_Server.send(200, "text/plain; charset=utf-8", Web_Log_Buffer);
}

void HandleClearLog()
{
    Web_Log_Buffer = "";
    Http_Server.send(200, "application/json", "{\"ok\":true}");
}

void Wifi_AP_STA_HTTP_Init(void)
{
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    BuildWifiAPSSID();
    Wifi_AP_Flag = WiFi.softAP(Wifi_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CONNECTIONS);
    IPAddress ip = WiFi.softAPIP();
    snprintf(g_ap_ip, sizeof(g_ap_ip), "%s", ip.toString().c_str());

    Http_Server.on("/", HTTP_GET, HandleRoot);
    Http_Server.on("/api/status", HTTP_GET, HandleStatus);
    Http_Server.on("/api/set", HTTP_GET, HandleSet);
    Http_Server.on("/log", HTTP_GET, HandleLogText);
    Http_Server.on("/clear", HTTP_GET, HandleClearLog);
    Http_Server.begin();

    AppLogPrintf("[WiFi] AP %s | SSID: %s | Password: %s | URL: http://%s/\n",
                 Wifi_AP_Flag ? "started" : "failed", Wifi_AP_SSID, WIFI_AP_PASSWORD, g_ap_ip);

    WiFi.begin(Wifi_Networks[g_wifi_network_index].ssid, Wifi_Networks[g_wifi_network_index].password);
    g_last_wifi_attempt_ms = millis();
    AppLogPrintf("[WiFi] Connecting STA to %s...\n", Wifi_Networks[g_wifi_network_index].ssid);
}

void ServiceWifiAndTime()
{
    uint32_t now = millis();
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected)
    {
        if (!g_wifi_sta_connected)
        {
            g_wifi_sta_connected = true;
            snprintf(g_sta_ssid, sizeof(g_sta_ssid), "%s", WiFi.SSID().c_str());
            snprintf(g_sta_ip, sizeof(g_sta_ip), "%s", WiFi.localIP().toString().c_str());
            g_sta_rssi = (int8_t)WiFi.RSSI();
            AppLogPrintf("[WiFi] STA connected: %s (%s, %d dBm)\n", g_sta_ssid, g_sta_ip, g_sta_rssi);
            if (!g_ntp_configured)
            {
                configTime(WIFI_TIME_ZONE_OFFSET_SEC, WIFI_DAYLIGHT_OFFSET_SEC,
                           "pool.ntp.org", "time.nist.gov", "ntp.aliyun.com");
                g_ntp_configured = true;
            }
        }
        else
        {
            g_sta_rssi = (int8_t)WiFi.RSSI();
        }
    }
    else
    {
        if (g_wifi_sta_connected)
        {
            g_wifi_sta_connected = false;
            snprintf(g_sta_ssid, sizeof(g_sta_ssid), "-");
            snprintf(g_sta_ip, sizeof(g_sta_ip), "0.0.0.0");
            g_sta_rssi = 0;
        }
        if (now - g_last_wifi_attempt_ms >= WIFI_RETRY_INTERVAL_MS)
        {
            g_wifi_network_index = (g_wifi_network_index + 1) % WIFI_NETWORK_COUNT;
            WiFi.begin(Wifi_Networks[g_wifi_network_index].ssid, Wifi_Networks[g_wifi_network_index].password);
            g_last_wifi_attempt_ms = now;
            AppLogPrintf("[WiFi] Switching STA to %s...\n", Wifi_Networks[g_wifi_network_index].ssid);
        }
    }

    if (now - g_last_second_ms >= 1000UL)
    {
        g_last_second_ms = now;
        if (strcmp(g_time_text, "syncing") != 0)
        {
            ++g_time_age_seconds;
        }
    }

    if (g_ntp_configured && (now - g_last_time_check_ms >= NTP_SYNC_INTERVAL_MS || strcmp(g_time_text, "syncing") == 0))
    {
        if (now - g_last_time_check_ms >= 1000UL)
        {
            g_last_time_check_ms = now;
            struct tm timeinfo = {};
            if (getLocalTime(&timeinfo, 10) && timeinfo.tm_year >= (2024 - 1900))
            {
                snprintf(g_time_text, sizeof(g_time_text), "%04d-%02d-%02d %02d:%02d:%02d",
                         timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                g_time_age_seconds = 0;
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    AppLogPrintln("Ciallo");
    AppLogPrintf("[%s_%s][%s]_firmware_%s\n", BOARD_NAME, BOARD_VERSION, SOFTWARE_NAME, SOFTWARE_LASTEDITTIME);
    AppLogPrintf("[Startup] CAN A module: %s, protocol: %s, speed: %s\n",
                 CAN_A_MODULE_NAME, CAN_A_PROTOCOL_NAME, CAN_A_SPEED_DESC);
    AppLogPrintf("[Startup] CAN B module: %s, protocol: %s, speed: %s\n",
                 CAN_B_MODULE_NAME, CAN_B_PROTOCOL_NAME, CAN_B_SPEED_DESC);

    Wifi_AP_STA_HTTP_Init();
    Can_A_Drive_Initialization();
    Can_B_Drive_Initialization();
}

void loop()
{
    Http_Server.handleClient();
    ServiceWifiAndTime();

    if (g_can_a_mode == CanMode::kAuto || g_can_a_mode == CanMode::kReceive)
    {
        Poll_Can_A_Receive();
    }
    Poll_Can_B_Twai();

    uint32_t now = millis();
    if (g_can_a_mode == CanMode::kAuto && g_can_b_mode == CanMode::kAuto)
    {
        if (now - g_last_auto_tick >= CAN_AUTO_INTERVAL_MS)
        {
            g_last_auto_tick = now;
            if (g_auto_send_a_turn)
            {
                Can_A_Send_Message();
            }
            else
            {
                Can_B_Twai_Send_Message();
            }
            g_auto_send_a_turn = !g_auto_send_a_turn;
        }
    }
    else
    {
        if ((g_can_a_mode == CanMode::kSend || g_can_a_mode == CanMode::kAuto) &&
            (now - g_last_can_a_tx_tick >= CAN_TX_INTERVAL_MS))
        {
            g_last_can_a_tx_tick = now;
            Can_A_Send_Message();
        }
        if ((g_can_b_mode == CanMode::kSend || g_can_b_mode == CanMode::kAuto) &&
            (now - g_last_can_b_tx_tick >= CAN_TX_INTERVAL_MS))
        {
            g_last_can_b_tx_tick = now;
            Can_B_Twai_Send_Message();
        }
    }
}
