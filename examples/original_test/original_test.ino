/*
 * @Description: original_test
 * @Author: LILYGO_L
 * @Date: 2024-11-07 10:04:14
 * @LastEditTime: 2026-07-16 16:04:39
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include <stdarg.h>
#include "driver/twai.h"
#include "pin_config.h"
#if defined T_2Can
#include "mcp2515.h"
#elif defined T_2Can_Fd
#include "mcp2518fd_can.h"
#else
#error "no macro definition is set"
#endif
#include <SPI.h>
#include "WiFi.h"
#include <WebServer.h>
#include <esp_mac.h>
#include <time.h>

// Intervall:
#define POLLING_RATE_MS 100

#define WIFI_SSID_1 "LilyGo-AABB"
#define WIFI_PASSWORD_1 "xinyuandianzi"
#define WIFI_SSID_2 "xinyuandianzi"
#define WIFI_PASSWORD_2 "AA15994823428"

#define WIFI_CONNECT_WAIT_MAX 40000UL
#define WIFI_NETWORK_SWITCH_INTERVAL 10000UL
#define WIFI_CONNECT_POLL_INTERVAL 50UL
#define WIFI_CONNECT_LOG_INTERVAL 500UL
#define WIFI_TIME_SYNC_WAIT_MAX 40000UL
#define WIFI_TIME_SYNC_POLL_TIMEOUT 500UL
#define WIFI_TIME_ZONE_OFFSET_SEC (8 * 60 * 60)
#define WIFI_DAYLIGHT_OFFSET_SEC 0
#define WIFI_AP_SSID_PREFIX "T-2Can_"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONNECTIONS 4
#define WEB_LOG_MAX_LENGTH 20000

#if defined T_2Can
#elif defined T_2Can_Fd
#define MAX_DATA_SIZE 64
#else
#error "no macro definition is set"
#endif

#define SOFTWARE_NAME "Original_Test"
#define SOFTWARE_LASTEDITTIME "202607161604"
#define BOARD_VERSION "V1.0"

size_t CycleTime = 0;

uint64_t Can_Count = 0;

bool Can_A_B_Send_Flag = true;

static bool Wifi_Connection_Flag = false;
static bool Wifi_AP_Flag = false;
static char Wifi_AP_SSID[sizeof(WIFI_AP_SSID_PREFIX) + 12];
static String Web_Log_Buffer;
WebServer Http_Server(80);

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

void AppLogPrint(const String &message);
template <typename T>
void AppLogPrint(const T &message);
void AppLogPrintln(const String &message = "");
template <typename T>
void AppLogPrintln(const T &message);
void AppLogPrintf(const char *format, ...);

#if defined T_2Can
struct can_frame Can_Receive_Package;
struct can_frame Can_Send_Package;

MCP2515 Can_A(MCP2515_CS, 10000000, &SPI);
#elif defined T_2Can_Fd
uint8_t Can_Receive_Package[MAX_DATA_SIZE];
uint8_t Can_Send_Package[MAX_DATA_SIZE];

mcp2518fd Can_A(MCP2518_CS);
#else
#error "no macro definition is set"
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

template <typename T>
void AppLogPrint(const T &message)
{
    Serial.print(message);
    AppendWebLog(String(message));
}

void AppLogPrintln(const String &message)
{
    Serial.println(message);
    AppendWebLog(message + "\n");
}

template <typename T>
void AppLogPrintln(const T &message)
{
    Serial.println(message);
    AppendWebLog(String(message) + "\n");
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

String HtmlEscape(const String &input)
{
    String output;
    output.reserve(input.length());
    for (size_t i = 0; i < input.length(); i++)
    {
        char c = input[i];
        if (c == '&')
        {
            output += "&amp;";
        }
        else if (c == '<')
        {
            output += "&lt;";
        }
        else if (c == '>')
        {
            output += "&gt;";
        }
        else
        {
            output += c;
        }
    }
    return output;
}

void HandleRoot()
{
    String html;
    html.reserve(Web_Log_Buffer.length() + 900);
    html += F("<!doctype html><html><head><meta charset='utf-8'>");
    html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += F("<title>T-2Can Log</title>");
    html += F("<style>body{font-family:Consolas,monospace;margin:16px;background:#101418;color:#e8eef2;}");
    html += F("a{color:#8cc7ff}pre{white-space:pre-wrap;word-break:break-word;border:1px solid #2d3942;padding:12px;min-height:70vh;background:#05080a;}");
    html += F(".bar{display:flex;gap:12px;align-items:center;margin-bottom:12px}</style>");
    html += F("<script>setInterval(()=>fetch('/log').then(r=>r.text()).then(t=>document.getElementById('log').textContent=t),1000);</script>");
    html += F("</head><body><div class='bar'><strong>T-2Can Serial Log</strong><a href='/log'>text</a><a href='/clear'>clear</a></div><pre id='log'>");
    html += HtmlEscape(Web_Log_Buffer);
    html += F("</pre></body></html>");
    Http_Server.send(200, "text/html; charset=utf-8", html);
}

void HandleLogText()
{
    Http_Server.send(200, "text/plain; charset=utf-8", Web_Log_Buffer);
}

void HandleClearLog()
{
    Web_Log_Buffer = "";
    Http_Server.sendHeader("Location", "/", true);
    Http_Server.send(302, "text/plain", "");
}

void BuildWifiAPSSID(void)
{
    uint8_t factory_mac[6] = {0};
    esp_err_t result = esp_efuse_mac_get_default(factory_mac);
    if (result != ESP_OK)
    {
        AppLogPrintf("Failed to read factory MAC: %d\n", result);
    }

    snprintf(Wifi_AP_SSID, sizeof(Wifi_AP_SSID),
             WIFI_AP_SSID_PREFIX "%02X%02X%02X%02X%02X%02X",
             factory_mac[0], factory_mac[1], factory_mac[2],
             factory_mac[3], factory_mac[4], factory_mac[5]);
}

void Wifi_AP_HTTP_Init(void)
{
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

    BuildWifiAPSSID();
    Wifi_AP_Flag = WiFi.softAP(Wifi_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CONNECTIONS);

    Http_Server.on("/", HTTP_GET, HandleRoot);
    Http_Server.on("/log", HTTP_GET, HandleLogText);
    Http_Server.on("/clear", HTTP_GET, HandleClearLog);
    Http_Server.begin();

    IPAddress ip = WiFi.softAPIP();
    AppLogPrintf("WiFi AP %s\n", Wifi_AP_Flag ? "started" : "start failed");
    AppLogPrintf("AP SSID: %s\n", Wifi_AP_SSID);
    AppLogPrintf("AP password: %s\n", WIFI_AP_PASSWORD);
    AppLogPrintf("HTTP log URL: http://%s/\n", ip.toString().c_str());
}

void Can_B_Drive_Initialization()
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        AppLogPrintln("can b: Driver installed");
    }
    else
    {
        AppLogPrintln("can b: Failed to install driver");
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        AppLogPrintln("can b: Driver started");
    }
    else
    {
        AppLogPrintln("can b: Failed to start driver");
    }

    // 配置
    uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS |
                                TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_DATA |
                                TWAI_ALERT_RX_QUEUE_FULL;

    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
    {
        AppLogPrintln("can b: CAN Alerts reconfigured");
    }
    else
    {
        AppLogPrintln("can b: Failed to reconfigure alerts");
    }
}

void Can_B_Twai_Send_Message()
{
    // Configure message to transmit
    twai_message_t message;
    message.identifier = 0xBB;
    // message.data_length_code = 1;
    // message.data[0] = Can_Count++;
    message.data_length_code = 8;
    message.data[0] = 1;
    message.data[1] = 2;
    message.data[2] = 3;
    message.data[3] = 4;
    message.data[4] = 5;
    message.data[5] = 6;
    message.data[6] = 7;
    message.data[7] = 8;

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        // printf("can b: Message queued for transmission\n");
    }
    else
    {
        AppLogPrintf("can b: Failed to queue message for transmission\n");
    }
}

void Can_B_Twai_Receive_Message(twai_message_t &message)
{
    // Process received message
    if (message.extd)
    {
        AppLogPrintln("can b: Message is in Extended Format");
        return;
    }
    else
    {
        // AppLogPrintln("can b: Message is in Standard Format");
    }
    AppLogPrintf("\ncan b received data\n");
    AppLogPrintf("can b receive id: 0x%X\n", message.identifier);
    AppLogPrintf("can b receive data length: %d\n", message.data_length_code);
    if (!(message.rtr))
    {
        for (int i = 0; i < message.data_length_code; i++)
        {
            AppLogPrintf("can b receive data [%d]: %d\n", i, message.data[i]);
        }
        AppLogPrintln("");
    }
}

void Wifi_STA_Test(void)
{
    String text;
    int wifi_num = 0;

    AppLogPrintf("\nScanning wifi");
    WiFi.mode(WIFI_AP_STA);
    delay(100);

    wifi_num = WiFi.scanNetworks();
    if (wifi_num == 0)
    {
        text = "\nWiFi scan complete !\nNo wifi discovered.\n";
    }
    else
    {
        text = "\nWiFi scan complete !\n";
        text += wifi_num;
        text += " wifi discovered.\n\n";

        for (int i = 0; i < wifi_num; i++)
        {
            text += (i + 1);
            text += ": ";
            text += WiFi.SSID(i);
            text += " (";
            text += WiFi.RSSI(i);
            text += ")";
            text += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " \n" : "*\n";
            delay(10);
        }
    }

    AppLogPrintln(text);
    WiFi.scanDelete();

    size_t network_index = 0;
    uint32_t start_tick = millis();
    uint32_t next_switch_tick = WIFI_NETWORK_SWITCH_INTERVAL;
    uint32_t last_log_tick = start_tick;

    Wifi_Connection_Flag = false;
    AppLogPrintf("Connecting to %s", Wifi_Networks[network_index].ssid);
    WiFi.begin(Wifi_Networks[network_index].ssid, Wifi_Networks[network_index].password);

    while (WiFi.status() != WL_CONNECTED)
    {
        Http_Server.handleClient();
        uint32_t now = millis();
        uint32_t elapsed = now - start_tick;

        if (elapsed >= WIFI_CONNECT_WAIT_MAX)
        {
            break;
        }

        if (elapsed >= next_switch_tick)
        {
            network_index = (network_index + 1) % WIFI_NETWORK_COUNT;
            AppLogPrintf("\nSwitching to %s", Wifi_Networks[network_index].ssid);
            WiFi.begin(Wifi_Networks[network_index].ssid, Wifi_Networks[network_index].password);
            next_switch_tick += WIFI_NETWORK_SWITCH_INTERVAL;
            last_log_tick = now;
        }

        if (now - last_log_tick >= WIFI_CONNECT_LOG_INTERVAL)
        {
            AppLogPrint(".");
            last_log_tick = now;
        }

        delay(WIFI_CONNECT_POLL_INTERVAL);
    }

    Wifi_Connection_Flag = (WiFi.status() == WL_CONNECTED);

    if (Wifi_Connection_Flag == true)
    {
        AppLogPrint("\nThe connection was successful ! \nTakes ");
        AppLogPrint(millis() - start_tick);
        AppLogPrintln(" ms\n");
        AppLogPrintf("Connected SSID: %s\n", WiFi.SSID().c_str());
        AppLogPrintf("STA IP: %s\n", WiFi.localIP().toString().c_str());
        AppLogPrintf("RSSI: %d dBm\n", WiFi.RSSI());
    }
    else
    {
        AppLogPrintf("\nWifi test error after %lu ms (status: %d)\n",
                     millis() - start_tick, WiFi.status());
    }
}

void WIFI_Time_Test(void)
{
    if (Wifi_Connection_Flag != true)
    {
        AppLogPrintln("Not connected to the network");
        return;
    }

    AppLogPrintln("Syncing time from NTP...");
    configTime(WIFI_TIME_ZONE_OFFSET_SEC, WIFI_DAYLIGHT_OFFSET_SEC,
               "pool.ntp.org", "time.nist.gov", "ntp.aliyun.com");

    struct tm timeinfo = {};
    uint32_t start_tick = millis();
    bool time_sync_success = false;

    while (millis() - start_tick < WIFI_TIME_SYNC_WAIT_MAX)
    {
        uint32_t elapsed = millis() - start_tick;
        uint32_t remaining = WIFI_TIME_SYNC_WAIT_MAX - elapsed;
        uint32_t poll_timeout = remaining < WIFI_TIME_SYNC_POLL_TIMEOUT
                                    ? remaining
                                    : WIFI_TIME_SYNC_POLL_TIMEOUT;

        if (getLocalTime(&timeinfo, poll_timeout))
        {
            time_sync_success = true;
            break;
        }

        Http_Server.handleClient();
        AppLogPrint(".");
    }

    if (!time_sync_success)
    {
        AppLogPrintf("\nNTP time sync failed after %lu ms\n", millis() - start_tick);
        return;
    }

    AppLogPrintln();
    AppLogPrintf("NTP time sync success, takes %lu ms\n", millis() - start_tick);
    AppLogPrintf("Local time: %04d-%02d-%02d %02d:%02d:%02d\n",
                  timeinfo.tm_year + 1900,
                  timeinfo.tm_mon + 1,
                  timeinfo.tm_mday,
                  timeinfo.tm_hour,
                  timeinfo.tm_min,
                  timeinfo.tm_sec);
}

void setup()
{
    Serial.begin(115200);
    AppLogPrintln("Ciallo");
#if defined T_2Can
    AppLogPrintln("[T_2Can_" + (String)BOARD_VERSION "][" + (String)SOFTWARE_NAME +
                   "]_firmware_" + (String)SOFTWARE_LASTEDITTIME);

#elif defined T_2Can_Fd
    AppLogPrintln("[T_2Can_Fd_" + (String)BOARD_VERSION "][" + (String)SOFTWARE_NAME +
                   "]_firmware_" + (String)SOFTWARE_LASTEDITTIME);
#else
#error "no macro definition is set"
#endif

    Wifi_AP_HTTP_Init();
    Wifi_STA_Test();
    WIFI_Time_Test();

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
    delay(100);
    digitalWrite(MCP2515_RST, LOW);
    delay(100);
    digitalWrite(MCP2515_RST, HIGH);
    delay(100);

    SPI.begin(MCP2515_SCLK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS); // SPI boots

    Can_A.reset();
    Can_A.setBitrate(CAN_500KBPS);
    Can_A.setNormalMode();

    AppLogPrintln("can a speed: 500kbps");

#elif defined T_2Can_Fd
    memset(Can_Send_Package, 'A', MAX_DATA_SIZE);

    Can_A.setMode(CAN_NORMAL_MODE);

    SPI.begin(MCP2518_SCLK, MCP2518_MISO, MCP2518_MOSI, MCP2518_CS); // SPI boots

    if (Can_A.begin(CAN_500K_5M) != CAN_OK)
    {
        AppLogPrintln("can a fd init fail");
    }
    else
    {
        AppLogPrintln("can a fd init success");
    }

    AppLogPrintln("can a fd speed: 5000kbps");
#else
#error "no macro definition is set"
#endif

    Can_B_Drive_Initialization();
    AppLogPrintln("can b speed: 500kbps");
}

void loop()
{
    Http_Server.handleClient();

    // 通信报警检测
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    // 总线状态信息
    // twai_status_info_t twai_status_info;
    // twai_get_status_info(&twai_status_info);

    // switch (alerts_triggered)
    // {
    // case TWAI_ALERT_ERR_PASS:
    //     AppLogPrintln("\ncan b: Alert: TWAI controller has become error passive.");
    //     delay(1000);
    //     break;
    // case TWAI_ALERT_BUS_ERROR:
    // {
    //     AppLogPrintln("\ncan b: Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    //     AppLogPrintf("can b: Bus error count: %d\n", twai_status_info.bus_error_count);

    //     uint8_t temp = 0;
    //     while (1)
    //     {
    //         uint32_t alerts_triggered;
    //         twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    //         // 总线状态信息
    //         twai_status_info_t twai_status_info;
    //         twai_get_status_info(&twai_status_info);

    //         temp++;
    //         if (temp > 3)
    //         {
    //             break;
    //         }

    //         delay(1000);
    //     }
    // }
    // break;
    // case TWAI_ALERT_TX_FAILED:
    //     AppLogPrintln("\ncan b: Alert: The Transmission failed.");
    //     AppLogPrintf("can b: TX buffered: %d\n", twai_status_info.msgs_to_tx);
    //     AppLogPrintf("can b: TX error: %d\n", twai_status_info.tx_error_counter);
    //     AppLogPrintf("can b: TX failed: %d\n", twai_status_info.tx_failed_count);
    //     delay(1000);
    //     break;
    // case TWAI_ALERT_TX_SUCCESS:
    //     AppLogPrintln("\ncan b: Alert: The Transmission was successful.");
    //     AppLogPrintf("can b: TX buffered: %d\n", twai_status_info.msgs_to_tx);
    //     break;
    // case TWAI_ALERT_RX_QUEUE_FULL:
    //     AppLogPrintln("\ncan b: Alert: The RX queue is full causing a received frame to be lost.");
    //     AppLogPrintf("can b: RX buffered: %d\n", twai_status_info.msgs_to_rx);
    //     AppLogPrintf("can b: RX missed: %d\n", twai_status_info.rx_missed_count);
    //     AppLogPrintf("can b: RX overrun %d\n", twai_status_info.rx_overrun_count);

    //     twai_clear_receive_queue();
    //     delay(1000);
    //     break;

    // default:
    //     break;
    // }

    // switch (twai_status_info.state)
    // {
    // case TWAI_STATE_RUNNING:
    //     AppLogPrintln("\ncan b: TWAI_STATE_RUNNING");
    //     break;
    // case TWAI_STATE_BUS_OFF:
    //     AppLogPrintln("\ncan b: TWAI_STATE_BUS_OFF");
    //     twai_initiate_recovery();
    //     // delay(1000);
    //     break;
    // case TWAI_STATE_STOPPED:
    //     AppLogPrintln("\ncan b: TWAI_STATE_STOPPED");
    //     twai_start();
    //     delay(1000);
    //     break;
    // case TWAI_STATE_RECOVERING:
    //     AppLogPrintln("\ncan b: TWAI_STATE_RECOVERING");
    //     delay(1000);
    //     break;

    // default:
    //     break;
    // }

    // 如果TWAI有信息接收到
    if (alerts_triggered & TWAI_ALERT_RX_DATA)
    {
        twai_message_t rx_buf;

        while (twai_receive(&rx_buf, 0) == ESP_OK)
        {
            Can_B_Twai_Receive_Message(rx_buf);
            delay(10);
        }
    }

    if (millis() > CycleTime)
    {
        if (Can_A_B_Send_Flag == true)
        {
#if defined T_2Can
            AppLogPrintf("can a: send data\n");
            Can_A.sendMessage(&Can_Send_Package);
#elif defined T_2Can_Fd
            AppLogPrintf("can a fd: send data\n");
            Can_A.sendMsgBuf(0xAA, 0, CANFD::len2dlc(MAX_DATA_SIZE), Can_Send_Package);
#else
#error "no macro definition is set"
#endif

        }
        else
        {
            AppLogPrintf("can b: send data\n");
            Can_B_Twai_Send_Message();
        }
        Can_A_B_Send_Flag = !Can_A_B_Send_Flag;

        CycleTime = millis() + 3000;
    }

#if defined T_2Can
    uint8_t irq = Can_A.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF)
    {
        if (Can_A.readMessage(MCP2515::RXB0, &Can_Receive_Package) == MCP2515::ERROR_OK)
        {
            // frame contains received from message

            AppLogPrintf("\ncan a received data\n");
            AppLogPrintf("can a receive id: 0x%X\n", Can_Receive_Package.can_id);
            AppLogPrintf("can a receive data length: %d\n", Can_Receive_Package.can_dlc);
            for (int i = 0; i < Can_Receive_Package.can_dlc; i++)
            {
                AppLogPrintf("can a receive data [%d]: %d\n", i, Can_Receive_Package.data[i]);
            }
            AppLogPrintln();
        }
    }
    else if (irq & MCP2515::CANINTF_RX1IF)
    {
        if (Can_A.readMessage(MCP2515::RXB1, &Can_Receive_Package) == MCP2515::ERROR_OK)
        {
            // frame contains received from message

            AppLogPrintf("\ncan a received data\n");
            AppLogPrintf("can a receive id: 0x%X\n", Can_Receive_Package.can_id);
            AppLogPrintf("can a receive data length: %d\n", Can_Receive_Package.can_dlc);
            for (int i = 0; i < Can_Receive_Package.can_dlc; i++)
            {
                AppLogPrintf("can a receive data [%d]: %d\n", i, Can_Receive_Package.data[i]);
            }
            AppLogPrintln();
        }
    }

    // if (Can_A.readMessage(&Can_Receive_Package) == MCP2515::ERROR_OK)
    // {
    //     AppLogPrintf("\ncan a received data\n");
    //     AppLogPrintf("can a receive id: 0x%X\n", Can_Receive_Package.can_id);
    //     AppLogPrintf("can a receive data length: %d\n", Can_Receive_Package.can_dlc);
    //     for (int i = 0; i < Can_Receive_Package.can_dlc; i++)
    //     {
    //         AppLogPrintf("can a receive data [%d]: %d\n", i, Can_Receive_Package.data[i]);
    //     }
    //     AppLogPrintln();
    // }
#elif defined T_2Can_Fd
    if (CAN_MSGAVAIL == Can_A.checkReceive())
    {
        uint8_t len = 0;
        Can_A.readMsgBuf(&len, Can_Receive_Package);
        unsigned long id = Can_A.getCanId();

        AppLogPrint("\ncan a fd received data\n");
        AppLogPrintf("can a fd receive id: %#X\n", id);
        AppLogPrintf("can a fd receive data length: %d\n", len);
        AppLogPrint("can a fd receive data: \n[");
        for (int i = 0; i < len; i++)
        {
            AppLogPrintf("%c", Can_Receive_Package[i]);
            if (i != len - 1)
            {
                if ((i + 1) % 10 == 0)
                {
                    AppLogPrintln(); // 每10个数据换行
                }
                else
                {
                    AppLogPrint(",");
                }
            }
        }
        AppLogPrintf("]\n");
    }
#else
#error "no macro definition is set"
#endif
}
