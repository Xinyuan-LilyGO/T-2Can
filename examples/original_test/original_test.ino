/*
 * @Description: original_test
 * @Author: LILYGO_L
 * @Date: 2024-11-07 10:04:14
 * @LastEditTime: 2025-07-07 14:54:46
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "pin_config.h"
#include "mcp2515.h"
#include <SPI.h>
#include "WiFi.h"
#include <HTTPClient.h>

// Intervall:
#define POLLING_RATE_MS 1000

#define WIFI_SSID "xinyuandianzi"
#define WIFI_PASSWORD "AA15994823428"
// #define WIFI_SSID "LilyGo-AABB"
// #define WIFI_PASSWORD "xinyuandianzi"

#define WIFI_CONNECT_WAIT_MAX 5000

size_t CycleTime = 0;

uint64_t Can_Count = 0;

bool Can_A_B_Send_Flag = true;

const char *fileDownloadUrl = "https://freetyst.nf.migu.cn/public/product9th/product45/2022/05/0716/2018%E5%B9%B409%E6%9C%8812%E6%97%A510%E7%82%B943%E5%88%86%E7%B4%A7%E6%80%A5%E5%86%85%E5%AE%B9%E5%87%86%E5%85%A5%E5%8D%8E%E7%BA%B3179%E9%A6%96/%E6%A0%87%E6%B8%85%E9%AB%98%E6%B8%85/MP3_128_16_Stero/6005751EPFG164228.mp3?channelid=02&msisdn=d43a7dcc-8498-461b-ba22-3205e9b6aa82&Tim=1728484238063&Key=0442fa065dacda7c";

static bool Wifi_Connection_Flag = false;

struct can_frame Can_Receive_Package;
struct can_frame Can_Send_Package;

MCP2515 Can_A(MCP2515_CS, 10000000, &SPI);

void Can_B_Drive_Initialization()
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        Serial.println("can b: Driver installed");
    }
    else
    {
        Serial.println("can b: Failed to install driver");
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        Serial.println("can b: Driver started");
    }
    else
    {
        Serial.println("can b: Failed to start driver");
    }

    // 配置
    uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS |
                                TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_DATA |
                                TWAI_ALERT_RX_QUEUE_FULL;

    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
    {
        Serial.println("can b: CAN Alerts reconfigured");
    }
    else
    {
        Serial.println("can b: Failed to reconfigure alerts");
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
        printf("can b: Failed to queue message for transmission\n");
    }
}

void Can_B_Twai_Receive_Message(twai_message_t &message)
{
    // Process received message
    if (message.extd)
    {
        Serial.println("can b: Message is in Extended Format");
        return;
    }
    else
    {
        // Serial.println("can b: Message is in Standard Format");
    }
    Serial.printf("\ncan b received data\n");
    Serial.printf("can b receive id: 0x%X\n", message.identifier);
    Serial.printf("can b receive data_length: %d\n", message.data_length_code);
    if (!(message.rtr))
    {
        for (int i = 0; i < message.data_length_code; i++)
        {
            Serial.printf("can b receive data [%d]: %d\n", i, message.data[i]);
        }
        Serial.println("");
    }
}

void Wifi_STA_Test(void)
{
    String text;
    int wifi_num = 0;

    Serial.printf("\nScanning wifi");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
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

    Serial.println(text);

    delay(3000);
    text.clear();

    text = "Connecting to ";
    Serial.print("Connecting to ");
    text += WIFI_SSID;
    text += "\n";

    Serial.print(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t last_tick = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        text += ".";
        delay(100);

        if (millis() - last_tick > WIFI_CONNECT_WAIT_MAX)
        {
            Wifi_Connection_Flag = false;
            break;
        }
        else
        {
            Wifi_Connection_Flag = true;
        }
    }

    if (Wifi_Connection_Flag == true)
    {
        text += "\nThe connection was successful ! \nTakes ";
        Serial.print("\nThe connection was successful ! \nTakes ");

        text += millis() - last_tick;
        Serial.print(millis() - last_tick);

        text += " ms\n";
        Serial.println(" ms\n");
    }
    else
    {
        Serial.printf("\nWifi test error!\n");
    }
}

void WIFI_STA_Test_Loop(void)
{
    if (Wifi_Connection_Flag == true)
    {
        // 初始化HTTP客户端
        HTTPClient http;
        http.begin(fileDownloadUrl);
        // 获取重定向的URL
        const char *headerKeys[] = {"Location"};
        http.collectHeaders(headerKeys, 1);

        // 记录下载开始时间
        size_t startTime = millis();
        // 无用时间
        size_t uselessTime = 0;

        // 发起GET请求
        int httpCode = http.GET();

        while (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND)
        {
            String newUrl = http.header("Location");
            Serial.printf("Redirecting to: %s\n", newUrl.c_str());
            http.end(); // 关闭旧的HTTP连接

            // 使用新的URL重新发起GET请求
            http.begin(newUrl);
            httpCode = http.GET();
        }

        if (httpCode == HTTP_CODE_OK)
        {
            // 获取文件大小
            size_t fileSize = http.getSize();
            Serial.printf("Starting file download...\n");
            Serial.printf("file size: %f MB\n", fileSize / 1024.0 / 1024.0);

            // 读取HTTP响应
            WiFiClient *stream = http.getStreamPtr();

            size_t temp_count_s = 0;
            size_t temp_fileSize = fileSize;
            uint8_t *buf_1 = (uint8_t *)heap_caps_malloc(100 * 1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            // uint8_t buf_1[4096] = {0};
            CycleTime = millis() + 1000; // 开始计时
            bool temp_count_flag = true;
            while (http.connected() && (temp_fileSize > 0 || temp_fileSize == -1))
            {
                // 获取可用数据的大小
                size_t availableSize = stream->available();
                if (availableSize)
                {
                    temp_fileSize -= stream->read(buf_1, min(availableSize, (size_t)(100 * 1024)));

                    if (millis() > CycleTime)
                    {
                        size_t temp_time_1 = millis();
                        temp_count_s++;
                        Serial.printf("Download speed: %f KB/s\n", ((fileSize - temp_fileSize) / 1024.0) / temp_count_s);
                        Serial.printf("Remaining file size: %f MB\n\n", temp_fileSize / 1024.0 / 1024.0);

                        CycleTime = millis() + 1000;
                        size_t temp_time_2 = millis();

                        uselessTime = uselessTime + (temp_time_2 - temp_time_1);
                    }
                }
                // delay(1);

                if (temp_count_s > 30)
                {
                    temp_count_flag = false;
                    break;
                }
            }

            // 关闭HTTP客户端
            http.end();

            // 记录下载结束时间并计算总花费时间
            size_t endTime = millis();

            if (temp_count_flag == true)
            {
                Serial.printf("Download completed!\n");
                Serial.printf("Total download time: %f s\n", (endTime - startTime - uselessTime) / 1000.0);
                Serial.printf("Average download speed: %f KB/s\n", (fileSize / 1024.0) / ((endTime - startTime - uselessTime) / 1000.0));
            }
            else
            {
                Serial.printf("Download incomplete!\n");
                Serial.printf("Download time: %f s\n", (endTime - startTime - uselessTime) / 1000.0);
                Serial.printf("Average download speed: %f KB/s\n", ((fileSize - temp_fileSize) / 1024.0) / ((endTime - startTime - uselessTime) / 1000.0));
            }
        }
        else
        {
            Serial.printf("Failed to download\n");
            Serial.printf("Error httpCode: %d \n", httpCode);
        }
    }
    else
    {
        Serial.print("Not connected to the network");
    }
    delay(1000);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

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

    Can_B_Drive_Initialization();

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

    Wifi_STA_Test();
    WIFI_STA_Test_Loop();

    Serial.println("can a speed: 500kbps");
    Serial.println("can b speed: 500kbps");
}

void loop()
{
    // 通信报警检测
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    // 总线状态信息
    twai_status_info_t twai_status_info;
    twai_get_status_info(&twai_status_info);

    switch (alerts_triggered)
    {
    case TWAI_ALERT_ERR_PASS:
        Serial.println("\ncan b: Alert: TWAI controller has become error passive.");
        delay(1000);
        break;
    case TWAI_ALERT_BUS_ERROR:
    {
        Serial.println("\ncan b: Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        Serial.printf("can b: Bus error count: %d\n", twai_status_info.bus_error_count);

        uint8_t temp = 0;
        while (1)
        {
            uint32_t alerts_triggered;
            twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
            // 总线状态信息
            twai_status_info_t twai_status_info;
            twai_get_status_info(&twai_status_info);

            temp++;
            if (temp > 3)
            {
                break;
            }

            delay(1000);
        }
    }
    break;
    case TWAI_ALERT_TX_FAILED:
        Serial.println("\ncan b: Alert: The Transmission failed.");
        Serial.printf("can b: TX buffered: %d\n", twai_status_info.msgs_to_tx);
        Serial.printf("can b: TX error: %d\n", twai_status_info.tx_error_counter);
        Serial.printf("can b: TX failed: %d\n", twai_status_info.tx_failed_count);
        delay(1000);
        break;
    case TWAI_ALERT_TX_SUCCESS:
        Serial.println("\ncan b: Alert: The Transmission was successful.");
        Serial.printf("can b: TX buffered: %d\n", twai_status_info.msgs_to_tx);
        break;
    case TWAI_ALERT_RX_QUEUE_FULL:
        Serial.println("\ncan b: Alert: The RX queue is full causing a received frame to be lost.");
        Serial.printf("can b: RX buffered: %d\n", twai_status_info.msgs_to_rx);
        Serial.printf("can b: RX missed: %d\n", twai_status_info.rx_missed_count);
        Serial.printf("can b: RX overrun %d\n", twai_status_info.rx_overrun_count);

        twai_clear_receive_queue();
        delay(1000);
        break;

    default:
        break;
    }

    switch (twai_status_info.state)
    {
    case TWAI_STATE_RUNNING:
        Serial.println("\ncan b: TWAI_STATE_RUNNING");
        break;
    case TWAI_STATE_BUS_OFF:
        Serial.println("\ncan b: TWAI_STATE_BUS_OFF");
        twai_initiate_recovery();
        // delay(1000);
        break;
    case TWAI_STATE_STOPPED:
        Serial.println("\ncan b: TWAI_STATE_STOPPED");
        twai_start();
        delay(1000);
        break;
    case TWAI_STATE_RECOVERING:
        Serial.println("\ncan b: TWAI_STATE_RECOVERING");
        delay(1000);
        break;

    default:
        break;
    }

    // 如果TWAI有信息接收到
    if (alerts_triggered & TWAI_ALERT_RX_DATA)
    {
        twai_message_t rx_buf;

        while (twai_receive(&rx_buf, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            Can_B_Twai_Receive_Message(rx_buf);
        }
    }

    if (millis() > CycleTime)
    {
        if (Can_A_B_Send_Flag == true)
        {
            Serial.printf("can a: send data\n");
            Can_A.sendMessage(&Can_Send_Package);
        }
        else
        {
            Serial.printf("can b: send data\n");
            Can_B_Twai_Send_Message();
        }
        Can_A_B_Send_Flag = !Can_A_B_Send_Flag;

        CycleTime = millis() + 3000;
    }

    uint8_t irq = Can_A.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF)
    {
        if (Can_A.readMessage(MCP2515::RXB0, &Can_Receive_Package) == MCP2515::ERROR_OK)
        {
            // frame contains received from RXB0 message

            Serial.printf("\ncan a received RXB0 data\n");
            Serial.printf("can a receive id: 0x%X\n", Can_Receive_Package.can_id);
            Serial.printf("can a receive data length: %d\n", Can_Receive_Package.can_dlc);
            for (int i = 0; i < Can_Receive_Package.can_dlc; i++)
            {
                Serial.printf("can a receive data [%d]: %d\n", i, Can_Receive_Package.data[i]);
            }
            Serial.println();
        }
    }
    else if (irq & MCP2515::CANINTF_RX1IF)
    {
        if (Can_A.readMessage(MCP2515::RXB1, &Can_Receive_Package) == MCP2515::ERROR_OK)
        {
            // frame contains received from RXB1 message

            Serial.printf("\ncan a received RXB1 data\n");
            Serial.printf("can a receive id: 0x%X\n", Can_Receive_Package.can_id);
            Serial.printf("can a receive data length: %d\n", Can_Receive_Package.can_dlc);
            for (int i = 0; i < Can_Receive_Package.can_dlc; i++)
            {
                Serial.printf("can a receive data [%d]: %d\n", i, Can_Receive_Package.data[i]);
            }
            Serial.println();
        }
    }

    // if (Can_A.readMessage(&Can_Receive_Package) == MCP2515::ERROR_OK)
    // {
    //     Serial.printf("\ncan a received data\n");
    //     Serial.printf("can a receive id: 0x%X\n", Can_Receive_Package.can_id);
    //     Serial.printf("can a receive data length: %d\n", Can_Receive_Package.can_dlc);
    //     for (int i = 0; i < Can_Receive_Package.can_dlc; i++)
    //     {
    //         Serial.printf("can a receive data [%d]: %d\n", i, Can_Receive_Package.data[i]);
    //     }
    //     Serial.println();
    // }
}
