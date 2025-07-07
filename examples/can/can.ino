/*
 * @Description: CAN test
 * @Author: LILYGO_L
 * @Date: 2024-11-07 10:04:14
 * @LastEditTime: 2025-06-21 09:46:11
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "pin_config.h"
#include "mcp2515.h"
#include <SPI.h>

// Intervall:
#define POLLING_RATE_MS 1000

size_t CycleTime = 0;

uint64_t Can_Count = 0;

bool Can_A_B_Send_Flag = true;

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

    Serial.println("can a speed: 1000kbps");
    Serial.println("can b speed: 1000kbps");
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
