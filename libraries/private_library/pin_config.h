/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2023-06-05 13:01:59
 * @LastEditTime: 2026-04-29 11:41:17
 */
#pragma once

// #define T_2Can
#define T_2Can_Fd

// CAN
#define CAN_TX 7
#define CAN_RX 6

// SPI
#define SPI_SCLK 12
#define SPI_MOSI 11
#define SPI_MISO 13

#if defined T_2Can
// MCP2515
#define MCP2515_CS 10
#define MCP2515_SCLK SPI_SCLK
#define MCP2515_MOSI SPI_MOSI
#define MCP2515_MISO SPI_MISO
#define MCP2515_RST 9
#define MCP2515_INT 8
#elif defined T_2Can_Fd
#define MCP2518_CS 10
#define MCP2518_SCLK SPI_SCLK
#define MCP2518_MOSI SPI_MOSI
#define MCP2518_MISO SPI_MISO

#define MCP2518_INT 8
#define MCP2518_INT_0 9
#define MCP2518_INT_1 3
#else
#error "no macro definition is set"
#endif

// ESPBOOT
#define ESP_BOOT 0
