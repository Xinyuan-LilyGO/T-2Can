#include <Arduino.h>
#include <SPI.h>
#include "driver/twai.h"
#include "pin_config.h"
#include "mcp2515.h"

static MCP2515 Can_A(MCP2515_CS, 10000000, &SPI);

static const char *state_name(twai_state_t s) {
    switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
    }
}

void setup() {
    Serial.begin(115200);
    delay(2500);
    Serial.println("\n=== CAN B (TWAI) self-test : NO_ACK + self reception ===");
    Serial.printf("TX pin=%d  RX pin=%d  bitrate=500k\n", CAN_TX, CAN_RX);

    // Silence CAN A so it cannot influence the bus.
    pinMode(MCP2515_RST, OUTPUT);
    digitalWrite(MCP2515_RST, HIGH); delay(10);
    digitalWrite(MCP2515_RST, LOW);  delay(10);
    digitalWrite(MCP2515_RST, HIGH); delay(50);
    SPI.begin(MCP2515_SCLK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS);
    Serial.println(Can_A.reset() == MCP2515::ERROR_OK
        ? "CAN A (MCP2515): reset OK -> Configuration mode (quiet)"
        : "CAN A (MCP2515): reset FAILED");
    delay(100);

    // --- Electrical check, before installing the TWAI driver ---
    // A working CAN transceiver pulls the bus dominant when TXD goes low and
    // reports that back on RXD. No library, no bitrate, no second node needed.
    pinMode(CAN_RX, INPUT);
    pinMode(CAN_TX, OUTPUT);
    int hi = 0, lo = 0, tx_hi = 0, tx_lo = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        digitalWrite(CAN_TX, HIGH);          // recessive
        delayMicroseconds(200);
        if (digitalRead(CAN_TX) == HIGH) tx_hi++;
        if (digitalRead(CAN_RX) == HIGH) hi++;
        digitalWrite(CAN_TX, LOW);           // dominant
        delayMicroseconds(200);
        if (digitalRead(CAN_TX) == LOW) tx_lo++;   // prove the pin really moved
        if (digitalRead(CAN_RX) == LOW) lo++;
    }
    digitalWrite(CAN_TX, HIGH);
    Serial.printf("GPIO_TX_READBACK high=%d/%d low=%d/%d\n", tx_hi, N, tx_lo, N);
    Serial.flush(); delay(120);
    Serial.printf("GPIO_RECESSIVE   rx_high=%d/%d\n", hi, N);
    Serial.flush(); delay(120);
    Serial.printf("GPIO_DOMINANT    rx_low=%d/%d\n", lo, N);
    Serial.flush(); delay(120);

    // --- Protocol check: NO_ACK + self reception needs no other node ---
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NO_ACK);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("install FAILED"); return; }
    Serial.println("driver installed");
    if (twai_start() != ESP_OK) { Serial.println("start FAILED"); return; }
    Serial.println("driver started");
}

void loop() {
    static uint32_t round = 0, tx = 0, txerr = 0, rx = 0;
    twai_message_t m = {};
    m.identifier = 0x123;
    m.data_length_code = 8;
    m.self = 1;                               // self reception request
    for (int i = 0; i < 8; i++) m.data[i] = (uint8_t)(round + i);
    if (twai_transmit(&m, pdMS_TO_TICKS(100)) == ESP_OK) tx++; else txerr++;
    twai_message_t r;
    while (twai_receive(&r, pdMS_TO_TICKS(30)) == ESP_OK) rx++;
    if (++round % 20 == 0) {
        twai_status_info_t s; twai_get_status_info(&s);
        Serial.printf("SELFTEST tx=%lu txerr=%lu rx=%lu | state=%s txerrcnt=%lu "
                      "rxerrcnt=%lu buserr=%lu arblost=%lu\n",
            (unsigned long)tx, (unsigned long)txerr, (unsigned long)rx,
            state_name(s.state), (unsigned long)s.tx_error_counter,
            (unsigned long)s.rx_error_counter, (unsigned long)s.bus_error_count,
            (unsigned long)s.arb_lost_count);
    }
    delay(40);
}