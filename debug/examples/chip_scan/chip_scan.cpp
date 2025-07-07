/*
 * @Description: None
 * @version: V1.0.0
 * @Author: None
 * @Date: 2023-09-12 17:09:04
 * @LastEditors: LILYGO_L
 * @LastEditTime: 2023-09-12 18:30:33
 * @License: GPL 3.0
 */
#include <Arduino.h>

void Chip_Scan(void)
{
    uint64_t chipid;

    chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).

    Serial.printf("\n------------------------------------------------\n");

    Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32)); // print High 2 bytes
    Serial.printf("%08X\n", (uint32_t)chipid);                       // print Low 4bytes.
    Serial.printf("Chip Revision = %u\n", ESP.getChipRevision());
    Serial.printf("CpuFreqMHz = %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Cycle Count = %u\n", ESP.getCycleCount());
    Serial.printf("SdkVersion = %s\n", ESP.getSdkVersion());

    Serial.printf("\n");

    Serial.printf("Total heap size = %u B\n", ESP.getHeapSize());
    Serial.printf("Available heap = %u B\n", ESP.getFreeHeap());
    Serial.printf("Lowest level of free heap since boot = %u B\n", ESP.getMinFreeHeap());
    Serial.printf("Largest block of heap that can be allocated at once = %u B\n", ESP.getMaxAllocHeap());

    Serial.printf("\n");

    Serial.printf("Total Psram size = %u B\n", ESP.getPsramSize());
    Serial.printf("Available Psram = %u B\n", ESP.getFreePsram());
    Serial.printf("Lowest level of free Psram since boot = %u B\n", ESP.getMinFreePsram());
    Serial.printf("Largest block of Psram that can be allocated at once = %u B\n", ESP.getMaxAllocPsram());

    Serial.printf("\n");

    Serial.printf("Flash chip size: %u B\n", ESP.getFlashChipSize());
    Serial.printf("Flash chip speed: %u Hz\n", ESP.getFlashChipSpeed());

    switch (ESP.getFlashChipMode())
    {
    case FM_QIO:
        Serial.printf("Flash chip mode: FM_QIO \n");
        break;
    case FM_QOUT:
        Serial.printf("Flash chip mode: FM_QOUT \n");
        break;
    case FM_DIO:
        Serial.printf("Flash chip mode: FM_DIO \n");
        break;
    case FM_DOUT:
        Serial.printf("Flash chip mode: FM_DOUT \n");
        break;
    case FM_FAST_READ:
        Serial.printf("Flash chip mode: FM_FAST_READ \n");
        break;
    case FM_SLOW_READ:
        Serial.printf("Flash chip mode: FM_SLOW_READ \n");
        break;
    case FM_UNKNOWN:
        Serial.printf("Flash chip mode: FM_UNKNOWN \n");
        break;

    default:
        break;
    }

    Serial.printf("------------------------------------------------\n");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");
}

void loop()
{
    Chip_Scan();
    delay(1000);
}