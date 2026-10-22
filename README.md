# Brain Ballast Rock

A low-power, deep-sleep enabled data logging node built around the Seeed Studio XIAO ESP32C3. This device remains in deep sleep to conserve power, waking periodically to listen for a LoRa trigger from the surface. Once triggered (or if a specific pressure threshold is met), it performs high-speed (50Hz) burst recording of pressure, temperature, and 3-axis acceleration to a MicroSD card, followed by chunked data transmission over LoRa.

## Hardware Requirements

*   **Microcontroller:** Seeed Studio XIAO ESP32C3
*   **Communication:** RF95 LoRa Module (915 MHz standard)
*   **Storage:** MicroSD Card Module
*   **Sensors:**
    *   BlueRobotics MS5837 Pressure/Depth Sensor
    *   ADXL345 3-Axis Accelerometer

## Software Dependencies

You will need to install the following libraries via the Arduino Library Manager to compile the code:

*   **RadioHead** (for `RH_RF95.h`)
*   **BlueRobotics MS5837 Library** (for `MS5837.h`)
*   **Adafruit Unified Sensor** (for `Adafruit_Sensor.h`)
*   **Adafruit ADXL345** (for `Adafruit_ADXL345_U.h`)
*   *Note: `Wire.h`, `SPI.h`, `SD.h`, and `esp_sleep.h` are included by default with the ESP32 board manager core.*

## Pin Mapping & Wiring Guide

The following tables detail the connections between the XIAO ESP32C3 and the various peripheral modules. 

> **Important Note:** Pin **D9** on the XIAO ESP32C3 is intentionally left disconnected/remapped in the SPI initialization to avoid interference with the ESP32C3's bootloader mode.

### Master Pin Mapping: XIAO ESP32C3

| XIAO ESP32C3 Pin | Function / Protocol | Connected Peripheral(s) |
| :--- | :--- | :--- |
| **3V3** | Power (3.3V) | VCC for LoRa, SD Card, MS5837, and ADXL345 |
| **GND** | Ground | GND for LoRa, SD Card, MS5837, and ADXL345 |
| **D1** | Digital Out | LoRa Reset (`LORA_RST`) |
| **D2** | Hardware Interrupt | LoRa Interrupt / DIO0 (`LORA_INT`) |
| **D3** | SPI Chip Select | LoRa Chip Select (`LORA_CS`) |
| **D4** | I2C SDA | Shared SDA (MS5837 & ADXL345) |
| **D5** | I2C SCL | Shared SCL (MS5837 & ADXL345) |
| **D6** | SPI Chip Select | SD Card Chip Select (`SD_CS`) |
| **D7** | SPI MISO | Shared MISO (LoRa & SD Card) |
| **D8** | SPI SCK | Shared Clock (LoRa & SD Card) |
| **D10** | SPI MOSI | Shared MOSI (LoRa & SD Card) |

---

### Peripheral Wiring Breakdown

#### 1. RF95 LoRa Module (SPI Bus)
*   **VIN / 3V3** ➔ XIAO **3V3**
*   **GND** ➔ XIAO **GND**
*   **SCK** ➔ XIAO **D8**
*   **MISO** ➔ XIAO **D7**
*   **MOSI** ➔ XIAO **D10**
*   **NSS / CS** ➔ XIAO **D3**
*   **RST** ➔ XIAO **D1**
*   **DIO0 / INT** ➔ XIAO **D2**

#### 2. MicroSD Card Module (SPI Bus)
*   **VCC / 3V3** ➔ XIAO **3V3**
*   **GND** ➔ XIAO **GND**
*   **SCK** ➔ XIAO **D8**
*   **MISO** ➔ XIAO **D7**
*   **MOSI** ➔ XIAO **D10**
*   **CS** ➔ XIAO **D6**

#### 3. ADXL345 Accelerometer (I2C Bus)
*   **VCC** ➔ XIAO **3V3**
*   **GND** ➔ XIAO **GND**
*   **SDA** ➔ XIAO **D4**
*   **SCL** ➔ XIAO **D5**
*   **CS** ➔ XIAO **3V3** *(Forces I2C mode)*
*   **SDO** ➔ XIAO **GND** *(Sets I2C address to 0x53)*
*   **INT1** ➔ *Not Connected*
*   **INT2** ➔ *Not Connected*

#### 4. MS5837 Pressure/Depth Sensor (I2C Bus)
*   **VIN / VCC** ➔ XIAO **3V3**
*   **GND** ➔ XIAO **GND**
*   **SDA** ➔ XIAO **D4**
*   **SCL** ➔ XIAO **D5**

## Operational Flow

1. **Deep Sleep Cycle:** The ESP32 stays in deep sleep, waking periodically to listen for a surface transmission.
2. **LoRa Listening:** Upon waking, the device powers up the LoRa module for 500ms. If it receives a configuration packet, it updates RTC variables and prepares to record.
3. **Threshold Checking (Optional):** If configured in threshold mode, the device powers the pressure sensor to check if the delta exceeds the set threshold.
4. **Burst Logging:** The device initializes the SD card, pressure sensor, and accelerometer, and logs 1500 samples at 50Hz (20ms intervals) in a pre-formatted format delimited by `~`.
5. **Chunked Transmission:** The ESP32 reads the logged packets back from the SD card and transmits them via LoRa sequentially.
6. **Sleep:** The device sends an `EOF` marker and returns to deep sleep.
