#include "MS5837.h"
#include <RadioHead.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SD.h>
#include <esp_sleep.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// Pin Definitions
#define LORA_CS    D3  
#define LORA_RST   D1
#define LORA_INT   D2
#define SD_CS      D6

#define RF95_FREQ 915.0
#define MAX_SAMPLES 12

MS5837 sensor;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// RTC memory variables
RTC_DATA_ATTR int boot_count = 0;
RTC_DATA_ATTR float last_recorded_pressure = 0.0;
RTC_DATA_ATTR int boots_left = 0;
RTC_DATA_ATTR bool threshold_mode = false;
RTC_DATA_ATTR uint64_t SLEEP_DURATION_SECONDS = 10;
RTC_DATA_ATTR float PRESSURE_THRESHOLD = 20.0;

// Constants
const uint16_t LORA_LISTEN_WINDOW_MS = 500;

class RH_RF95_Poll : public RH_RF95 {
public:
  RH_RF95_Poll(uint8_t slaveSelectPin, uint8_t interruptPin) 
    : RH_RF95(slaveSelectPin, interruptPin) {}
  
  void pollInterrupt() {
    this->handleInterrupt();
  }
};

RH_RF95_Poll rf95(LORA_CS, 255); 

void waitLoRaTx() {
  while (rf95.mode() == RHGenericDriver::RHModeTx) {
    if (digitalRead(LORA_INT) == HIGH) {
      rf95.pollInterrupt();
    }
    delay(1);
  }
}

// Data Compression Structs
struct __attribute__((packed)) SensorReading {
  uint32_t timestamp; 
  float pressure;
  float temperature;
  float abs_accel;
}; // 16 bytes per sample

struct __attribute__((packed)) DataPacket {
  uint16_t packet_id;
  uint8_t num_samples;
  uint8_t checksum;
  SensorReading samples[MAX_SAMPLES]; 
};

uint8_t calculateChecksum(uint8_t* data, uint16_t length) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
  }
  return crc;
}

void setup() {
  bool lora_initialized = false;

  SPI.begin(D8, D7, D10, -1);
  
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  
  pinMode(LORA_INT, INPUT);
  
  Serial.begin(115200);
  delay(1000);
  
  boot_count++;
  Serial.printf("\n--- Boot Count: %d ---\n", boot_count);

  bool wake = false;

  if(boots_left > 0) {
    if(threshold_mode) { 
      Wire.begin();
      if (!sensor.begin(0)) {
        Serial.println("MS5837 initialization failed!");
        goToSleep(); 
      }
      
      sensor.setDensity(0.997); 
      sensor.read();
      
      float current_pressure = sensor.getPressure();
      
      Serial.printf("Current Pressure: %.2f mbar\n", current_pressure);
      Serial.printf("Last Pressure: %.2f mbar\n", last_recorded_pressure);

      if (abs(current_pressure - last_recorded_pressure) >= PRESSURE_THRESHOLD || last_recorded_pressure == 0.0) {
        wake = true;
        Serial.println("Threshold exceeded! Initiating 50Hz Burst Record...");
        last_recorded_pressure = current_pressure; 
      }
    } else {
      wake = true; 
    }
  } else {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW); delay(10);
    digitalWrite(LORA_RST, HIGH); delay(10);
  
    if (!rf95.init()) {
      Serial.println("LoRa init failed. Going to sleep.");
      goToSleep();
    }
    lora_initialized = true; 
    
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(20, false);
    
    Serial.println("Listening for surface trigger...");
    
    unsigned long start_listen = millis();
    while (millis() - start_listen < LORA_LISTEN_WINDOW_MS) {
      if (digitalRead(LORA_INT) == HIGH) {
        rf95.pollInterrupt(); 
      }
      
      if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf) - 1; 
        
        if (rf95.recv(buf, &len)) {
          buf[len] = '\0'; 
          Serial.print("Signal received from surface: ");
          Serial.println((char*)buf);
          wake = true;

          int p_boots, p_mode;
          unsigned long long p_sleep;
          float p_thresh;
          
          if (sscanf((char*)buf, "%d,%llu,%f,%d", &p_boots, &p_sleep, &p_thresh, &p_mode) == 4) {
            boots_left = p_boots;
            SLEEP_DURATION_SECONDS = p_sleep;
            PRESSURE_THRESHOLD = p_thresh;
            threshold_mode = (p_mode > 0); 
            
            Serial.println("--- RTC Configuration Updated ---");
          } else {
            Serial.println("Standard wake signal (No config parameters parsed).");
          }

          rf95.sleep();
          break; 
        }
      }
      delay(1); 
    }
  }

  if (wake) {
    Serial.println("Trigger confirmed! Initializing Sensors...");

    if (!lora_initialized) {
      pinMode(LORA_RST, OUTPUT);
      digitalWrite(LORA_RST, LOW); delay(10);
      digitalWrite(LORA_RST, HIGH); delay(10);
      
      if (!rf95.init()) {
        Serial.println("LoRa init failed during wake up!");
        goToSleep();
      }
      rf95.setFrequency(RF95_FREQ);
      rf95.setTxPower(20, false);
    }

    Wire.begin();
    if (!sensor.begin(0)) {
      Serial.println("MS5837 initialization failed!");
      goToSleep(); 
    }
    sensor.setDensity(0.997); 

    if (!accel.begin()) {
      Serial.println("ADXL345 initialization failed!");
    } else {
      accel.setRange(ADXL345_RANGE_16_G);
    }
    
    if (SD.begin(SD_CS)) {
      // Changed to .bin as data is now binary
      File dataFile = SD.open("/burst_log.bin", FILE_WRITE); 
      
      if (dataFile) {
        Serial.println("Recording 500 samples and formatting packets to SD...");
        unsigned long next_sample_time = millis();
        
        DataPacket currentPacket;
        memset(&currentPacket, 0, sizeof(DataPacket));
        currentPacket.packet_id = 0;
        
        for (int i = 0; i < 500; i++) {
          while (millis() < next_sample_time) { 
            if (digitalRead(LORA_INT) == HIGH) {
              rf95.pollInterrupt();
            }
            delay(1); 
          }
          next_sample_time += 20; 
          
          sensor.read();
          sensors_event_t event; 
          accel.getEvent(&event);

          // Fixed the multiplication operators for abs_accel calculation
          float abs_accel = sqrt((event.acceleration.x * event.acceleration.x) + 
                                 (event.acceleration.y * event.acceleration.y) + 
                                 (event.acceleration.z * event.acceleration.z));
          
          unsigned long timestamp = millis();

          // Retain original real-time telemetry updates 
          if (i % 50 == 0) {
            char rtPacket[80];
            snprintf(rtPacket, sizeof(rtPacket), "RT|%lu,%.2f,%.2f,%.2f", 
                     timestamp, sensor.getPressure(), sensor.getTemperature(), abs_accel);
            Serial.println(rtPacket);
            rf95.send((uint8_t *)rtPacket, strlen(rtPacket) + 1);
          }

          // Append to the binary packet
          currentPacket.samples[currentPacket.num_samples].timestamp = timestamp;
          currentPacket.samples[currentPacket.num_samples].pressure = sensor.getPressure();
          currentPacket.samples[currentPacket.num_samples].temperature = sensor.getTemperature();
          currentPacket.samples[currentPacket.num_samples].abs_accel = abs_accel;
          currentPacket.num_samples++;

          // If the packet is full, calculate checksum and write binary to SD
          if (currentPacket.num_samples >= MAX_SAMPLES) {
            currentPacket.checksum = 0; // Zero before calc
            currentPacket.checksum = calculateChecksum((uint8_t*)&currentPacket, sizeof(DataPacket));
            
            dataFile.write((uint8_t*)&currentPacket, sizeof(DataPacket));
            
            // Setup for the next packet
            currentPacket.packet_id++;
            currentPacket.num_samples = 0;
            memset(&currentPacket.samples, 0, sizeof(currentPacket.samples)); // Zero out old samples
          }
        }
        
        // Catch any remaining samples left in the buffer when the loop ends
        if (currentPacket.num_samples > 0) {
            currentPacket.checksum = 0;
            currentPacket.checksum = calculateChecksum((uint8_t*)&currentPacket, sizeof(DataPacket));
            dataFile.write((uint8_t*)&currentPacket, sizeof(DataPacket));
        }
        
        dataFile.close();
        Serial.println("Burst recording complete.");
      } else {
         Serial.println("Error opening SD file!");
      }

      // Read binary packets back and transmit
      File readFile = SD.open("/burst_log.bin", FILE_READ);
      if (readFile) {
        Serial.println("Transmitting pre-formatted binary data in chunks...");
        
        DataPacket txPacket;
        // Read full structs direct from SD card
        while (readFile.available() >= sizeof(DataPacket)) {
          readFile.read((uint8_t*)&txPacket, sizeof(DataPacket));
          
          Serial.printf("Sending Packet Number %d\n", txPacket.packet_id);
          
          rf95.send((uint8_t *)&txPacket, sizeof(DataPacket));
          waitLoRaTx(); 
          
          delay(300); 
        }
        
        readFile.close();
        Serial.println("Bulk transmission complete.");
        
        delay(100);
        // Fixed pointer declaration syntax 
        const char* eofMarker = "EOF"; 
        rf95.send((uint8_t *)eofMarker, strlen(eofMarker) + 1);
        waitLoRaTx();
      }
    } else {
      Serial.println("SD Card initialization failed!");
    }
  } else {
    Serial.println("Not woken, going back to sleep.");
  }

  if(boots_left > 0) {
    boots_left--;
    if(boots_left == 0) {
      threshold_mode = false;
      SLEEP_DURATION_SECONDS = 10;
      PRESSURE_THRESHOLD = 20.0;
    }
  }

  rf95.sleep(); 
  goToSleep();
}

void loop() { }

void goToSleep() {
  Serial.println("Entering Deep Sleep...");
  Serial.flush(); 
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}