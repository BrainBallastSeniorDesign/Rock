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

// 1. Subclass RadioHead to bypass the ESP32 ISR crash
class RH_RF95_Poll : public RH_RF95 {
public:
  RH_RF95_Poll(uint8_t slaveSelectPin, uint8_t interruptPin) 
    : RH_RF95(slaveSelectPin, interruptPin) {}
  
  // Expose the protected interrupt handler so we can call it safely from the main thread
  void pollInterrupt() {
    this->handleInterrupt();
  }
};

// 2. Instantiate LoRa with 255 to PREVENT it from attaching a hardware interrupt
RH_RF95_Poll rf95(LORA_CS, 255); 

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

// 3. Helper to wait for TX without blocking the thread blindly
void waitLoRaTx() {
  while (rf95.mode() == RHGenericDriver::RHModeTx) {
    if (digitalRead(LORA_INT) == HIGH) {
      rf95.pollInterrupt();
    }
    delay(1);
  }
}

void setup() {
  bool lora_initialized = false;

  SPI.begin(D8, D7, D10, -1);
  
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  
  // Explicitly set the interrupt pin as input since RadioHead won't do it for pin 255
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
      float current_temp = sensor.getTemperature();
      float current_depth = sensor.getDepth();
      
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
    
    // 4. Polling listen loop replaces the crashing hardware interrupt
    unsigned long start_listen = millis();
    while (millis() - start_listen < LORA_LISTEN_WINDOW_MS) {
      if (digitalRead(LORA_INT) == HIGH) {
        rf95.pollInterrupt(); // Safely process the SPI read in the main thread
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
            Serial.printf("Boots Left: %d\n", boots_left);
            Serial.printf("Sleep Duration: %llu seconds\n", SLEEP_DURATION_SECONDS);
            Serial.printf("Threshold: %.2f\n", PRESSURE_THRESHOLD);
            Serial.printf("Mode: %s\n", threshold_mode ? "Threshold" : "Standard");
          } else {
            Serial.println("Standard wake signal (No config parameters parsed).");
          }

          rf95.sleep();
          break; 
        }
      }
      delay(1); // Small yield to prevent watchdog timeout
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
      File dataFile = SD.open("/burst_log.txt", FILE_WRITE); 
      
      if (dataFile) {
        Serial.println("Recording 500 samples and formatting packets to SD...");
        unsigned long next_sample_time = millis();
        
        int packetNum = 0;
        char payload[250] = ""; 
        char lineBuffer[80]; 
        
        snprintf(payload, sizeof(payload), "%i|\n", packetNum);
        
        for (int i = 0; i < 500; i++) {
          while (millis() < next_sample_time) { 
            // Keep polling LoRa in the background to clear TX Done flags for the 1Hz updates
            if (digitalRead(LORA_INT) == HIGH) {
              rf95.pollInterrupt();
            }
            delay(1); 
          }
          next_sample_time += 20; 
          
          sensor.read();
          sensors_event_t event; 
          accel.getEvent(&event);

          // Calculate absolute acceleration
          float abs_accel = sqrt((event.acceleration.x * event.acceleration.x) + 
                                 (event.acceleration.y * event.acceleration.y) + 
                                 (event.acceleration.z * event.acceleration.z));
          
          unsigned long timestamp = millis();

          // Format: Timestamp,Pressure,Temperature,AbsoluteAcceleration
          snprintf(lineBuffer, sizeof(lineBuffer), "%lu,%.2f,%.2f,%.2f\n", 
                   timestamp, sensor.getPressure(), sensor.getTemperature(), abs_accel);
          
          if (i % 50 == 0) {
            char rtPacket[80];
            snprintf(rtPacket, sizeof(rtPacket), "RT|%lu,%.2f,%.2f,%.2f", 
                     timestamp, sensor.getPressure(), sensor.getTemperature(), abs_accel);
            Serial.println(rtPacket);
            rf95.send((uint8_t *)rtPacket, strlen(rtPacket) + 1);
          }

          if (strlen(payload) + strlen(lineBuffer) > 200) {
            dataFile.print(payload);
            dataFile.print('~'); 
            
            payload[0] = '\0';
            packetNum++;
            snprintf(payload, sizeof(payload), "%i|\n", packetNum);
          }
          
          strcat(payload, lineBuffer);
        }
        
        if (strlen(payload) > 0) {
            dataFile.print(payload);
            dataFile.print('~');
        }
        
        dataFile.close();
        Serial.println("Burst recording complete.");
      } else {
         Serial.println("Error opening SD file!");
      }

      File readFile = SD.open("/burst_log.txt", FILE_READ);
      if (readFile) {
        Serial.println("Transmitting pre-formatted data in chunks...");
        
        while (readFile.available()) {
          String packet = readFile.readStringUntil('~');
          
          if (packet.length() > 0) {
            int pipeIndex = packet.indexOf('|');
            if (pipeIndex > 0) {
              Serial.printf("Sending Packet Number %s\n", packet.substring(0, pipeIndex).c_str());
            }
            
            rf95.send((uint8_t *)packet.c_str(), packet.length() + 1);
            waitLoRaTx(); // Use our non-blocking wait
            
            delay(300); 
          }
        }
        
        readFile.close();
        Serial.println("Bulk transmission complete.");
        
        delay(100);
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