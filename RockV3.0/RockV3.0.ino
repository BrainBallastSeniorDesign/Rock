#include "MS5837.h"
#include <RadioHead.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SD.h>
#include <esp_sleep.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pin Definitions
#define LORA_CS    D3  
#define LORA_RST   D1
#define LORA_INT   D2
#define SD_CS      D6

#define RF95_FREQ 915.0

// Instantiate modules
RH_RF95 rf95(LORA_CS, LORA_INT);
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

void setup() {
  bool lora_initialized = false;

  // Remap SPI bus to not use D9 (Bootload pin)
  SPI.begin(D8, D7, D10, -1);
  
  // 1. Setup CS pins immediately to keep SPI devices quiet
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  
  Serial.begin(115200);
  delay(1000);
  
  boot_count++;
  Serial.printf("\n--- Boot Count: %d ---\n", boot_count);

  bool wake = false;

  if(boots_left > 0) {
    if(threshold_mode) { 
      // 2. Initialize ONLY the I2C Bus and the Sensor
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
    // 2. Wake and Initialize LoRa FIRST to listen for surface signal
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW); delay(10);
    digitalWrite(LORA_RST, HIGH); delay(10);
  
    if (!rf95.init()) {
      Serial.println("LoRa init failed. Going to sleep.");
      goToSleep();
    }
    lora_initialized = true; // Mark as initialized
    
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(20, false);
    
    Serial.println("Listening for surface trigger...");
    
    // 3. THE DECISION: Wait up to LORA_LISTEN_WINDOW_MS for a packet
    if (rf95.waitAvailableTimeout(LORA_LISTEN_WINDOW_MS)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf) - 1; 
      
      if (rf95.recv(buf, &len)) {
        buf[len] = '\0'; 
        Serial.print("Signal received from surface: ");
        Serial.println((char*)buf);
        wake = true;

        // Parse variables
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

        // Put LoRa to sleep to save power
        rf95.sleep();
      }
    }
  }

  // 4. If triggered, proceed to record and transmit
  if (wake) {
    Serial.println("Trigger confirmed! Initializing Sensors...");

    // Initialize LoRa if it wasn't already initialized during the surface listen phase
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
    
    // --- PHASE 1: HIGH-SPEED SD CARD LOGGING & PACKET FORMATTING ---
    if (SD.begin(SD_CS)) {
      File dataFile = SD.open("/burst_log.txt", FILE_WRITE); 
      
      if (dataFile) {
        Serial.println("Recording 500 samples and formatting packets to SD...");
        unsigned long next_sample_time = millis();
        
        int packetNum = 0;
        char payload[250] = ""; 
        char lineBuffer[64]; 
        
        // Initialize the first payload string
        snprintf(payload, sizeof(payload), "%i|\n", packetNum);
        
        for (int i = 0; i < 500; i++) {
          while (millis() < next_sample_time) { delay(1); }
          next_sample_time += 20; 
          
          sensor.read();
          
          sensors_event_t event; 
          accel.getEvent(&event);

          snprintf(lineBuffer, sizeof(lineBuffer), "%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                   sensor.getPressure(), sensor.getTemperature(), 
                   event.acceleration.x, event.acceleration.y, event.acceleration.z);
          
          // >>> NEW ADDITION: Send Real-Time Update Every 50 Samples (1 Second at 50Hz) <<<
          if (i % 50 == 0) {
            char rtPacket[80];
            // Format with an "RT|" prefix so the surface receiver knows to parse it immediately
            snprintf(rtPacket, sizeof(rtPacket), "RT|%.2f,%.2f,%.2f,%.2f,%.2f", 
                     sensor.getPressure(), sensor.getTemperature(), 
                     event.acceleration.x, event.acceleration.y, event.acceleration.z);
            
            // Send the packet. We purposely omit waitPacketSent() here to prevent blocking
            // the 50Hz recording loop while the LoRa radio modulates the signal.
            rf95.send((uint8_t *)rtPacket, strlen(rtPacket) + 1);
          }
          // >>> END NEW ADDITION <<<

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

      // --- PHASE 2: CHUNKED LORA TRANSMISSION ---
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
            rf95.waitPacketSent();
            
            delay(300); 
          }
        }
        
        readFile.close();
        Serial.println("Bulk transmission complete.");
        
        delay(100);
        // Fixed syntax error from previous version here
        const char* eofMarker = "EOF"; 
        rf95.send((uint8_t *)eofMarker, strlen(eofMarker) + 1);
        rf95.waitPacketSent();
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

  // 5. Go back to deep sleep
  goToSleep();
}

void loop() {
  // Empty. The ESP32 never reaches this point.
}

void goToSleep() {
  Serial.println("Entering Deep Sleep...");
  Serial.flush(); 
  
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}