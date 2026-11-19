/*
Receiver 9.0

|
Receive LoRa packet
|
Identify packet type
|
+--------+---------+
| | |
RT BINARY EOF
| | |
Decode Validate Close burst
telemetry checksum and report
| | |
Display Decode Finalize
or log samples diagnostics
|
Save data
|
SD storage
*/


#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <string.h>
#include <stdio.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// =====================================================
// HARDWARE CONFIGURATION
// =====================================================

// TFT
#define TFT_CS 8
#define TFT_DC 9
#define TFT_RST 10

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// LoRa
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// SD
#define SD_CS 53

// Buttons
#define OPERATION_BUTTON_PIN 6
#define CONFIG_BUTTON_PIN 7

// =====================================================
// TRANSMITTER COMPATIBILITY
// =====================================================

#define MAX_SAMPLES 12

struct __attribute__((packed)) SensorReading
{
  uint32_t timestamp;
  float pressure;
  float temperature;
  float abs_accel;
};

struct __attribute__((packed)) DataPacket
{
  uint16_t packet_id;
  uint8_t num_samples;
  uint8_t checksum;

  SensorReading samples[MAX_SAMPLES];
};

// =====================================================
// CONFIGURATION WAKE PARAMETERS
// =====================================================

// These values are sent to the transmitter when
// the configuration button is held during startup.

const int CONFIG_BOOTS = 3;

const unsigned long CONFIG_SLEEP = 20;

const float CONFIG_THRESHOLD = 20.0;

const int CONFIG_MODE_VALUE = 1;

// =====================================================
// TIMING CONFIGURATION
// =====================================================

// The transmitter listens for approximately 500 ms.
// We repeatedly send wake messages and listen briefly.

const int WAKE_ATTEMPTS = 100;

const unsigned long WAKE_LISTEN_WINDOW_MS = 350;

// Allow gaps between incoming packets.
const unsigned long RECEIVE_INACTIVITY_TIMEOUT_MS = 10000;

// Maximum duration of one receiving session.
const unsigned long RECEIVE_TOTAL_TIMEOUT_MS = 120000;

// =====================================================
// RECEIVER STATE MACHINE
// =====================================================

enum ReceiverState
{
  IDLE,
  WAKE,
  RECEIVE,
  FINALIZE,
  ERROR_RECOVERY
};

ReceiverState currentState = IDLE;

// =====================================================
// SESSION VARIABLES
// =====================================================

File dataFile;

bool sdReady = false;
bool fileOpen = false;

bool configRequested = false;
bool sessionComplete = false;

char currentFilename[13] = "";

unsigned long sessionStartMillis = 0;
unsigned long lastPacketMillis = 0;

// Button edge detection
bool operationButtonArmed = true;

// =====================================================
// PACKET STATISTICS
// =====================================================

uint32_t packetsAccepted = 0;
uint32_t checksumFailures = 0;
uint32_t malformedPackets = 0;

uint32_t missingPackets = 0;
uint32_t duplicateOrOutOfOrderPackets = 0;

uint32_t samplesReceived = 0;
uint32_t samplesWritten = 0;

uint16_t expectedPacketId = 0;

// =====================================================
// SPI BUS MANAGEMENT
// =====================================================

void deselectAllSPI()
{
digitalWrite(TFT_CS, HIGH);
digitalWrite(SD_CS, HIGH);
digitalWrite(RFM95_CS, HIGH);
}

// =====================================================
// DISPLAY FUNCTIONS
// =====================================================

void drawScreen()
{
  deselectAllSPI();

  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  tft.setCursor(10, 10);
  tft.println("ROCK RECEIVER");

  tft.setCursor(10, 40);
  tft.println("State:");

  tft.setCursor(10, 80);
  tft.println("Pressure:");

  tft.setCursor(10, 110);
  tft.println("Temp:");

  tft.setCursor(10, 140);
  tft.println("Accel:");

  tft.setCursor(10, 170);
  tft.println("RSSI:");

  tft.setCursor(10, 205);
  tft.println("Packets:");
}

void updateState(const char* state)
{
  deselectAllSPI();

  tft.fillRect(150, 40, 165, 22, ST77XX_BLACK);

  tft.setCursor(150, 40);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);

  tft.print(state);
}

void updateTelemetry(
  float pressure,
  float temperature,
  float acceleration,
  int rssi
)
{
  deselectAllSPI();

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  tft.fillRect(150, 80, 165, 22, ST77XX_BLACK);
  tft.setCursor(150, 80);
  tft.print(pressure, 2);

  tft.fillRect(150, 110, 165, 22, ST77XX_BLACK);
  tft.setCursor(150, 110);
  tft.print(temperature, 2);

  tft.fillRect(150, 140, 165, 22, ST77XX_BLACK);
  tft.setCursor(150, 140);
  tft.print(acceleration, 2);

  tft.fillRect(150, 170, 165, 22, ST77XX_BLACK);
  tft.setCursor(150, 170);
  tft.print(rssi);

  tft.fillRect(150, 205, 165, 22, ST77XX_BLACK);
  tft.setCursor(150, 205);
  tft.print(packetsAccepted);
}

// =====================================================
// SERIAL DIAGNOSTICS
// =====================================================

void printStartupDiagnostics()
{
  Serial.println();
  Serial.println("==================================");
  Serial.println("ROCK RECEIVER 8.0");
  Serial.println("==================================");

  Serial.print("SensorReading size: ");
  Serial.println(sizeof(SensorReading));

  Serial.print("DataPacket size: ");
  Serial.println(sizeof(DataPacket));

  Serial.print("RadioHead maximum payload: ");
  Serial.println(RH_RF95_MAX_MESSAGE_LEN);

  Serial.print("SD card: ");
  Serial.println(sdReady ? "READY" : "FAILED");

  Serial.println("==================================");
  Serial.println();
}

void resetStatistics()
{
  packetsAccepted = 0;
  checksumFailures = 0;
  malformedPackets = 0;

  missingPackets = 0;
  duplicateOrOutOfOrderPackets = 0;

  samplesReceived = 0;
  samplesWritten = 0;

  expectedPacketId = 0;
}

void printSessionSummary()
{
  Serial.println();
  Serial.println("==================================");
  Serial.println("TRANSMISSION SUMMARY");
  Serial.println("==================================");

  Serial.print("Completion: ");
  Serial.println(sessionComplete ? "SUCCESSFUL" : "INCOMPLETE");

  Serial.print("File: ");
  Serial.println(fileOpen ? "STILL OPEN" :
  (currentFilename[0] ? currentFilename : "NONE"));

  Serial.print("Accepted packets: ");
  Serial.println(packetsAccepted);

  Serial.print("Checksum failures: ");
  Serial.println(checksumFailures);

  Serial.print("Malformed packets: ");
  Serial.println(malformedPackets);

  Serial.print("Missing packets: ");
  Serial.println(missingPackets);

  Serial.print("Duplicate/out-of-order packets: ");
  Serial.println(duplicateOrOutOfOrderPackets);

  Serial.print("Samples received: ");
  Serial.println(samplesReceived);

  Serial.print("Samples written: ");
  Serial.println(samplesWritten);

  Serial.println("==================================");
  Serial.println();
}

// =====================================================
// CHECKSUM
// =====================================================

uint8_t calculateChecksum(
uint8_t* data,
uint16_t length
)
  {
  uint8_t checksum = 0;

  for(uint16_t i = 0; i < length; i++)
  {
    checksum ^= data[i];
  }

  return checksum;
}


bool verifyPacketChecksum(DataPacket& packet)
{
  uint8_t receivedChecksum = packet.checksum;

  packet.checksum = 0;

  uint8_t calculatedChecksum =
  calculateChecksum(
    (uint8_t*)&packet,
    sizeof(DataPacket)
  );

  packet.checksum = receivedChecksum;

  Serial.print("Received checksum: ");
  Serial.println(receivedChecksum, HEX);

  Serial.print("Calculated checksum: ");
  Serial.println(calculatedChecksum, HEX);

  return receivedChecksum == calculatedChecksum;
}

// =====================================================
// SD CARD / FILE MANAGEMENT
// =====================================================

bool startFile()
{
  if(!sdReady)
  {
  Serial.println("ERROR: SD card unavailable.");
  return false;
  }

  currentFilename[0] = '\0';

  for(unsigned int i = 0; i < 1000; i++)
  {
    snprintf(
      currentFilename,
      sizeof(currentFilename),
      "burst%03u.csv",
      i
    );

    if(!SD.exists(currentFilename))
    {
      break;
    }

    currentFilename[0] = '\0';
  }

  if(currentFilename[0] == '\0')
  {
    Serial.println("ERROR: No available filename.");
    return false;
  }

  deselectAllSPI();

  dataFile = SD.open(
  currentFilename,
  FILE_WRITE
  );

  if(!dataFile)
  {
    Serial.println("ERROR: Could not create CSV.");
    currentFilename[0] = '\0';
    return false;
  }

  fileOpen = true;

  dataFile.println(
    "timestamp,pressure,temperature,accel"
  );

  dataFile.flush();

  Serial.print("Created file: ");
  Serial.println(currentFilename);

  return true;
}

void closeFile()
{
  if(fileOpen)
  {
    deselectAllSPI();

    dataFile.flush();
    dataFile.close();

    fileOpen = false;

    Serial.println("CSV file closed.");
  }
}

void savePacket(DataPacket& packet)
{
if(!fileOpen)
{
Serial.println("WARNING: Packet received but no CSV is open.");
return;
}

deselectAllSPI();

for(uint8_t i = 0; i < packet.num_samples; i++)
{
dataFile.print(packet.samples[i].timestamp);
dataFile.print(",");

dataFile.print(packet.samples[i].pressure, 2);
dataFile.print(",");

dataFile.print(packet.samples[i].temperature, 2);
dataFile.print(",");

dataFile.println(packet.samples[i].abs_accel, 2);

samplesWritten++;
}

// Commit this packet's data before the next one arrives.
dataFile.flush();
}

// =====================================================
// SESSION MANAGEMENT
// =====================================================

void beginSession()
{
  resetStatistics();

  sessionComplete = false;

  currentFilename[0] = '\0';

  sessionStartMillis = 0;
  lastPacketMillis = 0;

  Serial.println();
  Serial.println("==================================");
  Serial.println("NEW RECEIVING SESSION");
  Serial.println("==================================");

  Serial.print("Configuration requested: ");
  Serial.println(configRequested ? "YES" : "NO");

  currentState = WAKE;
}

void finalizeSession()
{
  closeFile();

  printSessionSummary();

  updateState(
    sessionComplete ? "COMPLETE" : "INCOMPLETE"
  );

  delay(1500);

  updateState("IDLE");

  Serial.println("Receiver returning to IDLE.");

  currentFilename[0] = '\0';

  currentState = IDLE;
}

// =====================================================
// WAKE SEQUENCE
// =====================================================

bool performWakeSequence(bool configMode)
{
  Serial.println();
  Serial.println("Beginning wake sequence...");

  for(int attempt = 0; attempt < WAKE_ATTEMPTS; attempt++)
  {
    deselectAllSPI();

    if(configMode)
    {
      char message[64];
      char threshStr[16];

      // Convert float to string: (value, min_width, decimal_places, buffer)
      dtostrf(CONFIG_THRESHOLD, 1, 2, threshStr);

      snprintf(
        message,
        sizeof(message),
        "%d,%lu,%s,%d",
        CONFIG_BOOTS,
        CONFIG_SLEEP,
        threshStr,
        CONFIG_MODE_VALUE
      );

      // Include the null terminator for the
      // transmitter's string-based parser.
      rf95.send(
      (uint8_t*)message,
      strlen(message) + 1
      );

      Serial.print("CONFIG WAKE ");
      Serial.println(attempt + 1);
    }
    else
    {
      const char* message = "WAKE";

      rf95.send(
      (uint8_t*)message,
      strlen(message) + 1
      );

      Serial.print("WAKE ");
      Serial.println(attempt + 1);
    }

    rf95.waitPacketSent();

    rf95.setModeRx();

    unsigned long listenStart = millis();

    while(
      millis() - listenStart <
      WAKE_LISTEN_WINDOW_MS
    )
    {
      if(rf95.available())
      {
        Serial.println();
        Serial.println("Transmitter response detected.");

        return true;
      }
    }
  }

  Serial.println();
  Serial.println("ERROR: No transmitter response.");

  return false;
}

// =====================================================
// REAL-TIME TELEMETRY
// =====================================================

void processRealtimePacket(uint8_t* buf)
{
  char* ptr = (char*)buf + 3;

  char* tokTime  = strtok(ptr, ",");
  char* tokPres  = strtok(NULL, ",");
  char* tokTemp  = strtok(NULL, ",");
  char* tokAccel = strtok(NULL, ",");

  if (!tokTime || !tokPres || !tokTemp || !tokAccel)
  {
    Serial.println("Malformed RT telemetry packet.");
    malformedPackets++;
    return;
  }

  unsigned long timestamp = strtoul(tokTime, NULL, 10);
  float pressure          = atof(tokPres);
  float temperature       = atof(tokTemp);
  float acceleration      = atof(tokAccel);

  Serial.println();
  Serial.println("===== REAL-TIME TELEMETRY =====");
  Serial.print("Timestamp: ");    Serial.println(timestamp);
  Serial.print("Pressure: ");     Serial.println(pressure);
  Serial.print("Temperature: ");  Serial.println(temperature);
  Serial.print("Acceleration: "); Serial.println(acceleration);
  Serial.print("RSSI: ");         Serial.println(rf95.lastRssi());
  Serial.println("================================");

  updateTelemetry(pressure, temperature, acceleration, rf95.lastRssi());
}

// =====================================================
// BINARY PACKET PROCESSING
// =====================================================

void processBinaryPacket(
uint8_t* buf,
uint8_t len
)
{
if(len != sizeof(DataPacket))
{
malformedPackets++;
return;
}

DataPacket packet;

memcpy(
&packet,
buf,
sizeof(DataPacket)
);

// Validate checksum first.
if(!verifyPacketChecksum(packet))
{
Serial.println("CHECKSUM FAILED");

checksumFailures++;

return;
}

// Validate sample count.
if(
packet.num_samples == 0 ||
packet.num_samples > MAX_SAMPLES
)
{
Serial.println("INVALID SAMPLE COUNT");

malformedPackets++;

return;
}

// Validate packet sequence.
uint16_t difference =
packet.packet_id - expectedPacketId;

if(difference >= 0x8000)
{
Serial.print("DUPLICATE/OUT-OF-ORDER PACKET: ");
Serial.println(packet.packet_id);

duplicateOrOutOfOrderPackets++;

return;
}

if(difference > 0)
{
Serial.print("MISSING PACKETS BEFORE ID ");
Serial.print(packet.packet_id);
Serial.print(": ");
Serial.println(difference);

missingPackets += difference;
}

expectedPacketId = packet.packet_id + 1;

packetsAccepted++;

samplesReceived += packet.num_samples;

Serial.println();
Serial.println("===== BINARY PACKET =====");

Serial.print("Packet ID: ");
Serial.println(packet.packet_id);

Serial.print("Samples: ");
Serial.println(packet.num_samples);

Serial.println("Checksum: PASSED");

Serial.print("RSSI: ");
Serial.println(rf95.lastRssi());

Serial.println("=========================");

savePacket(packet);
}

// =====================================================
// INSPECT PACKET
// =====================================================

void dumpPacket(uint8_t* buf, uint8_t len)
{
Serial.println("--- RAW PACKET DUMP ---");

Serial.print("Length: ");
Serial.println(len);

Serial.print("HEX: ");

uint8_t limit = len < 40 ? len : 40;

for(uint8_t i = 0; i < limit; i++)
{
if(buf[i] < 0x10)
Serial.print("0");

Serial.print(buf[i], HEX);
Serial.print(" ");
}

Serial.println();

Serial.print("ASCII: ");

for(uint8_t i = 0; i < limit; i++)
{
if(buf[i] >= 32 && buf[i] <= 126)
Serial.write(buf[i]);
else
Serial.print(".");
}

Serial.println();
Serial.println("-----------------------");
}

// =====================================================
// RECEIVE PACKET DISPATCHER
// =====================================================

void receivePacket()
{
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 1];

uint8_t len = RH_RF95_MAX_MESSAGE_LEN;

if(!rf95.recv(buf, &len))
{
return;
}

dumpPacket(buf, len);

lastPacketMillis = millis();

// Safely terminate text messages.
buf[len] = '\0';

Serial.print("Received packet. Length: ");
Serial.print(len);

Serial.print(" | RSSI: ");
Serial.println(rf95.lastRssi());

// -------------------------------------------------
// REAL-TIME TELEMETRY
// -------------------------------------------------

if(
len >= 3 &&
memcmp(buf, "RT|", 3) == 0
)
{
processRealtimePacket(buf);
return;
}

// -------------------------------------------------
// END OF TRANSMISSION
// -------------------------------------------------

// The transmitter sends "EOF" including its
// terminating null byte.
if(
len == 4 &&
memcmp(buf, "EOF", 3) == 0 &&
buf[3] == '\0'
)
{
Serial.println();
Serial.println("EOF DETECTED.");

sessionComplete =
packetsAccepted > 0 &&
samplesWritten > 0 &&
checksumFailures == 0 &&
malformedPackets == 0;

currentState = FINALIZE;

return;
}

// -------------------------------------------------
// BINARY DATA
// -------------------------------------------------

if(len == sizeof(DataPacket))
{
processBinaryPacket(buf, len);
return;
}

// -------------------------------------------------
// UNKNOWN MESSAGE
// -------------------------------------------------

Serial.println("WARNING: Unknown packet format.");

malformedPackets++;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
Serial.begin(115200);

delay(1500);

Serial.println();
Serial.println("BOOTING ROCK RECEIVER 8.0");

// Buttons
pinMode(OPERATION_BUTTON_PIN, INPUT_PULLUP);
pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);

// Ensure every SPI device is deselected.
pinMode(TFT_CS, OUTPUT);
pinMode(SD_CS, OUTPUT);
pinMode(RFM95_CS, OUTPUT);

deselectAllSPI();

// Mega hardware SPI initialization.
SPI.begin();

// -------------------------------------------------
// SD INITIALIZATION
// -------------------------------------------------

Serial.println("Initializing SD card...");

sdReady = SD.begin(SD_CS);

Serial.println(
sdReady ? "SD initialization OK" :
"SD initialization FAILED"
);

// -------------------------------------------------
// LORA RESET
// -------------------------------------------------

pinMode(RFM95_RST, OUTPUT);

digitalWrite(RFM95_RST, LOW);
delay(10);

digitalWrite(RFM95_RST, HIGH);
delay(10);

// -------------------------------------------------
// LORA INITIALIZATION
// -------------------------------------------------

Serial.println("Initializing LoRa...");

if(!rf95.init())
{
Serial.println("FATAL: LoRa initialization failed.");

while(1);
}

if(!rf95.setFrequency(RF95_FREQ))
{
Serial.println("FATAL: Frequency configuration failed.");

while(1);
}

rf95.setTxPower(20, false);

Serial.println("LoRa initialization OK.");
Serial.println("Frequency: 915 MHz");

// -------------------------------------------------
// TFT INITIALIZATION
// -------------------------------------------------

Serial.println("Initializing TFT...");

deselectAllSPI();

tft.init(240, 320);

tft.setRotation(1);

drawScreen();

updateState("IDLE");

// -------------------------------------------------
// STARTUP DIAGNOSTICS
// -------------------------------------------------

printStartupDiagnostics();

Serial.println("Receiver ready.");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
switch(currentState)
{
// =============================================
// IDLE
// =============================================

case IDLE:
{
bool buttonPressed =
digitalRead(OPERATION_BUTTON_PIN) == LOW;

if(!buttonPressed)
{
operationButtonArmed = true;
break;
}

if(!operationButtonArmed)
{
break;
}

delay(30);

if(
digitalRead(OPERATION_BUTTON_PIN) != LOW
)
{
break;
}

operationButtonArmed = false;

// Capture configuration button state
// at the moment operation is requested.
configRequested =
digitalRead(CONFIG_BUTTON_PIN) == LOW;

beginSession();

updateState("WAKING");

break;
}

// =============================================
// WAKE
// =============================================

case WAKE:
{
if(!performWakeSequence(configRequested))
{
currentState = ERROR_RECOVERY;
break;
}

// A response is already waiting in the
// radio receive buffer. Do not consume it
// here; RECEIVE will process it normally.

startFile();

sessionStartMillis = millis();
lastPacketMillis = millis();

updateState("RECEIVING");

Serial.println();
Serial.println("Entering RECEIVE state.");

currentState = RECEIVE;

break;
}

// =============================================
// RECEIVE
// =============================================

case RECEIVE:
{
if(rf95.available())
{
receivePacket();
}

if(currentState != RECEIVE)
{
break;
}

// Inactivity timeout.
// if(
// millis() - lastPacketMillis >
// RECEIVE_INACTIVITY_TIMEOUT_MS
// )
// {
// Serial.println();
// Serial.println("ERROR: Receive inactivity timeout.");

// currentState = ERROR_RECOVERY;

// break;
// }

// Overall session timeout.
if(
millis() - sessionStartMillis >
RECEIVE_TOTAL_TIMEOUT_MS
)
{
Serial.println();
Serial.println("ERROR: Overall session timeout.");

currentState = ERROR_RECOVERY;

break;
}

break;
}

// =============================================
// FINALIZE
// =============================================

case FINALIZE:
{
finalizeSession();

break;
}

// =============================================
// ERROR / RECOVERY
// =============================================

case ERROR_RECOVERY:
{
Serial.println();
Serial.println("SESSION FAILED OR INCOMPLETE.");

sessionComplete = false;

closeFile();

currentState = FINALIZE;

break;
}
}
}