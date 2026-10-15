// ============================================================
// For 5.0 demo sketch:
//1) Assume rock wakes every 10 seconds by default.
//2) Send a configuration wake to make the rock sleep for 20 seconds as opposed to 10
// and that makes the rock go through 3 cycles before returning to standard operation, where
// the delay is 10 seconds once again. 
// This means the surface module should refrain from sending any wake signals for the entire time the rock
// is obeying its current configuration that dictates boot cycles. 
// 3 boot cycles * (20 seconds  waiting to wake + ~ 20 seconds of rock recording then transmitting and surface receiving)
// 
// ============================================================
// Config mode can now handle EOF misses. After config time runs out it returns to standard operation regardless.
// ============================================================
// Receiver 5.0
// Arduino Mega 2560
//
// LoRa + SD burst receiver with:
//
//   - Standard WAKE command
//   - Configuration WAKE command via button on pin 7
//   - Packet-number diagnostics
//   - Packet timing diagnostics
//   - RSSI reporting
//   - Missing/duplicate packet detection
//   - SD storage of sensor data only
//   - Automatic sequential filenames
//   - EOF detection
//   - Burst summary report
//
// Standard command sent to transmitter:
//
//     WAKE
//
// Configuration command:
//
//     boots,sleep_seconds,threshold,mode
//
// Example:
//
//     5,120,20.0,1
//
// Transmitted data packets:
//
//     0|
//     1012.34,22.51
//     1012.30,22.49
//     ...
//
// EOF:
//
//     EOF
// ============================================================

#include <RadioHead.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <SD.h>

// ============================================================
// LoRa configuration
// ============================================================

#define RFM95_CS  4
#define RFM95_RST 2
#define RFM95_INT 3

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// ============================================================
// SD configuration
// ============================================================

#define SD_CS 53

// ============================================================
// Configuration button
// ============================================================

#define CONFIG_BUTTON_PIN 7

// ============================================================
// Transmitter configuration
//
// These values are sent when the configuration button is
// pressed.
//
// Format:
//
//     boots,sleep_seconds,threshold,mode
//
// Example:
//
//     5,120,20.0,1
// ============================================================

// specifically for the demo presentation
const int DEMO_CONFIG_BOOTS = 3;

const unsigned long DEMO_CONFIG_SLEEP_SECONDS = 20;

const int CONFIG_MODE = 1;

// ============================================================
// Configured boot-cycle tracking
// ============================================================

bool configModeActive = false;

int remainingConfiguredBursts = 0;

const float CONFIG_PRESSURE_THRESHOLD = 20.0;


// ============================================================
// Wake behavior
// ============================================================

// How long the receiver repeatedly sends wake messages.
const unsigned long WAKE_DURATION_MS = 20000;

// Time between individual wake messages.
const unsigned long WAKE_INTERVAL_MS = 200;

// How long to wait for the transmitter's burst after
// finishing the wake sequence.
const unsigned long WAKE_RETRY_INTERVAL_MS = 180000;

// ============================================================
// EOF miss catcher for config rock cycles
// ============================================================

unsigned long configModeStartTime = 0;

const unsigned long CONFIG_MODE_TIMEOUT_MS = 200000;

// ============================================================
// File state
// ============================================================

File dataFile;

bool recordingFile = false;

int fileNumber = 0;

String filename;

// ============================================================
// Burst diagnostic state
// ============================================================

bool recordingBurst = false;

long packetsReceived = 0;

long missingPackets = 0;

long duplicatePackets = 0;

long highestPacket = -1;

long expectedPacket = 0;

long totalBytesReceived = 0;

unsigned long burstStartTime = 0;

unsigned long lastPacketTime = 0;

// ============================================================
// Setup
// ============================================================

void setup() {

  Serial.begin(9600);

  while (!Serial);

  Serial.println();
  Serial.println("=================================");
  Serial.println("LoRa + SD Burst Receiver");
  Serial.println("Receiver 5.0");
  Serial.println("=================================");
  Serial.println();

  // ----------------------------------------------------------
  // Configuration button
  // ----------------------------------------------------------

  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);

  // ----------------------------------------------------------
  // Mega SPI master configuration
  // ----------------------------------------------------------

  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  // ----------------------------------------------------------
  // LoRa CS
  // ----------------------------------------------------------

  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_CS, HIGH);

  // ----------------------------------------------------------
  // SD card
  // ----------------------------------------------------------

  Serial.println("Initializing SD card...");

  if (!SD.begin(SD_CS)) {

    Serial.println("SD initialization FAILED!");
    Serial.println("Check SD wiring.");

    while (1);
  }

  Serial.println("SD initialization successful.");
  Serial.println();

  // ----------------------------------------------------------
  // LoRa reset
  // ----------------------------------------------------------

  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(RFM95_RST, HIGH);

  // ----------------------------------------------------------
  // LoRa initialization
  // ----------------------------------------------------------

  Serial.println("Initializing LoRa...");

  digitalWrite(RFM95_RST, LOW);
  delay(10);

  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {

    Serial.println("LoRa initialization FAILED!");
    Serial.println("Check LoRa wiring.");

    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {

    Serial.println("setFrequency FAILED!");

    while (1);
  }

  rf95.setTxPower(23, false);

  Serial.println("LoRa initialized successfully.");
  Serial.println();

  Serial.println("Configuration button:");
  Serial.println("  Pin 7 -> GND = CONFIGURATION WAKE");
  Serial.println("  Pin 7 open  = STANDARD WAKE");
  Serial.println();
}

// ============================================================
// Main loop
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Decide which type of wake command to send.
  //
  // INPUT_PULLUP means:
  //
  //   HIGH = button not pressed
  //   LOW  = button pressed
  // ----------------------------------------------------------

  bool configurationRequested =
      (digitalRead(CONFIG_BUTTON_PIN) == LOW);

  // Small debounce if button is pressed.
  if (configurationRequested) {

    delay(50);

    configurationRequested =
        (digitalRead(CONFIG_BUTTON_PIN) == LOW);
  }

  // ----------------------------------------------------------
  // Send appropriate wake command
  // ----------------------------------------------------------

  if (configModeActive) {

      Serial.println();
      Serial.println("Configuration mode active.");

      Serial.print("Configured bursts remaining: ");
      Serial.println(remainingConfiguredBursts);
  }
  else {

      if (configurationRequested) {

          sendConfigurationWake();

      } else {

          sendStandardWake();
      }
  }

  // ----------------------------------------------------------
  // Now wait for transmitter burst
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("=================================");
  Serial.println("Waiting for burst transmission...");
  Serial.println("=================================");
  Serial.println();

  unsigned long waitStart = millis();

  while (millis() - waitStart < WAKE_RETRY_INTERVAL_MS) {

    // timeout check
    if (configModeActive &&
    millis() - configModeStartTime >= CONFIG_MODE_TIMEOUT_MS)
    {
    Serial.println();
    Serial.println("CONFIGURATION TIMEOUT");
    Serial.println("Returning to standard operation.");

    configModeActive = false;
    remainingConfiguredBursts = 0;

    break;
    }

    // --------------------------------------------------------
    // Check whether a packet is available.
    // --------------------------------------------------------

    if (!rf95.available()) {
      continue;
    }

    // --------------------------------------------------------
    // Receive packet
    // --------------------------------------------------------

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 1];

    uint8_t len = RH_RF95_MAX_MESSAGE_LEN;

    if (!rf95.recv(buf, &len)) {

      Serial.println("Receive failed.");

      continue;
    }

    // IMPORTANT:
    // Capture the receive time immediately.
    //
    // We do this before SD operations so that packet timing
    // tells us about gaps between actual radio receptions.
    unsigned long receiveTime = millis();

    // Add string terminator for diagnostics/parsing.
    buf[len] = '\0';

    // --------------------------------------------------------
    // Basic packet information
    // --------------------------------------------------------

    Serial.print("Received packet (");
    Serial.print(len);
    Serial.print(" bytes)");

    Serial.print(" | RSSI: ");
    Serial.print(rf95.lastRssi());
    Serial.println(" dBm");

    // ========================================================
    // Check for EOF
    // ========================================================

    if (strcmp((char*)buf, "EOF") == 0) {

    Serial.println();
    Serial.println("EOF received.");

    finishFile();

    finishBurst();

    if (configModeActive) {

        remainingConfiguredBursts--;

        Serial.print(
            "Configured bursts remaining: "
        );

        Serial.println(
            remainingConfiguredBursts
        );

        if (remainingConfiguredBursts <= 0) {

            configModeActive = false;

            Serial.println();
            Serial.println(
                "Configuration sequence complete."
            );

            Serial.println(
                "Returning to standard operation."
            );
        }
    }

    Serial.println();
    Serial.println("Burst transmission complete.");
    Serial.println();

    return;
    }

    // ========================================================
    // Start diagnostic burst state
    // ========================================================

    if (!recordingBurst) {

      startBurst(receiveTime);
    }

    // ========================================================
    // Find packet-number separator
    //
    // Expected:
    //
    //     0|
    //     1|
    //     2|
    //
    // ========================================================

    char* separator = strchr((char*)buf, '|');

    if (separator == NULL) {

      Serial.println("WARNING: No '|' separator found.");
      Serial.println("Packet ignored.");

      continue;
    }

    // --------------------------------------------------------
    // Separate packet number from payload.
    // --------------------------------------------------------

    *separator = '\0';

    long packetNumber = atol((char*)buf);

    char* payload = separator + 1;

    // --------------------------------------------------------
    // The transmitter currently creates:
    //
    //     packetNumber|\n
    //
    // So remove that newline before writing to SD.
    // --------------------------------------------------------

    if (payload[0] == '\r') {
      payload++;
    }

    if (payload[0] == '\n') {
      payload++;
    }

    // --------------------------------------------------------
    // Calculate actual payload length.
    //
    // 'len' includes the packet-number header and the
    // transmitter's null terminator.
    // --------------------------------------------------------

    int payloadOffset =
        payload - (char*)buf;

    int payloadLength = len;

    // Remove transmitted null terminator if present.
    if (payloadLength > 0 &&
        buf[payloadLength - 1] == '\0') {

      payloadLength--;
    }

    payloadLength -= payloadOffset;

    // --------------------------------------------------------
    // Safety check
    // --------------------------------------------------------

    if (payloadLength < 0) {

      Serial.println("WARNING: Invalid payload length.");
      Serial.println("Packet ignored.");

      continue;
    }

    // ========================================================
    // Packet-number diagnostics
    // ========================================================

    packetsReceived++;

    // Track highest packet number.
    if (packetNumber > highestPacket) {

      highestPacket = packetNumber;
    }

    // --------------------------------------------------------
    // First packet
    // --------------------------------------------------------

    if (packetsReceived == 1) {

      if (packetNumber > 0) {

        missingPackets += packetNumber;

        Serial.print("WARNING: Missing packets 0-");

        Serial.println(packetNumber - 1);
      }

      expectedPacket = packetNumber + 1;
    }

    // --------------------------------------------------------
    // Subsequent packets
    // --------------------------------------------------------

    else {

      // Exactly the packet we expected.
      if (packetNumber == expectedPacket) {

        expectedPacket++;
      }

      // Packet number jumped ahead.
      else if (packetNumber > expectedPacket) {

        long gap =
            packetNumber - expectedPacket;

        missingPackets += gap;

        Serial.print("WARNING: Missing packets ");

        Serial.print(expectedPacket);

        Serial.print("-");

        Serial.print(packetNumber - 1);

        Serial.print(" (");

        Serial.print(gap);

        Serial.println(" packets)");

        expectedPacket =
            packetNumber + 1;
      }

      // Packet number is behind what we expected.
      else {

        duplicatePackets++;

        Serial.print(
            "WARNING: Duplicate/out-of-order packet "
        );

        Serial.println(packetNumber);
      }
    }

    // ========================================================
    // Packet timing
    // ========================================================

    totalBytesReceived += payloadLength;

    unsigned long deltaTime =
        receiveTime - lastPacketTime;

    lastPacketTime = receiveTime;

    Serial.print("t=");
    Serial.print(receiveTime);

    Serial.print(" ms | dt=");
    Serial.print(deltaTime);

    Serial.print(" ms | Packet=");
    Serial.print(packetNumber);

    Serial.print(" | Data=");
    Serial.print(payloadLength);

    Serial.print(" bytes | RSSI=");
    Serial.print(rf95.lastRssi());

    Serial.println(" dBm");

    // ========================================================
    // SD file
    // ========================================================

    if (!recordingFile) {

      startNewFile();
    }

    if (recordingFile) {

      // Write ONLY the sensor data.
      //
      // Packet number is diagnostic information and does
      // not go into the reconstructed data file.

      dataFile.write(
          (uint8_t*)payload,
          payloadLength
      );

      Serial.print("  SD write: ");
      Serial.print(payloadLength);
      Serial.println(" bytes");
    }
  }

  // ==========================================================
  // No complete burst
  // ==========================================================

  Serial.println();
  Serial.println("No complete burst received.");
  Serial.println("Retrying wake cycle...");
  Serial.println();
}

// ============================================================
// Send standard WAKE
// ============================================================

void sendStandardWake() {

  Serial.println();
  Serial.println("==============================");
  Serial.println("STANDARD WAKE");
  Serial.println("==============================");

  const char* wakeMessage = "WAKE";

  unsigned long startTime = millis();

  int wakeCount = 0;

  while (millis() - startTime < WAKE_DURATION_MS) {

    rf95.send(
        (uint8_t*)wakeMessage,
        strlen(wakeMessage)
    );

    rf95.waitPacketSent();

    wakeCount++;

    delay(WAKE_INTERVAL_MS);
  }

  Serial.print("Standard WAKE packets sent: ");
  Serial.println(wakeCount);
}

// ============================================================
// Send configuration wake
// ============================================================

void sendConfigurationWake() {

  Serial.println();
  Serial.println("==============================");
  Serial.println("CONFIGURATION WAKE");
  Serial.println("==============================");

  Serial.println("Configuration to send:");

  Serial.print("  Boots:      ");
  Serial.println(DEMO_CONFIG_BOOTS);

  Serial.print("  Sleep:      ");
  Serial.print(DEMO_CONFIG_SLEEP_SECONDS);
  Serial.println(" seconds");

  Serial.print("  Threshold:  ");
  Serial.println(CONFIG_PRESSURE_THRESHOLD);

  Serial.print("  Mode:       ");
  Serial.println(CONFIG_MODE);

  char configMessage[64];

  String configString =
    String(DEMO_CONFIG_BOOTS) + "," +
    String(DEMO_CONFIG_SLEEP_SECONDS) + "," +
    String(CONFIG_PRESSURE_THRESHOLD, 2) + "," +
    String(CONFIG_MODE);

  configString.toCharArray(
    configMessage,
    sizeof(configMessage)
  );

  Serial.print("Sending: ");
  Serial.println(configMessage);

  unsigned long startTime = millis();

  int configCount = 0;

  while (millis() - startTime < WAKE_DURATION_MS) {

    rf95.send(
        (uint8_t*)configMessage,
        strlen(configMessage)
    );

    rf95.waitPacketSent();

    configCount++;

    delay(WAKE_INTERVAL_MS);
  }

  Serial.print("Configuration packets sent: ");
  Serial.println(configCount);

  configModeActive = true;

  remainingConfiguredBursts = DEMO_CONFIG_BOOTS;

  configModeStartTime = millis();

  Serial.println();
  Serial.print("Entering configuration mode. Waiting for ");
  Serial.print(remainingConfiguredBursts);
  Serial.println(" configured bursts.");
}

// ============================================================
// Start a new diagnostic burst
// ============================================================

void startBurst(unsigned long startTime) {

  recordingBurst = true;

  packetsReceived = 0;

  missingPackets = 0;

  duplicatePackets = 0;

  highestPacket = -1;

  expectedPacket = 0;

  totalBytesReceived = 0;

  burstStartTime = startTime;

  lastPacketTime = startTime;

  Serial.println();
  Serial.println("=================================");
  Serial.println("NEW BURST DETECTED");
  Serial.println("=================================");
}

// ============================================================
// Create next available SD file
// ============================================================

void startNewFile() {

  fileNumber = 0;

  while (true) {

    filename =
        "file" +
        String(fileNumber) +
        ".txt";

    if (!SD.exists(filename.c_str())) {

      break;
    }

    fileNumber++;
  }

  Serial.println();
  Serial.print("Starting new burst file: ");
  Serial.println(filename);

  dataFile =
      SD.open(
          filename.c_str(),
          FILE_WRITE
      );

  if (!dataFile) {

    Serial.println(
        "ERROR: Could not create file!"
    );

    recordingFile = false;

    return;
  }

  recordingFile = true;

  Serial.println(
      "File opened successfully."
  );
}

// ============================================================
// Finish SD file
// ============================================================

void finishFile() {

  if (recordingFile) {

    dataFile.close();

    Serial.print("Finished writing ");

    Serial.println(filename);

    recordingFile = false;
  }

  else {

    Serial.println(
        "EOF received, but no file was open."
    );
  }
}

// ============================================================
// Finish burst and print diagnostics
// ============================================================

void finishBurst() {

  if (!recordingBurst) {

    Serial.println(
        "No active burst diagnostics."
    );

    return;
  }

  unsigned long duration =
      millis() - burstStartTime;

  Serial.println();
  Serial.println("========== BURST REPORT ==========");

  Serial.print("File:                   ");
  Serial.println(filename);

  Serial.print("Packets received:       ");
  Serial.println(packetsReceived);

  Serial.print("Highest packet number:  ");
  Serial.println(highestPacket);

  Serial.print("Packets expected:       ");
  Serial.println(highestPacket + 1);

  Serial.print("Missing packets:        ");
  Serial.println(missingPackets);

  Serial.print("Duplicate/out-of-order: ");
  Serial.println(duplicatePackets);

  Serial.print("Data bytes received:    ");
  Serial.println(totalBytesReceived);

  Serial.print("Burst duration:         ");
  Serial.print(duration);
  Serial.println(" ms");

  Serial.println("EOF received:           YES");

  Serial.println("==================================");

  recordingBurst = false;
}
