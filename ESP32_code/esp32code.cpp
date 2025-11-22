/**
 * @file esp32code.cpp
 * @brief ESP32 firmware for hotel room access control, power monitoring, and
 *        reliable event delivery to the backend over HTTPS.
 */

#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "time.h"
#include <FS.h>
#include <SPIFFS.h>

// ---- Pins and Config ----
#define RST_PIN 22
#define SS_PIN 21

const char* ssid = "Yahya";
const char* password = "@@@@@@@@";

// WebSocket / HTTP connection (matches your backend)
const char* websocketHost = "hotel-backend-5kcn.onrender.com"; // Render backend host
const int   websocketPort = 443; // HTTPS port for Render backend
const char* websocketPath = "/mqtt";

const char* roomNumber  = "101";
const char* building    = "main";
const char* floorNumber = "1"; // This maps to hotelId in your backend

// ---- NTP Config ----
const char* ntpServer1       = "pool.ntp.org";
const char* ntpServer2       = "time.nist.gov";
const char* ntpServer3       = "time.google.com";
const long  gmtOffset_sec    = 19800; // IST +5:30
const int   daylightOffset_sec = 0;
unsigned long lastSyncAttempt    = 0;
const unsigned long SYNC_INTERVAL = 3600000UL; // 1 hour
const int CURRENT_SENSOR_PIN = 34;
const unsigned long POWER_SAMPLE_INTERVAL = 15000;
unsigned long lastPowerSample = 0;
const float CURRENT_CALIBRATION = 30.0f;
const int RELAY_PIN = 25;
bool roomPowerOn = false;

// ---- RFID/WebSocket objects ----
MFRC522 mfrc522(SS_PIN, RST_PIN);
WebSocketsClient webSocket;

// ---- UIDs and Roles ----
/**
 * @brief Mapping between RFID card UIDs and logical user roles.
 */
struct UserAuth {
  byte uid[4];
  const char* role;
};

UserAuth users[] = {
  {{0xAF, 0x4D, 0x99, 0x1F}, "Maintenance"},
  {{0xBF, 0xD1, 0x07, 0x1F}, "Manager"},
  {{0xB2, 0xF9, 0x7C, 0x00}, "Guest"}
};
const int numUsers = sizeof(users) / sizeof(users[0]);

// ---- Presence Detection State ----
byte         presentCardUID[4] = {0};
int          presentUserIndex  = -1;
unsigned long checkedInTime    = 0;
bool         checkedIn         = false;
int          cardAbsentCount   = 0;
const int    CARD_ABSENT_THRESHOLD = 5;
bool         websocketConnected = false;

// ---- Offline Event Queue State ----
/**
 * @brief Simple queued event structure persisted in SPIFFS when offline.
 */
struct QueuedEvent {
  String type;
  String json;
};

const int   MAX_QUEUE_SIZE = 50;
QueuedEvent eventQueue[MAX_QUEUE_SIZE];
int         queueHead      = 0;
int         queueTail      = 0;
int         queueCount     = 0;
bool        storageReady   = false;
const char* QUEUE_FILE     = "/event_queue.txt";
const unsigned long QUEUE_PROCESS_INTERVAL = 5000; // ms between queue flush attempts
unsigned long       lastQueueProcess       = 0;
const char* SESSION_FILE  = "/active_session.json";

// ---- Function prototypes ----
/** @brief Initialize WiFi, time sync, and system banner. */
void setupSystem();
/** @brief Ensure the ESP32 is connected to WiFi. */
void connectWiFi();
/** @brief Synchronize RTC from NTP servers. */
bool syncTime();
/** @brief Get current timestamp as "YYYY-MM-DD HH:MM:SS". */
String getTimestamp();
/**
 * @brief Find index of user by RFID UID.
 * @param uid Pointer to UID bytes.
 * @param length UID length in bytes.
 * @return Index in users[] or -1 if not found.
 */
int  getUserIndex(byte *uid, byte length);
/** @brief Stop any active RFID communication gracefully. */
void cleanupRFID();
/** @brief Connect WebSocket client to backend MQTT-over-WebSocket endpoint. */
void connectWebSocket();
/**
 * @brief Publish an event to backend via HTTPS /api/mqtt-data.
 * @param type     Event type ("attendance", "power", "alerts", "denied_access").
 * @param jsonData JSON payload string.
 * @return true if HTTP POST succeeded (2xx), false otherwise.
 */
bool publishToMQTT(const char* type, const char* jsonData);
/** @brief Check if a card is currently present on the RFID reader. */
bool isCardPresent();
/** @brief WebSocket event handler for low-level MQTT WebSocket client. */
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
/** @brief Handle an unauthorized RFID access attempt. */
void handleUnauthorizedAccess(const char* cardUID);
/** @brief Initialize SPIFFS storage and load any queued events. */
void initStorage();
/** @brief Enqueue an event (type + JSON) into the offline queue. */
void enqueueEvent(const char* type, const char* jsonData);
/** @brief Process and flush queued events when WiFi/back-end is available. */
void processEventQueue();
/** @brief Measure AC current using RMS calculation over NUM_SAMPLES. */
float readCurrent();
/** @brief Control the room mains relay. */
void setRoomPower(bool on);

/**
 * @brief Arduino setup: initialize peripherals and system state.
 */
void setup() {
  Serial.begin(115200);
  delay(100);
  SPI.begin();
  mfrc522.PCD_Init();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  initStorage();
  
  setupSystem();
}

/**
 * @brief Main loop: maintain WiFi, flush queue, sample power, and handle
 *        continuous card presence logic for check-in / check-out.
 */
void loop() {
  // Maintain connections
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // Process any queued events when online (throttled to avoid log spam)
  unsigned long now = millis();
  if (now - lastQueueProcess >= QUEUE_PROCESS_INTERVAL) {
    lastQueueProcess = now;
    processEventQueue();
  }
  // Reuse now for other periodic work
  if (now - lastPowerSample >= POWER_SAMPLE_INTERVAL) {
    lastPowerSample = now;
    float current = readCurrent();
    DynamicJsonDocument powerDoc(256);
    powerDoc["current"] = current;
    powerDoc["timestamp"] = getTimestamp();
    String powerJson;
    serializeJson(powerDoc, powerJson);
    enqueueEvent("power", powerJson.c_str());
  }

  // Periodic time sync
  if (millis() - lastSyncAttempt > SYNC_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED && syncTime()) {
      lastSyncAttempt = millis();
    }
  }

  // ---- Continuous Card Presence Detection ----
  if (isCardPresent()) {
    cardAbsentCount = 0;

    if (!checkedIn) {
      int userIdx = getUserIndex(mfrc522.uid.uidByte, mfrc522.uid.size);
      
      char cardUID[9];
      snprintf(cardUID, sizeof(cardUID),
               "%02X%02X%02X%02X",
               mfrc522.uid.uidByte[0],
               mfrc522.uid.uidByte[1],
               mfrc522.uid.uidByte[2],
               mfrc522.uid.uidByte[3]);

      if (userIdx != -1) {
        String role = users[userIdx].role;
        String checkInTime = getTimestamp();
        time_t checkInEpoch = time(nullptr);
        
        DynamicJsonDocument doc(512);
        doc["card_uid"] = cardUID;
        doc["role"] = role;
        doc["check_in"] = checkInTime;
        doc["room"] = roomNumber;
        
        String jsonString;
        serializeJson(doc, jsonString);
        
        enqueueEvent("attendance", jsonString.c_str());
        Serial.printf("%s Checked IN at %s\n", role.c_str(), checkInTime.c_str());

        saveActiveSession(cardUID, role.c_str(), checkInTime.c_str(), checkInEpoch);

        setRoomPower(true);

        memcpy(presentCardUID, mfrc522.uid.uidByte, 4);
        presentUserIndex = userIdx;
        checkedInTime = millis();
        checkedIn = true;
      } else {
        // Unauthorized access attempt
        handleUnauthorizedAccess(cardUID);
      }
    }
  } else {
    cardAbsentCount++;

    if (checkedIn && cardAbsentCount >= CARD_ABSENT_THRESHOLD) {
      if (presentUserIndex != -1) {
        String role = users[presentUserIndex].role;
        unsigned long duration = (millis() - checkedInTime) / 1000;
        
        char cardUID[9];
        snprintf(cardUID, sizeof(cardUID),
                 "%02X%02X%02X%02X",
                 presentCardUID[0],
                 presentCardUID[1],
                 presentCardUID[2],
                 presentCardUID[3]);

        DynamicJsonDocument doc(512);
        String checkOutTime = getTimestamp();
        doc["card_uid"] = cardUID;
        doc["role"] = role;
        doc["check_out"] = checkOutTime;
        doc["duration"] = duration;
        doc["room"] = roomNumber;
        
        String jsonString;
        serializeJson(doc, jsonString);
        
        enqueueEvent("attendance", jsonString.c_str());
        Serial.printf("%s Checked OUT at %s (duration: %lu seconds)\n",
                      role.c_str(), checkOutTime.c_str(), duration);

        clearActiveSession();

        setRoomPower(false);
      }

      // Reset state
      checkedIn = false;
      presentUserIndex = -1;
      checkedInTime = 0;
      memset(presentCardUID, 0, sizeof(presentCardUID));
      cardAbsentCount = 0;
    }
  }

  delay(100);
}

/**
 * @brief Build and enqueue denied_access + alert events for unknown cards.
 * @param cardUID Hex string representation of the unknown card UID.
 */
void handleUnauthorizedAccess(const char* cardUID) {
  DynamicJsonDocument doc(512);
  doc["card_uid"] = cardUID;
  doc["role"] = "Unknown";
  doc["denial_reason"] = "Unauthorized card";
  doc["attempted_at"] = getTimestamp();
  doc["room"] = roomNumber;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  enqueueEvent("denied_access", jsonString.c_str());
  Serial.printf("DENIED ACCESS: Unknown card %s at %s\n", cardUID, getTimestamp().c_str());
  
  // Also send security alert
  DynamicJsonDocument alertDoc(512);
  alertDoc["card_uid"] = cardUID;
  alertDoc["role"] = "Security";
  alertDoc["alert_message"] = "Unauthorized access attempt detected";
  alertDoc["triggered_at"] = getTimestamp();
  alertDoc["room"] = roomNumber;
  
  String alertString;
  serializeJson(alertDoc, alertString);
  
  enqueueEvent("alerts", alertString.c_str());
}

/**
 * @brief Fast check for card presence and load UID into mfrc522.uid.
 * @return true if a card is present and UID was read, false otherwise.
 */
bool isCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);

  MFRC522::StatusCode result = mfrc522.PICC_REQA_or_WUPA(
    MFRC522::PICC_CMD_WUPA, bufferATQA, &bufferSize);

  if (result == MFRC522::STATUS_OK) {
    if (mfrc522.PICC_ReadCardSerial()) {
      return true;
    }
  }

  mfrc522.PICC_HaltA();
  return false;
}

/**
 * @brief Initialize WiFi, block until NTP time is synced, and print banner.
 */
void setupSystem() {
  Serial.println();
  connectWiFi();
  
  while (!syncTime()) {
    Serial.println("NTP sync failed, retrying in 5s...");
    delay(5000);
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
  }
  
  lastSyncAttempt = millis();
  Serial.println("\n====================");
  Serial.printf("Room %s Access Control System\n", roomNumber);
  Serial.printf("Hotel ID: %s\n", floorNumber);
  Serial.printf("Backend API: https://%s:%d/api/mqtt-data\n", websocketHost, websocketPort);
  Serial.println("====================\n");
  Serial.println("Ready to read cards...");
}

/**
 * @brief Connect to configured WiFi network with basic retry loop.
 */
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("WiFi Connection Failed");
  }
}

/**
 * @brief Synchronize local time from configured NTP servers.
 * @return true if time sync succeeded, false otherwise.
 */
bool syncTime() {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  configTime(gmtOffset_sec, daylightOffset_sec,
             ntpServer1, ntpServer2, ntpServer3);
  
  Serial.print("Syncing NTP time");
  time_t now = time(nullptr);
  int attempts = 0;
  
  while (now < 8 * 3600 * 2 && attempts < 20) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    attempts++;
  }
  
  Serial.println();
  if (attempts >= 20) return false;
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return false;
  
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.printf("Current Time: %s\n", buf);
  return true;
}

/**
 * @brief Return current local time as "YYYY-MM-DD HH:MM:SS".
 * @return Timestamp string, or epoch fallback on error.
 */
String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "1970-01-01 00:00:00";
  
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

/**
 * @brief Find user index in users[] array for a given 4-byte UID.
 * @param uid Pointer to UID bytes.
 * @param length Length of UID (must be 4).
 * @return Index in users[] or -1 if not found.
 */
int getUserIndex(byte *uid, byte length) {
  if (length != 4) return -1;
  
  for (int i = 0; i < numUsers; i++) {
    if (memcmp(uid, users[i].uid, 4) == 0) return i;
  }
  return -1;
}

/**
 * @brief Halt the current RFID card and stop cryptography.
 */
void cleanupRFID() {
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

/**
 * @brief Connect the WebSocket client to the backend MQTT WebSocket route.
 */
void connectWebSocket() {
  if (websocketConnected) return;
  
  Serial.printf("Connecting to WebSocket: ws://%s:%d%s\n", 
                websocketHost, websocketPort, websocketPath);
  
  // Use non-SSL WebSocket for local backend connection
  webSocket.begin(websocketHost, websocketPort, websocketPath);
  
  // Set reconnect interval
  webSocket.setReconnectInterval(5000);
  
  // Enable heartbeat
  webSocket.enableHeartbeat(15000, 3000, 2);
}

/**
 * @brief WebSocket event handler for MQTT WebSocket connection.
 */
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      websocketConnected = false;
      break;
      
    case WStype_CONNECTED:
      Serial.printf("WebSocket Connected to: %s\n", payload);
      websocketConnected = true;
      break;
      
    case WStype_TEXT:
      Serial.printf("Received: %s\n", payload);
      break;
      
    case WStype_ERROR:
      Serial.printf("WebSocket Error: %s\n", payload);
      websocketConnected = false;
      break;
      
    case WStype_PONG:
      Serial.println("WebSocket Pong received");
      break;
      
    default:
      break;
  }
}

/**
 * @brief Control the mains relay that powers the room.
 * @param on true to enable power, false to cut power.
 */
void setRoomPower(bool on) {
  roomPowerOn = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

/**
 * @brief Read AC current using RMS over multiple ADC samples.
 * @return Estimated current in amps.
 */
float readCurrent() {
  const int NUM_SAMPLES = 200;
  uint32_t sum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    int adc = analogRead(CURRENT_SENSOR_PIN);
    int centered = adc - 2048;  // Center around mid-scale (12-bit ADC)
    sum += (uint32_t)(centered * centered);
    delayMicroseconds(200);
  }

  float mean = sum / (float)NUM_SAMPLES;
  float rms = sqrtf(mean);
  float voltage_rms = (rms / 4095.0f) * 3.3f;
  float current = voltage_rms * CURRENT_CALIBRATION;
  return current;
}

// ---- Persistent Offline Queue Helpers ----

/**
 * @brief Serialize the in-memory queue to SPIFFS file.
 */
void saveQueueToFile() {
  if (!storageReady) return;

  File f = SPIFFS.open(QUEUE_FILE, "w");
  if (!f) {
    Serial.println("Failed to open queue file for writing");
    return;
  }

  int idx = queueHead;
  for (int i = 0; i < queueCount; i++) {
    const QueuedEvent &ev = eventQueue[idx];
    String line = ev.type + "|" + ev.json + "\n";
    f.print(line);
    idx = (idx + 1) % MAX_QUEUE_SIZE;
  }

  f.close();
}

/**
 * @brief Load queued events from SPIFFS file if present.
 */
void loadQueueFromFile() {
  if (!storageReady) return;

  File f = SPIFFS.open(QUEUE_FILE, "r");
  if (!f) {
    Serial.println("No existing queue file, starting fresh");
    return;
  }

  queueHead = 0;
  queueTail = 0;
  queueCount = 0;

  while (f.available() && queueCount < MAX_QUEUE_SIZE) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int sep = line.indexOf('|');
    if (sep == -1) continue;

    String type = line.substring(0, sep);
    String json = line.substring(sep + 1);

    eventQueue[queueTail].type = type;
    eventQueue[queueTail].json = json;
    queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
    queueCount++;
  }

  f.close();
  Serial.printf("Loaded %d pending events from storage\n", queueCount);
}

/**
 * @brief Initialize SPIFFS and load any pending events.
 */
void initStorage() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS, offline queue persistence disabled");
    storageReady = false;
    return;
  }

  storageReady = true;
  loadQueueFromFile();
  restoreActiveSession(); // Call restoreActiveSession after loading queue from file
}

/**
 * @brief Restore active session from SPIFFS file if present.
 */
void restoreActiveSession() {
  if (!storageReady) return;

  File f = SPIFFS.open(SESSION_FILE, "r");
  if (!f) {
    Serial.println("[SESSION] No existing session file, starting fresh");
    return;
  }

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, f);
  if (err) {
    Serial.print("[SESSION] Failed to parse JSON session file: ");
    Serial.println(err.c_str());
    f.close();
    return;
  }
  f.close();

  const char* cardUID = doc["card_uid"];
  const char* role = doc["role"];
  const char* checkInTime = doc["check_in"];
  time_t checkInEpoch = doc["check_in_epoch"];
  bool sessionActive = doc["sessionActive"];

  if (sessionActive) {
    Serial.println("[SESSION] Restoring active session");
    // Generate synthetic checkout event and queue it as a normal attendance checkout
    time_t nowEpoch = time(nullptr);
    if (checkInEpoch > 0 && nowEpoch > checkInEpoch) {
      unsigned long duration = (unsigned long)(nowEpoch - checkInEpoch);

      DynamicJsonDocument checkoutDoc(256);
      checkoutDoc["card_uid"] = cardUID;
      checkoutDoc["role"] = role;
      checkoutDoc["check_out"] = getTimestamp();
      checkoutDoc["duration"] = duration;
      checkoutDoc["room"] = roomNumber;

      String jsonData;
      serializeJson(checkoutDoc, jsonData);
      enqueueEvent("attendance", jsonData.c_str());
      Serial.printf("[SESSION] Queued synthetic checkout for %s (duration %lu seconds)\n", role, duration);
    }

    // Clear the session so we don't emit multiple synthetic checkouts
    clearActiveSession();
  }
}

/**
 * @brief Append a new event to the offline queue and persist it.
 * @param type Event type (e.g. "attendance").
 * @param jsonData Serialized JSON payload.
 */
void enqueueEvent(const char* type, const char* jsonData) {
  // If queue is full, drop the oldest event
  if (queueCount >= MAX_QUEUE_SIZE) {
    Serial.println("[QUEUE] Event queue full, dropping oldest event");
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    queueCount--;
  }

  Serial.printf("[QUEUE] Enqueue event type=%s (len=%d) currentCount=%d\n",
                type,
                (int)strlen(jsonData),
                queueCount);

  eventQueue[queueTail].type = String(type);
  eventQueue[queueTail].json = String(jsonData);
  queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
  queueCount++;

  saveQueueToFile();
}

void saveActiveSession(const char* cardUID, const char* role, const char* checkInTime, time_t checkInEpoch) {
  if (!storageReady) return;

  File f = SPIFFS.open(SESSION_FILE, "w");
  if (!f) {
    Serial.println("[SESSION] Failed to open session file for writing");
    return;
  }

  DynamicJsonDocument doc(256);
  doc["card_uid"] = cardUID;
  doc["role"] = role;
  doc["check_in"] = checkInTime;
  doc["check_in_epoch"] = (long)checkInEpoch;
  doc["room"] = roomNumber;
  doc["sessionActive"] = true;

  serializeJson(doc, f);
  f.close();
  Serial.println("[SESSION] Active session saved");
}

void clearActiveSession() {
  if (!storageReady) return;
  if (SPIFFS.exists(SESSION_FILE)) {
    SPIFFS.remove(SESSION_FILE);
    Serial.println("[SESSION] Active session cleared");
  }
}

/**
 * @brief Attempt to send all queued events to the backend when online.
 */
void processEventQueue() {
  if (queueCount == 0) return;

  if (WiFi.status() != WL_CONNECTED) {
    // We have events but still offline; keep them and try again later
    Serial.printf("[QUEUE] %d pending events but WiFi is not connected, will retry later\n",
                  queueCount);
    return;
  }

  Serial.printf("[QUEUE] Flushing %d queued events to backend...\n", queueCount);

  int attempts = queueCount;
  while (queueCount > 0 && attempts > 0) {
    QueuedEvent &ev = eventQueue[queueHead];
    Serial.printf("[QUEUE] Sending queued event type=%s\n", ev.type.c_str());

    bool success = publishToMQTT(ev.type.c_str(), ev.json.c_str());
    if (!success) {
      // Stop processing if we fail to publish; keep remaining events for retry
      Serial.println("[QUEUE] Failed to send queued event, will retry later");
      break;
    }

    Serial.println("[QUEUE] Queued event sent successfully, removing from queue");
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    queueCount--;
    attempts--;
  }

  saveQueueToFile();
}

/**
 * @brief Publish an event to the backend over HTTPS using HTTPClient.
 * @param type Event type string.
 * @param jsonData JSON body to embed under data{}.
 * @return true if HTTP response is 2xx, false otherwise.
 */
bool publishToMQTT(const char* type, const char* jsonData) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MQTT] Cannot publish: WiFi disconnected");
    return false;
  }

  // Create MQTT-style topic that matches backend expectation
  char topic[128];
  snprintf(topic, sizeof(topic),
           "campus/room/%s/%s/%s/%s",
           building, floorNumber, roomNumber, type);

  // Build HTTP body: { "topic": "...", "data": { ...jsonData... } }
  DynamicJsonDocument root(1024);
  root["topic"] = topic;

  DynamicJsonDocument dataDoc(512);
  DeserializationError err = deserializeJson(dataDoc, jsonData);
  if (err) {
    Serial.print("[MQTT] Failed to parse JSON payload for HTTP publish: ");
    Serial.println(err.c_str());
    return false;
  }
  root["data"] = dataDoc;

  String body;
  serializeJson(root, body);

  HTTPClient http;
  String url = String("https://") + websocketHost + ":" + String(websocketPort) + "/api/mqtt-data";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(body);
  if (httpCode > 0 && httpCode >= 200 && httpCode < 300) {
    Serial.printf("[MQTT] Published via HTTP to %s (status %d) topic=%s\n", url.c_str(), httpCode, topic);
    http.end();
    return true;
  } else {
    if (httpCode == -1) {
      Serial.printf("[MQTT] HTTP publish failed to %s (status -1: connection failed or backend offline)\n", url.c_str());
    } else {
      Serial.printf("[MQTT] HTTP publish failed to %s (status %d)\n", url.c_str(), httpCode);
    }
    http.end();
    return false;
  }
}

void onWiFiConnected() {
  Serial.println("[QUEUE] WiFi reconnected, retrying pending events");
  processEventQueue();
}