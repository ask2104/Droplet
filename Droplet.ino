// ---------- FirebaseClient build options (must be before the include) ----------
#define ENABLE_USER_AUTH       // use email/password auth
#define ENABLE_DATABASE        // enable Realtime Database

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FirebaseClient.h>

// ---------------- WiFi Credentials ----------------
const char* ssid     = "yaggi";
const char* password = "yaggi123";

// ---------------- Firebase Credentials ----------------
#define Web_API_KEY  "AIzaSyAqzpMhxTOIVjFwCVcy7h7m1IILL-dDH4c"
#define DATABASE_URL "https://aavishkar-8f0b9-default-rtdb.asia-southeast1.firebasedatabase.app"

// REPLACE THESE
#define USER_EMAIL   "esp32@gmail.com"
#define USER_PASS    "12345678"

// ---------------- Firebase Objects ----------------
void processData(AsyncResult &aResult);

UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

// ---------------- Web Server ----------------
WebServer server(80);

// ---------------- Pin Definitions ----------------
#define ONE_WIRE_BUS     25
#define TDS_PIN          35
#define DO_PIN           34
#define TURBIDITY_PIN    32

#define VREF     3.3f
#define ADC_RES  4095.0f
#define SCOUNT   30

// ---------------- DO Calibration ----------------
#define TWO_POINT_CALIBRATION 0
#define CAL1_V 1455
#define CAL1_T 25
#define CAL2_V 1300
#define CAL2_T 15

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int   analogBuffer[SCOUNT];
int   analogBufferTemp[SCOUNT];

float gTemperature   = 0;
float gTDS           = 0;
float gDO            = 0;
float gTurbidityNTU  = 0;

unsigned long lastFirebaseSend = 0;
const unsigned long firebaseInterval = 5000;

// ---------------------------------------------------------------------
// ✔ ENABLE LINEAR TURBIDITY CALIBRATION
// ---------------------------------------------------------------------
#define USE_LINEAR_TURBIDITY_MAP 1   // <--- ENABLED

// Linear calibration constants (Clear = 1 NTU, Milky = 35 NTU)
float LIN_M = -87.02f;      // slope
float LIN_C = 204.14f;      // intercept

#ifndef ADC_11db
#define ADC_11db 3
#endif

// ---------------- MEDIAN FILTER ----------------
int getMedianNum(int bArray[], int iFilterLen) {
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bArray[i] > bArray[i + 1]) {
        int bTemp   = bArray[i];
        bArray[i]   = bArray[i + 1];
        bArray[i+1] = bTemp;
      }
    }
  }
  if (iFilterLen & 1)
    return bArray[(iFilterLen - 1) / 2];
  else
    return (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 - 1]) / 2;
}

// ---------------- DO FUNCTION ----------------
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
  uint16_t V_saturation;
#if TWO_POINT_CALIBRATION == 0
  V_saturation = (uint32_t)CAL1_V + 35 * temperature_c - CAL1_T * 35;
#else
  V_saturation = ((temperature_c - CAL2_T) * (CAL1_V - CAL2_V)) /
                 (CAL1_T - CAL2_T) + CAL2_V;
#endif
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
}

// ---------------- READ SENSORS ----------------
void readSensors() {
  sensors.requestTemperatures();
  gTemperature = sensors.getTempCByIndex(0);
  uint8_t tempInt = constrain((int)gTemperature, 0, 40);

  // ---- TDS ----
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(TDS_PIN);
    delay(2);
  }
  for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];

  float avgVoltage  = getMedianNum(analogBufferTemp, SCOUNT) * VREF / ADC_RES;
  float compCoeff   = 1.0 + 0.02 * (gTemperature - 25.0);
  float compVoltage = avgVoltage / compCoeff;

  gTDS = (133.42 * compVoltage * compVoltage * compVoltage
      - 255.86 * compVoltage * compVoltage
      + 857.39 * compVoltage) * 0.5;

  // ---- DO ----
  float voltageDO = analogRead(DO_PIN) * VREF * 1000 / ADC_RES;
  gDO = readDO(voltageDO, tempInt) / 1000.0;

  // ---- TURBIDITY (LINEAR CALIBRATION) ----
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(TURBIDITY_PIN);
    delay(2);
  }
  for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];

  int turbidityMedianRaw = getMedianNum(analogBufferTemp, SCOUNT);
  float turbidityVoltage = turbidityMedianRaw * VREF / ADC_RES;

  float ntu = (LIN_M * turbidityVoltage) + LIN_C;

  if (ntu < 0) ntu = 0;
  if (ntu > 5000) ntu = 5000;

  gTurbidityNTU = ntu;

  Serial.print("Turbidity raw: ");
  Serial.print(turbidityMedianRaw);
  Serial.print("  V: ");
  Serial.print(turbidityVoltage, 4);
  Serial.print("  NTU: ");
  Serial.println(gTurbidityNTU, 3);
}

// ---------------- PROCESS FIREBASE DATA ----------------
void processData(AsyncResult &aResult) {
  if (!aResult.isResult()) return;

  if (aResult.isEvent()) {
    Firebase.printf("Event: %s, msg: %s, code: %d\n",
                    aResult.uid().c_str(),
                    aResult.eventLog().message().c_str(),
                    aResult.eventLog().code());
  }

  if (aResult.isDebug()) {
    Firebase.printf("Debug: %s, msg: %s\n",
                    aResult.uid().c_str(),
                    aResult.debug().c_str());
  }

  if (aResult.isError()) {
    Firebase.printf("Error task: %s, msg: %s, code: %d\n",
                    aResult.uid().c_str(),
                    aResult.error().message().c_str(),
                    aResult.error().code());
  }

  if (aResult.available()) {
    Firebase.printf("Task: %s, payload: %s\n",
                    aResult.uid().c_str(),
                    aResult.c_str());
  }
}

// ---------------- WEB ----------------
void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
    <html>
    <head>
      <title>Water Quality Monitor</title>
      <meta http-equiv="refresh" content="30" />
      <script>
        function fetchData() {
          fetch("/data")
            .then(response => response.json())
            .then(data => {
              document.getElementById("temp").innerHTML = data.temp + " °C";
              document.getElementById("tds").innerHTML  = data.tds + " ppm";
              document.getElementById("do").innerHTML   = data.do  + " mg/L";
              document.getElementById("ntu").innerHTML  = data.ntu + " NTU";
            });
        }
        setInterval(fetchData, 1000);
        window.onload = fetchData;
      </script>
    </head>
    <body>
      <h2>Water Quality Monitoring Dashboard</h2>
      <p><strong>Temperature:</strong> <span id="temp">--</span></p>
      <p><strong>TDS:</strong> <span id="tds">--</span></p>
      <p><strong>DO:</strong> <span id="do">--</span></p>
      <p><strong>Turbidity:</strong> <span id="ntu">--</span></p>
    </body>
    </html>
  )rawliteral");
}

void handleData() {
  readSensors();
  String json = "{";
  json += "\"temp\":" + String(gTemperature, 2) + ",";
  json += "\"tds\":"  + String(gTDS, 2) + ",";
  json += "\"do\":"   + String(gDO, 2) + ",";
  json += "\"ntu\":"  + String(gTurbidityNTU, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  sensors.begin();

#if defined(analogSetPinAttenuation)
  analogSetPinAttenuation(TURBIDITY_PIN, ADC_11db);
  analogSetPinAttenuation(TDS_PIN, ADC_11db);
  analogSetPinAttenuation(DO_PIN, ADC_11db);
#endif

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  ssl_client.setInsecure();
  ssl_client.setConnectionTimeout(1000);
  ssl_client.setHandshakeTimeout(5);

  Serial.println("Initializing Firebase...");

  initializeApp(aClient, app, getAuth(user_auth), processData, "authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  Serial.println("Firebase init done.");

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  app.loop();
  server.handleClient();

  if (app.ready()) {
    static bool printedReady = false;
    if (!printedReady) {
      Serial.println("Firebase app READY");
      printedReady = true;
    }

    unsigned long now = millis();
    if (now - lastFirebaseSend >= firebaseInterval) {
      lastFirebaseSend = now;

      readSensors();
      Serial.println("Sending data to Firebase...");

      Database.set<float>(aClient, "/waterQuality/temp",
          gTemperature, processData, "send_temp");
      Database.set<float>(aClient, "/waterQuality/tds",
          gTDS, processData, "send_tds");
      Database.set<float>(aClient, "/waterQuality/do",
          gDO, processData, "send_do");
      Database.set<float>(aClient, "/waterQuality/ntu",
          gTurbidityNTU, processData, "send_ntu");
    }
  }
}
