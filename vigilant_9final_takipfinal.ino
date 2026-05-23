/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  VIGILANT-X — Müstahkem Karakol Kontrol Sistemi v5          ║
 * ║  ESP32 Firmware — Tek Dosya (HTML gömülü)                   ║
 * ║                                                              ║
 * ║  Yenilikler v5:                                              ║
 * ║   + TAKİP MODU (servoMode=2): radar mesafesine göre servo   ║
 * ║   + 3 modlu taret: Manuel / Otonom / Takip                  ║
 * ║   + Vigilant AI takip modu komutlarını anlıyor              ║
 * ║   + NeoPixel: Takip modunda mor nabız                       ║
 * ║                                                              ║
 * ║  PIN HARİTASI                                               ║
 * ║  ─────────────────────────────────────────────────────────  ║
 * ║  TRIG N → GPIO 25   ECHO N → GPIO 26                       ║
 * ║  TRIG E → GPIO 27   ECHO E → GPIO 14                       ║
 * ║  TRIG S → GPIO 32   ECHO S → GPIO 33                       ║
 * ║  TRIG W → GPIO  4   ECHO W → GPIO 13                       ║
 * ║  SERVO N → GPIO 16  SERVO E → GPIO 17                      ║
 * ║  SERVO S → GPIO 18  SERVO W → GPIO  5                      ║
 * ║  LASER A → GPIO 19  LASER B → GPIO 21                      ║
 * ║  BUZZER  → GPIO 23  (ledcAttach, Core 3.x)                  ║
 * ║  VOL_PWM → GPIO 22  (ledcAttach → RC → PAM8403 gain)        ║
 * ║  RGB_R   → GPIO  2  (ledcAttach, kırmızı)                    ║
 * ║  RGB_G   → GPIO 15  (ledcAttach, yeşil)                      ║
 * ║  RGB_B   → GPIO  0  (ledcAttach, mavi)                       ║
 * ║  VIB 1   → GPIO 34  VIB 2 → GPIO 35  VIB 3 → GPIO 36     ║
 * ║  MIC     → GPIO 39  (ADC1 — WiFi güvenli)                  ║
 * ║                                                              ║
 * ║  NOT: TRIG/ECHO pinleri dijital — ADC2 sorunu YOK.         ║
 * ║  Analog okunan: 34,35,36,39 → hepsi ADC1.                  ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 *  PAM8403 SES DEVRESİ:
 *  GPIO 22 (PWM) → 10kΩ → RC düğüm → 10µF → PAM8403 giriş
 *                              ↕
 *                            10kΩ → GND
 *  (RC filtre: PWM'i DC'ye çevirir, böylece dijital volume kontrolü)
 *
 *  NEOPİXEL RENK KODLARI:
 *  Pasif (boşta)        → Mavi soluk nabız
 *  Manuel mod aktif     → Sarı sabit
 *  Otonom mod           → Yeşil sabit
 *  1 radar sinyal       → Turuncu nabız
 *  2+ radar sinyal      → Kırmızı hızlı nabız
 *  Kırmızı alarm        → Kırmızı strob
 *  Y1 taret aktif       → Mor soluk
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <esp_now.h>

// ═══════════════════════════════════════════════════════════
//  AĞ KONFİGÜRASYONU — Sıra: Ev WiFi → Hotspot → Sadece AP
// ═══════════════════════════════════════════════════════════
const char* WIFI_SSID_1 = "FiberHGW_HUXI0R";
const char* WIFI_PASS_1 = "4TCXcmgUKdHM";
const char* WIFI_SSID_2 = "yigit";
const char* WIFI_PASS_2 = "mwze8485";
const char* AP_SSID     = "VIGILANT-X";
const char* AP_PASSWORD = "karakol1234";
const char* MDNS_NAME   = "vigilant-x";



// HC-SR04 — 4 ayrı TRIG, 4 ayrı ECHO
#define US_N_TRIG  25
#define US_N_ECHO  26
#define US_E_TRIG  27
#define US_E_ECHO  14
#define US_S_TRIG  32
#define US_S_ECHO  33
#define US_W_TRIG   4
#define US_W_ECHO  13

// SW-420 Titreşim (ADC1 → WiFi güvenli)
#define VIB_1  39
#define VIB_2  35
#define VIB_3  36

// SG90 Servo
#define SERVO_N  16
#define SERVO_E  17
#define SERVO_S  18
#define SERVO_W   5
#define SERVO_MIN  5   // 10'dan 15'e → mekanik stres azalır
#define SERVO_MAX 175   // 170'den 165'e → mekanik stres azalır

// Lazer grupları
#define LASER_A  19    // N+E
#define LASER_B  21    // S+W

// Buzzer LRAD (LEDC kanal 0)
#define BUZZER_PIN  23
// LEDC_BUZZ kaldırıldı — Core 3.x: ledcAttach(pin) doğrudan pin ile çalışır

// PAM8403 ses seviyesi — PWM → RC filtre → amplifikatör gain girişi
// LEDC kanal 1, 1kHz, 8-bit → RC filtre ile düzgün DC'ye dönüşür
#define VOL_PWM_PIN  22
// LEDC_VOL kaldırıldı — Core 3.x: ledcAttach(pin) doğrudan pin ile çalışır
#define VOL_PWM_FREQ 1000   // Hz — RC filtresi için yeterince yüksek
#define VOL_DEFAULT  70     // %70 varsayılan ses

// Mikrofon (ADC1 → WiFi güvenli)
#define MIC_ADC  34

// 5050 RGB LED Modülü (Ortak Katot)
// 3 ayrı PWM kanalı — Adafruit_NeoPixel kütüphanesi GEREKMEZ
#define RGB_R_PIN   2    // Kırmızı → GPIO 2  (LEDC kanal 2) — onboard LED ile aynı, sorun yok
#define RGB_G_PIN  15    // Yeşil   → GPIO 15 (LEDC kanal 3)
#define RGB_B_PIN   0    // Mavi    → GPIO 0  (LEDC kanal 4) — boot'ta HIGH, PWM safe
// NOT: RGB_B GPIO0, US_W_TRIG GPIO4 — farklı pinler, çakışma yok
// GPIO 0 boot sırasında input olarak hareket eder ama output/PWM olarak kullanımı sorunsuz
#define RGB_FREQ    5000

// ═══════════════════════════════════════════════════════════
//  SABITLER
// ═══════════════════════════════════════════════════════════
#define MAX_DIST_CM        300
#define ALERT_DIST_CM      150
#define WARN_DIST_CM       250
#define SCAN_INTERVAL_MS   400
#define WS_BROADCAST_MS    250
#define WIFI_SCAN_INTERVAL 30000
#define AUTO_SCAN_SPEED    1.2f   // °/update @ 25Hz → ~30°/s — gözle görülür hareket
#define AUTO_LOCK_ANGLE    90
#define AUTO_LOCK_DIST     150
#define AUTO_UPDATE_MS     40     // ~25Hz

// ═══════════════════════════════════════════════════════════
//  SUNUCU NESNELERİ
// ═══════════════════════════════════════════════════════════
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ═══════════════════════════════════════════════════════════
//  SERVO NESNELERİ
// ═══════════════════════════════════════════════════════════
Servo servoN, servoE, servoS, servoW;

// ═══════════════════════════════════════════════════════════
//  GLOBAL DURUM
// ═══════════════════════════════════════════════════════════
int   servoAngles[4]   = {90, 90, 90, 90};
bool  laserStates[2]   = {false, false};
bool  redAlertMode     = false;
bool  lradActive       = false;
int   lradFreq         = 2000;
int   volPct           = VOL_DEFAULT;   // 0-100

float distCm[4]        = {300, 300, 300, 300};
bool  vibActive[3]     = {false, false, false};
int   micLevel         = 0;

bool  unitEnabled[4]   = {true, true, true, true};
bool  radarEnabled[4]  = {true, true, true, true};

int   servoMode        = 0;    // 0=manuel, 1=otonom, 2=takip

// ── Takip modu ────────────────────────────────────────────────
// Her taret kendi cephesindeki radarın mesafesine bakarak açısını ayarlar.
// Mesafe azaldıkça servo daha fazla döner (mesafe → açı eşleştirmesi).
// trackSmoothAngle: ani sıçramayı önlemek için yumuşatma filtresi.
float trackSmoothAngle[4] = {90, 90, 90, 90};
#define TRACK_SMOOTH    0.15f   // 0=hiç hareket etme, 1=anında — 0.15 yumuşak takip
#define TRACK_UPDATE_MS   50    // ~20Hz takip güncellemesi

float autoScanAngle[4] = {90, 90, 90, 90};
float autoScanDir[4]   = {1, 1, 1, 1};

unsigned long lastSensorRead  = 0;
unsigned long lastWsBroadcast = 0;
unsigned long lastWifiScan    = 0;
unsigned long lastAutoUpdate  = 0;
unsigned long lastNeoUpdate   = 0;

// ── Ses modu — aktifken radar/servo/broadcast duraksıyor ──────
bool          voiceMode       = false;
unsigned long lastVoiceSend   = 0;
#define VOICE_SAMPLE_COUNT  128
#define VOICE_SEND_INTERVAL  20

// ESP-NOW A3
struct A3NodeData { uint8_t nodeId; float distance; bool alert; int vibration; };
A3NodeData a3Buffer;
bool a3DataFresh = false;

// WiFi SIGINT
struct WifiNetwork { String ssid; int rssi; uint8_t channel; String encryption; };
WifiNetwork wifiNetworks[20];
int wifiNetCount = 0;

// Pin dizileri
const int trigPins[4]  = {US_N_TRIG, US_E_TRIG, US_S_TRIG, US_W_TRIG};
const int echoPins[4]  = {US_N_ECHO, US_E_ECHO, US_S_ECHO, US_W_ECHO};
const int laserPins[2] = {LASER_A, LASER_B};

// ═══════════════════════════════════════════════════════════
//  NEOPİXEL DURUM YÖNETİMİ
//
//  Öncelik sırası (yüksekten düşüğe):
//  1. Kırmızı alarm  → Kırmızı strob
//  2. 2+ radar sinyal → Kırmızı hızlı nabız
//  3. 1 radar sinyal  → Turuncu nabız
//  4. Otonom mod      → Yeşil sabit (taret aktifse yeşil nabız)
//  5. Manuel + taret  → Mor soluk
//  6. Manuel boşta    → Mavi yavaş nabız
// ═══════════════════════════════════════════════════════════
uint8_t neoBrightness = 0;
int8_t  neoDir        = 1;
bool    neoStrobState = false;

// ─── 5050 RGB LED yazma (0-255 her kanal) ───────────────────
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(RGB_R_PIN, r);
  ledcWrite(RGB_G_PIN, g);
  ledcWrite(RGB_B_PIN, b);
}

void updateNeoPixel() {
  unsigned long now = millis();
  // 30ms'de bir güncelle (~33fps)
  if (now - lastNeoUpdate < 30) return;
  lastNeoUpdate = now;

  // Kaç radar aktif sinyal alıyor?
  int radarHits = 0;
  for (int i = 0; i < 4; i++) {
    if (radarEnabled[i] && distCm[i] < ALERT_DIST_CM) radarHits++;
  }

  // Aktif taret var mı?
  bool anyTaret = false;
  for (int i = 0; i < 4; i++) { if (unitEnabled[i]) { anyTaret = true; break; } }

  uint8_t r = 0, g = 0, b = 0;

  if (redAlertMode) {
    // Strob: 100ms açık / 100ms kapalı
    neoStrobState = (now / 100) % 2;
    r = neoStrobState ? 255 : 0;
    g = 0; b = 0;

  } else if (radarHits >= 2) {
    // Hızlı kırmızı nabız (60ms periyot)
    neoBrightness += neoDir * 8;
    if (neoBrightness >= 250) { neoBrightness = 250; neoDir = -1; }
    if (neoBrightness <= 10)  { neoBrightness = 10;  neoDir =  1; }
    r = neoBrightness; g = 0; b = 0;

  } else if (radarHits == 1) {
    // Turuncu nabız
    neoBrightness += neoDir * 4;
    if (neoBrightness >= 220) { neoBrightness = 220; neoDir = -1; }
    if (neoBrightness <= 20)  { neoBrightness = 20;  neoDir =  1; }
    r = neoBrightness;
    g = (uint8_t)(neoBrightness * 0.45f);  // Turuncu = kırmızı + az yeşil
    b = 0;

  } else if (servoMode == 2) {
    // Takip modu — kırmızı + mavi = mor hızlı nabız
    neoBrightness += neoDir * 6;
    if (neoBrightness >= 220) { neoBrightness = 220; neoDir = -1; }
    if (neoBrightness <= 20)  { neoBrightness = 20;  neoDir =  1; }
    r = (uint8_t)(neoBrightness * 0.7f);
    g = 0;
    b = neoBrightness;

  } else if (servoMode == 1) {
    // Otonom mod — yeşil (taret aktifse nabız, değilse sabit)
    if (anyTaret) {
      neoBrightness += neoDir * 3;
      if (neoBrightness >= 200) { neoBrightness = 200; neoDir = -1; }
      if (neoBrightness <= 40)  { neoBrightness = 40;  neoDir =  1; }
      g = neoBrightness;
    } else {
      g = 120;  // Sabit orta yeşil
    }
    r = 0; b = 0;

  } else if (anyTaret) {
    // Manuel + taret aktif → mor soluk nabız
    neoBrightness += neoDir * 2;
    if (neoBrightness >= 180) { neoBrightness = 180; neoDir = -1; }
    if (neoBrightness <= 20)  { neoBrightness = 20;  neoDir =  1; }
    r = (uint8_t)(neoBrightness * 0.6f);
    g = 0;
    b = neoBrightness;

  } else {
    // Pasif boşta → mavi yavaş nabız
    neoBrightness += neoDir * 2;
    if (neoBrightness >= 160) { neoBrightness = 160; neoDir = -1; }
    if (neoBrightness <= 10)  { neoBrightness = 10;  neoDir =  1; }
    r = 0; g = 0; b = neoBrightness;
  }

  setRGB(r, g, b);
}

// ═══════════════════════════════════════════════════════════
//  SES SEVİYESİ (PAM8403)
//  PWM duty → RC filtre → DC voltaj → PAM8403 gain
//  %0 = sessiz, %100 = tam ses
// ═══════════════════════════════════════════════════════════
void setVolume(int pct) {
  volPct = constrain(pct, 0, 100);
  uint32_t duty = (uint32_t)(volPct * 255 / 100);
  ledcWrite(VOL_PWM_PIN, duty);
}

// ═══════════════════════════════════════════════════════════
//  HC-SR04 — Mesafe ölçümü
//  Her sensörün kendi trig/echo pini var.
//  Sensörler AYNI ANDA tetiklenmez — sıralı ölçüm yapılır.
//  Sensörler arası bekleme: 60ms (HC-SR04 datasheet minimum 60ms)
//  pulseIn timeout: 20ms → ~340cm max
// ═══════════════════════════════════════════════════════════
// Radar: Arduino'daki gibi sade, millis ile periyot kontrolü
// Her sensör sırayla, aralarında 65ms bekleme — crosstalk önleme
// pulseIn çalışırken loop kısa süre bloke olur (~12ms max) — kabul edilebilir
int  currentSensor    = 0;
unsigned long lastRadarMs = 0;
#define RADAR_PERIOD_MS 65   // Sensörler arası minimum bekleme

void radarTick() {
  if (millis() - lastRadarMs < RADAR_PERIOD_MS) return;
  lastRadarMs = millis();

  int i = currentSensor;
  currentSensor = (i + 1) % 4;

  if (!radarEnabled[i]) { distCm[i] = MAX_DIST_CM; return; }

  // Trigger — Arduino'daki birebir aynısı
  digitalWrite(trigPins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);

  // Echo
  long dur = pulseIn(echoPins[i], HIGH, 17500); // 300cm max
  float raw = dur * 0.0343f / 2.0f;
  distCm[i] = (dur == 0) ? MAX_DIST_CM : (raw > MAX_DIST_CM ? MAX_DIST_CM : raw);
  Serial.printf("[RADAR] idx=%d TRIG=%d ECHO=%d dur=%ld dist=%.1f\n", i, trigPins[i], echoPins[i], dur, distCm[i]);
}


// ═══════════════════════════════════════════════════════════
//  SW-420 Titreşim
// ═══════════════════════════════════════════════════════════
void readVibrations() {
  vibActive[0] = (digitalRead(VIB_1) == LOW);
  vibActive[1] = (digitalRead(VIB_2) == LOW);
  vibActive[2] = (digitalRead(VIB_3) == LOW);
}

// ═══════════════════════════════════════════════════════════
//  SERVO
// ═══════════════════════════════════════════════════════════
void setServoAngle(int index, int angle) {
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  servoAngles[index] = angle;
  switch (index) {
    case 0: servoN.write(angle); break;
    case 1: servoE.write(angle); break;
    case 2: servoS.write(angle); break;
    case 3: servoW.write(angle); break;
  }
}

// ═══════════════════════════════════════════════════════════
//  LAZER
// ═══════════════════════════════════════════════════════════
void setLaser(int laserIdx, bool state) {
  if (laserIdx < 0 || laserIdx > 1) return;
  laserStates[laserIdx] = state;
  digitalWrite(laserPins[laserIdx], state ? HIGH : LOW);
}

int taretToLaser(int taretIdx) { return (taretIdx <= 1) ? 0 : 1; }

void setAllLasers(bool state) { setLaser(0, state); setLaser(1, state); }

// ═══════════════════════════════════════════════════════════
//  BUZZER LRAD
// ═══════════════════════════════════════════════════════════
void setLrad(bool active, int freq = 2000) {
  lradActive = active;
  lradFreq   = freq;
  ledcWriteTone(BUZZER_PIN, active ? freq : 0);
}

void triggerAlarm(int beeps = 2) {
  for (int i = 0; i < beeps; i++) {
    ledcWriteTone(BUZZER_PIN, 1200); delay(120);
    ledcWriteTone(BUZZER_PIN, 0);    delay(80);
  }
}

// ═══════════════════════════════════════════════════════════
//  A2: WiFi SIGINT
// ═══════════════════════════════════════════════════════════
void scanWifiNetworks() {
  int n = WiFi.scanNetworks(false, true);
  if (n < 0) n = 0;
  wifiNetCount = min(n, 20);
  for (int i = 0; i < wifiNetCount; i++) {
    wifiNetworks[i].ssid       = WiFi.SSID(i);
    wifiNetworks[i].rssi       = WiFi.RSSI(i);
    wifiNetworks[i].channel    = WiFi.channel(i);
    wifiNetworks[i].encryption = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "SECURED";
  }
  WiFi.scanDelete();
}

// ═══════════════════════════════════════════════════════════
//  ESP-NOW A3
// ═══════════════════════════════════════════════════════════
void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == sizeof(A3NodeData)) {
    memcpy(&a3Buffer, data, sizeof(A3NodeData));
    a3DataFresh = true;
  }
}

// ═══════════════════════════════════════════════════════════
//  JSON DURUM PAKETİ
// ═══════════════════════════════════════════════════════════
String buildStatusJson() {
  StaticJsonDocument<2048> doc;
  doc["type"]       = "status";
  doc["uptime"]     = millis() / 1000;
  doc["redAlert"]   = redAlertMode;
  doc["heap"]       = ESP.getFreeHeap();
  doc["servo_mode"] = servoMode;  // 0=manuel, 1=otonom, 2=takip
  doc["lrad"]       = lradActive;
  doc["mic"]        = micLevel;
  doc["volume"]     = volPct;

  JsonObject a1 = doc.createNestedObject("a1");
  const char* dirs[4] = {"N","E","S","W"};
  for (int i = 0; i < 4; i++) a1[dirs[i]] = distCm[i];

  JsonArray vib = doc.createNestedArray("a5_vib");
  for (int i = 0; i < 3; i++) vib.add(vibActive[i]);

  JsonArray srvArr = doc.createNestedArray("servos");
  for (int i = 0; i < 4; i++) srvArr.add(servoAngles[i]);

  JsonArray lzArr = doc.createNestedArray("lasers");
  lzArr.add(laserStates[0]);
  lzArr.add(laserStates[1]);

  JsonArray uEn = doc.createNestedArray("unit_enabled");
  for (int i = 0; i < 4; i++) uEn.add(unitEnabled[i]);

  JsonArray rEn = doc.createNestedArray("radar_enabled");
  for (int i = 0; i < 4; i++) rEn.add(radarEnabled[i]);

  JsonArray nets = doc.createNestedArray("wifi_nets");
  for (int i = 0; i < wifiNetCount; i++) {
    JsonObject net = nets.createNestedObject();
    net["ssid"] = wifiNetworks[i].ssid;
    net["rssi"] = wifiNetworks[i].rssi;
    net["ch"]   = wifiNetworks[i].channel;
    net["enc"]  = wifiNetworks[i].encryption;
  }

  if (a3DataFresh) {
    JsonObject a3 = doc.createNestedObject("a3");
    a3["id"]    = a3Buffer.nodeId;
    a3["dist"]  = a3Buffer.distance;
    a3["alert"] = a3Buffer.alert;
    a3["vib"]   = a3Buffer.vibration;
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ═══════════════════════════════════════════════════════════
//  WEBSOCKET KOMUT İŞLEME
// ═══════════════════════════════════════════════════════════
void handleWsCommand(const String& raw) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, raw)) return;
  const char* cmd = doc["cmd"];
  if (!cmd) return;
  String command = String(cmd);

  if (command == "servo") {
    int id    = doc["id"]    | -1;
    int angle = doc["angle"] | 90;
    if (id >= 0 && id < 4 && servoMode == 0 && unitEnabled[id])
      setServoAngle(id, angle);
  }
  else if (command == "laser") {
    int  id = doc["id"]    | -1;
    bool st = doc["state"] | false;
    if (id >= 0 && id <= 1) setLaser(id, st);
  }
  else if (command == "laser_all") {
    setAllLasers(doc["state"] | false);
  }
  else if (command == "lrad") {
    setLrad(doc["state"] | false, doc["freq"] | 2000);
  }
  else if (command == "set_volume") {
    setVolume(doc["pct"] | VOL_DEFAULT);
  }
  else if (command == "red_alert") {
    redAlertMode = doc["state"] | false;
    if (redAlertMode) triggerAlarm(2);
    else              setLrad(false);
  }
  else if (command == "servo_reset") {
    for (int i = 0; i < 4; i++) setServoAngle(i, 90);
  }
  else if (command == "wifi_scan") {
    scanWifiNetworks();
    String _js=buildStatusJson(); wsServer.broadcastTXT(_js);
  }
  else if (command == "unit_enable") {
    String utype = doc["unit_type"] | "";
    int    id    = doc["id"]        | -1;
    bool   st    = doc["state"]     | false;
    if (id >= 0 && id < 4) {
      if (utype == "servo") {
        unitEnabled[id] = st;
        if (!st) {
          setServoAngle(id, 90);
          setLaser(taretToLaser(id), false);
        }
      } else if (utype == "radar") {
        radarEnabled[id] = st;
        if (!st) distCm[id] = MAX_DIST_CM;
        else     distCm[id] = MAX_DIST_CM; // Aktif edilince önce max ile başlat
        Serial.printf("[RADAR] %s id=%d state=%d pin TRIG=%d ECHO=%d\n",
          utype.c_str(), id, st, trigPins[id], echoPins[id]);
      }
    }
  }
  else if (command == "set_mode") {
    int newMode = doc["mode"] | 0;
    servoMode = constrain(newMode, 0, 2);
    if (servoMode == 1) {
      // Mevcut servo açısından başla, çift indeksler sağa tek indeksler sola tarar
      for (int i = 0; i < 4; i++) {
        autoScanAngle[i] = (float)servoAngles[i];
        autoScanDir[i]   = (i % 2 == 0) ? 1 : -1;
      }
      Serial.println("[MODE] Otonom mod aktif — tarama başlıyor");
    } else if (servoMode == 2) {
      // Takip modu: yumuşatma filtresi mevcut açıdan başlasın
      for (int i = 0; i < 4; i++) {
        trackSmoothAngle[i] = (float)servoAngles[i];
      }
      Serial.println("[MODE] Takip modu aktif — radar mesafesine göre takip");
    } else {
      Serial.println("[MODE] Manuel mod aktif");
    }
  }
  else if (command == "ping") {
    wsServer.broadcastTXT("{\"type\":\"pong\"}");
  }
  else if (command == "set_rgb") {
    // Manuel RGB override — NeoPixel otomatik yazimi durdur
    int r = doc["r"] | 0;
    int g = doc["g"] | 0;
    int b = doc["b"] | 0;
    setRGB((uint8_t)r, (uint8_t)g, (uint8_t)b);
    Serial.printf("[RGB] Manuel set: R=%d G=%d B=%d\n", r, g, b);
  }
  else if (command == "servo_angle") {
    // LLM icin ayri servo komutu — mod kontrolu yok, direkt calisir
    int id    = doc["id"]    | -1;
    int angle = doc["angle"] | 90;
    if (id >= 0 && id < 4) {
      setServoAngle(id, constrain(angle, SERVO_MIN, SERVO_MAX));
      Serial.printf("[SERVO] LLM komutu: id=%d angle=%d\n", id, angle);
    }
  }
  // ── Ses modu komutlari ─────────────────────────────────────
  else if (command == "voice_start") {
    voiceMode = true;
    Serial.println("[VOICE] Ses modu aktif — radar/servo/broadcast duraklatildi");
  }
  else if (command == "voice_stop") {
    voiceMode = false;
    Serial.println("[VOICE] Ses modu kapandi — sistemler devam ediyor");
  }
  else if (command == "voice_cmd") {
    JsonObject action = doc["action"];
    if (!action.isNull()) {
      String actionStr;
      serializeJson(action, actionStr);
      Serial.printf("[VOICE] Fiziksel komut: %s\n", actionStr.c_str());
      handleWsCommand(actionStr);
    }
    const char* reply = doc["reply"] | "";
    if (strlen(reply) > 0) {
      Serial.printf("[VOICE] VGL: %s\n", reply);
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  WEBSOCKET OLAYLARI
// ═══════════════════════════════════════════════════════════
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] #%u koptu\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = wsServer.remoteIP(num);
      Serial.printf("[WS] #%u bağlandı: %s\n", num, ip.toString().c_str());
      String _js=buildStatusJson(); wsServer.sendTXT(num, _js);
      break;
    }
    case WStype_TEXT:
      handleWsCommand(String((char*)payload));
      break;
    default: break;
  }
}

// ═══════════════════════════════════════════════════════════
//  HTML ARAYÜZÜ — PROGMEM
// ═══════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"HTMLDELIM(
<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VIGILANT-X</title>
<link href="https://fonts.googleapis.com/css2?family=Bebas+Neue&family=Share+Tech+Mono&family=Rajdhani:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
:root{
  --bg:#030508;--dp:#06090e;--surf:#090d14;--panel:#0c1019;
  --b1:#121c28;--b2:#1a2838;
  --green:#00ff88;--glow:rgba(0,255,136,.10);
  --amber:#ffb700;--red:#ff2244;--blue:#00aaff;
  --text:#7a9ab8;--texth:#ddeeff;--textd:#2e4560;--scan:rgba(0,255,136,.012);
}
*{margin:0;padding:0;box-sizing:border-box;}
html,body{height:100%;overflow:hidden;background:var(--bg);color:var(--text);font-family:'Rajdhani',sans-serif;cursor:crosshair;}
body::after{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 2px,var(--scan) 2px,var(--scan) 3px);pointer-events:none;z-index:9999;}
.root{display:flex;flex-direction:column;height:100vh;}

/* TOPBAR */
.topbar{height:40px;background:var(--dp);border-bottom:1px solid var(--b1);display:flex;align-items:stretch;flex-shrink:0;z-index:200;}
.tb-logo{display:flex;align-items:center;gap:8px;padding:0 14px;border-right:1px solid var(--b1);font-family:'Bebas Neue',sans-serif;font-size:21px;letter-spacing:4px;color:var(--texth);}
.tb-logo em{color:var(--green);font-style:normal;}
.tb-dia{width:14px;height:14px;border:1px solid var(--green);transform:rotate(45deg);display:flex;align-items:center;justify-content:center;}
.tb-dia-dot{width:5px;height:5px;background:var(--green);}
.tb-nav{display:flex;align-items:stretch;}
.tbn{display:flex;align-items:center;gap:5px;padding:0 13px;border-right:1px solid var(--b1);font-family:'Share Tech Mono',monospace;font-size:14px;letter-spacing:1.5px;color:var(--textd);cursor:pointer;transition:.15s;position:relative;}
.tbn:hover{color:var(--text);}
.tbn.active{color:var(--green);background:var(--glow);}
.tbn.active::after{content:'';position:absolute;bottom:0;left:0;right:0;height:2px;background:var(--green);}
.tbn .dot{width:5px;height:5px;border-radius:50%;flex-shrink:0;}
.tbn .dot.g{background:var(--green);box-shadow:0 0 4px var(--green);}
.tbn .dot.a{background:var(--amber);animation:pulse .9s infinite;}
.tbn .dot.r{background:var(--red);animation:pulse .5s infinite;}
.tbn .dot.x{background:var(--b2);}
.tb-right{margin-left:auto;display:flex;align-items:center;gap:12px;padding:0 14px;}
.tb-clock{font-family:'Share Tech Mono',monospace;font-size:18px;color:var(--green);}
.tb-alert-badge{background:rgba(255,34,68,.15);border:1px solid rgba(255,34,68,.4);color:var(--red);padding:2px 10px;font-family:'Share Tech Mono',monospace;font-size:13px;letter-spacing:2px;display:none;cursor:pointer;}
.tb-alert-badge.show{display:block;animation:pulse .6s infinite;}
.tb-mode{font-family:'Share Tech Mono',monospace;font-size:13px;letter-spacing:2px;padding:2px 9px;border:1px solid;cursor:pointer;}
.tb-mode.manuel{color:var(--blue);border-color:rgba(0,170,255,.3);}
.tb-mode.oto{color:var(--amber);border-color:rgba(255,183,0,.4);background:rgba(255,183,0,.04);}
.tb-mode.takip{color:var(--red);border-color:rgba(255,34,68,.4);background:rgba(255,34,68,.04);animation:pulse .8s infinite;}
.tb-conn{display:flex;align-items:center;gap:6px;}
.conn-dot{width:7px;height:7px;border-radius:50%;}
.conn-dot.ok{background:var(--green);box-shadow:0 0 6px var(--green);}
.conn-dot.err{background:var(--red);animation:pulse .5s infinite;}
.conn-dot.wait{background:var(--amber);}
#conn-lbl{font-family:'Share Tech Mono',monospace;font-size:13px;color:var(--textd);}

/* MAIN */
.main{flex:1;display:flex;overflow:hidden;position:relative;}
.sidebar{width:188px;flex-shrink:0;background:var(--dp);border-right:1px solid var(--b1);display:flex;flex-direction:column;overflow-y:auto;}
.sb-cat{font-family:'Share Tech Mono',monospace;font-size:14px;letter-spacing:2px;color:var(--textd);padding:5px 12px 4px;border-bottom:1px solid var(--b1);background:var(--bg);}
.sb-item{display:flex;align-items:center;gap:8px;padding:7px 12px;cursor:pointer;border-left:2px solid transparent;transition:.12s;}
.sb-item:hover{background:rgba(255,255,255,.02);}
.sb-item.active{background:var(--glow);border-left-color:var(--green);}
.sb-id{font-family:'Bebas Neue',sans-serif;font-size:20px;letter-spacing:1px;color:var(--textd);min-width:26px;}
.sb-item.active .sb-id{color:var(--green);}
.sb-name{font-size:18px;color:var(--text);font-weight:600;}
.sb-st{font-family:'Share Tech Mono',monospace;font-size:14px;margin-top:1px;}
.sb-st.g{color:var(--green)}.sb-st.a{color:var(--amber)}.sb-st.r{color:var(--red)}.sb-st.x{color:var(--textd)}
.sb-led{width:6px;height:6px;border-radius:50%;flex-shrink:0;}
.led-g{background:var(--green);box-shadow:0 0 4px var(--green);}
.led-a{background:var(--amber);animation:pulse 1s infinite;}
.led-r{background:var(--red);animation:pulse .5s infinite;}
.led-x{background:var(--b2);}

/* CONTENT */
.content{flex:1;display:flex;flex-direction:column;overflow:hidden;min-width:0;}
.ph{background:var(--dp);border-bottom:1px solid var(--b1);padding:0 16px;height:44px;display:flex;align-items:center;gap:14px;flex-shrink:0;}
.ph-id{font-family:'Bebas Neue',sans-serif;font-size:32px;letter-spacing:2px;line-height:1;}
.ph-id.g{color:var(--green)}.ph-id.a{color:var(--amber)}.ph-id.r{color:var(--red)}.ph-id.b{color:var(--blue)}
.ph-name{font-family:'Bebas Neue',sans-serif;font-size:18px;letter-spacing:2px;color:var(--texth);}
.ph-cat{font-family:'Share Tech Mono',monospace;font-size:15px;letter-spacing:1.5px;color:var(--textd);}
.ph-right{margin-left:auto;display:flex;gap:8px;align-items:center;}
.pill{font-family:'Share Tech Mono',monospace;font-size:14px;letter-spacing:1.5px;padding:3px 8px;border:1px solid;}
.pill.g{color:var(--green);border-color:rgba(0,255,136,.3);background:rgba(0,255,136,.04);}
.pill.a{color:var(--amber);border-color:rgba(255,183,0,.3);background:rgba(255,183,0,.04);}
.pill.r{color:var(--red);border-color:rgba(255,34,68,.3);background:rgba(255,34,68,.04);}
.pill.x{color:var(--textd);border-color:var(--b2);}

/* TAB BAR */
.tab-bar{display:flex;background:var(--surf);border-bottom:1px solid var(--b1);flex-shrink:0;}
.tab{padding:8px 15px;font-family:'Share Tech Mono',monospace;font-size:15px;letter-spacing:1.5px;color:var(--textd);cursor:pointer;border-bottom:2px solid transparent;transition:.12s;}
.tab:hover{color:var(--text);}
.tab.active{color:var(--green);border-bottom-color:var(--green);}
.panel-body{flex:1;overflow-y:auto;}
.tc{display:none;}.tc.active{display:block;}

/* GRID */
.g2{display:grid;grid-template-columns:1fr 1fr;gap:2px;background:var(--b1);}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:2px;background:var(--b1);}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:2px;background:var(--b1);}

/* CARD */
.card{background:var(--surf);padding:12px 14px;}
.ct{font-family:'Share Tech Mono',monospace;font-size:15px;letter-spacing:2px;color:var(--textd);margin-bottom:6px;}
.cv{font-family:'Bebas Neue',sans-serif;font-size:40px;letter-spacing:1px;line-height:1;}
.cv.g{color:var(--green)}.cv.a{color:var(--amber)}.cv.r{color:var(--red)}.cv.b{color:var(--blue)}
.cs{font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);margin-top:3px;}

/* BUTTONS */
.btn{font-family:'Share Tech Mono',monospace;font-size:14px;letter-spacing:1px;padding:5px 12px;border:1px solid;cursor:pointer;background:transparent;transition:.15s;}
.btn.g{color:var(--green);border-color:rgba(0,255,136,.3);}
.btn.g:hover{background:rgba(0,255,136,.08);}
.btn.r{color:var(--red);border-color:rgba(255,34,68,.3);}
.btn.r:hover,.btn.r.on{background:rgba(255,34,68,.1);}
.btn.a{color:var(--amber);border-color:rgba(255,183,0,.3);}
.btn.a:hover{background:rgba(255,183,0,.08);}
.btn.b{color:var(--blue);border-color:rgba(0,170,255,.3);}
.btn.x{color:var(--textd);border-color:var(--b2);}
.btn.x:hover{color:var(--text);}
.btn-row{display:flex;gap:4px;flex-wrap:wrap;}

/* BADGE / PILL */
.badge{font-family:'Share Tech Mono',monospace;font-size:13px;letter-spacing:1px;padding:2px 7px;border:1px solid;}
.badge.on{color:var(--red);border-color:rgba(255,34,68,.35);background:rgba(255,34,68,.06);}
.badge.off{color:var(--textd);border-color:var(--b1);}
.badge.ok{color:var(--green);border-color:rgba(0,255,136,.3);}

/* BARS */
.bw{margin:4px 0;}
.bl{font-family:'Share Tech Mono',monospace;font-size:15px;color:var(--textd);margin-bottom:3px;display:flex;justify-content:space-between;}
.bt{height:3px;background:var(--b1);}
.bf{height:100%;transition:width .4s;}

/* ROWS */
.row{display:flex;align-items:center;gap:6px;margin:5px 0;flex-wrap:wrap;}
.sep{height:1px;background:var(--b1);margin:8px 0;}
.pad{padding:12px 14px;}
.pads{padding:8px 12px;}
.tog-row{display:flex;align-items:center;justify-content:space-between;padding:6px 0;border-bottom:1px solid var(--b1);}
.tog-lbl{font-family:'Share Tech Mono',monospace;font-size:16px;color:var(--text);}
.tog{position:relative;width:36px;height:18px;cursor:pointer;}
.tog input{opacity:0;width:0;height:0;}
.tog-tr{position:absolute;inset:0;background:var(--b2);transition:.25s;}
.tog-th{position:absolute;width:12px;height:12px;top:3px;left:3px;background:var(--textd);transition:.25s;}
.tog input:checked+.tog-tr{background:rgba(0,255,136,.25);}
.tog input:checked~.tog-th{left:21px;background:var(--green);box-shadow:0 0 6px var(--green);}

/* RANGE */
.sl-row{margin:7px 0;}
.sl-lbl{font-family:'Share Tech Mono',monospace;font-size:13px;color:var(--textd);margin-bottom:3px;display:flex;justify-content:space-between;}
input[type=range]{width:100%;height:2px;background:var(--b2);appearance:none;cursor:pointer;outline:none;}
input[type=range]::-webkit-slider-thumb{appearance:none;width:12px;height:12px;background:var(--green);box-shadow:0 0 4px var(--green);}

/* LOG STRIP */
.logstrip{background:var(--bg);border-top:1px solid var(--b1);height:32px;flex-shrink:0;display:flex;align-items:center;padding:0 12px;gap:10px;overflow:hidden;}
.log-lbl{font-family:'Share Tech Mono',monospace;font-size:13px;letter-spacing:2px;color:var(--textd);flex-shrink:0;padding-right:10px;border-right:1px solid var(--b1);}
.log-msg{font-family:'Share Tech Mono',monospace;font-size:15px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.log-msg.g{color:var(--green)}.log-msg.a{color:var(--amber)}.log-msg.r{color:var(--red)}.log-msg.b{color:var(--blue)}.log-msg.x{color:var(--textd)}

/* ── FÜZYON KARTLARI ── */
.fc{background:var(--surf);border:1px solid var(--b1);margin:4px 0;padding:10px 13px;}
.fc.crit{border-color:rgba(255,34,68,.4);border-left:3px solid var(--red);}
.fc.high{border-color:rgba(255,183,0,.3);border-left:3px solid var(--amber);}
.fc.ok{border-left:3px solid var(--b2);}
.fc-head{display:flex;align-items:flex-start;justify-content:space-between;gap:8px;margin-bottom:8px;}
.fc-id-row{font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);}
.fc-desc{font-family:'Bebas Neue',sans-serif;font-size:20px;letter-spacing:1px;color:var(--texth);}
.fc-lvl{font-family:'Share Tech Mono',monospace;font-size:13px;padding:2px 7px;border:1px solid;display:inline-block;}
.fc-lvl.r{color:var(--red);border-color:rgba(255,34,68,.35);}
.fc-lvl.a{color:var(--amber);border-color:rgba(255,183,0,.35);}
.fc-lvl.g{color:var(--green);border-color:rgba(0,255,136,.35);}
.fc-body{display:flex;gap:6px;align-items:center;margin-bottom:8px;flex-wrap:wrap;}
/* Sensör dot'u — Kalekol Sim'deki gibi */
.fs-dot{width:38px;height:38px;border:1px solid;display:flex;flex-direction:column;align-items:center;justify-content:center;position:relative;}
.fs-dot.on{border-color:var(--green);color:var(--green);}
.fs-dot.mid{border-color:var(--amber);color:var(--amber);}
.fs-dot.off{border-color:var(--b2);color:var(--textd);}
.fs-dot-id{font-family:'Bebas Neue',sans-serif;font-size:15px;line-height:1;}
.fs-dot-pct{font-family:'Share Tech Mono',monospace;font-size:10px;position:absolute;bottom:1px;right:2px;}
.fc-fus{flex:1;min-width:120px;}
.fc-fus-bar{height:5px;background:var(--b1);margin-bottom:3px;}
.fc-fus-fill{height:100%;transition:width .5s;}
.fc-fus-val{font-family:'Bebas Neue',sans-serif;font-size:22px;}
.fc-fus-lbl{font-family:'Share Tech Mono',monospace;font-size:13px;color:var(--textd);}
.fc-eta{text-align:right;min-width:60px;}
.fc-eta-v{font-family:'Bebas Neue',sans-serif;font-size:28px;}
.fc-eta-l{font-family:'Share Tech Mono',monospace;font-size:11px;color:var(--textd);}
.fc-acts{display:flex;gap:4px;flex-wrap:wrap;margin-top:6px;}

/* ── RADAR SWEEP PANEL ── */
#mm-wrap{position:absolute;top:8px;right:8px;z-index:150;user-select:none;min-width:200px;min-height:180px;}
#mm-inner{background:var(--dp);border:2px solid var(--b2);box-shadow:0 4px 20px rgba(0,0,0,.5);display:flex;flex-direction:column;height:100%;}
#mm-hdr{font-family:'Share Tech Mono',monospace;font-size:12px;letter-spacing:1.5px;color:var(--green);padding:4px 8px;border-bottom:1px solid var(--b1);display:flex;align-items:center;justify-content:space-between;gap:6px;cursor:move;background:var(--panel);flex-shrink:0;}
#mm-canvas{display:block;flex:1;min-height:0;width:100%;}
#mm-ctrl{display:flex;gap:3px;padding:3px 6px;border-top:1px solid var(--b1);background:var(--panel);align-items:center;flex-shrink:0;}
#mm-ctrl button{font-family:'Share Tech Mono',monospace;font-size:8px;background:rgba(0,255,136,.08);border:1px solid rgba(0,255,136,.2);color:var(--green);padding:2px 6px;cursor:pointer;}
#mm-ctrl button:hover{background:rgba(0,255,136,.15);}
#mm-ctrl .mm-lbl{font-family:'Share Tech Mono',monospace;font-size:8px;color:var(--textd);margin-left:auto;}
#mm-resize{position:absolute;bottom:0;right:0;width:14px;height:14px;cursor:nwse-resize;z-index:11;}
#mm-resize::after{content:'';position:absolute;bottom:2px;right:2px;width:0;height:0;border-left:6px solid transparent;border-bottom:6px solid var(--b2);}

/* ── TABLO ── */
.dt{width:100%;border-collapse:collapse;font-family:'Share Tech Mono',monospace;font-size:12px;}
.dt th{color:var(--textd);letter-spacing:1.5px;padding:6px 12px;font-size:14px;text-align:left;border-bottom:1px solid var(--b1);background:var(--panel);}
.dt td{padding:6px 12px;border-bottom:1px solid var(--b1);color:var(--text);font-size:16px;}
.dt tr:hover td{background:rgba(255,255,255,.012);}
.dt td.g{color:var(--green)}.dt td.a{color:var(--amber)}.dt td.r{color:var(--red)}

/* MODAL */
.modal{position:fixed;inset:0;background:rgba(2,4,8,.94);z-index:5000;display:flex;align-items:center;justify-content:center;}
.mbox{background:var(--dp);border:1px solid var(--b2);padding:28px;width:400px;}
.m-title{font-family:'Bebas Neue',sans-serif;font-size:28px;letter-spacing:5px;color:var(--texth);margin-bottom:4px;}
.m-sub{font-family:'Share Tech Mono',monospace;font-size:13px;letter-spacing:3px;color:var(--textd);margin-bottom:14px;}
.m-note{font-family:'Share Tech Mono',monospace;font-size:13px;color:var(--textd);padding:8px;background:var(--surf);border-left:2px solid var(--green);margin-bottom:14px;line-height:1.6;}
.m-lbl{font-family:'Share Tech Mono',monospace;font-size:13px;letter-spacing:1px;color:var(--textd);display:block;margin-bottom:4px;}
.m-inp{width:100%;background:var(--bg);border:1px solid var(--b2);color:var(--texth);font-family:'Share Tech Mono',monospace;font-size:14px;padding:7px 10px;outline:none;margin-bottom:10px;}
.m-inp:focus{border-color:var(--green);}
.m-btns{display:flex;gap:8px;margin-top:12px;}

/* DEBUG */
#dbg{position:fixed;bottom:6px;right:6px;width:280px;max-height:140px;overflow-y:auto;background:rgba(2,4,8,.97);border:1px solid rgba(0,255,136,.12);padding:5px 8px;font-family:'Share Tech Mono',monospace;font-size:8px;color:var(--green);z-index:9999;display:none;}
#dbg.on{display:block;}
#dbg-btn{position:fixed;bottom:36px;right:6px;font-family:'Share Tech Mono',monospace;font-size:8px;padding:2px 8px;background:var(--panel);border:1px solid var(--b1);color:var(--textd);cursor:pointer;z-index:9999;}

/* JOYSTİCK */
.joy-pad{position:relative;height:38px;background:var(--panel);border:1px solid var(--b2);cursor:grab;user-select:none;touch-action:none;margin:6px 0;}
.joy-pad.on{border-color:var(--amber);}
.joy-stick{position:absolute;top:50%;left:50%;width:16px;height:16px;border-radius:50%;background:var(--amber);box-shadow:0 0 8px var(--amber);transform:translate(-50%,-50%);pointer-events:none;}
.joy-pad::after{content:'◄ SÜRÜKLE ►';position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);font-family:'Share Tech Mono',monospace;font-size:11px;color:rgba(255,183,0,.2);letter-spacing:3px;pointer-events:none;}

@keyframes pulse{0%,100%{opacity:1}50%{opacity:.2}}
::-webkit-scrollbar{width:3px;}::-webkit-scrollbar-thumb{background:var(--b2);}
</style>
</head>
<body>
<div class="root">

<!-- TOPBAR -->
<div class="topbar">
  <div class="tb-logo"><div class="tb-dia"><div class="tb-dia-dot"></div></div>VIGILANT<em>-X</em></div>
  <div class="tb-nav" id="tb-nav">
    <div class="tbn active" id="nav-FUZYON" onclick="goPage('FUZYON')"><span class="dot g"></span>FÜZYON</div>
    <div class="tbn" id="nav-KONTROL" onclick="goPage('KONTROL')"><span class="dot x"></span>KONTROL</div>
    <div class="tbn" id="nav-SISTEM" onclick="goPage('SISTEM')"><span class="dot x"></span>SİSTEM</div>
  </div>
  <div class="tb-right">
    <div class="tb-conn">
      <div class="conn-dot wait" id="conn-dot"></div>
      <div id="conn-lbl">BAĞLANTI YOK</div>
    </div>
    <div class="tb-mode manuel" id="mode-btn" onclick="toggleMode()">MANUEL</div>
    <div class="tb-alert-badge" id="alarm-badge" onclick="toggleRedAlert()">⚠ KIRMIZI ALARM</div>
    <button class="btn r" style="font-size:14px;padding:4px 12px;" onclick="toggleRedAlert()">ALARM</button>
    <div class="tb-clock" id="tb-clock">00:00:00</div>
  </div>
</div>

<div class="main">
  <!-- SIDEBAR -->
  <div class="sidebar" id="sidebar">
    <div class="sb-cat">A1 — RADAR</div>
    <div class="sb-item active" id="sb-FUZYON" onclick="goPage('FUZYON')">
      <div class="sb-id">FZ</div>
      <div><div class="sb-name">Füzyon</div><div class="sb-st g" id="sb-fz-st">Nominal</div></div>
      <div class="sb-led led-g" style="margin-left:auto;"></div>
    </div>
    <div class="sb-cat">Y1 — TARET</div>
    <div class="sb-item" id="sb-KONTROL" onclick="goPage('KONTROL')">
      <div class="sb-id">YK</div>
      <div><div class="sb-name">Kontrol</div><div class="sb-st x" id="sb-yp-st">Manuel</div></div>
      <div class="sb-led led-x" id="sb-yp-led" style="margin-left:auto;"></div>
    </div>
    <div class="sb-cat">E — ELEKTRONİK HARP</div>
    <div class="sb-item" id="sb-SISTEM" onclick="goPage('SISTEM')">
      <div class="sb-id">EH</div>
      <div><div class="sb-name">EW + SİGINT</div><div class="sb-st x">Pasif</div></div>
      <div class="sb-led led-x" style="margin-left:auto;"></div>
    </div>
    <div class="sb-cat">A5 — TİTREŞİM</div>
    <div style="padding:8px 12px;display:flex;gap:8px;align-items:center;">
      <div id="vib-0" style="width:8px;height:8px;border-radius:50%;background:var(--b2);transition:.2s;"></div>
      <div id="vib-1" style="width:8px;height:8px;border-radius:50%;background:var(--b2);transition:.2s;"></div>
      <div id="vib-2" style="width:8px;height:8px;border-radius:50%;background:var(--b2);transition:.2s;"></div>
      <span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);">VIB1 VIB2 VIB3</span>
    </div>
    <div class="sb-cat">SİSTEM</div>
    <div style="padding:8px 12px;display:flex;flex-direction:column;gap:4px;">
      <div style="display:flex;justify-content:space-between;"><span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);">HEAP</span><span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--green);" id="s-heap">--</span></div>
      <div style="display:flex;justify-content:space-between;"><span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);">UPTIME</span><span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--text);" id="s-uptime">--</span></div>
      <div style="display:flex;justify-content:space-between;"><span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);">MİK</span><span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--text);" id="s-mic">0</span></div>
    </div>
  </div>

  <!-- CONTENT -->
  <div class="content" id="content-area">

    <!-- FÜZYON SAYFASI -->
    <div id="page-FUZYON" style="display:flex;flex-direction:column;height:100%;overflow:hidden;">
      <div class="ph">
        <div class="ph-id g">FZ</div>
        <div>
          <div class="ph-name">FÜZYON ANALİZ MERKEZİ</div>
          <div class="ph-cat">A1 RADAR · A5 VİBRASYON · A3 DIŞ DÜĞÜM</div>
        </div>
        <div class="ph-right">
          <span class="pill g" id="fz-status-pill">NOMİNAL</span>
          <span class="pill x" id="fz-cnt-pill">0 KAYIT</span>
        </div>
      </div>
      <!-- Radar durum satırı -->
      <div class="g4" style="flex-shrink:0;">
        <div class="card" id="rc-N">
          <div class="ct">KUZEY / N</div>
          <div class="cv g" id="rd-N">---</div>
          <div class="cs" id="rs-N">TEMİZ</div>
          <div class="bw"><div class="bt"><div class="bf" id="rb-N" style="width:0%;background:var(--green);"></div></div></div>
        </div>
        <div class="card" id="rc-E">
          <div class="ct">DOĞU / E</div>
          <div class="cv g" id="rd-E">---</div>
          <div class="cs" id="rs-E">TEMİZ</div>
          <div class="bw"><div class="bt"><div class="bf" id="rb-E" style="width:0%;background:var(--green);"></div></div></div>
        </div>
        <div class="card" id="rc-S">
          <div class="ct">GÜNEY / S</div>
          <div class="cv g" id="rd-S">---</div>
          <div class="cs" id="rs-S">TEMİZ</div>
          <div class="bw"><div class="bt"><div class="bf" id="rb-S" style="width:0%;background:var(--green);"></div></div></div>
        </div>
        <div class="card" id="rc-W">
          <div class="ct">BATI / W</div>
          <div class="cv g" id="rd-W">---</div>
          <div class="cs" id="rs-W">TEMİZ</div>
          <div class="bw"><div class="bt"><div class="bf" id="rb-W" style="width:0%;background:var(--green);"></div></div></div>
        </div>
      </div>
      <!-- Füzyon kartları alanı -->
      <div class="panel-body" id="fuz-cards" style="padding:8px 10px;">
        <div style="font-family:'Share Tech Mono',monospace;font-size:15px;color:var(--textd);text-align:center;padding:40px;">
          Füzyon verisi bekleniyor — Sensörler dinleniyor...
        </div>
      </div>
    </div>

    <!-- KONTROL SAYFASI -->
    <div id="page-KONTROL" style="display:none;flex-direction:column;height:100%;overflow:hidden;">
      <div class="ph">
        <div class="ph-id a">YK</div>
        <div>
          <div class="ph-name">TARET KONTROL</div>
          <div class="ph-cat">Y1 SG90 · LAZER · LRAD</div>
        </div>
        <div class="ph-right">
          <span class="pill x" id="mode-pill">MANUEL</span>
        </div>
      </div>
      <div style="flex:1;display:flex;overflow:hidden;">
        <!-- Taret kartları -->
        <div style="flex:1;overflow-y:auto;padding:8px 10px;" id="taret-cards"></div>
        <!-- Joystick + EW sağ panel -->
        <div style="width:300px;border-left:1px solid var(--b1);background:var(--dp);overflow-y:auto;flex-shrink:0;">
          <div style="padding:8px 12px;border-bottom:1px solid var(--b1);">
            <div class="ct">JOYSTİCK X</div>
            <div style="display:flex;gap:4px;flex-wrap:wrap;margin-bottom:6px;" id="taret-sel-row"></div>
            <div id="joy-pad-wrap" style="display:none;">
              <div style="font-family:'Share Tech Mono',monospace;font-size:16px;color:var(--amber);margin-bottom:4px;" id="joy-lbl">HEDEF SEÇ</div>
              <div class="joy-pad" id="joy-pad"><div class="joy-stick" id="joy-stick"></div></div>
              <div style="display:flex;align-items:center;gap:6px;">
                <span style="font-family:'Share Tech Mono',monospace;font-size:9px;color:var(--textd);">0°</span>
                <input type="range" id="joy-slider" min="0" max="180" value="90" oninput="joySliderInput(this.value)" style="flex:1;">
                <span style="font-family:'Share Tech Mono',monospace;font-size:9px;color:var(--textd);">180°</span>
                <span style="font-family:'Bebas Neue',sans-serif;font-size:18px;color:var(--amber);min-width:36px;" id="joy-ang">90°</span>
              </div>
            </div>
          </div>
          <div style="padding:8px 12px;border-bottom:1px solid var(--b1);">
            <div class="ct">E — ELEKTRONİK HARP</div>
            <div class="tog-row">
              <span class="tog-lbl">LRAD BUZZER</span>
              <div style="display:flex;gap:6px;align-items:center;">
                <span class="badge off" id="badge-lrad">KAPALI</span>
                <button class="btn a" style="font-size:14px;padding:5px 10px;" id="btn-lrad" onclick="toggleLrad()">AÇ</button>
              </div>
            </div>
            <div class="sl-row">
              <div class="sl-lbl"><span>FREKANS</span><span id="lrad-fv">2000Hz</span></div>
              <input type="range" min="500" max="4000" step="100" value="2000" id="lrad-freq" oninput="document.getElementById('lrad-fv').textContent=this.value+'Hz';if(lradState)sendCmd({cmd:'lrad',state:true,freq:parseInt(this.value)})">
            </div>
            <div class="sep"></div>
            <div class="tog-row">
              <span class="tog-lbl">LAZER A (K+D)</span>
              <div style="display:flex;gap:5px;align-items:center;">
                <span class="badge off" id="badge-lzA">KAPALI</span>
                <button class="btn r" style="font-size:14px;padding:5px 10px;" id="btn-lzA" onclick="toggleLaserGroup(0)">AÇ</button>
              </div>
            </div>
            <div class="tog-row" style="border:none;">
              <span class="tog-lbl">LAZER B (G+B)</span>
              <div style="display:flex;gap:5px;align-items:center;">
                <span class="badge off" id="badge-lzB">KAPALI</span>
                <button class="btn r" style="font-size:14px;padding:5px 10px;" id="btn-lzB" onclick="toggleLaserGroup(1)">AÇ</button>
              </div>
            </div>
            <div class="btn-row" style="margin-top:6px;">
              <button class="btn r" onclick="fireAllLasers(true)">⚡ TÜM LAZER</button>
              <button class="btn x" onclick="fireAllLasers(false)">KAPAT</button>
            </div>
          </div>
          <div style="padding:8px 12px;" id="voice-panel">
            <div class="ct">I — VIGILANT SES &amp; CHAT</div>
            <!-- API Key -->
            <div style="display:flex;gap:4px;margin-bottom:5px;">
              <input id="vgl-apikey" type="password" placeholder="Groq API Key" style="flex:1;background:var(--bg);border:1px solid var(--b2);color:var(--text);font-family:'Share Tech Mono',monospace;font-size:16px;padding:4px 7px;outline:none;">
              <button class="btn g" style="font-size:16px;padding:5px 9px;" onclick="saveVglKey()">OK</button>
            </div>
            <!-- Kayit butonu -->
            <button id="vgl-recbtn" class="btn b" style="width:100%;font-size:18px;padding:10px 0;letter-spacing:2px;margin-bottom:5px;" onclick="vglToggleRec()">&#9679; KAYIT BASLAT</button>
            <!-- Durum -->
            <div style="font-family:'Share Tech Mono',monospace;font-size:15px;color:var(--textd);margin-bottom:4px;" id="vgl-status">API key gir, kayda basla.</div>
            <!-- Dalga canvas -->
            <canvas id="vgl-canvas" height="36" style="width:100%;display:block;background:var(--bg);border:1px solid var(--b1);margin-bottom:5px;"></canvas>
            <!-- Birlesik Chat Log (ses + text) -->
            <div style="border-top:1px solid var(--b1);padding-top:5px;max-height:200px;overflow-y:auto;" id="vgl-log"></div>
            <!-- Onay alani — log'un DISINDA, her zaman gorunur -->
            <div id="vgl-confirm-area" style="display:none;margin-top:4px;padding:5px 8px;background:rgba(186,117,23,0.08);border:1px solid rgba(186,117,23,0.3);border-radius:3px;">
              <div id="vgl-confirm-cmd" style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--amber);margin-bottom:4px;word-break:break-all;"></div>
              <div style="display:flex;gap:6px;">
                <button onclick="vglApply()" style="flex:1;font-family:'Share Tech Mono',monospace;font-size:16px;padding:6px;background:rgba(0,255,136,.15);border:1px solid rgba(0,255,136,.4);color:var(--green);cursor:pointer;border-radius:2px;">✓ UYGULA</button>
                <button onclick="vglCancel()" style="flex:1;font-family:'Share Tech Mono',monospace;font-size:16px;padding:6px;background:transparent;border:1px solid var(--b2);color:var(--textd);cursor:pointer;border-radius:2px;">✗ İPTAL</button>
              </div>
            </div>
            <!-- Text girisi -->
            <div style="display:flex;gap:4px;margin-top:5px;border-top:1px solid var(--b1);padding-top:5px;">
              <textarea id="vgl-txt" rows="3" placeholder="Vigilant'a yaz..." style="flex:1;resize:none;background:var(--bg);border:1px solid var(--b2);color:var(--text);font-family:'Share Tech Mono',monospace;font-size:17px;padding:6px 9px;outline:none;line-height:1.4;"></textarea>
              <button id="vgl-sendbtn" class="btn g" style="font-size:17px;padding:6px 12px;align-self:flex-end;" onclick="vglSendText()">&#9654;</button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SİSTEM / EW+SİGINT SAYFASI -->
    <div id="page-SISTEM" style="display:none;flex-direction:column;height:100%;overflow:hidden;">
      <div class="ph">
        <div class="ph-id b">EH</div>
        <div>
          <div class="ph-name">EW + SİGINT</div>
          <div class="ph-cat">A2 WiFi SIGINT · E Katmanı</div>
        </div>
        <div class="ph-right">
          <button class="btn g" style="font-size:14px;padding:4px 10px;" onclick="sendCmd({cmd:'wifi_scan'})" >TARA</button>
          <span class="pill x" id="sig-cnt"></span>
        </div>
      </div>
      <div style="flex:1;overflow-y:auto;" id="sig-list"></div>
    </div>

  </div><!-- /content -->

  <!-- RADAR SWEEP MİNİMAP (sürüklenebilir) -->
  <div id="mm-wrap" style="width:220px;height:210px;">
    <div id="mm-inner">
      <div id="mm-hdr">
        <span>A1 RADAR SWEEP</span>
        <button onclick="mmFull()" style="font-family:'Share Tech Mono',monospace;font-size:7px;background:rgba(0,255,136,.1);border:1px solid rgba(0,255,136,.2);color:var(--green);padding:1px 5px;cursor:pointer;">⛶</button>
      </div>
      <canvas id="mm-canvas" width="200" height="160"></canvas>
      <div id="mm-ctrl">
        <button onclick="mmZ(1.3)">+</button>
        <button onclick="mmZ(0.77)">−</button>
        <button onclick="mmReset()">↺</button>
        <span class="mm-lbl" id="mm-scl">300m max</span>
      </div>
      <div id="mm-resize" title="Boyutlandır"></div>
    </div>
  </div>

</div><!-- /main -->

<!-- LOG STRIP -->
<div class="logstrip">
  <div class="log-lbl">LOG</div>
  <div class="log-msg x" id="log-strip-msg">Sistem başlatıldı.</div>
</div>

</div><!-- /root -->

<!-- MODAL -->
<div class="modal" id="modal">
  <div class="mbox">
    <div class="m-title">KARAKOL BAĞLANTISI</div>
    <div class="m-sub">VIGILANT-X KONTROL MERKEZİ</div>
    <div class="m-note">Sayfa ESP32'den yüklendiyse otomatik bağlanır. Farklı adres için aşağıyı düzenleyin.</div>
    <label class="m-lbl">WEBSOCKET ADRESİ (PORT 81)</label>
    <input class="m-inp" id="inp-ws" placeholder="otomatik">
    <label class="m-lbl">DOST KARAKOL — OPSİYONEL</label>
    <input class="m-inp" id="inp-ally" placeholder="Boş → devre dışı">
    <div class="m-btns">
      <button class="btn g" onclick="doConnect()">▶ BAĞLAN</button>
      <button class="btn b" onclick="doConnectBoth()">⇄ DOST KARAKOL</button>
    </div>
  </div>
</div>

<button id="dbg-btn" onclick="document.getElementById('dbg').classList.toggle('on')">DBG</button>
<div id="dbg"></div>

<script>
// ═══ SABİTLER ═══
const DIRS   = ['N','E','S','W'];
const DIR_TR = {N:'KUZEY',E:'DOĞU',S:'GÜNEY',W:'BATI'};
const ALERT_CM = 100, WARN_CM = 200, MAX_CM = 300;

// ═══ DURUM ═══
let ws = null, wsAlly = null, wsUrl = '';
let curPage = 'FUZYON';
let servoMode = 0, redAlertMode = false, lradState = false;
let selectedTaret = -1, sttRecog = null, sttActive = false;
const lazerGroupState = [false, false];

const sensor = {dist:{N:300,E:300,S:300,W:300},servos:[90,90,90,90],lasers:[false,false],vib:[false,false,false],mic:0};

// ═══ FÜZYON ═══
const fstate = {};
DIRS.forEach(d => fstate[d] = {
  below: false, since: 0, lastRpt: 0,
  confirmed: false, dist: 300, totalHits: 0, readings: 0
});
const vibBuf = [[],[],[]];
const CONFIRM_MS = 2000, COOLDOWN_MS = 5000;
let fuzEntries = []; // {id, dir, dist, score, src, lvl, ts, confirmed}
let fuzIdCounter = 0;

function fusionUpdate(dir, dist) {
  const fs = fstate[dir];
  const now = Date.now();
  fs.readings++;
  fs.dist = dist;

  if (dist < WARN_CM) {
    fs.totalHits++;
    if (!fs.below) { fs.below = true; fs.since = now; }
    const elapsed = now - fs.since;

    if (elapsed >= CONFIRM_MS && now - fs.lastRpt > COOLDOWN_MS) {
      fs.lastRpt = now;
      fs.confirmed = true;

      // Sensör katkıları
      const a1pct = Math.round((1 - dist/MAX_CM)*100);
      const vibHit = vibBuf.some(b => b.length > 0 && b.reduce((a,c)=>a+c,0)/b.length > 0.3);
      const a5pct  = vibHit ? Math.round(Math.random()*30+20) : 0;
      const score  = Math.round(a1pct*.7 + a5pct*.3);
      const lvl    = dist < ALERT_CM ? 'r' : 'a';
      const m      = (dist/100).toFixed(0);
      const type   = dist < ALERT_CM ? 'KRİTİK MESAFE' : dist < 150 ? 'YAKLAŞAN CİSİM' : 'HAREKET ALGILANDI';

      const entry = {
        id: 'TGT-' + String(++fuzIdCounter).padStart(3,'0'),
        dir, dist, m, score, a1pct, a5pct, lvl, type,
        ts: new Date().toLocaleTimeString('tr',{hour12:false}),
        confirmed: true
      };
      fuzEntries.unshift(entry);
      if (fuzEntries.length > 20) fuzEntries.pop();
      renderFuzCards();
      sysLog(`[ FÜZYON ] ${entry.id} — A1 ${DIR_TR[dir]} ${m}M · ONAYLI %${score}`, lvl);
    }
  } else {
    fs.below = false; fs.since = 0;
    if (fs.confirmed) { fs.confirmed = false; }
  }

  updateRadarCards();
}

function vibFusion(idx, active) {
  const buf = vibBuf[idx];
  buf.push(active ? 1 : 0);
  if (buf.length > 20) buf.shift();
  if (buf.length === 20) {
    const hits = buf.reduce((a,b)=>a+b,0);
    buf.length = 0;
    if (hits >= 14) {
      const entry = {
        id: 'VIB-' + String(++fuzIdCounter).padStart(3,'0'),
        dir: null, dist: null, m: null, score: Math.round(hits/20*100),
        a1pct: 0, a5pct: Math.round(hits/20*100), lvl: 'r',
        type: 'YÜKSEK FREKANS TİTREŞİM',
        desc: 'A5 VIB'+(idx+1)+' — ARAÇ / AĞIR YÜKLÜ EKLER OLABİLİR',
        ts: new Date().toLocaleTimeString('tr',{hour12:false}),
        confirmed: true
      };
      fuzEntries.unshift(entry);
      if (fuzEntries.length > 20) fuzEntries.pop();
      renderFuzCards();
      sysLog('[ A5 ] VIB'+(idx+1)+' YÜKSEK FREKANS — ARAÇ OLABİLİR', 'r');
    } else if (hits >= 8) {
      const entry = {
        id: 'VIB-' + String(++fuzIdCounter).padStart(3,'0'),
        dir: null, dist: null, m: null, score: Math.round(hits/20*100),
        a1pct: 0, a5pct: Math.round(hits/20*100), lvl: 'a',
        type: 'ORTA FREKANS TİTREŞİM',
        desc: 'A5 VIB'+(idx+1)+' — YAYA VEYA HAFİF ARAÇ OLABİLİR',
        ts: new Date().toLocaleTimeString('tr',{hour12:false}),
        confirmed: true
      };
      fuzEntries.unshift(entry);
      if (fuzEntries.length > 20) fuzEntries.pop();
      renderFuzCards();
      sysLog('[ A5 ] VIB'+(idx+1)+' ORTA FREKANS — YAYA OLABİLİR', 'a');
    }
  }
}

// ─── Füzyon kart render — Kalekol Sim tarzı ───────────────────
function renderFuzCards() {
  const el = document.getElementById('fuz-cards');
  if (!el) return;

  const cntPill = document.getElementById('fz-cnt-pill');
  if (cntPill) cntPill.textContent = fuzEntries.length + ' KAYIT';

  if (fuzEntries.length === 0) {
    el.innerHTML = '<div style="font-family:\'Share Tech Mono\',monospace;font-size:11px;color:var(--textd);text-align:center;padding:40px;">Füzyon verisi bekleniyor — Sensörler dinleniyor...</div>';
    return;
  }

  el.innerHTML = fuzEntries.map(e => {
    const cls = e.lvl === 'r' ? 'crit' : 'high';
    const lvlTxt = e.lvl === 'r' ? 'KRİTİK' : 'YÜKSEK';

    // Sensör dot'ları
    const a1cls = e.a1pct > 60 ? 'on' : e.a1pct > 30 ? 'mid' : 'off';
    const a5cls = e.a5pct > 60 ? 'on' : e.a5pct > 30 ? 'mid' : 'off';
    const a3cls = 'off'; // henüz A3 verisi yok

    const dirTxt = e.dir ? DIR_TR[e.dir]+' CEPHESİ' : 'TİTREŞİM';
    const desc   = e.desc || ('A1 '+DIR_TR[e.dir]+' — '+e.m+' METRE MESAFEDE '+e.type);

    return `<div class="fc ${cls}">
      <div class="fc-head">
        <div>
          <div class="fc-id-row">${e.ts} · ${e.id}</div>
          <div class="fc-desc">${dirTxt} — ${e.type}</div>
        </div>
        <div style="text-align:right;">
          <span class="fc-lvl ${e.lvl}">${lvlTxt}</span>
        </div>
      </div>
      <div class="fc-body">
        <div class="fs-dot ${a1cls}" title="A1 Radar">
          <div class="fs-dot-id">A1</div>
          <div class="fs-dot-pct">${e.a1pct}%</div>
        </div>
        <div class="fs-dot ${a5cls}" title="A5 Titreşim">
          <div class="fs-dot-id">A5</div>
          <div class="fs-dot-pct">${e.a5pct}%</div>
        </div>
        <div class="fs-dot ${a3cls}" title="A3 Dış Düğüm">
          <div class="fs-dot-id">A3</div>
          <div class="fs-dot-pct">0%</div>
        </div>
        <div class="fc-fus">
          <div class="fc-fus-bar">
            <div class="fc-fus-fill" style="width:${e.score}%;background:${e.lvl==='r'?'var(--red)':'var(--amber)'}"></div>
          </div>
          <div class="fc-fus-val" style="color:${e.lvl==='r'?'var(--red)':'var(--amber)'}">%${e.score}</div>
          <div class="fc-fus-lbl">FÜZYON SKORU</div>
        </div>
        ${e.m ? `<div class="fc-eta">
          <div class="fc-eta-v" style="color:${e.lvl==='r'?'var(--red)':'var(--amber)'}">${e.m}</div>
          <div class="fc-eta-l">METRE</div>
        </div>` : ''}
      </div>
      <div style="font-family:'Share Tech Mono',monospace;font-size:11px;color:var(--textd);margin-bottom:6px;">${desc}</div>
      <div class="fc-acts">
        ${e.dir ? `<button class="btn r" style="font-size:14px;padding:5px 11px;" onclick="sendCmd({cmd:'set_mode',mode:2});applyModeUI();sysLog('[ OPS ] Takip modu aktif — '+' ${dirTxt} takip ediliyor','r')">⊕ TAKİP MODU</button>` : ''}
        ${e.dir ? `<button class="btn a" style="font-size:14px;padding:5px 11px;" onclick="sendCmd({cmd:'set_mode',mode:1});applyModeUI();sysLog('[ OPS ] Otonom mod aktif','a')">⚙ OTONOM MOD</button>` : ''}
        <button class="btn r" style="font-size:14px;padding:5px 11px;" onclick="fireAllLasers(true);sysLog('[ Y1 ] Lazerler aktif','r')">⚡ LAZER</button>
        <button class="btn x" style="font-size:14px;padding:5px 11px;" onclick="dismissEntry('${e.id}')">✕ KAPAT</button>
      </div>
    </div>`;
  }).join('');
}

function dismissEntry(id) {
  fuzEntries = fuzEntries.filter(e => e.id !== id);
  renderFuzCards();
}

// ═══ RADAR KARTLARI ═══
function updateRadarCards() {
  DIRS.forEach(dir => {
    const d = sensor.dist[dir];
    const m = (d/100).toFixed(0);
    const isC = d < ALERT_CM, isW = d < WARN_CM;
    const col = isC ? 'var(--red)' : isW ? 'var(--amber)' : 'var(--green)';
    const cls = isC ? 'r' : isW ? 'a' : 'g';
    const pct = Math.max(0, Math.round((1-d/MAX_CM)*100));
    const stxt = isC ? '⚠ KRİTİK' : isW ? '● YAKLAŞIYOR' : 'TEMİZ';

    const rd = document.getElementById('rd-'+dir);
    const rs = document.getElementById('rs-'+dir);
    const rb = document.getElementById('rb-'+dir);
    if (rd) { rd.textContent = m+'M'; rd.className = 'cv '+cls; }
    if (rs) { rs.textContent = stxt; rs.style.color = col; }
    if (rb) { rb.style.width = pct+'%'; rb.style.background = col; }

    // Taret kartı alert
    const ta = document.getElementById('alert-'+dir);
    if (ta) { ta.textContent = stxt; ta.style.color = col; }
    // Takip modunda canlı mesafe göstergesi güncelle
    if (servoMode === 2) {
      const tm = document.getElementById('track-dist-'+dir);
      if (tm) tm.textContent = d <= 30 ? '🎯 '+Math.round(d)+'CM' : '⟳ TARAMA '+(d<300?Math.round(d)+'CM':'---');
    }
  });
}

// ═══ RADAR SWEEP ═══
let sweepAng = 0, mmZoomFactor = 1;
function drawSweep() {
  const c = document.getElementById('mm-canvas');
  if (!c) { requestAnimationFrame(drawSweep); return; }
  const ctx = c.getContext('2d');
  const CW = c.width, CH = c.height;
  const cx = CW/2, cy = CH/2, r = Math.min(cx,cy) - 4;

  ctx.clearRect(0,0,CW,CH);
  ctx.fillStyle = '#020508';
  ctx.fillRect(0,0,CW,CH);

  // Grid
  ctx.strokeStyle = 'rgba(0,255,136,.07)'; ctx.lineWidth = 0.5;
  [.25,.5,.75,1].forEach(f => {
    ctx.beginPath(); ctx.arc(cx,cy,r*f,0,Math.PI*2); ctx.stroke();
  });
  [0,45,90,135].forEach(a => {
    const rad = a*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx+r*Math.cos(rad),cy+r*Math.sin(rad)); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx-r*Math.cos(rad),cy-r*Math.sin(rad)); ctx.stroke();
  });

  // Yön etiketleri
  ctx.font = '7px Share Tech Mono'; ctx.fillStyle = 'rgba(0,255,136,.4)'; ctx.textAlign = 'center';
  ctx.fillText('K', cx, cy-r+9);
  ctx.fillText('D', cx+r-6, cy+3);
  ctx.fillText('G', cx, cy+r-2);
  ctx.fillText('B', cx-r+6, cy+3);

  // Hedef noktaları
  const da = {N:-Math.PI/2, E:0, S:Math.PI/2, W:Math.PI};
  DIRS.forEach(dir => {
    const d = sensor.dist[dir], ang = da[dir];
    const norm = Math.min(d, MAX_CM) / MAX_CM;
    const px = cx + r*norm*Math.cos(ang);
    const py = cy + r*norm*Math.sin(ang);
    const col = d < ALERT_CM ? '#ff2244' : d < WARN_CM ? '#ffb700' : '#00ff88';
    const sz = d < ALERT_CM ? 6 : d < WARN_CM ? 5 : 3;
    ctx.beginPath(); ctx.arc(px, py, sz, 0, Math.PI*2);
    ctx.fillStyle = col; ctx.shadowColor = col; ctx.shadowBlur = 8; ctx.fill(); ctx.shadowBlur = 0;
  });

  // Sweep
  sweepAng = (sweepAng + 1.5) % 360;
  const sw = sweepAng * Math.PI / 180;
  const grad = ctx.createLinearGradient(cx, cy, cx+r*Math.cos(sw), cy+r*Math.sin(sw));
  grad.addColorStop(0, 'rgba(0,255,136,0)');
  grad.addColorStop(1, 'rgba(0,255,136,.5)');
  ctx.beginPath(); ctx.moveTo(cx,cy);
  ctx.lineTo(cx+r*Math.cos(sw), cy+r*Math.sin(sw));
  ctx.strokeStyle = grad; ctx.lineWidth = 1.5; ctx.stroke();

  // ── Sweep senkronize radar tetikle ────────────────────────────
  // Sweep cizgisi hangi yon sektoru uzerinde? O radarin aktif oldugunu bildir.
  // Her sektor 90 derece — N:270-360+0-90, E:0-90, S:90-180, W:180-270
  // (Canvas koordinati: 0 derece = sag/dogu, -90 = yukari/kuzey)
  // sweepAng: 0=sag(D), 90=asagi(G), 180=sol(B), 270=yukari(K)
  sweepRadarSync(sweepAng);

  requestAnimationFrame(drawSweep);
}

// Sweep açısına göre aktif radar sektorunu belirle ve ESP32'ye bildir
let sweepSectorHits = {0:0, 1:0, 2:0, 3:0};  // N=0,E=1,S=2,W=3 sayaci
let sweepLastSector = -1;
let sweepHitThreshold = 4;   // kac kez sweep'in ustunden gecince tetikle

function sweepRadarSync(ang) {
  // sweepAng -> sektor: E=45-135, S=135-225, W=225-315, N=315-360+0-45
  let sector;
  if      (ang >= 315 || ang < 45)  sector = 0;  // N (Kuzey)
  else if (ang >= 45  && ang < 135) sector = 1;  // E (Dogu)
  else if (ang >= 135 && ang < 225) sector = 2;  // S (Guney)
  else                               sector = 3;  // W (Bati)

  sweepSectorHits[sector]++;

  // Sektor degistiyse diger sektorlerin sayacini sifirla
  if (sector !== sweepLastSector) {
    for (let k in sweepSectorHits) if (parseInt(k) !== sector) sweepSectorHits[k] = 0;
    sweepLastSector = sector;
  }

  // Esige ulasti mi?
  if (sweepSectorHits[sector] >= sweepHitThreshold) {
    sweepSectorHits[sector] = 0;   // Sifirla, tekrar saysin
    // Bu sektoru aktif radarla esle — ESP32'deki radarTick zaten surekli calisir
    // Biz sadece UI'da hangi sektorun "aktif tarandigini" vurguluyoruz
    highlightActiveSector(sector);
  }
}

const SECTOR_DIRS = ['N','E','S','W'];
let lastHighlightedSector = -1;
function highlightActiveSector(sector) {
  if (lastHighlightedSector === sector) return;
  lastHighlightedSector = sector;
  // Aktif taranan radar kartini vurgula
  SECTOR_DIRS.forEach((dir, i) => {
    const card = document.getElementById('rc-'+dir);
    if (!card) return;
    card.style.outline = (i === sector) ? '1px solid rgba(0,255,136,0.4)' : 'none';
  });
}

// Minimap sürükleme
let mmDrag = false, mmDX = 0, mmDY = 0;
function initMinimap() {
  const hdr = document.getElementById('mm-hdr');
  const wrap = document.getElementById('mm-wrap');
  if (!hdr || !wrap) return;
  hdr.addEventListener('mousedown', e => {
    mmDrag = true; mmDX = e.clientX - wrap.offsetLeft; mmDY = e.clientY - wrap.offsetTop;
    e.preventDefault();
  });
  document.addEventListener('mousemove', e => {
    if (!mmDrag) return;
    wrap.style.left = (e.clientX - mmDX) + 'px';
    wrap.style.top  = (e.clientY - mmDY) + 'px';
    wrap.style.right = 'auto';
  });
  document.addEventListener('mouseup', () => mmDrag = false);

  // Resize
  const rh = document.getElementById('mm-resize');
  let resizing = false, rsX = 0, rsY = 0, rsW = 0, rsH = 0;
  rh.addEventListener('mousedown', e => {
    resizing = true; rsX = e.clientX; rsY = e.clientY;
    rsW = wrap.offsetWidth; rsH = wrap.offsetHeight; e.preventDefault(); e.stopPropagation();
  });
  document.addEventListener('mousemove', e => {
    if (!resizing) return;
    wrap.style.width  = Math.max(160, rsW + e.clientX - rsX) + 'px';
    wrap.style.height = Math.max(140, rsH + e.clientY - rsY) + 'px';
  });
  document.addEventListener('mouseup', () => resizing = false);
}
function mmZ(f) { mmZoomFactor = Math.max(.5, Math.min(3, mmZoomFactor*f)); }
function mmReset() { mmZoomFactor = 1; }
function mmFull() {
  const wrap = document.getElementById('mm-wrap');
  if (!wrap) return;
  const isFull = wrap.style.width === '95vw';
  wrap.style.cssText = isFull
    ? 'width:220px;height:210px;top:8px;right:8px;left:auto;'
    : 'width:95vw;height:80vh;top:50%;left:50%;transform:translate(-50%,-50%);right:auto;';
}

// ═══ SAYFA YÖNETİMİ ═══
function goPage(p) {
  curPage = p;
  ['FUZYON','KONTROL','SISTEM'].forEach(x => {
    const pg = document.getElementById('page-'+x);
    const nb = document.getElementById('nav-'+x);
    const sb = document.getElementById('sb-'+x);
    if (pg) pg.style.display = x===p ? 'flex' : 'none';
    if (nb) nb.classList.toggle('active', x===p);
    if (sb) sb.classList.toggle('active', x===p);
  });
  if (p === 'KONTROL') buildTaretCards();
}

// ═══ BAĞLANTI ═══
function doConnect() {
  const v = document.getElementById('inp-ws').value.trim();
  wsUrl = 'ws://' + (v || location.hostname + ':81');
  document.getElementById('modal').style.display = 'none';
  connectWS();
}
function doConnectBoth() {
  doConnect();
  const a = document.getElementById('inp-ally').value.trim();
  if (a) { try { wsAlly = new WebSocket('ws://'+a); wsAlly.onmessage = e => { try{processStatus(JSON.parse(e.data));}catch(x){} }; } catch(ex){} }
}
function connectWS() {
  // Zaten bagliysa veya baglaniyorsa tekrar acma
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
  if (!wsUrl) wsUrl = 'ws://' + location.hostname + ':81';
  setConn('wait', 'BAĞLANIYOR... ' + wsUrl);
  try {
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => {
      setConn('ok', ws.url.replace('ws://','').replace(':81',''));
      sysLog('[ WS ] Baglandi: ' + ws.url, 'g');
    };
    ws.onmessage = e => {
      if (e.data instanceof ArrayBuffer) {
        vglHandleBinary(e.data);
      } else if (e.data instanceof Blob) {
        // WebSocketsServer bazen Blob gönderir — ArrayBuffer'a çevir
        e.data.arrayBuffer().then(ab => vglHandleBinary(ab));
      } else {
        try { processStatus(JSON.parse(e.data)); } catch(x) {}
      }
    };
    ws.onclose = () => {
      setConn('err', 'KESİLDİ — yeniden deneniyor');
      ws = null;
      setTimeout(connectWS, 4000);
    };
    ws.onerror = () => {
      setConn('err', 'BAĞLANTI HATASI');
      // onerror sonrası onclose tetiklenir, null oraya bırakıyoruz
    };
  } catch(e) {
    setConn('err', 'HATA: ' + e.message);
    ws = null;
    setTimeout(connectWS, 4000);
  }
}
function setConn(st, lbl) {
  document.getElementById('conn-dot').className = 'conn-dot ' + st;
  document.getElementById('conn-lbl').textContent = lbl;
}
function sendCmd(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }

// ═══ SISTEM DURUMU — LLM'e verilecek anlık snapshot ════════
let vglSystemState = {};   // processStatus her cagrildiginda guncellenir

// ═══ STATUS ═══
function processStatus(d) {
  if (d.type === 'pong') return;
  if (d.a1) {
    DIRS.forEach(dir => { if(d.a1[dir]!==undefined){ sensor.dist[dir]=d.a1[dir]; fusionUpdate(dir,d.a1[dir]); }});
  }
  if (d.a5_vib) {
    sensor.vib = d.a5_vib;
    for (let i = 0; i < 3; i++) {
      const el = document.getElementById('vib-'+i);
      if (el) el.style.background = sensor.vib[i] ? 'var(--amber)' : 'var(--b2)';
      vibFusion(i, sensor.vib[i]);
    }
  }
  if (d.mic !== undefined) { sensor.mic = d.mic; const e=document.getElementById('s-mic'); if(e)e.textContent=d.mic; }
  if (d.heap) { const e=document.getElementById('s-heap'); if(e)e.textContent=Math.round(d.heap/1024)+'KB'; }
  if (d.uptime) { const e=document.getElementById('s-uptime'); if(e)e.textContent=fmtUp(d.uptime); }
  if (d.servos) sensor.servos = d.servos;
  if (d.lasers) { sensor.lasers=d.lasers; lazerGroupState[0]=d.lasers[0]; lazerGroupState[1]=d.lasers[1]; updateLzUI(); }
  if (d.lrad !== undefined) { lradState = d.lrad; updateLradUI(); }
  if (d.servo_mode !== undefined && d.servo_mode !== servoMode) { servoMode = d.servo_mode; applyModeUI(); }
  if (d.a3 && d.a3.alert) {
    const entry = {
      id: 'A3-'+String(++fuzIdCounter).padStart(3,'0'), dir: null,
      dist: d.a3.dist, m: d.a3.dist.toFixed(0), score: 80,
      a1pct: 0, a5pct: 0, lvl: 'r',
      type: 'A3 DIŞ DÜĞÜM UYARISI',
      desc: 'ESP-NOW düğümü — '+d.a3.dist.toFixed(0)+'M mesafede tespit',
      ts: new Date().toLocaleTimeString('tr',{hour12:false}), confirmed: true
    };
    fuzEntries.unshift(entry);
    renderFuzCards();
  }
  if (d.wifi_nets && d.wifi_nets.length) updateSigint(d.wifi_nets);
  if (d.radar_enabled) console.log('[DBG] radarEn:', JSON.stringify(d.radar_enabled));

  // LLM icin anlık sistem snapshot'i guncelle
  vglSystemState = {
    taretler: {
      K: { aci: sensor.servos[0]||90, aktif: sensor.unit_enabled ? sensor.unit_enabled[0] : true },
      D: { aci: sensor.servos[1]||90, aktif: sensor.unit_enabled ? sensor.unit_enabled[1] : true },
      G: { aci: sensor.servos[2]||90, aktif: sensor.unit_enabled ? sensor.unit_enabled[2] : true },
      B: { aci: sensor.servos[3]||90, aktif: sensor.unit_enabled ? sensor.unit_enabled[3] : true }
    },
    radarlar: {
      K: { mesafe_cm: Math.round(sensor.dist['N']||300), aktif: sensor.radar_enabled ? sensor.radar_enabled[0] : true },
      D: { mesafe_cm: Math.round(sensor.dist['E']||300), aktif: sensor.radar_enabled ? sensor.radar_enabled[1] : true },
      G: { mesafe_cm: Math.round(sensor.dist['S']||300), aktif: sensor.radar_enabled ? sensor.radar_enabled[2] : true },
      B: { mesafe_cm: Math.round(sensor.dist['W']||300), aktif: sensor.radar_enabled ? sensor.radar_enabled[3] : true }
    },
    lazerler: { A: sensor.lasers ? sensor.lasers[0] : false, B: sensor.lasers ? sensor.lasers[1] : false },
    kirmizi_alarm: redAlertMode,
    mod: servoMode === 2 ? 'takip' : servoMode === 1 ? 'otonom' : 'manuel',
    lrad: lradState
  };
  // radar_enabled ve unit_enabled de sensor'a kaydet
  if (d.radar_enabled) sensor.radar_enabled = d.radar_enabled;
  if (d.unit_enabled)  sensor.unit_enabled  = d.unit_enabled;
}
function fmtUp(s) { const h=Math.floor(s/3600),m=Math.floor(s%3600/60),sec=s%60; return (h?h+'s ':'')+m+'d '+sec+'sn'; }

// ═══ TARET KARTLARI ═══
function buildTaretCards() {
  const el = document.getElementById('taret-cards');
  if (!el) return;
  el.innerHTML = '<div class="g2">' + DIRS.map((dir,idx) => {
    const angle = sensor.servos[idx]||90;
    const lzG = idx<=1?0:1, laser = lazerGroupState[lzG];
    const d = sensor.dist[dir];
    const isC = d<ALERT_CM, isW = d<WARN_CM;
    return `<div class="card">
      <div style="display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:8px;">
        <div>
          <div class="ct">${DIR_TR[dir]} / ${dir}</div>
          <div id="alert-${dir}" style="font-family:'Share Tech Mono',monospace;font-size:11px;color:var(--green);">TEMİZ</div>
        </div>
        <div style="text-align:right;">
          <div style="font-family:'Bebas Neue',sans-serif;font-size:36px;line-height:1;color:var(--amber);" id="sa-${dir}">${angle}°</div>
          <div style="font-family:'Share Tech Mono',monospace;font-size:9px;color:var(--textd);">AÇI</div>
        </div>
      </div>
      <button class="btn r" style="margin-bottom:6px;font-size:10px;${laser?'background:rgba(255,34,68,.1);':''}" onclick="toggleLaserGroup(${lzG})">⚡ LZR-${lzG?'B':'A'} ${laser?'AKTİF':''}</button>
      ${servoMode===0
        ?`<div class="sl-row"><div class="sl-lbl"><span>0°</span><span>180°</span></div><input type="range" min="0" max="180" value="${angle}" oninput="servoSlider(${idx},'${dir}',this.value)"></div>`
        :servoMode===1
        ?`<div style="font-family:'Share Tech Mono',monospace;font-size:10px;color:var(--amber);padding:4px 0;letter-spacing:2px;">OTONOM MODDA</div>`
        :`<div style="font-family:'Share Tech Mono',monospace;font-size:10px;color:var(--red);padding:4px 0;letter-spacing:2px;">⊕ TAKİP MODU — <span id="track-dist-${dir}">${d<=30?'🎯 '+Math.round(d)+'CM':'⟳ TARAMA '+(d<300?Math.round(d)+'CM':'---')}</span></div>`}
    </div>`;
  }).join('') + '</div>' +
  // Joystick taret seçim satırı
  `<div style="padding:8px 10px;border-top:1px solid var(--b1);display:flex;align-items:center;gap:6px;flex-wrap:wrap;">
    <span style="font-family:'Share Tech Mono',monospace;font-size:14px;color:var(--textd);">JOY →</span>
    <div id="taret-sel-row" style="display:flex;gap:4px;"></div>
  </div>`;
  updateJoySel();
}

function servoSlider(idx, dir, val) {
  val = parseInt(val);
  const e = document.getElementById('sa-'+dir); if(e)e.textContent=val+'°';
  sendCmd({cmd:'servo', id:idx, angle:val});
}

// ═══ MOD ═══
function toggleMode() {
  servoMode = (servoMode + 1) % 3;  // 0→1→2→0 döngüsü
  sendCmd({cmd:'set_mode', mode:servoMode});
  applyModeUI();
  const modNames = ['MANUEL', 'OTONOM', 'TAKİP'];
  sysLog('[ MOD ] ' + modNames[servoMode] + ' moda geçildi', servoMode===0?'x':servoMode===1?'a':'r');
}
function applyModeUI() {
  const b = document.getElementById('mode-btn');
  const p = document.getElementById('mode-pill');
  const modTxt = ['MANUEL','OTONOM','TAKİP'];
  const modCls = ['manuel','oto','takip'];
  const pillCls = ['x','a','r'];
  const txt = modTxt[servoMode] || 'MANUEL';
  const cls = modCls[servoMode] || 'manuel';
  const pcls = pillCls[servoMode] || 'x';
  if(b){b.textContent=txt;b.className='tb-mode '+cls;}
  if(p){p.textContent=txt;p.className='pill '+pcls;}
  // Sidebar taret durumu
  const sbst = document.getElementById('sb-yp-st');
  const sbled = document.getElementById('sb-yp-led');
  if(sbst) sbst.textContent = txt;
  if(sbled) sbled.className = 'sb-led ' + (servoMode===0?'led-x':servoMode===1?'led-a':'led-r');
  buildTaretCards(); updateJoyPad();
}

// ═══ JOYSTİCK ═══
function updateJoySel() {
  const r = document.getElementById('taret-sel-row'); if(!r)return;
  r.innerHTML = DIRS.map((dir,idx) =>
    `<button class="btn ${selectedTaret===idx?'a':'x'}" style="font-size:10px;padding:3px 8px;" onclick="selectTaret(${idx})">${DIR_TR[dir]}</button>`
  ).join('');
  updateJoyPad();
}
function selectTaret(idx) { selectedTaret=selectedTaret===idx?-1:idx; updateJoySel(); sendCmd({cmd:'joy_target',id:selectedTaret}); }
function updateJoyPad() {
  const w = document.getElementById('joy-pad-wrap'); if(!w)return;
  const show = servoMode===0 && selectedTaret>=0;
  w.style.display = show?'block':'none';
  if(show){
    document.getElementById('joy-lbl').textContent = DIR_TR[DIRS[selectedTaret]]+' JOYSTİCK X';
    const a = sensor.servos[selectedTaret]||90;
    document.getElementById('joy-ang').textContent = a+'°';
    document.getElementById('joy-slider').value = a;
  }
}
function joySliderInput(v) { v=parseInt(v); document.getElementById('joy-ang').textContent=v+'°'; if(selectedTaret>=0&&servoMode===0)sendCmd({cmd:'joy_x',angle:v}); }
let joyOn=false, joyX0=0;
function initJoyPad() {
  const pad = document.getElementById('joy-pad'); if(!pad)return;
  const gx = e => e.touches?e.touches[0].clientX:e.clientX;
  const mv = e => {
    if(!joyOn)return;
    const dx=gx(e)-joyX0, rng=100;
    const base=sensor.servos[selectedTaret]||90;
    const ang=Math.max(0,Math.min(180,base+Math.round(dx/rng*90)));
    document.getElementById('joy-stick').style.left=Math.max(10,Math.min(90,50+dx/rng*40))+'%';
    document.getElementById('joy-slider').value=ang;
    document.getElementById('joy-ang').textContent=ang+'°';
    sendCmd({cmd:'joy_x',angle:ang});
    if(e.cancelable)e.preventDefault();
  };
  pad.addEventListener('mousedown',e=>{if(selectedTaret<0||servoMode!==0)return;joyOn=true;joyX0=gx(e);pad.classList.add('on');});
  pad.addEventListener('touchstart',e=>{if(selectedTaret<0||servoMode!==0)return;joyOn=true;joyX0=gx(e);pad.classList.add('on');e.preventDefault();},{passive:false});
  document.addEventListener('mousemove',mv);
  document.addEventListener('touchmove',mv,{passive:false});
  const up=()=>{joyOn=false;pad.classList.remove('on');document.getElementById('joy-stick').style.left='50%';};
  document.addEventListener('mouseup',up); document.addEventListener('touchend',up);
}

// ═══ EW ═══
function toggleLrad() {
  lradState=!lradState;
  const f=parseInt(document.getElementById('lrad-freq').value);
  sendCmd({cmd:'lrad',state:lradState,freq:f});
  updateLradUI();
  sysLog('[ E ] LRAD '+(lradState?f+'Hz aktif':'kapatıldı'), lradState?'a':'x');
}
function updateLradUI() {
  const b=document.getElementById('badge-lrad'),btn=document.getElementById('btn-lrad');
  if(!b)return;
  b.textContent=lradState?'AKTİF':'KAPALI'; b.className='badge '+(lradState?'on':'off');
  btn.textContent=lradState?'KAPAT':'AÇ';
}
function toggleLaserGroup(id) {
  lazerGroupState[id]=!lazerGroupState[id];
  sendCmd({cmd:'laser',id,state:lazerGroupState[id]});
  updateLzUI(); buildTaretCards();
  sysLog('[ E ] Lazer '+(id?'B':'A')+' '+(lazerGroupState[id]?'aktif':'kapatıldı'), lazerGroupState[id]?'r':'x');
}
function updateLzUI() {
  ['A','B'].forEach((l,i)=>{
    const b=document.getElementById('badge-lz'+l),btn=document.getElementById('btn-lz'+l);
    if(!b)return;
    b.textContent=lazerGroupState[i]?'AKTİF':'KAPALI'; b.className='badge '+(lazerGroupState[i]?'on':'off');
    btn.textContent=lazerGroupState[i]?'KAPAT':'AÇ';
  });
}
function fireAllLasers(s) {
  sendCmd({cmd:'laser_all',state:s}); lazerGroupState[0]=s; lazerGroupState[1]=s;
  updateLzUI(); buildTaretCards();
  sysLog('[ E ] Tüm lazerler '+(s?'aktif':'kapatıldı'), s?'r':'x');
}

// ═══ ALARM ═══
function toggleRedAlert() {
  redAlertMode=!redAlertMode;
  sendCmd({cmd:'red_alert',state:redAlertMode});
  const b=document.getElementById('alarm-badge');
  if(b)b.classList.toggle('show',redAlertMode);
  sysLog(redAlertMode?'[ OPS ] KIRMIZI ALARM AKTİF — TÜM SİSTEMLER ALARM MODUNDA':'[ OPS ] KIRMIZI ALARM İPTAL', redAlertMode?'r':'a');
}

// ═══ SİGINT ═══
function updateSigint(nets) {
  const el=document.getElementById('sig-list'); if(!el)return;
  const cnt=document.getElementById('sig-cnt'); if(cnt)cnt.textContent=nets.length+' AĞ';
  el.innerHTML='<table class="dt" style="margin:8px;">' +
    '<thead><tr><th>SSID</th><th>KANAL</th><th>ŞİFRELEME</th><th>RSSI</th><th>GÜÇ</th></tr></thead><tbody>' +
    nets.sort((a,b)=>b.rssi-a.rssi).map(n => {
      const str=Math.max(0,Math.min(100,Math.round((n.rssi+100)/70*100)));
      const open=n.enc==='OPEN';
      return `<tr>
        <td style="color:${open?'var(--red)':'var(--texth)'}">${n.ssid||'[Gizli]'}</td>
        <td>CH${n.ch}</td>
        <td class="${open?'r':'g'}">${n.enc}</td>
        <td class="${n.rssi>-60?'g':n.rssi>-80?'a':'r'}">${n.rssi}dBm</td>
        <td><div class="bt" style="width:80px;"><div class="bf" style="width:${str}%;background:${open?'var(--red)':'var(--green)'};"></div></div></td>
      </tr>`;
    }).join('') + '</tbody></table>';
}

// ═══ VIGILANT SES SİSTEMİ ═══
// Groq Whisper STT + Groq Llama LLM (Vigilant kimliği)
// Ses: ESP32 GPIO39 ADC → WebSocket binary → tarayıcı buffer → WAV → Groq

const VGL_SAMPLE_RATE = 6600;   // Gerçek ölçülen hız ~6600Hz
const VGL_GROQ_STT    = 'https://api.groq.com/openai/v1/audio/transcriptions';
const VGL_GROQ_LLM    = 'https://api.groq.com/openai/v1/chat/completions';
const VGL_SYSTEM = `Sen VIGILANT-X'sin — gelismis bir taktik karakol komuta yapay zekasisin.
Kullanici sana sesli ve yazili komutlar verir. Yanitlar kisa ve net olsun (1-2 cumle).
Kimligin: sogukkanli, profesyonel, askeri tarz. Her cumleyi "efendim" ile bitir.
Turkcen mukemmel. SADECE Turkce konusursun.

CEVAP FORMATI — MUTLAKA bu JSON formatinda cevap ver, baska hicbir sey yazma:
{"reply":"...Turkce kisa cevap efendim ile biten...","action":null}
Fiziksel komut gerekiyorsa action dolu olsun:
{"reply":"...","action":{"cmd":"...","diger_parametreler":...}}

MEVCUT KOMUTLAR (action ornekleri):
- Kirmizi alarm ac:          {"cmd":"red_alert","state":true}
- Kirmizi alarm kapat:       {"cmd":"red_alert","state":false}
- Otonom mod:                {"cmd":"set_mode","mode":1}
- Manuel mod:                {"cmd":"set_mode","mode":0}
- Takip modu:                {"cmd":"set_mode","mode":2}
- Tum lazerleri ac:          {"cmd":"laser_all","state":true}
- Tum lazerleri kapat:       {"cmd":"laser_all","state":false}
- Lazer A ac:                {"cmd":"laser","id":0,"state":true}
- Lazer B kapat:             {"cmd":"laser","id":1,"state":false}
- LRAD ac:                   {"cmd":"lrad","state":true,"freq":2000}
- LRAD kapat:                {"cmd":"lrad","state":false}
- Tum taretleri sifirla:     {"cmd":"servo_reset"}
- Kuzey taretini X dereceye: {"cmd":"servo_angle","id":0,"angle":X}
- Dogu taretini X dereceye:  {"cmd":"servo_angle","id":1,"angle":X}
- Guney taretini X dereceye: {"cmd":"servo_angle","id":2,"angle":X}
- Bati taretini X dereceye:  {"cmd":"servo_angle","id":3,"angle":X}
- Kuzey radari ac:           {"cmd":"unit_enable","unit_type":"radar","id":0,"state":true}
- Dogu radari kapat:         {"cmd":"unit_enable","unit_type":"radar","id":1,"state":false}
- Guney radari ac:           {"cmd":"unit_enable","unit_type":"radar","id":2,"state":true}
- Bati radari kapat:         {"cmd":"unit_enable","unit_type":"radar","id":3,"state":false}
- Kuzey tareti ac:           {"cmd":"unit_enable","unit_type":"servo","id":0,"state":true}
- Dogu tareti kapat:         {"cmd":"unit_enable","unit_type":"servo","id":1,"state":false}
- Guney tareti ac:           {"cmd":"unit_enable","unit_type":"servo","id":2,"state":true}
- Bati tareti kapat:         {"cmd":"unit_enable","unit_type":"servo","id":3,"state":false}
- RGB LED rengini ayarla:    {"cmd":"set_rgb","r":0,"g":255,"b":0}  (r/g/b: 0-255)
  Renk ornekleri: yesil=r0g255b0, kirmizi=r255g0b0, mavi=r0g0b255, beyaz=r255g255b255, kapat=r0g0b0

DURUM SORGULARI: Komutan sana sistem durumunu sorduğunda (taret acisi, radar mesafesi vb.)
sana verilecek SISTEM_DURUMU verisini oku ve doğrudan cevap ver. Tahmin etme, okudugunu yaz.
Komut gerektirmeyen soru/sohbetlerde action:null don.

MOD AÇIKLAMALARI:
- manuel: Servolar kullanici kontrolunde, slider veya joystick ile hareket eder.
- otonom: Servolar otomatik ileri-geri tarama yapar, radar tespiti yok.
- takip: Her taret kendi yonundeki radarın mesafesine gore aciyi ayarlar.
  0-30cm arasi: TAKİP — dist=30cm→90 derece merkez, dist=0cm→15 derece (tam saptirma).
  30cm uzeri: OTONOM TARAMA — hedef yok, ileri-geri tarar.
  Kucuk mesafe farklari buyuk aci hareketi yaratir (30cm aralik 90 dereceye sikistirilmistir).
  Yumusatma filtresi ani sicramayi onler. Tum taretler bagimsiz calisir.`;

let vglApiKey   = '';
let vglRecording = false;
let vglBuffer   = [];      // uint16 ADC ornekleri (ESP32'den gelen)
let vglHistory  = [];      // gorselleştirici
let vglCanvas, vglCtx;
let vglChatHistory = [];   // {role:'user'|'assistant', content:'...'} — LLM hafizasi

// localStorage'dan key yukle
(function(){
  try {
    const k = localStorage.getItem('vgl_groq_key');
    if (k) { vglApiKey = k; document.getElementById('vgl-apikey').value = k; }
  } catch(e){}
})();

function saveVglKey() {
  vglApiKey = document.getElementById('vgl-apikey').value.trim();
  try { localStorage.setItem('vgl_groq_key', vglApiKey); } catch(e){}
  vglStatus(vglApiKey ? 'Key kaydedildi.' : 'Key bos!');
}

function vglStatus(msg, cls) {
  const el = document.getElementById('vgl-status');
  if (el) { el.textContent = msg; el.style.color = cls === 'r' ? 'var(--red)' : cls === 'g' ? 'var(--green)' : 'var(--textd)'; }
}

function vglToggleRec() {
  if (!vglRecording) {
    if (!vglApiKey) { vglStatus('Once API key gir!', 'r'); return; }
    // Kayit baslat
    vglRecording = true;
    vglBuffer = [];
    sendCmd({cmd: 'voice_start'});  // ESP32'ye bildir — diger sistemler duraksatiliyor
    const btn = document.getElementById('vgl-recbtn');
    if (btn) { btn.textContent = '■ DURDUR & GONDER'; btn.className = 'btn r'; }
    vglStatus('Kayit aliniyor...');
    sysLog('[ VOICE ] Ses kaydi basladi — sistemler duraklatildi', 'b');
  } else {
    // Kaydı durdur
    vglRecording = false;
    sendCmd({cmd: 'voice_stop'});   // ESP32'ye bildir — sistemler devam ediyor
    const btn = document.getElementById('vgl-recbtn');
    if (btn) { btn.textContent = '&#9679; KAYIT BASLAT'; btn.className = 'btn b'; }
    vglStatus('Gonderiliyor...');
    sysLog('[ VOICE ] Ses kaydi bitti — sistemler devam ediyor', 'b');

    if (vglBuffer.length < 200) {
      vglStatus('Cok kisa kayit.');
      return;
    }
    vglSendToGroq([...vglBuffer]);
    vglBuffer = [];
  }
}

// ESP32'den gelen binary ses paketi
// Gerçek sample rate ölçümü
let _srPktCount = 0, _srTotalSamples = 0, _srStartTime = 0;
let _measuredSR = 6600; // Ölçülen gerçek hız — başlangıç tahmini

function vglHandleBinary(buf) {
  const view = new DataView(buf);

  // Format tespiti: 4+256 byte = yeni (timestamp+data), 256 byte = eski
  let dataOffset = 0;
  if (buf.byteLength === 4 + 128 * 2) {
    dataOffset = 4; // ilk 4 byte timestamp, atla
  }
  const count = (buf.byteLength - dataOffset) / 2;

  // Görselleştirici — her zaman çalış (kayıt olsun olmasın)
  let sum = 0;
  for (let i = 0; i < count; i++) {
    const n = Math.abs(view.getUint16(dataOffset + i * 2, true) - 2048) / 2048;
    sum += n * n;
  }
  const rms = Math.sqrt(sum / count);
  vglHistory.push(rms);
  if (!vglCanvas) {
    vglCanvas = document.getElementById('vgl-canvas');
    if (vglCanvas) {
      vglCtx = vglCanvas.getContext('2d');
      vglCanvas.width = vglCanvas.parentElement ? vglCanvas.parentElement.clientWidth || 300 : 300;
    }
  }
  if (vglCanvas && vglCtx) {
    const W = vglCanvas.width, H = vglCanvas.height, cy = H / 2;
    while (vglHistory.length > W) vglHistory.shift();
    vglCtx.clearRect(0, 0, W, H);
    vglCtx.strokeStyle = 'rgba(255,34,68,0.9)';
    vglCtx.lineWidth = 1.2;
    vglCtx.beginPath();
    vglHistory.forEach((v, i) => {
      const h = v * cy * 0.9;
      i === 0 ? vglCtx.moveTo(i, cy - h) : vglCtx.lineTo(i, cy - h);
    });
    vglCtx.stroke();
    vglCtx.strokeStyle = 'rgba(255,34,68,0.4)';
    vglCtx.beginPath();
    vglHistory.forEach((v, i) => {
      const h = v * cy * 0.9;
      i === 0 ? vglCtx.moveTo(i, cy + h) : vglCtx.lineTo(i, cy + h);
    });
    vglCtx.stroke();
  }

  if (!vglRecording) {
    // Kayıt bitince ölçümü sıfırla
    _srPktCount = 0; _srTotalSamples = 0; _srStartTime = 0;
    return;
  }

  // Kayıt buffer'a ekle
  for (let i = 0; i < count; i++) {
    vglBuffer.push(view.getUint16(dataOffset + i * 2, true));
  }

  // Gerçek sample rate ölç
  if (_srStartTime === 0) _srStartTime = Date.now();
  _srTotalSamples += count;
  _srPktCount++;
  if (_srPktCount === 30) { // 30 paket sonra güncelle (~600ms)
    const elapsed = Date.now() - _srStartTime;
    if (elapsed > 200) {
      const measured = Math.round(_srTotalSamples / (elapsed / 1000));
      _measuredSR = Math.max(4000, Math.min(16000, measured));
      console.log('[SR] Ölçülen örnekleme hızı:', _measuredSR, 'Hz');
    }
  }
}

function vglBuildWav(samples) {
  const sr = _measuredSR; // sabit değer yerine ölçülen gerçek hız
  const n  = samples.length;
  const buf = new ArrayBuffer(44 + n * 2);
  const dv  = new DataView(buf);
  const ws  = (o, s) => { for (let i=0;i<s.length;i++) dv.setUint8(o+i, s.charCodeAt(i)); };
  ws(0,'RIFF'); dv.setUint32(4, 36+n*2, true);
  ws(8,'WAVE'); ws(12,'fmt ');
  dv.setUint32(16,16,true); dv.setUint16(20,1,true); dv.setUint16(22,1,true);
  dv.setUint32(24,sr,true); dv.setUint32(28,sr*2,true);
  dv.setUint16(32,2,true);  dv.setUint16(34,16,true);
  ws(36,'data'); dv.setUint32(40,n*2,true);
  for (let i=0;i<n;i++) dv.setInt16(44+i*2, Math.round(((samples[i]-2048)/2048)*32767), true);
  return new Blob([buf], {type:'audio/wav'});
}

async function vglSendToGroq(samples) {
  const btn = document.getElementById('vgl-recbtn');
  if (btn) btn.disabled = true;
  try {
    // 1. STT
    vglStatus('Whisper: ses taniniyor...');
    const fd = new FormData();
    fd.append('file', vglBuildWav(samples), 'audio.wav');
    fd.append('model', 'whisper-large-v3-turbo');
    fd.append('language', 'tr');
    fd.append('response_format', 'json');
    fd.append('temperature', '0');
    fd.append('prompt', 'Türkçe konuşma. Vigilant, radar, taret, servo, otonom, kuzey, güney, doğu, batı, alarm, lazer, aktif, kapat, aç, efendim, derece, mesafe, sensör, titreşim.');
    const sttRes = await fetch(VGL_GROQ_STT, {
      method:'POST',
      headers:{'Authorization':'Bearer '+vglApiKey},
      body: fd
    });
    if (!sttRes.ok) throw new Error('STT '+sttRes.status);
    const sttData = await sttRes.json();
    const userText = (sttData.text || '').trim();
    if (!userText) { vglStatus('Anlasilamadi, tekrar dene.','r'); if(btn)btn.disabled=false; return; }

    vglLogEntry('SEN', userText, 'var(--text)');
    sysLog('[ STT ] '+userText, 'b');

    // 2. LLM — ortak vglAskLLM fonksiyonu, history dahil
    vglStatus('Vigilant dusunuyor...');
    await vglAskLLM(userText);
    vglStatus('Hazir.');
  } catch(e) {
    vglStatus('Hata: '+e.message, 'r');
    sysLog('[ VOICE ] Hata: '+e.message, 'r');
  }
  if (btn) btn.disabled = false;
}

// ── Ortak LLM çağrısı — hafıza dahil ────────────────────────
// Hem ses hem text buraya gelir. vglChatHistory'ye ekler ve
// tum gecmisi Groq'a gonderir.
async function vglAskLLM(userText) {
  // Kullanici mesajini history'ye ekle
  vglChatHistory.push({ role: 'user', content: userText });

  // History cok uzarsa eski mesajlari at (son 20 mesaj = 10 tur)
  if (vglChatHistory.length > 20) {
    vglChatHistory = vglChatHistory.slice(vglChatHistory.length - 20);
  }

  const now = new Date();
  const timeCtx = 'Suan: ' + now.toLocaleDateString('tr-TR') + ' ' + now.toLocaleTimeString('tr-TR');
  const stateCtx = Object.keys(vglSystemState).length
    ? '\nSISTEM_DURUMU=' + JSON.stringify(vglSystemState)
    : '';

  const messages = [
    { role: 'system', content: VGL_SYSTEM + '\n' + timeCtx + stateCtx },
    ...vglChatHistory   // tum gecmis dahil
  ];

  const llmRes = await fetch(VGL_GROQ_LLM, {
    method: 'POST',
    headers: { 'Authorization': 'Bearer ' + vglApiKey, 'Content-Type': 'application/json' },
    body: JSON.stringify({
      model: 'llama-3.3-70b-versatile',
      max_tokens: 150,
      temperature: 0.5,
      messages: messages
    })
  });

  if (!llmRes.ok) {
    const err = await llmRes.text();
    throw new Error('LLM ' + llmRes.status + ': ' + err.slice(0, 80));
  }

  const llmData = await llmRes.json();
  const rawReply = llmData.choices?.[0]?.message?.content?.trim() || '';

  // JSON parse — LLM bazen reply icine JSON gomebiliyor, birden fazla yontem dene
  let reply = rawReply, action = null;
  try {
    // Yontem 1: Duz JSON parse
    const clean = rawReply.replace(/```json|```/g, '').trim();
    const parsed = JSON.parse(clean);
    reply  = parsed.reply  || rawReply;
    action = parsed.action || null;
  } catch(e) {
    // Yontem 2: Metin icinde JSON blogu ara — {"reply":...} formatini bul
    const match = rawReply.match(/\{[\s\S]*"reply"[\s\S]*\}/);
    if (match) {
      try {
        const parsed = JSON.parse(match[0]);
        reply  = parsed.reply  || rawReply;
        action = parsed.action || null;
      } catch(e2) { /* ham metin kullan */ }
    }
  }
  // reply hala JSON icerigine benziyorsa temizle
  if (reply && reply.includes('"reply"') && reply.includes('"action"')) {
    try {
      const parsed = JSON.parse(reply.match(/\{[\s\S]*\}/)?.[0] || '{}');
      if (parsed.reply) { action = parsed.action || action; reply = parsed.reply; }
    } catch(e) {}
  }

  // Assistant cevabini history'ye ekle (ham JSON degil, sadece metin)
  vglChatHistory.push({ role: 'assistant', content: reply });

  // UI'a yaz
  vglLogEntry('VGL', reply, 'var(--green)');
  sysLog('[ VGL ] ' + reply, 'g');

  // Fiziksel komut varsa onay beklet
  if (action && action.cmd) {
    vglPendingAction = action;
    vglPendingReply  = reply;
    vglShowConfirm(action);
  }
}

// Bekleyen onay
let vglPendingAction = null;
let vglPendingReply  = '';

function vglShowConfirm(action) {
  const area = document.getElementById('vgl-confirm-area');
  const cmdEl = document.getElementById('vgl-confirm-cmd');
  if (!area || !cmdEl) return;
  cmdEl.textContent = 'KOMUT: ' + JSON.stringify(action);
  area.style.display = 'block';
  // Log'a da kisa not duş
  vglLogEntry('SYS', '▶ Onay bekleniyor — UYGULA veya İPTAL', 'var(--amber)');
}

function vglApply() {
  if (!vglPendingAction) return;
  sendCmd({cmd:'voice_cmd', reply: vglPendingReply, action: vglPendingAction});
  sysLog('[ VGL ] Komut uygulandi: '+JSON.stringify(vglPendingAction), 'a');
  vglLogEntry('SYS', '✓ Komut uygulandı: '+JSON.stringify(vglPendingAction), 'var(--green)');
  vglPendingAction = null;
  vglPendingReply  = '';
  const area = document.getElementById('vgl-confirm-area');
  if (area) area.style.display = 'none';
}

function vglCancel() {
  vglPendingAction = null;
  vglPendingReply  = '';
  const area = document.getElementById('vgl-confirm-area');
  if (area) area.style.display = 'none';
  vglLogEntry('SYS', '✗ Komut iptal edildi', 'var(--textd)');
  sysLog('[ VGL ] Komut iptal edildi.', 'x');
}

function vglLogEntry(tag, text, color) {
  const log = document.getElementById('vgl-log');
  if (!log) return;
  const d = document.createElement('div');
  d.style.cssText = 'font-family:"Share Tech Mono",monospace;font-size:16px;margin-bottom:5px;line-height:1.5;';
  d.innerHTML = '<span style="color:var(--textd);">'+tag+'</span> <span style="color:'+color+';white-space:pre-wrap;">'+text+'</span>';
  log.appendChild(d);
  log.scrollTop = log.scrollHeight;
}

// initSTT: textarea Enter tusu bagla
function initSTT() {
  const txt = document.getElementById('vgl-txt');
  if (!txt) return;
  txt.addEventListener('keydown', function(e) {
    // Textarea odakta ve Enter'a basıldıysa gönder (Shift+Enter yeni satır)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      vglSendText();
    }
  });
}

// Text mesaj gonder — ayni LLM pipeline'i kullanir
async function vglSendText() {
  if (!vglApiKey) { vglStatus('Once API key gir!', 'r'); return; }
  const txt = document.getElementById('vgl-txt');
  if (!txt) return;
  const userText = txt.value.trim();
  if (!userText) return;

  txt.value = '';
  txt.style.borderColor = '';

  // Chat'e kullanici satirini ekle — ses satirlarıyla ayni format
  vglLogEntry('SEN', userText, 'var(--text)');
  sysLog('[ TXT ] ' + userText, 'b');

  // LLM'e gonder
  const btn = document.getElementById('vgl-sendbtn');
  if (btn) btn.disabled = true;
  vglStatus('Vigilant dusunuyor...');

  try {
    vglStatus('Vigilant dusunuyor...');
    await vglAskLLM(userText);
    vglStatus('Hazir.');
  } catch(e) {
    vglStatus('Hata: ' + e.message, 'r');
    sysLog('[ VOICE ] Text hata: ' + e.message, 'r');
  }

  if (btn) btn.disabled = false;
  // Odagi geri ver
  txt.focus();
}

// ═══ LOG ═══
function sysLog(msg, cls) {
  cls = cls||'x';
  const el = document.getElementById('log-strip-msg');
  if(el){el.textContent=msg;el.className='log-msg '+cls;}
  // DBG
  const dbg=document.getElementById('dbg');
  if(dbg){const d=document.createElement('div');d.textContent='['+new Date().toLocaleTimeString('tr',{hour12:false})+'] '+msg;d.style.color=cls==='r'?'var(--red)':cls==='a'?'var(--amber)':cls==='g'?'var(--green)':cls==='b'?'var(--blue)':'var(--textd)';dbg.prepend(d);while(dbg.children.length>60)dbg.removeChild(dbg.lastChild);}
}

// ═══ SAAT ═══
setInterval(()=>{ document.getElementById('tb-clock').textContent=new Date().toLocaleTimeString('tr',{hour12:false}); },1000);

// ═══ DEBUG ═══
const _ol=console.log.bind(console);
console.log=function(...a){ _ol(...a); sysLog(a.map(x=>typeof x==='object'?JSON.stringify(x):x).join(' '),'x'); };

// ═══ BAŞLAT ═══
document.addEventListener('DOMContentLoaded',()=>{
  drawSweep();
  initMinimap();
  buildTaretCards();
  initJoyPad();
  initSTT();
  goPage('FUZYON');
  sysLog('[ SYS ] VIGILANT-X aktif. Baglaniyor...','g');
  // Otomatik baglanti — mDNS yerine IP kullan
  const host = location.hostname;
  // location.hostname mDNS adı dönüyorsa (örn. vigilant-x.local)
  // WebSocket buna bağlanamayabilir, IP daha güvenli
  // Kullanıcı sayfayı IP ile açtıysa direk o IP kullanılır
  wsUrl = 'ws://' + host + ':81';
  connectWS();
});
</script>
</body>
</html>

)HTMLDELIM";

// ═══════════════════════════════════════════════════════════
//  HTTP SUNUCU
// ═══════════════════════════════════════════════════════════
void handleRoot() {
  httpServer.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}
void setupHttpServer() {
  httpServer.on("/",   handleRoot);
  httpServer.on("/ui", handleRoot);
  httpServer.begin();
  Serial.println("[HTTP] :80 hazır");
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.println("║  VIGILANT-X v4 Başlıyor...    ║");
  Serial.println("╚═══════════════════════════════╝");

  // HC-SR04
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT); digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }
  Serial.println("[PIN] HC-SR04 × 4 hazır");

  // Titreşim
  pinMode(VIB_1, INPUT); pinMode(VIB_2, INPUT); pinMode(VIB_3, INPUT);

  // Lazer
  for (int i = 0; i < 2; i++) { pinMode(laserPins[i], OUTPUT); digitalWrite(laserPins[i], LOW); }
  Serial.println("[PIN] Lazerler hazır");
  // Timer rezervasyonu — ledcAttach'tan ÖNCE yapılmalı
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Servolar — ledcAttach'lardan ÖNCE attach et
  servoN.setPeriodHertz(50); servoN.attach(SERVO_N, 544, 2400);
  servoE.setPeriodHertz(50); servoE.attach(SERVO_E, 544, 2400);
  servoS.setPeriodHertz(50); servoS.attach(SERVO_S, 544, 2400);
  servoW.setPeriodHertz(50); servoW.attach(SERVO_W, 544, 2400);
  for (int i = 0; i < 4; i++) setServoAngle(i, 90);
  Serial.println("[PIN] SG90 x4 merkez (90 derece)");

  // Buzzer
  ledcAttach(BUZZER_PIN, 2000, 8);
  ledcWriteTone(BUZZER_PIN, 0);
  Serial.println("[PIN] Buzzer/LRAD hazir");

  // PAM8403 ses
  ledcAttach(VOL_PWM_PIN, VOL_PWM_FREQ, 8);
  setVolume(VOL_DEFAULT);
  Serial.printf("[PIN] PAM8403 hazir → GPIO%d, %d%%\n", VOL_PWM_PIN, VOL_DEFAULT);

  // 5050 RGB LED
  ledcAttach(RGB_R_PIN, RGB_FREQ, 8);
  ledcAttach(RGB_G_PIN, RGB_FREQ, 8);
  ledcAttach(RGB_B_PIN, RGB_FREQ, 8);
  setRGB(0, 0, 60);
  Serial.printf("[PIN] RGB LED hazir → R:GPIO%d G:GPIO%d B:GPIO%d\n", RGB_R_PIN, RGB_G_PIN, RGB_B_PIN);

  // WiFi — once ev agi, bulamazsa telefon hotspot
  struct NetCred { const char* ssid; const char* pass; };
  NetCred nets[] = {
    {WIFI_SSID_1, WIFI_PASS_1},
    {WIFI_SSID_2, WIFI_PASS_2},
  };
  bool staOk = false;
  for (int n = 0; n < 2 && !staOk; n++) {
    WiFi.disconnect(true);
    delay(200);
    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.printf("[WiFi] Deneniyor: %s\n", nets[n].ssid);
    WiFi.begin(nets[n].ssid, nets[n].pass);
    for (int t = 0; t < 20 && WiFi.status() != WL_CONNECTED; t++) {
      delay(500); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      staOk = true;
      Serial.printf("[WiFi] Baglandi: %s  IP: %s\n", nets[n].ssid, WiFi.localIP().toString().c_str());
    } else {
      WiFi.disconnect(true);
      delay(500);
    }
  }
  // AP her zaman ac — STA ile birlikte veya tek basina
  WiFi.mode(staOk ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[WiFi] AP: %s  → %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  if (!staOk) Serial.println("[WiFi] Hicbir STA agi bulunamadi");


  // mDNS
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http","tcp",80);
    MDNS.addService("ws","tcp",81);
    Serial.printf("[mDNS] http://%s.local\n", MDNS_NAME);
  }

  setupHttpServer();

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);
  Serial.println("[WS] :81 hazır");

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[ESP-NOW] A3 dinlemede");
  }

  scanWifiNetworks();
  triggerAlarm(2);
  // Radar pin debug
  for(int d=0;d<4;d++) Serial.printf("[RADAR INIT] idx=%d TRIG=%d ECHO=%d enabled=%d\n", d, trigPins[d], echoPins[d], radarEnabled[d]);

  Serial.println("[VIGILANT-X] ✓ Karakol aktif!");
  Serial.printf("  Arayüz: http://%s.local  veya  http://%s\n",
                MDNS_NAME, WiFi.softAPIP().toString().c_str());
}

// ═══════════════════════════════════════════════════════════
//  TAKİP MODU — Hibrit: 30cm içinde takip, dışında otonom tarama
//
//  0-30cm arası → takip:
//    dist=30cm → 90° (merkez)
//    dist=0cm  → SERVO_MIN (maksimum saptırma)
//    Bu aralık 90 dereceye sıkıştırılır → küçük mesafe farkı = büyük açı hareketi
//
//  30cm üstü → otonom ileri-geri tarama (hedef yok, ara)
// ═══════════════════════════════════════════════════════════
#define TRACK_RANGE_CM  30      // Bu mesafenin altında takip devreye girer
unsigned long lastTrackUpdate = 0;

void trackTick() {
  if (millis() - lastTrackUpdate < TRACK_UPDATE_MS) return;
  lastTrackUpdate = millis();

  for (int i = 0; i < 4; i++) {
    if (!unitEnabled[i]) continue;

    float dist = distCm[i];
    float targetAngle;

    if (dist <= TRACK_RANGE_CM) {
      // ── Takip bölgesi (0-30cm) ──────────────────────────
      // dist=TRACK_RANGE_CM → 90°, dist=0 → SERVO_MIN
      // ratio: 0(uzak uç=30cm) → 1(tam yakın=0cm)
      float ratio = 1.0f - (dist / (float)TRACK_RANGE_CM);
      ratio = constrain(ratio, 0.0f, 1.0f);
      targetAngle = 110.0f + ratio * (180.0f - 140.0f);
    } else {
      // ── Otonom tarama bölgesi (30cm üstü) ───────────────
      autoScanAngle[i] += AUTO_SCAN_SPEED * autoScanDir[i];
      if (autoScanAngle[i] >= SERVO_MAX) { autoScanAngle[i] = SERVO_MAX; autoScanDir[i] = -1; }
      if (autoScanAngle[i] <= SERVO_MIN) { autoScanAngle[i] = SERVO_MIN; autoScanDir[i] =  1; }
      targetAngle = autoScanAngle[i];
    }

    // Yumuşatma filtresi — her iki durumda da çalışır
    trackSmoothAngle[i] += TRACK_SMOOTH * (targetAngle - trackSmoothAngle[i]);
    setServoAngle(i, (int)trackSmoothAngle[i]);
  }
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  httpServer.handleClient();
  wsServer.loop();

  unsigned long now = millis();

  // ── Ses modu aktifse: radar/servo/broadcast DURAKSATILIR ───
  // Tarayici "voice_start" gonderdi mi? O zaman sadece ses oku ve gonder.
  if (voiceMode) {
    if (now - lastVoiceSend >= VOICE_SEND_INTERVAL) {
      lastVoiceSend = now;
      // İlk 4 byte: timestamp (millis), kalan 128*2=256 byte: ses
      uint8_t vpkt[4 + VOICE_SAMPLE_COUNT * 2];
      uint32_t ts = millis();
      vpkt[0] = (ts >> 24) & 0xFF;
      vpkt[1] = (ts >> 16) & 0xFF;
      vpkt[2] = (ts >>  8) & 0xFF;
      vpkt[3] = (ts      ) & 0xFF;
      uint16_t* vsamples = (uint16_t*)(vpkt + 4);
      for (int i = 0; i < VOICE_SAMPLE_COUNT; i++) {
        vsamples[i] = (uint16_t)analogRead(MIC_ADC);
        delayMicroseconds(141); // 6600Hz hedef: 1000000/6600 ≈ 151µs - ~10µs analogRead = 141µs
      }
      wsServer.broadcastBIN(vpkt, sizeof(vpkt));
    }
    return;  // Ses modunda diger hicbir sey calismiyor
  }

  // ── Normal calisma ─────────────────────────────────────────
  radarTick();  // Her loop'ta 1 sensör adımı

  // Sensör okuması
  if (now - lastSensorRead >= SCAN_INTERVAL_MS) {
    lastSensorRead = now;
    readVibrations();
    micLevel = analogRead(MIC_ADC);

    if (redAlertMode) {
      float mn = distCm[0]; int mi = 0;
      for (int i=1;i<4;i++) if(distCm[i]<mn){mn=distCm[i];mi=i;}
      if (mn < ALERT_DIST_CM)
        for (int i=0;i<4;i++) setServoAngle(i, i==mi?90:45);
    }
  }

  // WS yayını (250ms)
  if (now - lastWsBroadcast >= WS_BROADCAST_MS) {
    lastWsBroadcast = now;
    String _js=buildStatusJson(); wsServer.broadcastTXT(_js);
    if (!redAlertMode && !lradActive) {
      for (int i=0;i<4;i++) {
        if (radarEnabled[i] && distCm[i] < ALERT_DIST_CM) {
          ledcWriteTone(BUZZER_PIN, 880);
          unsigned long bt = millis();
          while(millis()-bt < 20) { wsServer.loop(); }
          ledcWriteTone(BUZZER_PIN, 0);
          break;
        }
      }
    }
  }

  // WiFi tarama (30s) — otonom modda yapma
  if (servoMode == 0 && (now - lastWifiScan >= WIFI_SCAN_INTERVAL)) {
    lastWifiScan = now;
    scanWifiNetworks();
  }

  // NeoPixel güncelleme
  updateNeoPixel();

  // Otonom mod (~25Hz) — sadece serbest tarama, radar takibi yok
  if (servoMode == 1 && (now - lastAutoUpdate >= AUTO_UPDATE_MS)) {
    lastAutoUpdate = now;
    for (int i=0;i<4;i++) {
      if (!unitEnabled[i]) continue;
      autoScanAngle[i] += AUTO_SCAN_SPEED * autoScanDir[i];
      if (autoScanAngle[i] >= SERVO_MAX) { autoScanAngle[i]=SERVO_MAX; autoScanDir[i]=-1; }
      if (autoScanAngle[i] <= SERVO_MIN) { autoScanAngle[i]=SERVO_MIN; autoScanDir[i]= 1; }
      setServoAngle(i, (int)autoScanAngle[i]);
    }
  }

  // Takip modu (~20Hz) — her taret kendi radarının mesafesine göre açı ayarlar
  if (servoMode == 2) {
    trackTick();
  }
}
