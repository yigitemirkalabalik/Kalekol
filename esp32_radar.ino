/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║           ESP32 RADAR SİSTEMİ                           ║
 * ║  HC-SR04 + Servo + 6 LED + WebSocket Arayüzü            ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * Gerekli Kütüphaneler (Arduino IDE → Kütüphane Yöneticisi):
 *   - ESP32Servo
 *   - ESPAsyncWebServer  (me-no-dev)
 *   - AsyncTCP           (me-no-dev)
 *
 * Bağlantılar:
 *   Servo  Signal → GPIO 13
 *   TRIG          → GPIO 18
 *   ECHO          → GPIO 19  (voltaj bölücü ile!)
 *   LED 1 (sol)   → GPIO 27
 *   LED 2         → GPIO 26
 *   LED 3         → GPIO 25
 *   LED 4         → GPIO 33
 *   LED 5         → GPIO 32
 *   LED 6 (sağ)   → GPIO 23
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

// ─── WiFi ────────────────────────────────────────────────────
const char* SSID     = "WIFI_ADINIZ";
const char* PASSWORD = "WIFI_SIFRENIZ";

// ─── PİNLER ──────────────────────────────────────────────────
const int SERVO_PIN       = 13;
const int TRIG_PIN        = 18;
const int ECHO_PIN        = 19;
const int LED_PINS[6]     = {27, 26, 25, 33, 32, 23}; // sol → sağ

// ─── RADAR AYARLARI ──────────────────────────────────────────
const int   STEP_DELAY      = 20;   // ms, her 1° adım arası (azalt = hızlı tarama)
const int   DETECTION_LIMIT = 40;   // cm, bu mesafenin ötesi yok sayılır
const int   SAMPLE_COUNT    = 5;    // Filtreleme için ölçüm sayısı

// ─── LED MESAFE EŞİKLERİ ─────────────────────────────────────
const int LED_CLOSE  = 15;   // cm altı  → LED yanar
const int LED_MEDIUM = 40;  // cm altı  → LED yanar (> 40cm → yok sayılır)

// ─── LED AÇI ZONEları (sol → sağ, her LED 30° aralık) ────────
// LED 0 (GPIO 27): 150°–180°  (sol uç)
// LED 1 (GPIO 26): 120°–150°
// LED 2 (GPIO 25):  90°–120°
// LED 3 (GPIO 33):  60°– 90°
// LED 4 (GPIO 32):  30°– 60°
// LED 5 (GPIO 23):   0°– 30°  (sağ uç)
const int LED_ZONE_MIN[6] = {150, 120, 90, 60, 30,  0};
const int LED_ZONE_MAX[6] = {180, 150, 120, 90, 60, 30};

// ─────────────────────────────────────────────────────────────

Servo myServo;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

float currentAngle = 0.0;
int   sweepDir     = 1;

// ─── FİLTRELİ MESAFE ─────────────────────────────────────────
long readDistanceCm() {
  long samples[SAMPLE_COUNT];
  int  valid = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 25000);
    if (dur > 0) samples[valid++] = dur / 58;
    delay(4);
  }

  if (valid == 0) return -1;

  // Bubble sort → ortanca değer
  for (int i = 0; i < valid - 1; i++)
    for (int j = 0; j < valid - i - 1; j++)
      if (samples[j] > samples[j + 1]) {
        long t = samples[j]; samples[j] = samples[j + 1]; samples[j + 1] = t;
      }

  return samples[valid / 2]; // medyan
}

// ─── LED KONTROL ─────────────────────────────────────────────
// Her LED yalnızca kendi 30°'lik zone'unda nesne varsa yanar.
// Mesafe < LED_CLOSE  → LED yanar
// Mesafe < LED_MEDIUM → LED yanar
// Mesafe >= LED_MEDIUM veya tespit yok → LED söner
void updateLEDs(float angle, long dist) {
  bool detected = (dist > 0 && dist <= LED_MEDIUM);

  for (int i = 0; i < 6; i++) {
    bool inZone = detected &&
                  ((int)angle >= LED_ZONE_MIN[i]) &&
                  ((int)angle <  LED_ZONE_MAX[i]);
    digitalWrite(LED_PINS[i], inZone ? HIGH : LOW);
  }
}




// ─── HTML ARAYÜZÜ (PROGMEM) ──────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Radar</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #050f05;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    min-height: 100vh;
    font-family: 'Courier New', monospace;
    color: #00ff41;
  }
  h1 {
    font-size: 14px;
    letter-spacing: 4px;
    color: #00aa29;
    margin-bottom: 12px;
    text-transform: uppercase;
  }
  canvas { display: block; }
  #info {
    margin-top: 14px;
    font-size: 15px;
    letter-spacing: 2px;
    color: #00cc33;
    min-height: 22px;
  }
  #status {
    margin-top: 6px;
    font-size: 11px;
    color: #006614;
    letter-spacing: 1px;
  }
  .dot-blink {
    animation: blink 1s infinite;
  }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }
</style>
</head>
<body>
<h1>◈ ESP32 RADAR SİSTEMİ</h1>
<canvas id="radar" width="500" height="280"></canvas>
<div id="info">Bağlanıyor<span class="dot-blink">...</span></div>
<div id="status">IP: <span id="ip">—</span> &nbsp;|&nbsp; WebSocket: <span id="wsstate">BEKLEMEDE</span></div>

<script>
const canvas = document.getElementById('radar');
const ctx    = canvas.getContext('2d');

// Yarı daire (servo 0-180°)
const cx = 250, cy = 268, R = 240;
const MAX_DIST  = 40;
const FADE_MS   = 4000;

let curAngle    = 0;
let curDist     = 0;
let detections  = [];

function angleToXY(deg, dist) {
  // 0° = sağ, 90° = yukarı, 180° = sol
  const rad = (180 - deg) * Math.PI / 180;
  const dr  = Math.min(dist, MAX_DIST) / MAX_DIST * R;
  return {
    x: cx + dr * Math.cos(rad),
    y: cy - dr * Math.sin(rad)
  };
}

function sweepXY(deg) {
  const rad = (180 - deg) * Math.PI / 180;
  return {
    x: cx + R * Math.cos(rad),
    y: cy - R * Math.sin(rad)
  };
}

function draw() {
  ctx.clearRect(0, 0, 500, 280);

  // ── Arka plan yarım daire ────────────────────────────────
  ctx.beginPath();
  ctx.arc(cx, cy, R, Math.PI, 0, false);
  ctx.lineTo(cx, cy);
  ctx.closePath();
  ctx.fillStyle = '#020d02';
  ctx.fill();

  // ── Izgara halkaları ─────────────────────────────────────
  for (let i = 1; i <= 4; i++) {
    ctx.beginPath();
    ctx.arc(cx, cy, R * i / 4, Math.PI, 0, false);
    ctx.strokeStyle = i === 4 ? '#004d00' : '#002800';
    ctx.lineWidth = i === 4 ? 1.5 : 1;
    ctx.stroke();
    // Mesafe etiketi
    ctx.fillStyle = '#005500';
    ctx.font = '10px Courier New';
    ctx.fillText(`${MAX_DIST * i / 4}cm`, cx + R * i / 4 + 3, cy - 3);
  }

  // ── Açı çizgileri ────────────────────────────────────────
  for (let a = 0; a <= 180; a += 30) {
    const rad = (180 - a) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + R * Math.cos(rad), cy - R * Math.sin(rad));
    ctx.strokeStyle = '#002200';
    ctx.lineWidth = 1;
    ctx.stroke();
    // Açı etiketi
    const lx = cx + (R + 14) * Math.cos(rad);
    const ly = cy - (R + 14) * Math.sin(rad);
    ctx.fillStyle = '#005500';
    ctx.font = '10px Courier New';
    ctx.textAlign = 'center';
    ctx.fillText(`${a}°`, lx, ly + 4);
  }
  ctx.textAlign = 'left';

  // ── Taban çizgisi ────────────────────────────────────────
  ctx.beginPath();
  ctx.moveTo(cx - R, cy);
  ctx.lineTo(cx + R, cy);
  ctx.strokeStyle = '#004d00';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // ── Tespit noktaları (fade out) ──────────────────────────
  const now = Date.now();
  detections = detections.filter(d => now - d.t < FADE_MS);
  detections.forEach(d => {
    const age   = (now - d.t) / FADE_MS;
    const alpha = Math.pow(1 - age, 1.5);
    const p     = angleToXY(d.angle, d.dist);

    // Dış parlama
    const grd = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, 10);
    grd.addColorStop(0, `rgba(255,30,0,${alpha * 0.6})`);
    grd.addColorStop(1, `rgba(255,0,0,0)`);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
    ctx.fillStyle = grd;
    ctx.fill();

    // Merkez nokta
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = `rgba(255,60,0,${alpha})`;
    ctx.fill();
  });

  // ── Sweep çizgisi ────────────────────────────────────────
  const sp  = sweepXY(curAngle);
  const grd = ctx.createLinearGradient(cx, cy, sp.x, sp.y);
  grd.addColorStop(0,   'rgba(0,255,65,0.9)');
  grd.addColorStop(0.6, 'rgba(0,255,65,0.3)');
  grd.addColorStop(1,   'rgba(0,255,65,0)');
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(sp.x, sp.y);
  ctx.strokeStyle = grd;
  ctx.lineWidth   = 2.5;
  ctx.stroke();

  // Sweep izi (soluk koni efekti)
  for (let trail = 1; trail <= 12; trail++) {
    const ta   = curAngle - trail * 1.5;
    if (ta < 0 || ta > 180) continue;
    const tp   = sweepXY(ta);
    const alp  = (1 - trail / 12) * 0.08;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tp.x, tp.y);
    ctx.strokeStyle = `rgba(0,200,50,${alp})`;
    ctx.lineWidth   = 2;
    ctx.stroke();
  }

  // ── Merkez nokta ─────────────────────────────────────────
  ctx.beginPath();
  ctx.arc(cx, cy, 4, 0, Math.PI * 2);
  ctx.fillStyle = '#00ff41';
  ctx.fill();

  requestAnimationFrame(draw);
}

// ── WebSocket ──────────────────────────────────────────────
function connect() {
  const wsUrl = 'ws://' + location.host + '/ws';
  document.getElementById('wsstate').textContent = 'BAĞLANIYOR';
  const sock = new WebSocket(wsUrl);

  sock.onopen = () => {
    document.getElementById('wsstate').textContent = 'BAĞLI ●';
    document.getElementById('info').innerHTML      = 'Sinyal bekleniyor...';
    document.getElementById('ip').textContent      = location.hostname;
  };

  sock.onmessage = (e) => {
    const d = JSON.parse(e.data);
    curAngle = d.angle;
    curDist  = d.distance;

    if (curDist > 0 && curDist <= MAX_DIST) {
      detections.push({ angle: curAngle, dist: curDist, t: Date.now() });
      document.getElementById('info').textContent =
        `AÇI: ${curAngle}°  |  MESAFE: ${curDist} cm`;
    } else {
      document.getElementById('info').textContent =
        `AÇI: ${curAngle}°  |  MESAFE: ---`;
    }
  };

  sock.onclose = () => {
    document.getElementById('wsstate').textContent = 'KESİLDİ';
    setTimeout(connect, 2000); // Otomatik yeniden bağlan
  };

  sock.onerror = () => sock.close();
}

connect();
draw();
</script>
</body>
</html>
)rawliteral";

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 RADAR BAŞLIYOR ===");

  // LED pinleri
  for (int i = 0; i < 6; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  // LED başlangıç testi: soldan sağa
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PINS[i], HIGH); delay(80); }
  for (int i = 0; i < 6; i++) { digitalWrite(LED_PINS[i], LOW);  delay(60); }

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(0);
  delay(300);

  // WiFi bağlantısı
  Serial.print("WiFi bağlanıyor: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✓ Bağlandı! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("✗ WiFi bağlantısı başarısız!");
  }

  // WebSocket & HTTP sunucu
  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* c,
                AwsEventType t, void*, uint8_t*, size_t) {
    if (t == WS_EVT_CONNECT)    Serial.println("WS: Client bağlandı");
    if (t == WS_EVT_DISCONNECT) Serial.println("WS: Client ayrıldı");
  });

  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();

  // IP adresi — setup bittikten sonra 3 kez tekrar yazdır, kaybetme
  String ip = WiFi.localIP().toString();
  for (int i = 0; i < 3; i++) {
    Serial.println("════════════════════════════════════");
    Serial.println("  >>> Tarayıcıda aç: http://" + ip);
    Serial.println("════════════════════════════════════");
    delay(200);
  }
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  ws.cleanupClients();

  // Mesafe ölç
  long dist = readDistanceCm();
  if (dist < 0 || dist > DETECTION_LIMIT) dist = 0;

  // LED güncelle
  updateLEDs(currentAngle, dist);

  // WebSocket veri gönder
  if (ws.count() > 0) {
    String json = "{\"angle\":" + String((int)currentAngle) +
                  ",\"distance\":" + String(dist) + "}";
    ws.textAll(json);
  }

  // Servo adım
  myServo.write((int)currentAngle);
  currentAngle += sweepDir;
  if (currentAngle >= 180) { currentAngle = 180; sweepDir = -1; }
  if (currentAngle <= 0)   { currentAngle = 0;   sweepDir =  1; }

  delay(STEP_DELAY);
}
