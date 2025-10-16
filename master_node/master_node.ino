#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "../common/common.h"

// ===== Versi Arduino Core guard (kompatibel v2/v3) =====
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Pada core v3, recv-callback pakai esp_now_recv_info (punya rx_ctrl: RSSI & channel)
  static inline const uint8_t* recv_src_mac(const esp_now_recv_info* info) {
    return info ? info->src_addr : nullptr;
  }
  // Beberapa header untuk tipe rx_ctrl
  #include "esp_wifi_types.h"
#endif

// === Wi-Fi creds ===
const char* WIFI_SSID = "Fxx";
const char* WIFI_PASS = "123456789";

// Server
WebServer http(80);
WebSocketsServer ws(81);

// State data
volatile uint16_t RAW[NUM_SLAVES];
volatile float    KAL[NUM_SLAVES];
volatile float    PCT[NUM_SLAVES];
volatile uint32_t SLAVE_MS[NUM_SLAVES];   // millis() di slave (terakhir)
volatile uint32_t SEQ[NUM_SLAVES];
volatile bool     HAS[NUM_SLAVES];

uint32_t lastSeenMs[NUM_SLAVES];          // millis() master saat terakhir terima
uint32_t lostCount[NUM_SLAVES];           // total "stale" hitungan health (opsional)

// Timing
unsigned long lastBroadcast = 0;
const unsigned long WS_PERIOD_MS     = 2000;
unsigned long lastBeacon   = 0;
const unsigned long BEACON_PERIOD_MS = 2000;

// Periodic serial summary
unsigned long lastSummary = 0;
const unsigned long SUMMARY_PERIOD_MS = 5000;

uint8_t masterMac[6];
uint8_t wifiChannel = 1;

// === HTML test (placeholder) ===
const char* TEST_HTML = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><title>Soil Monitor WS</title></head>
<body>
<h2>WebSocket Live Data</h2>
<pre id="log">connecting...</pre>
<script>
let log = document.getElementById('log');
let ws = new WebSocket(`ws://${location.hostname}:81/`);
ws.onopen=()=>log.textContent='connected\n';
ws.onmessage=(e)=>{
  try{ log.textContent = JSON.stringify(JSON.parse(e.data), null, 2); }
  catch{ log.textContent = e.data; }
};
ws.onclose=()=>log.textContent+='\nclosed';
</script></body></html>
)HTML";

void handleRoot(){ http.send(200, "text/html", TEST_HTML); }

// === Helpers ===
static inline void macToStr(const uint8_t mac[6], char* out /* >=18 */) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // stabilkan radio
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi connected. IP=%s\n", WiFi.localIP().toString().c_str());
  WiFi.macAddress(masterMac);
  wifiChannel = WiFi.channel(); // <— ambil channel AP
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // opsional tapi membantu
  Serial.printf("Master STA MAC: %02X:%02X:%02X:%02X:%02X:%02X ch=%u\n",
    masterMac[0],masterMac[1],masterMac[2],masterMac[3],masterMac[4],masterMac[5], wifiChannel);
}

// ===== RECV CALLBACK (kompatibel v2 & v3) =====
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  const uint8_t* mac = recv_src_mac(info);
  int rxChannel = -1;
  int rxRssi    = 0;
  if (info && info->rx_ctrl) {
    // rx_ctrl adalah wifi_pkt_rx_ctrl_t*
    rxChannel = info->rx_ctrl->channel;
    rxRssi    = info->rx_ctrl->rssi;
  }
#else
void onEspNowRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  int rxChannel = -1;  // tidak tersedia di core v2
  int rxRssi    = 0;   // tidak tersedia di core v2
#endif
  if (len < 1) return;
  uint8_t type = incomingData[0];

  if (type == PKT_DATA && len == sizeof(SoilPacket)) {
    SoilPacket pkt; memcpy(&pkt, incomingData, sizeof(pkt));
    int idx = (int)pkt.slaveId - 1;
    if (idx >= 0 && idx < NUM_SLAVES) {
      RAW[idx] = pkt.raw;
      KAL[idx] = pkt.kalman;
      PCT[idx] = pkt.percent;
      SLAVE_MS[idx] = pkt.millis_ts;
      SEQ[idx] = pkt.seq;
      HAS[idx]  = true;
      lastSeenMs[idx] = millis();

      // --- Serial monitoring detail per-paket ---
      char macbuf[18] = "??:??:??:??:??:??";
      if (mac) macToStr(mac, macbuf);
      Serial.printf("[RX] t=%lu ms | Slave=%d seq=%lu | RAW=%u KAL=%.2f PCT=%.2f%% | MAC=%s | APch=%u RXch=%d RSSI=%d dBm\n",
                    (unsigned long)lastSeenMs[idx],
                    pkt.slaveId, (unsigned long)pkt.seq,
                    (unsigned)pkt.raw, pkt.kalman, pkt.percent,
                    macbuf, (unsigned)wifiChannel, rxChannel, rxRssi);

      // Kirim ACK langsung ke pengirim (mac)
      if (mac) {
        AckPacket ack{};
        ack.type    = PKT_ACK;
        ack.slaveId = pkt.slaveId;
        ack.seq     = pkt.seq;
        esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
      }
    }
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);

  // Pastikan ESP-NOW berada pada channel AP aktif
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // re-init bersih
  esp_now_deinit();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED"); ESP.restart();
  }

  esp_now_register_recv_cb(onEspNowRecv);

  // Tambahkan peer broadcast untuk siarkan beacon
  esp_now_peer_info_t b{}; memcpy(b.peer_addr, ESPNOW_BROADCAST_ADDR, 6);
  b.channel = wifiChannel; b.encrypt = false;
  esp_now_add_peer(&b);
}

void sendBeacon() {
  BeaconPacket bp{};
  bp.type = PKT_BEACON;
  bp.channel = wifiChannel;
  memcpy(bp.masterMac, masterMac, 6);
  bp.masterUptimeMs = millis();
  esp_now_send(ESPNOW_BROADCAST_ADDR, (uint8_t*)&bp, sizeof(bp));
}

String makeJsonPayload() {
  // Tambah health: seq, slaveMillis, lastSeenMaster, ageMs, ok, lostCount
  // ok = data dianggap "fresh" jika diterima < 5*WS_PERIOD_MS
  uint32_t now = millis();
  String json = "{";

  json += "\"raw\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(RAW[i]); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"kal\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(KAL[i],2); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"pct\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(PCT[i],2); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"seq\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(SEQ[i]); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"slaveMillis\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(SLAVE_MS[i]); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"lastSeenMs\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(lastSeenMs[i]); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"ageMs\":[";
  for (int i=0;i<NUM_SLAVES;i++){
    uint32_t age = lastSeenMs[i] ? (now - lastSeenMs[i]) : 0xFFFFFFFF;
    json += String(age);
    if(i<NUM_SLAVES-1) json+=",";
  }
  json += "],\"ok\":[";
  for (int i=0;i<NUM_SLAVES;i++){
    bool ok = (lastSeenMs[i] && (now - lastSeenMs[i] < 5*WS_PERIOD_MS));
    json += ok ? "true":"false";
    if(i<NUM_SLAVES-1) json+=",";
  }
  json += "],\"lostCount\":[";
  for (int i=0;i<NUM_SLAVES;i++){ json += String(lostCount[i]); if(i<NUM_SLAVES-1) json+=","; }
  json += "],\"meta\":{";
  json += "\"masterIP\":\""+WiFi.localIP().toString()+"\",";
  json += "\"masterMac\":\"";
  char macbuf[18]; macToStr(masterMac, macbuf);
  json += macbuf; json += "\",";
  json += "\"channel\":"+String(wifiChannel)+",";
  json += "\"uptimeMs\":"+String(now);
  json += "}}";
  return json;
}

// Helper: send snapshot to a single WS client as lvalue String
inline void sendWsSnapshot(uint8_t num) {
  String payload = makeJsonPayload();
  ws.sendTXT(num, payload);  // takes String&
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = ws.remoteIP(num);
    Serial.printf("[WS] Client %u from %s\n", num, ip.toString().c_str());
    sendWsSnapshot(num);
  }
}

void setup() {
  Serial.begin(115200); delay(300);

  for (int i=0;i<NUM_SLAVES;i++){
    RAW[i]=0; KAL[i]=NAN; PCT[i]=NAN; SLAVE_MS[i]=0; SEQ[i]=0; HAS[i]=false;
    lastSeenMs[i]=0; lostCount[i]=0;
  }

  // 1) WiFi connect -> ambil channel
  connectWiFi();

  // 2) ESP-NOW di channel AP
  setupEspNow();

  // 3) HTTP + WS
  http.on("/", handleRoot);
  http.begin();

  ws.begin(); ws.onEvent(onWsEvent);

  Serial.println("Master ready (beacon + WS).");
}

void loop() {
  http.handleClient();
  ws.loop();

  unsigned long now = millis();

  // Beacon rutin (untuk discovery/resync slave)
  if (now - lastBeacon >= BEACON_PERIOD_MS) {
    lastBeacon = now;
    sendBeacon();
  }

  // Broadcast WebSocket payload (use lvalue String)
  if (now - lastBroadcast >= WS_PERIOD_MS) {
    lastBroadcast = now;
    String payload = makeJsonPayload();
    ws.broadcastTXT(payload);   // takes String&
  }

  // Periodic serial summary
  if (now - lastSummary >= SUMMARY_PERIOD_MS) {
    lastSummary = now;
    Serial.println("----- Summary: slaves status -----");
    Serial.println("ID | age(ms) | seq   | pct   | has");
    for (int i=0;i<NUM_SLAVES;i++){
      uint32_t age = lastSeenMs[i] ? (now - lastSeenMs[i]) : 0xFFFFFFFF;
      Serial.printf("%2d | %7lu | %5lu | %5.2f | %s\n",
        i+1, (unsigned long)age, (unsigned long)SEQ[i], PCT[i], HAS[i] ? "Y":"N");
      // contoh health counter: kalau terlalu lama stale, tambahkan lostCount
      if (age > 5*WS_PERIOD_MS && HAS[i]) {
        lostCount[i]++;
      }
    }
    Serial.println("----------------------------------");
  }
}
