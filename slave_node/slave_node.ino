#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "KalmanFilter.h"
#include "../common/common.h"

// ===== Versi Arduino Core guard (untuk kompatibilitas v2/v3) =====
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Pada core v3, recv-callback pakai esp_now_recv_info
  static inline const uint8_t* recv_src_mac(const esp_now_recv_info* info) {
    return info ? info->src_addr : nullptr;
  }
#endif

// === KONFIGURASI UNIK PER SLAVE ===
#define SLAVE_ID 1  // ganti 1..7 di tiap board

// === Pin & kalibrasi sensor ===
const int SOIL_PIN = 34;
const int ADC_DRY = 3200;  // kalibrasi!
const int ADC_WET = 1200;  // kalibrasi!

// === Timing ===
// Lebih longgar untuk koeksistensi & multi-slave
const unsigned long SAMPLE_PERIOD_MS       = 2000; // period kirim per slave
const unsigned long BEACON_WAIT_MS         = 250;  // waktu dengar per channel saat scan
const unsigned long BEACON_VALID_WINDOW_MS = 6000; // jika >6s tak dengar beacon -> boleh rescan
const unsigned long ACK_TIMEOUT_MS         = 200;  // tunggu ACK sekali kirim
const int           MAX_RETRY              = 5;    // max retransmit per sampel
const int           MAX_CONSECUTIVE_LOST   = 10;   // jika berkali2 gagal + beacon hilang, rescan

// TDMA-lite: setiap slave punya slot sendiri agar tidak tabrakan
const unsigned long SLOT_MS = 120; // pastikan NUM_SLAVES * SLOT_MS < SAMPLE_PERIOD_MS

// === State ===
uint8_t MASTER_MAC[6] = {0};   // diisi setelah terima beacon
uint8_t MASTER_CHANNEL = 1;    // diisi setelah terima beacon
bool    linked = false;

KalmanFilter kf;
uint32_t seqCounter = 0;
int lostStreak = 0;

volatile bool acked = false;
volatile uint32_t ackSeq = 0;
volatile uint32_t lastBeaconSeenMs = 0;  // kapan terakhir beacon terdengar

// Penjadwalan slot
unsigned long nextSendMs = 0;

// --- PRNG sederhana untuk jitter (xorshift-like) ---
static uint32_t prng_state = 0xA5A5A5A5u ^ (uint32_t)SLAVE_ID;
static inline uint32_t prng16() {
  prng_state ^= prng_state << 13;
  prng_state ^= prng_state >> 17;
  prng_state ^= prng_state << 5;
  return prng_state & 0xFFFFu;
}

float adcToPercent(int adc) {
  if (adc < 0) adc = 0;
  if (adc > 4095) adc = 4095;
  float pct = 100.0f * ((float)ADC_DRY - (float)adc) / ((float)ADC_DRY - (float)ADC_WET);
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return pct;
}

// Jadwalkan pengiriman berikutnya berdasarkan period + slot + jitter
void scheduleNextSend(unsigned long nowMs) {
  // periode dasar selaras dengan SAMPLE_PERIOD_MS
  unsigned long periodStart = (nowMs / SAMPLE_PERIOD_MS) * SAMPLE_PERIOD_MS;
  unsigned long slotOffset  = (unsigned long)(SLAVE_ID - 1) * SLOT_MS;

  // jitter kecil 0..14 ms untuk hindari perfect sync
  uint32_t jitter = prng16() % 15;

  unsigned long candidate = periodStart + slotOffset + jitter;

  if (candidate <= nowMs) {
    // jika slot periode ini sudah lewat, jadwalkan periode berikutnya
    candidate = periodStart + SAMPLE_PERIOD_MS + slotOffset + jitter;
  }

  nextSendMs = candidate;
}

// ===== RECV CALLBACK (kompatibel v2 & v3) =====
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  const uint8_t* mac = recv_src_mac(info);
#else
void onRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len < 1) return;
  uint8_t type = incomingData[0];

  if (type == PKT_BEACON) {
    if (len == sizeof(BeaconPacket)) {
      BeaconPacket b; memcpy(&b, incomingData, sizeof(b));
      lastBeaconSeenMs = millis();

      // Saat belum linked, lakukan locking channel + add peer
      if (!linked) {
        MASTER_CHANNEL = b.channel;
        memcpy(MASTER_MAC, b.masterMac, 6);

        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(MASTER_CHANNEL, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);

        esp_now_peer_info_t p{}; memcpy(p.peer_addr, MASTER_MAC, 6);
        p.channel = MASTER_CHANNEL; p.encrypt = false;
        esp_now_add_peer(&p);

        linked = true;
        Serial.printf("[Slave %d] Linked to master ch=%u MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
          SLAVE_ID, MASTER_CHANNEL,
          MASTER_MAC[0],MASTER_MAC[1],MASTER_MAC[2],MASTER_MAC[3],MASTER_MAC[4],MASTER_MAC[5]);

        // Reset scheduler saat link terbentuk
        scheduleNextSend(millis());
      }
    }
    return;
  }

  if (type == PKT_ACK && linked) {
    if (len == sizeof(AckPacket)) {
      AckPacket a; memcpy(&a, incomingData, sizeof(a));
      if (a.slaveId == SLAVE_ID) {
        acked = true; ackSeq = a.seq;
      }
    }
  }
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  // tidak digunakan (ACK diproses via onRecv)
}

void initEspNowOnChannel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // penting: hindari modem sleep
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // opsional: dorong margin SNR

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // re-init bersih
  esp_now_deinit();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    delay(500); ESP.restart();
  }

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  // Peer broadcast (untuk kirim/terima beacon)
  esp_now_peer_info_t b{}; memcpy(b.peer_addr, ESPNOW_BROADCAST_ADDR, 6);
  b.channel = ch; b.encrypt = false;
  esp_now_add_peer(&b);
}

void enterScanMode() {
  linked = false;
  memset(MASTER_MAC, 0, sizeof(MASTER_MAC));
  lastBeaconSeenMs = 0;

  // Non-rekursif: loop sampai linked
  while (!linked) {
    for (uint8_t ch=1; ch<=13 && !linked; ch++) {
      initEspNowOnChannel(ch);
      Serial.printf("[Slave %d] Scanning channel %u ...\n", SLAVE_ID, ch);
      unsigned long t0 = millis();
      while (!linked && (millis() - t0 < BEACON_WAIT_MS)) {
        delay(1); // biar callback jalan
      }
    }
    if (!linked) {
      Serial.printf("[Slave %d] Beacon not found. Retry...\n", SLAVE_ID);
      delay(500);
    }
  }
}

bool sendWithAck(const SoilPacket& pkt) {
  acked = false;
  esp_err_t rs = esp_now_send(MASTER_MAC, (const uint8_t*)&pkt, sizeof(pkt));
  if (rs != ESP_OK) return false;

  unsigned long t0 = millis();
  while (millis() - t0 < ACK_TIMEOUT_MS) {
    if (acked && ackSeq == pkt.seq) return true;
    delay(1);
  }
  return false;
}

void setup() {
  Serial.begin(115200); delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);
  pinMode(SOIL_PIN, INPUT);

  // mulai scan beacon (mulai dari ch=1)
  initEspNowOnChannel(1);
  enterScanMode();

  // seed PRNG sedikit unik per board
  prng_state ^= (uint32_t)millis();
  scheduleNextSend(millis());
}

void loop() {
  if (!linked) {
    enterScanMode();
    return;
  }

  unsigned long now = millis();

  // Jika beacon tidak terdengar terlalu lama dan kita juga sering gagal, barulah rescan
  bool beaconStale = lastBeaconSeenMs == 0 || (now - lastBeaconSeenMs) > BEACON_VALID_WINDOW_MS;
  if (beaconStale && lostStreak >= MAX_CONSECUTIVE_LOST) {
    Serial.printf("[Slave %d] Lost too many & no beacon. Resync...\n", SLAVE_ID);
    lostStreak = 0;
    enterScanMode();
    scheduleNextSend(millis());
    return;
  }

  // Penjadwalan TDMA-lite: kirim hanya saat masuk slot & waktu terjadwal
  if (now < nextSendMs) return;

  // Baca sensor & proses
  int raw = analogRead(SOIL_PIN);
  float kal = kf.update((float)raw);
  float pct = adcToPercent(raw);

  Serial.printf("ID=%d RAW=%d KAL=%.2f PCT=%.2f%%\n", SLAVE_ID, raw, kal, pct);

  SoilPacket pkt{};
  pkt.type      = PKT_DATA;
  pkt.slaveId   = SLAVE_ID;
  pkt.raw       = (uint16_t)raw;
  pkt.kalman    = kal;
  pkt.percent   = pct;
  pkt.millis_ts = now;
  pkt.seq       = ++seqCounter;

  bool ok = false;
  for (int i=0; i<=MAX_RETRY; i++){
    ok = sendWithAck(pkt);
    if (ok) break;
    // exponential backoff mini: 5ms,10ms,20ms,40ms,80ms...
    delay(5 * (1 << i));
  }

  if (!ok) {
    lostStreak++;
    Serial.printf("[Slave %d] No ACK (seq=%lu) streak=%d\n",
                  SLAVE_ID, (unsigned long)pkt.seq, lostStreak);
  } else {
    lostStreak = 0;
  }

  // Jadwalkan pengiriman periode berikutnya (TDMA + jitter)
  scheduleNextSend(now + 1);
}
