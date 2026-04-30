// ===== ESP-NOW Receiver (ESP32-C3, Arduino core 3.x / IDF 5.x) =====
#include <WiFi.h>
#include <esp_now.h>

struct Payload {
  uint32_t seq;
  uint16_t adc;
  float volts;
  float bend_pct;
};

volatile uint32_t pktCount = 0;

void onRecv(const esp_now_recv_info* info, const uint8_t* incoming, int len) {
  if (!info || !incoming) return;
  if (len == sizeof(Payload)) {
    Payload p;
    memcpy(&p, incoming, sizeof(Payload));
    pktCount++;
    // (valgfritt) hent MAC: info->src_addr (6 bytes) 
    Serial.printf("SEQ:%lu\tADC:%u\tV:%.3f\tBendPct:%.1f\n",
                  (unsigned long)p.seq, p.adc, p.volts,
                  isnan(p.bend_pct) ? 0.0f : p.bend_pct);
  } else {
    Serial.printf("RX len=%d (unexpected)\n", len);
  }
}

void setup() {
  Serial.begin(115200);   
  delay(200);
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress()); // lim inn i sender

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while (1) { delay(1000); }
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("ESP-NOW receiver ready.");
}

void loop() {
  static uint32_t t0 = millis();
  if (millis() - t0 > 5000) {
    t0 = millis();
    Serial.printf("[status] packets=%lu\n", (unsigned long)pktCount);
  }
}
