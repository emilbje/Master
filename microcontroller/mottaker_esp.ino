#include <WiFi.h>
#include <esp_now.h>

// ------------------ ESP-NOW Mottaker + "z"-videresending ------------------

// Vi lagrer MAC-adressen til senderen når vi mottar første pakke
uint8_t senderMac[6];
bool senderKnown = false;

// Mottaks-callback (ny signatur for esp32 core / IDF 5.x)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  // Send alt videre til PC via USB-serial, uendret
  if (data_len > 0) {
    Serial.write(data, data_len);
  }

  // Hvis vi ikke allerede har registrert en peer, gjør det nå
  if (!senderKnown && info != nullptr) {
    memcpy(senderMac, info->src_addr, 6);
    senderKnown = true;

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, senderMac, 6);
    peerInfo.channel = 0;       // 0 = samme som WiFi STA
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.print("Registrerte ESP-NOW peer (sender MAC): ");
      for (int i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        Serial.print(senderMac[i], HEX);
      }
      Serial.println();
    } else {
      Serial.println("Klarte ikke å legge til ESP-NOW peer (sender)!");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init feilet");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("ESP-NOW mottaker klar.");
  Serial.println("Skriv 'z' i Serial for å sende 'z' til sender (rekalibrering).");
}

void loop() {
  // Lytt på USB-serial fra PC
  while (Serial.available() > 0) {
    int c = Serial.read();

    if (c == 'z' || c == 'Z') {
      if (senderKnown) {
        uint8_t msg = 'z';
        esp_err_t result = esp_now_send(senderMac, &msg, 1);
        if (result != ESP_OK) {
          Serial.print("Feil ved sending av 'z' til sender: ");
          Serial.println(result);
        } else {
          Serial.println("Sendte 'z' til sender (rekalibrer).");
        }
      } else {
        Serial.println("Ingen ESP-NOW-avsender registrert enda (har ikke mottatt data).");
      }
    }

    // (ev. håndter andre tegn her hvis du vil)
  }

  delay(10);
}

