#include <Wire.h>

const int SDA_PIN = 4;   // prøv (8,9) først. Alternativt (0,1) hvis du bruker M5 Port A.
const int SCL_PIN = 5;
const uint8_t TCA_ADDR = 0x70;

void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void scanBus(const char* label){
  Serial.println(label);
  for (uint8_t addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0){
      Serial.print("  found 0x"); Serial.println(addr, HEX);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);          // 100 kHz for robusthet først

  Serial.println("Scanning root bus:");
  scanBus("Root:");
  // Forvent å se 0x70 her

  // Skann hver TCA-kanal for MPU6050 (0x68)
  for (uint8_t ch = 0; ch < 8; ch++){
    tcaSelect(ch);
    Serial.print("Channel "); Serial.println(ch);
    scanBus("  devices:");
  }
  // Sett tilbake til ingen kanal aktiv
  tcaSelect(0xFF);
}

void loop(){}
