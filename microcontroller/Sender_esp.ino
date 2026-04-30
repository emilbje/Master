#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// ------------------ ESP-NOW -------------------
uint8_t peerAddress[] = { 0x60, 0x55, 0xF9, 0x57, 0x4A, 0x60 }; // Mottakerens MAC

// Flag for re-nullpunkt via ESP-NOW
volatile bool reZeroRequested = false;

// Ny signatur for esp_now_register_send_cb i ESP-IDF 5.x / ny esp32-core
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Valgfritt debug
  // Serial.print("Send status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FEIL");
}

// ESP-NOW mottaks-callback: ser etter 'z' i payload
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  if (data_len <= 0 || data == nullptr) return;

  for (int i = 0; i < data_len; i++) {
    char c = (char)data[i];
    if (c == 'z' || c == 'Z') {
      reZeroRequested = true;
      // Valgfritt debug:
      // Serial.println("Mottok 'z' via ESP-NOW (rekalibrer).");
      break;
    }
  }
}

// ------------------ TCA9548A ------------------
const uint8_t TCA_ADDR = 0x70; // standardadresse
void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);      // velg kanal 0..7
  Wire.endTransmission();
}

// ------------------ MPU-6050 ------------------
const uint8_t MPU_ADDR = 0x68; // alle IMU'er kan stå på 0x68
// Registre
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_INT_ENABLE   = 0x38;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

const float ACCEL_LSB_PER_G  = 16384.0f; // ±2 g
const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps

// ------------------ Oppsett -------------------
const uint8_t NIMU = 5;
const uint8_t IMU_CH[NIMU] = {0,1,2,3,4};   // TCA-kanaler i bruk
const unsigned long PRINT_INTERVAL_MS = 50; // 20 Hz utskrift

// Komplementært filter
const float ALPHA = 0.98f;

// Tilstandsdata
float gx_bias[NIMU]={0}, gy_bias[NIMU]={0}, gz_bias[NIMU]={0};
float roll_fused[NIMU]={0}, pitch_fused[NIMU]={0};
float roll_zero[NIMU]={0}, pitch_zero[NIMU]={0};
bool  fused_init[NIMU]={false};

unsigned long last_us=0, last_print_ms=0;

// ------------------ Hjelpefunksjoner ----------
int16_t toI16(uint8_t hi, uint8_t lo){ 
  return (int16_t)((hi<<8)|lo); 
}

bool wreg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(reg); 
  Wire.write(val);
  return Wire.endTransmission()==0;
}

bool rburst(uint8_t startReg, uint8_t* buf, uint8_t len){
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(startReg);
  if (Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom((int)MPU_ADDR, (int)len);
  for(uint8_t i=0;i<len;i++){ 
    if(!Wire.available()) return false; 
    buf[i]=Wire.read(); 
  }
  return true;
}

bool initOneIMU(uint8_t ch){
  tcaSelect(ch);
  // WHO_AM_I (ikke strengt nødvendig, men greit å sjekke buss)
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false)!=0 || Wire.requestFrom((int)MPU_ADDR,1)!=1) return false;
  uint8_t who = Wire.read();
  if (who!=0x68 && who!=0x69) return false;

  bool ok=true;
  ok&=wreg(REG_PWR_MGMT_1,0x00); delay(50); // vekke
  ok&=wreg(REG_SMPLRT_DIV,9);               // 1 kHz/(1+9)=100 Hz
  ok&=wreg(REG_CONFIG,0x03);                // DLPF ~44 Hz
  ok&=wreg(REG_GYRO_CONFIG,0x00);           // ±250 dps
  ok&=wreg(REG_ACCEL_CONFIG,0x00);          // ±2 g
  ok&=wreg(REG_INT_ENABLE,0x00);
  return ok;
}

void calibrateGyro(uint8_t ch, unsigned long ms=1500){
  tcaSelect(ch);
  unsigned long endt = millis()+ms;
  long n=0; 
  double sx=0, sy=0, sz=0;
  uint8_t b[14];
  while(millis()<endt){
    if(rburst(REG_ACCEL_XOUT_H,b,14)){
      sx += (toI16(b[8],b[9])  / GYRO_LSB_PER_DPS);
      sy += (toI16(b[10],b[11])/ GYRO_LSB_PER_DPS);
      sz += (toI16(b[12],b[13])/ GYRO_LSB_PER_DPS);
      n++;
    }
    delay(5);
  }
  if(n>0){ 
    gx_bias[ch]=sx/n; 
    gy_bias[ch]=sy/n; 
    gz_bias[ch]=sz/n; 
  }
}

void stepIMU(uint8_t i, float dt){
  tcaSelect(IMU_CH[i]);
  uint8_t b[14];
  if(!rburst(REG_ACCEL_XOUT_H,b,14)) return;

  int16_t axr=toI16(b[0],b[1]), ayr=toI16(b[2],b[3]), azr=toI16(b[4],b[5]);
  int16_t gxr=toI16(b[8],b[9]),  gyr=toI16(b[10],b[11]);

  float ax = axr/ACCEL_LSB_PER_G;
  float ay = ayr/ACCEL_LSB_PER_G;
  float az = azr/ACCEL_LSB_PER_G;
  float gx = (gxr/GYRO_LSB_PER_DPS) - gx_bias[i];
  float gy = (gyr/GYRO_LSB_PER_DPS) - gy_bias[i];

  float roll_acc  = atan2(ay, az) * 180.0f/PI;
  float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f/PI;

  if(!fused_init[i]){
    roll_fused[i]  = roll_acc;
    pitch_fused[i] = pitch_acc;
    fused_init[i]  = true;
  }else{
    roll_fused[i]  = ALPHA*(roll_fused[i]  + gx*dt) + (1.0f-ALPHA)*roll_acc;
    pitch_fused[i] = ALPHA*(pitch_fused[i] + gy*dt) + (1.0f-ALPHA)*pitch_acc;
  }
}

// ------------------ SETUP ---------------------
void setup(){
  Serial.begin(115200);

  // ESP32-C3: WiFi + ESP-NOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init feilet");
  } else {
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);   // <-- NY!

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;    // samme kanal
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Kunne ikke legge til peer");
    }
  }

  // I2C på M5STAMP-C3: SDA=4, SCL=5
  Wire.begin(4, 5);
  // Wire.setClock(400000); // om du vil prøve 400 kHz
  delay(100);

  // Init og gyro-kalibrering for alle 5 (hold planken i ro)
  for(uint8_t i=0;i<NIMU;i++){
    if(!initOneIMU(IMU_CH[i])){
      Serial.print("Init-feil på IMU "); 
      Serial.println(i);
      while(1){} // stopp hvis en IMU feiler
    }
  }
  Serial.println("Kalibrerer gyro (hold i ro)...");
  for(uint8_t i=0;i<NIMU;i++) calibrateGyro(IMU_CH[i], 1500);
  Serial.println("OK. Trykk 'z' i Serial ELLER send 'z' via ESP-NOW for å sette 0° (alle IMU).");

  last_us = micros();
  last_print_ms = millis();
}

// ------------------ LOOP ----------------------
void loop(){
  bool doZero = false;

  // 'z' for nytt nullpunkt (alle) via USB-serial (om koblet til)
  if (Serial.available()) {
    char c = Serial.read();
    if(c=='z' || c=='Z'){
      doZero = true;
    }
  }

  // 'z' mottatt via ESP-NOW
  if (reZeroRequested) {
    reZeroRequested = false;
    doZero = true;
  }

  if (doZero) {
    for(uint8_t i=0;i<NIMU;i++){
      roll_zero[i]=roll_fused[i];
      pitch_zero[i]=pitch_fused[i];
    }
    Serial.println("Nullpunkt satt (alle IMU).");
  }

  unsigned long now_us = micros();
  float dt = (now_us - last_us)/1e6f; 
  last_us = now_us;

  for(uint8_t i=0;i<NIMU;i++) stepIMU(i, dt);

  if(millis() - last_print_ms >= PRINT_INTERVAL_MS){
    last_print_ms = millis();

    // Bygg linje i samme format som før: p0..p4 og r0..r4 (tab-separert)
    char line[200];
    size_t idx = 0;

    for(uint8_t i=0;i<NIMU;i++){
      float p = (pitch_fused[i] - pitch_zero[i]);
      idx += snprintf(line + idx, sizeof(line) - idx, "p%u:%.1f\t", i, p);
      if (idx >= sizeof(line)) break;
    }
    for(uint8_t i=0;i<NIMU;i++){
      float r = (roll_fused[i] - roll_zero[i]);
      if (i < NIMU-1) {
        idx += snprintf(line + idx, sizeof(line) - idx, "r%u:%.1f\t", i, r);
      } else {
        idx += snprintf(line + idx, sizeof(line) - idx, "r%u:%.1f", i, r);
      }
      if (idx >= sizeof(line)) break;
    }
    // legg til newline
    if (idx < sizeof(line)-2) {
      line[idx++] = '\n';
      line[idx]   = '\0';
    } else {
      line[sizeof(line)-2] = '\n';
      line[sizeof(line)-1] = '\0';
    }

    // Send over ESP-NOW
    esp_err_t result = esp_now_send(peerAddress, (uint8_t*)line, strlen(line));
    // Valgfritt: også skrive til Serial for debugging
    // Serial.print(line);
  }

  delay(2); // litt pusterom
}
