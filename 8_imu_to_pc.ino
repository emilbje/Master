#include <Wire.h>

// ---------- PINNER / ADRESSER ----------
const int SDA_PIN = 4;       // endre til (8,9) eller (0,1) hvis du bruker andre pinner
const int SCL_PIN = 5;
const uint8_t TCA_ADDR = 0x70;   // TCA9548A
const uint8_t MPU_ADDR = 0x68;   // MPU6050 (0x69 hvis AD0=HIGH)

// ---------- MPU6050 REGISTRE ----------
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

const float ACCEL_LSB_PER_G  = 16384.0f; // ±2 g
const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps

// ---------- OPPSETT ----------
const uint8_t NIMU = 8;
const uint8_t IMU_CH[NIMU] = {0,1,2,3,4,5,6,7};
const unsigned long PRINT_INTERVAL_MS = 10; // ~20 Hz
const float ALPHA = 0.98f;                   // komplementært filter

// ---------- TILSTAND ----------
bool   imu_ok[NIMU]      = {false};
bool   fused_init[NIMU]  = {false};
float  gx_bias[NIMU]={0}, gy_bias[NIMU]={0}, gz_bias[NIMU]={0};
float  roll_fused[NIMU]={0}, pitch_fused[NIMU]={0};
float  roll_zero[NIMU]={0}, pitch_zero[NIMU]={0};
int    fail_cnt[NIMU]={0};

unsigned long last_us=0, last_print_ms=0;

// ---------- HJELP ----------
static inline int16_t toI16(uint8_t hi, uint8_t lo){ return (int16_t)((hi<<8)|lo); }

void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

bool wreg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool rburst(uint8_t startReg, uint8_t* buf, uint8_t len){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((int)MPU_ADDR, (int)len);
  if (n != len) return false;
  for (uint8_t i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}

bool initOneIMU(uint8_t ch){
  tcaSelect(ch);

  // WHO_AM_I
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false)!=0 || Wire.requestFrom((int)MPU_ADDR,1)!=1) return false;
  uint8_t who = Wire.read();
  if (who != 0x68 && who != 0x69) return false;

  bool ok = true;
  ok &= wreg(REG_PWR_MGMT_1, 0x00); delay(50);   // vekke
  ok &= wreg(REG_SMPLRT_DIV, 9);                 // 100 Hz
  ok &= wreg(REG_CONFIG, 0x03);                  // DLPF ~44 Hz
  ok &= wreg(REG_GYRO_CONFIG, 0x00);             // ±250 dps
  ok &= wreg(REG_ACCEL_CONFIG, 0x00);            // ±2 g
  return ok;
}

void calibrateGyro(uint8_t idx, uint16_t nsamples=600){
  tcaSelect(IMU_CH[idx]);
  double sx=0, sy=0, sz=0;
  uint8_t b[6];
  uint16_t n=0;
  delay(50);
  while (n < nsamples){
    if (rburst(REG_GYRO_XOUT_H, b, 6)){
      sx += toI16(b[0],b[1]) / GYRO_LSB_PER_DPS;
      sy += toI16(b[2],b[3]) / GYRO_LSB_PER_DPS;
      sz += toI16(b[4],b[5]) / GYRO_LSB_PER_DPS;
      n++;
    }
    delay(2);
  }
  gx_bias[idx]=sx/n; gy_bias[idx]=sy/n; gz_bias[idx]=sz/n;
}

void stepIMU(uint8_t i, float dt){
  if (!imu_ok[i]) return;
  tcaSelect(IMU_CH[i]);

  uint8_t b[14];
  if (!rburst(REG_ACCEL_XOUT_H, b, 14)){
    if (++fail_cnt[i] > 10){ imu_ok[i]=false; }
    return;
  }
  fail_cnt[i]=0;

  int16_t axr=toI16(b[0],b[1]);
  int16_t ayr=toI16(b[2],b[3]);
  int16_t azr=toI16(b[4],b[5]);
  int16_t gxr=toI16(b[8],b[9]);
  int16_t gyr=toI16(b[10],b[11]);

  float ax = axr/ACCEL_LSB_PER_G;
  float ay = ayr/ACCEL_LSB_PER_G;
  float az = azr/ACCEL_LSB_PER_G;

  float gx = (gxr/GYRO_LSB_PER_DPS) - gx_bias[i];
  float gy = (gyr/GYRO_LSB_PER_DPS) - gy_bias[i];

  // vinkel fra akselerometer
  float roll_acc  = atan2(ay, az) * 180.0f / PI;
  float pitch_acc = atan2(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  if (!fused_init[i]){
    roll_fused[i]  = roll_acc;
    pitch_fused[i] = pitch_acc;
    fused_init[i]  = true;
  } else {
    float roll_pred  = roll_fused[i]  + gx * dt;
    float pitch_pred = pitch_fused[i] + gy * dt;
    roll_fused[i]  = ALPHA*roll_pred  + (1.0f-ALPHA)*roll_acc;
    pitch_fused[i] = ALPHA*pitch_pred + (1.0f-ALPHA)*pitch_acc;
  }
}

void setZeroAll(){
  for (uint8_t i=0;i<NIMU;i++){
    if (!imu_ok[i]) continue;
    roll_zero[i]  = roll_fused[i];
    pitch_zero[i] = pitch_fused[i];
  }
  Serial.println("Zero set");
}

// ---------- SETUP ----------
void setup(){
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // start konservativt; øk til 400 kHz når stabilt

  // Init alle IMU’er
  for (uint8_t i=0; i<NIMU; i++){
    bool ok = initOneIMU(IMU_CH[i]);
    imu_ok[i] = ok;
    if (!ok){
      Serial.printf("IMU %u (ch %u) ikke funnet, hopper over.\n", i, IMU_CH[i]);
    }
  }

  // Kalibrer gyro (hold systemet i ro)
  for (uint8_t i=0; i<NIMU; i++){
    if (imu_ok[i]) calibrateGyro(i, 800);
  }

  last_us = micros();
  last_print_ms = millis();
  setZeroAll(); // start med nullpunkt
}

// ---------- LOOP ----------
void loop(){
  // 'z' i seriellmonitor for nytt nullpunkt
  if (Serial.available()){
    char c = Serial.read();
    if (c=='z' || c=='Z') setZeroAll();
  }

  unsigned long now_us = micros();
  float dt = (now_us - last_us)/1e6f;
  last_us = now_us;

  for (uint8_t i=0;i<NIMU;i++) stepIMU(i, dt);

  if (millis() - last_print_ms >= PRINT_INTERVAL_MS){
    last_print_ms = millis();

    char line[400];
    size_t idx = 0;

    // pitch (p0..p7)
    for (uint8_t i=0;i<NIMU;i++){
      float p = imu_ok[i] ? (pitch_fused[i] - pitch_zero[i]) : NAN;
      idx += snprintf(line+idx, sizeof(line)-idx, "p%u:%s%.1f\t",
                      i, isnan(p) ? "" : "", isnan(p) ? -999.9f : p);
      if (idx >= sizeof(line)) break;
    }
    // roll (r0..r7)
    for (uint8_t i=0;i<NIMU;i++){
      float r = imu_ok[i] ? (roll_fused[i] - roll_zero[i]) : NAN;
      if (i < NIMU-1)
        idx += snprintf(line+idx, sizeof(line)-idx, "r%u:%s%.1f\t",
                        i, isnan(r) ? "" : "", isnan(r) ? -999.9f : r);
      else
        idx += snprintf(line+idx, sizeof(line)-idx, "r%u:%s%.1f",
                        i, isnan(r) ? "" : "", isnan(r) ? -999.9f : r);
      if (idx >= sizeof(line)) break;
    }

    // newline
    if (idx < sizeof(line)-2){ line[idx++] = '\n'; line[idx] = '\0'; }
    else { line[sizeof(line)-2] = '\n'; line[sizeof(line)-1] = '\0'; }

    Serial.print(line);
  }

  delay(2);
}
