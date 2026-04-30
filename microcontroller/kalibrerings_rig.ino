#include <Wire.h>
#include <math.h>

static float defl_mm = 0.0f;
static const float DEFL_STEP_MM = 0.1f;
// ---------- PINNER / ADRESSER ----------
const int SDA_PIN = 4;
const int SCL_PIN = 5;
const uint8_t TCA_ADDR = 0x70;   // TCA9548A
const uint8_t MPU_ADDR = 0x68;   // MPU6050 (0x69 hvis AD0=HIGH)

// ---------- MPU6050 REGISTRE ----------
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

// ---------- SENSOR SKALA ----------
const float ACCEL_LSB_PER_G  = 8192.0f; // ±4 g
const float GYRO_LSB_PER_DPS = 65.5f;   // ±500 dps

// ---------- OPPSETT ----------
const uint8_t NIMU = 8;
const uint8_t IMU_CH[NIMU] = {0,1,2,3,4,5,6,7};

// Madgwick tuning
const float MADGWICK_BETA = 0.08f;

// Accel “gate”
const float ACC_GATE_G = 0.45f;

// ---------- RASK GYRO-BIAS ----------
static const uint16_t GYRO_BIAS_SAMPLES = 250;

// ---------- TILSTAND ----------
bool   imu_ok[NIMU]     = {false};
bool   q_init[NIMU]     = {false};

float  gx_bias[NIMU]={0}, gy_bias[NIMU]={0}, gz_bias[NIMU]={0};

// Quaternion per IMU
float  q0[NIMU]={1}, q1[NIMU]={0}, q2[NIMU]={0}, q3[NIMU]={0};

// ABS Euler (deg)
float  roll_deg[NIMU]={0}, pitch_deg[NIMU]={0};

// Stable Euler branch
float roll_last[NIMU]={0}, pitch_last[NIMU]={0};
bool  rp_has_last[NIMU]={false};

int    fail_cnt[NIMU]={0};
unsigned long last_us=0;

// ---------- HJELP ----------
static inline int16_t toI16(uint8_t hi, uint8_t lo){ return (int16_t)((hi<<8)|lo); }
static inline float invSqrt(float x){ return 1.0f / sqrtf(x); }

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

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false)!=0 || Wire.requestFrom((int)MPU_ADDR,1)!=1) return false;
  uint8_t who = Wire.read();
  if (who != 0x68 && who != 0x69) return false;

  bool ok = true;
  ok &= wreg(REG_PWR_MGMT_1, 0x00); delay(30);

  ok &= wreg(REG_SMPLRT_DIV, 4);     // 200Hz intern
  ok &= wreg(REG_CONFIG, 0x04);      // ~21Hz
  ok &= wreg(REG_GYRO_CONFIG,  0x08);// ±500 dps
  ok &= wreg(REG_ACCEL_CONFIG, 0x08);// ±4g
  return ok;
}

// Leser én sample: accel(g) + gyro(dps). Inkl. flip for i>=3.
bool readIMUSample(uint8_t i, float &ax, float &ay, float &az,
                   float &gx_dps, float &gy_dps, float &gz_dps){
  if (!imu_ok[i]) return false;

  tcaSelect(IMU_CH[i]);

  uint8_t b[14];
  if (!rburst(REG_ACCEL_XOUT_H, b, 14)) return false;

  int16_t axr=toI16(b[0],b[1]);
  int16_t ayr=toI16(b[2],b[3]);
  int16_t azr=toI16(b[4],b[5]);
  int16_t gxr=toI16(b[8],b[9]);
  int16_t gyr=toI16(b[10],b[11]);
  int16_t gzr=toI16(b[12],b[13]);

  ax = axr/ACCEL_LSB_PER_G;
  ay = ayr/ACCEL_LSB_PER_G;
  az = azr/ACCEL_LSB_PER_G;

  gx_dps = (gxr/GYRO_LSB_PER_DPS);
  gy_dps = (gyr/GYRO_LSB_PER_DPS);
  gz_dps = (gzr/GYRO_LSB_PER_DPS);

  if (i >= 3) {
    ax = -ax;  ay = -ay;
    gx_dps = -gx_dps; gy_dps = -gy_dps;
  }
  return true;
}

// ---------- Euler stable ----------
static inline float wrap180(float a){
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void quatFromAccel(uint8_t i, float ax, float ay, float az){
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);

  q0[i] = cr*cp;
  q1[i] = sr*cp;
  q2[i] = cr*sp;
  q3[i] = -sr*sp;

  float n = invSqrt(q0[i]*q0[i] + q1[i]*q1[i] + q2[i]*q2[i] + q3[i]*q3[i]);
  q0[i]*=n; q1[i]*=n; q2[i]*=n; q3[i]*=n;

  rp_has_last[i] = false;
}

void quatToRollPitchStable(uint8_t i, float &rollOut, float &pitchOut){
  float _q0=q0[i], _q1=q1[i], _q2=q2[i], _q3=q3[i];

  float sinr_cosp = 2.0f * (_q0*_q1 + _q2*_q3);
  float cosr_cosp = 1.0f - 2.0f * (_q1*_q1 + _q2*_q2);
  float roll  = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float sinp = 2.0f * (_q0*_q2 - _q3*_q1);
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  float pitch = asinf(sinp) * 180.0f / PI;

  float rollB  = wrap180(roll + 180.0f);
  float pitchB = wrap180(180.0f - pitch);

  float rollC  = wrap180(roll - 180.0f);
  float pitchC = wrap180(-180.0f - pitch);

  if (!rp_has_last[i]){
    rp_has_last[i] = true;
    roll_last[i] = roll;
    pitch_last[i] = pitch;
    rollOut = roll;
    pitchOut = pitch;
    return;
  }

  float r0 = wrap180(roll  - roll_last[i]);
  float p0 = wrap180(pitch - pitch_last[i]);
  float cost0 = fabsf(r0) + fabsf(p0);

  float r1 = wrap180(rollB  - roll_last[i]);
  float p1 = wrap180(pitchB - pitch_last[i]);
  float cost1 = fabsf(r1) + fabsf(p1);

  float r2 = wrap180(rollC  - roll_last[i]);
  float p2 = wrap180(pitchC - pitch_last[i]);
  float cost2 = fabsf(r2) + fabsf(p2);

  if (cost1 < cost0 && cost1 <= cost2){
    roll = rollB; pitch = pitchB;
  } else if (cost2 < cost0 && cost2 < cost1){
    roll = rollC; pitch = pitchC;
  }

  roll_last[i]  = roll;
  pitch_last[i] = pitch;

  rollOut = roll;
  pitchOut = pitch;
}

// ---------- Madgwick ----------
void madgwickUpdateIMU_core(uint8_t i, float gx, float gy, float gz,
                            float ax, float ay, float az, float dt){
  float _q0 = q0[i], _q1 = q1[i], _q2 = q2[i], _q3 = q3[i];

  float a2 = ax*ax + ay*ay + az*az;
  if (a2 <= 1e-12f) return;

  float inva = invSqrt(a2);
  ax *= inva; ay *= inva; az *= inva;

  float _2q0 = 2.0f*_q0;
  float _2q1 = 2.0f*_q1;
  float _2q2 = 2.0f*_q2;
  float _2q3 = 2.0f*_q3;
  float _4q0 = 4.0f*_q0;
  float _4q1 = 4.0f*_q1;
  float _4q2 = 4.0f*_q2;
  float _8q1 = 8.0f*_q1;
  float _8q2 = 8.0f*_q2;
  float q0q0 = _q0*_q0;
  float q1q1 = _q1*_q1;
  float q2q2 = _q2*_q2;
  float q3q3 = _q3*_q3;

  float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
  float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*_q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
  float s2 = 4.0f*q0q0*_q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
  float s3 = 4.0f*q1q1*_q3 - _2q1*ax + 4.0f*q2q2*_q3 - _2q2*ay;

  float s2norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
  if (s2norm > 1e-12f){
    float invs = invSqrt(s2norm);
    s0*=invs; s1*=invs; s2*=invs; s3*=invs;
  }

  float qDot0 = 0.5f * (-_q1*gx - _q2*gy - _q3*gz) - MADGWICK_BETA*s0;
  float qDot1 = 0.5f * ( _q0*gx + _q2*gz - _q3*gy) - MADGWICK_BETA*s1;
  float qDot2 = 0.5f * ( _q0*gy - _q1*gz + _q3*gx) - MADGWICK_BETA*s2;
  float qDot3 = 0.5f * ( _q0*gz + _q1*gy - _q2*gx) - MADGWICK_BETA*s3;

  _q0 += qDot0 * dt;
  _q1 += qDot1 * dt;
  _q2 += qDot2 * dt;
  _q3 += qDot3 * dt;

  float n = invSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
  q0[i]=_q0*n; q1[i]=_q1*n; q2[i]=_q2*n; q3[i]=_q3*n;
}

void quatIntegrateGyroOnly(uint8_t i, float gx, float gy, float gz, float dt){
  float _q0 = q0[i], _q1 = q1[i], _q2 = q2[i], _q3 = q3[i];

  float qDot0 = 0.5f * (-_q1*gx - _q2*gy - _q3*gz);
  float qDot1 = 0.5f * ( _q0*gx + _q2*gz - _q3*gy);
  float qDot2 = 0.5f * ( _q0*gy - _q1*gz + _q3*gx);
  float qDot3 = 0.5f * ( _q0*gz + _q1*gy - _q2*gx);

  _q0 += qDot0 * dt; _q1 += qDot1 * dt; _q2 += qDot2 * dt; _q3 += qDot3 * dt;

  float n = invSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
  q0[i]=_q0*n; q1[i]=_q1*n; q2[i]=_q2*n; q3[i]=_q3*n;
}

// ---------- RASK GYRO-BIAS ----------
void calibrateGyroFast(uint8_t idx, uint16_t nsamples){
  double sx=0, sy=0, sz=0;
  uint16_t n=0;

  while (n < nsamples){
    float ax,ay,az,gx,gy,gz;
    if (readIMUSample(idx, ax,ay,az,gx,gy,gz)){
      sx += gx; sy += gy; sz += gz;
      n++;
    }
    delay(1);
  }

  gx_bias[idx] = sx/n;
  gy_bias[idx] = sy/n;
  gz_bias[idx] = sz/n;
}

// ---------- IMU STEP ----------
void stepIMU(uint8_t i, float dt){
  if (!imu_ok[i]) return;

  float ax,ay,az,gx_dps,gy_dps,gz_dps;
  if (!readIMUSample(i, ax,ay,az,gx_dps,gy_dps,gz_dps)){
    if (++fail_cnt[i] > 20) imu_ok[i] = false;
    return;
  }
  fail_cnt[i]=0;

  float gx_corr = gx_dps - gx_bias[i];
  float gy_corr = gy_dps - gy_bias[i];
  float gz_corr = gz_dps - gz_bias[i];

  if (!q_init[i]){
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(an - 1.0f) <= 0.18f) {
      quatFromAccel(i, ax, ay, az);
      q_init[i] = true;
      float r,p;
      quatToRollPitchStable(i, r, p);
      roll_deg[i]=r;
      pitch_deg[i]=p;
    }
    return;
  }

  float gx = gx_corr * (PI/180.0f);
  float gy = gy_corr * (PI/180.0f);
  float gz = gz_corr * (PI/180.0f);

  float an = sqrtf(ax*ax + ay*ay + az*az);
  bool accelGood = fabsf(an - 1.0f) <= ACC_GATE_G;

  if (accelGood) madgwickUpdateIMU_core(i, gx, gy, gz, ax, ay, az, dt);
  else           quatIntegrateGyroOnly(i, gx, gy, gz, dt);

  float r,p;
  quatToRollPitchStable(i, r, p);
  roll_deg[i]=r;
  pitch_deg[i]=p;
}

// ---------- SERIAL COMMAND ----------
void handleSerialCommands(){
  while (Serial.available()){
    char c = (char)Serial.read();

    if (c == 'p' || c == 'P'){
      // Print snapshot (defl_mm + p0..p7)
      Serial.print(defl_mm, 1);

      for (uint8_t i=0;i<NIMU;i++){
        float v = (!imu_ok[i] || !q_init[i]) ? NAN : roll_deg[i]; // "pitch" hos deg
        Serial.print('\t');
        Serial.print(isnan(v) ? -999.9f : v, 1);
      }
      Serial.println();

      defl_mm += DEFL_STEP_MM;

      // Tøm evt. \r/\n (og annet) som Serial Monitor sendte med
      while (Serial.available()) Serial.read();

      return; // én utskrift per send
    }

    // Ignorer linjeslutt og annet tull
  }
}

// ---------- SETUP ----------
void setup(){
  Serial.begin(115200);
  delay(150);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("Boot: Snapshot on Serial command.");
  Serial.println("Send 'p' to print: defl_mm\\t(p0..p7)  [prints ROLL as your pitch]");

  for (uint8_t i=0; i<NIMU; i++){
    imu_ok[i] = initOneIMU(IMU_CH[i]);
    if (!imu_ok[i]) Serial.printf("IMU %u (ch %u) ikke funnet.\n", i, IMU_CH[i]);
  }

  Serial.println("### Gyro bias fast (hold still) ###");
  for (uint8_t i=0; i<NIMU; i++){
    if (!imu_ok[i]) continue;
    calibrateGyroFast(i, GYRO_BIAS_SAMPLES);
    Serial.printf("IMU%u bias dps: %.3f %.3f %.3f\n", i, gx_bias[i], gy_bias[i], gz_bias[i]);
  }

  uint32_t t0 = millis();
  while (millis() - t0 < 1200){
    bool allInit = true;
    for (uint8_t i=0;i<NIMU;i++){
      if (!imu_ok[i] || q_init[i]) continue;

      float ax,ay,az,gx,gy,gz;
      if (readIMUSample(i, ax,ay,az,gx,gy,gz)){
        float an = sqrtf(ax*ax + ay*ay + az*az);
        if (fabsf(an - 1.0f) <= 0.18f) {
          quatFromAccel(i, ax, ay, az);
          q_init[i] = true;
          float r,p; quatToRollPitchStable(i, r, p);
          roll_deg[i]=r; pitch_deg[i]=p;
        } else allInit = false;
      } else allInit = false;
    }
    if (allInit) break;
    delay(10);
  }

  Serial.println("### Ready. Send 'p' to print snapshot. ###");
  last_us = micros();
}

// ---------- LOOP ----------
void loop(){
  unsigned long now_us = micros();
  float dt = (now_us - last_us)/1e6f;
  last_us = now_us;

  if (dt < 0.0005f) dt = 0.0005f;
  if (dt > 0.0500f) dt = 0.0500f;

  for (uint8_t i=0;i<NIMU;i++) stepIMU(i, dt);

  handleSerialCommands();
  delay(2);
}