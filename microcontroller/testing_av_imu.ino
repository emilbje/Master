#include <Wire.h>

// ---- PINNER ----
#define SDA_PIN 4   // din SDA
#define SCL_PIN 5   // din SCL

// ---- MPU6050 ----
const uint8_t ADDR_68 = 0x68;
const uint8_t ADDR_69 = 0x69;
uint8_t MPU_ADDR = ADDR_68;

const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_GYRO_XOUT_H  = 0x43;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

const float ACCEL_LSB_PER_G  = 16384.0f; // ±2g
const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps

// ---- HJELP ----
int16_t toI16(uint8_t hi, uint8_t lo){ return (int16_t)((hi<<8)|lo); }

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
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
  for (uint8_t i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}

void scanI2C(){
  Serial.println("I2C-scan:");
  for (uint8_t a=1; a<127; a++){
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0){
      Serial.print("  0x"); Serial.println(a, HEX);
    }
  }
}

bool initMPU(){
  // velg adresse som svarer
  Wire.beginTransmission(ADDR_68);
  if (Wire.endTransmission() == 0) MPU_ADDR = ADDR_68;
  else {
    Wire.beginTransmission(ADDR_69);
    if (Wire.endTransmission() == 0) MPU_ADDR = ADDR_69;
    else return false;
  }

  // WHO_AM_I
  Wire.beginTransmission(MPU_ADDR); Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false)!=0 || Wire.requestFrom((int)MPU_ADDR,1)!=1) return false;
  uint8_t who = Wire.read();
  Serial.printf("WHO_AM_I=0x%02X (addr 0x%02X)\n", who, MPU_ADDR);

  // konfig
  bool ok = true;
  ok &= wreg(REG_PWR_MGMT_1, 0x00); delay(50); // vekke
  ok &= wreg(REG_SMPLRT_DIV, 9);               // 100 Hz
  ok &= wreg(REG_CONFIG, 0x03);                // DLPF ~44 Hz
  ok &= wreg(REG_GYRO_CONFIG, 0x00);           // ±250 dps
  ok &= wreg(REG_ACCEL_CONFIG, 0x00);          // ±2 g
  return ok;
}

void setup(){
  Serial.begin(115200); delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);  // SDA=21, SCL=20
  Wire.setClock(100000);         // start trygt
  scanI2C();

  if (!initMPU()){
    Serial.println("Fant ikke MPU6050. Sjekk SDA=21, SCL=20, GND, og 3.3V.");
    while(1){ delay(1000); }
  }
  Serial.println("MPU6050 klar. Leser data...");
}

void loop(){
  static uint32_t t0=0; if (millis()-t0 < 100) return; t0 = millis();

  uint8_t b[14];
  if (!rburst(REG_ACCEL_XOUT_H, b, 14)){
    Serial.println("Lesefeil");
    return;
  }

  int16_t axr=toI16(b[0],b[1]), ayr=toI16(b[2],b[3]), azr=toI16(b[4],b[5]);
  int16_t gxr=toI16(b[8],b[9]),  gyr=toI16(b[10],b[11]), gzr=toI16(b[12],b[13]);

  float ax=axr/ACCEL_LSB_PER_G, ay=ayr/ACCEL_LSB_PER_G, az=azr/ACCEL_LSB_PER_G;
  float gx=gxr/GYRO_LSB_PER_DPS, gy=gyr/GYRO_LSB_PER_DPS, gz=gzr/GYRO_LSB_PER_DPS;

  float roll_acc  = atan2(ay, az) * 180.0f / PI;
  float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;

  Serial.printf("P:%.1f R:%.1f\n",
                pitch_acc, roll_acc);
}
