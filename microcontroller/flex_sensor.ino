/******************************************************************************
  Flex sensor -> vinkel (−90..+90) til Serial Plotter
  Støyreduksjon som fortsatt fanger ~2° endringer:
   - Median(5) på råvinkel
   - Leaky deadband (±0.6°) med liten gain under terskel, stor over
   - EMA-glatting for jevn kurve
  Re-kalibrer "rett" med 's' i serie/plotter.
******************************************************************************/
const int   FLEX_PIN = A0;       // til A0
const float VCC      = 4.98;     // mål faktisk 5V/3.3V
const float R_DIV    = 47500.0;  // mål faktisk ohm

// Motstandsforhold for ~90° (SparkFun: 90000/37300 ≈ 2.41)
const float BEND_RATIO_AT_90 = 90000.0f / 37300.0f;

// Samplings-/filteroppsett
const int   NUM_SAMPLES   = 10;     // ADC-avg for mindre støy
const unsigned long DT_MS = 50;     // ~20 Hz

// Referanselinjer til plotter (låser y-aksen)
const float REF_HI =  8.0f;
const float REF_LO = -8.0f;

// --- Støyfiltre (tweak disse) ---
const float DB_EPS_DEG = 0.6f;   // deadband ±0.6° (under dette: svært tregt)
const float K_SMALL    = 0.05f;  // gain når |feil| < DB_EPS_DEG
const float K_LARGE    = 0.35f;  // gain når |feil| ≥ DB_EPS_DEG
const float EMA_ALPHA  = 0.15f;  // ekstra glatting etter deadband (0..1)

// Kalibrerte verdier
float R_straight = NAN;          // motstand ved “rett”

// Tilstand
unsigned long tPrev = 0;
float angle_deg_smooth = 0.0f;

// Medianbuffer
float a_buf[5] = {0,0,0,0,0};
int   a_idx = 0;
bool  a_filled = false;

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);
  delay(200);
  calibrateStraight();          // hold sensoren rett ved oppstart
  angle_deg_smooth = 0.0f;
}

void loop() {
  // Re-kalibrer med 's'
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 's' || c == 'S') {
      calibrateStraight();
      angle_deg_smooth = 0.0f;
    }
  }

  unsigned long now = millis();
  if (now - tPrev < DT_MS) return;
  tPrev = now;

  // 1) Les og konverter til signert vinkel
  float R     = readFlexResistance();
  float a_raw = resistanceToAngleSigned(R);      // kan være −90..+90
  a_raw       = constrain(a_raw, -90.0f, 90.0f);

  // 2) Median(5) mot spikes
  pushAngle(a_raw);
  float a_med = median5();

  // 3) Leaky deadband: liten gain for mikrobevegelser, stor for reelle endringer
  float err = a_med - angle_deg_smooth;
  float k   = (fabs(err) < DB_EPS_DEG) ? K_SMALL : K_LARGE;
  angle_deg_smooth += k * err;

  // 4) Ekstra jevning (EMA) for pen kurve
  angle_deg_smooth = angle_deg_smooth + EMA_ALPHA * (a_med - angle_deg_smooth);
  angle_deg_smooth = constrain(angle_deg_smooth, -90.0f, 90.0f);

  // --- Utskrift: én linje med 3 serier (låser skala) ---
  Serial.print("angle:");
  Serial.print(angle_deg_smooth, 2);
  Serial.print('\t');
  Serial.print("ref_hi:");
  Serial.print(REF_HI, 0);
  Serial.print('\t');
  Serial.print("ref_lo:");
  Serial.println(REF_LO, 0);
}

/*** Hjelpefunksjoner ***/

// Kalibrer "rett": gjennomsnitt over kort tid
void calibrateStraight() {
  const int N = 50;
  const int step_ms = 40;
  double acc = 0.0;
  for (int i = 0; i < N; i++) {
    acc += readFlexResistance();
    delay(step_ms);
  }
  R_straight = acc / N;

  // Nullstill medianbuffer og vinkel
  for (int i = 0; i < 5; i++) a_buf[i] = 0.0f;
  a_idx = 0; a_filled = false;
}

// Les ADC og regn om til motstand
float readFlexResistance() {
  long acc = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    acc += analogRead(FLEX_PIN);
    delayMicroseconds(200);
  }
  float adc = (float)acc / NUM_SAMPLES;

  float flexV = adc * VCC / 1023.0f;
  if (flexV < 0.001f) flexV = 0.001f; // vern mot div/0

  // R_flex = R_DIV * (Vcc/Vout - 1)
  float R_flex = R_DIV * (VCC / flexV - 1.0f);
  return R_flex;
}

// Tosidig vinkel via log-forhold rundt "rett"
float resistanceToAngleSigned(float R_flex) {
  if (isnan(R_straight) || R_straight <= 0.0f) return 0.0f;

  float ratio = R_flex / R_straight;
  if (ratio < 0.001f) ratio = 0.001f;

  float scale = log(BEND_RATIO_AT_90);
  if (scale <= 0.0f) return 0.0f;

  float angle = 90.0f * (log(ratio) / scale); // negativ om R < R_straight
  return angle;
}

// --- Median(5) verktøy ---
void pushAngle(float a) {
  a_buf[a_idx] = a;
  a_idx = (a_idx + 1) % 5;
  if (!a_filled && a_idx == 0) a_filled = true;
}

float median5() {
  // Hvis ikke fylt ennå, bruk gjennomsnitt av det vi har
  int n = a_filled ? 5 : max(1, a_idx);
  float tmp[5];
  for (int i = 0; i < n; i++) tmp[i] = a_buf[i];

  // Enkel sortering (n ≤ 5)
  for (int i = 0; i < n-1; i++) {
    for (int j = i+1; j < n; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t;
      }
    }
  }

  if (n == 1) return tmp[0];
  if (n == 2) return 0.5f * (tmp[0] + tmp[1]);
  if (n == 3) return tmp[1];
  if (n == 4) return 0.5f * (tmp[1] + tmp[2]);
  return tmp[2]; // n==5 median
}
