// Flex/Bend sensor -> prints ONLY bend % (0..100) once every 200 ms.
// Calibrate: press 's' when straight, 'b' when at your typical max bend.
// Reset calibration: 'r'.

const int PIN_IN = A0;
const float VREF = 5.0;
const unsigned long PERIOD_MS = 200;   // 5 Hz
const int NUM_SAMPLES = 10;            // average to reduce noise
const int SAMPLE_DELAY_US = 500;       // between fast samples

float adc_straight = NAN;
float adc_bent     = NAN;

float bend_pct_smooth = NAN;           // exponential smoothing
const float ALPHA = 0.3f;              // 0..1 (higher = faster, noisier)

unsigned long tPrev = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  // analogReference(DEFAULT); // Uno default 5V
  Serial.println(F("Ready: press 's' (straight), then 'b' (bent). Prints only % bend."));
}

int readAvg(int pin, int n) {
  long acc = 0;
  for (int i = 0; i < n; i++) {
    acc += analogRead(pin);
    delayMicroseconds(SAMPLE_DELAY_US);
  }
  return (int)(acc / n);
}

float constrainf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

void handleSerialCommands(int adc_current) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S') {
      adc_straight = (float)adc_current;
      Serial.println(F("STRAIGHT set"));   // one-time hint
    } else if (c == 'b' || c == 'B') {
      adc_bent = (float)adc_current;
      Serial.println(F("BENT set"));       // one-time hint
    } else if (c == 'r' || c == 'R') {
      adc_straight = NAN;
      adc_bent = NAN;
      bend_pct_smooth = NAN;
      Serial.println(F("Calibration reset"));
    }
  }
}

void loop() {
  unsigned long now = millis();
  if (now - tPrev >= PERIOD_MS) {
    tPrev = now;

    int adc = readAvg(PIN_IN, NUM_SAMPLES);
    handleSerialCommands(adc);

    float bend_pct = NAN;
    if (!isnan(adc_straight) && !isnan(adc_bent) && adc_straight != adc_bent) {
      float num   = (float)adc - adc_straight;
      float denom = adc_bent - adc_straight;      // sign auto-handled
      bend_pct = 100.0f * (num / denom);
      bend_pct = constrainf(bend_pct, 0.0f, 100.0f);

      if (isnan(bend_pct_smooth)) bend_pct_smooth = bend_pct;
      bend_pct_smooth += ALPHA * (bend_pct - bend_pct_smooth);

      Serial.println(bend_pct_smooth, 1); // <-- ONLY % bend printed
    } else {
      // Not calibrated yet: print nothing (or uncomment next line if you prefer a hint)
      // Serial.println("NaN");
    }
  }
}
