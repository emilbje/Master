#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>

// --------- WiFi AP / Webserver ----------
const char* AP_SSID = "SkiLogger";
const char* AP_PASS = "skitest123";   // kan være tom, men fint med passord

WebServer server(80);

// Siste data fra sender
String latestLine = "";
bool   hasData    = false;
float  p[5] = {0}, r[5] = {0};

// --------- ESP-NOW mottak ----------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  if (data_len <= 0 || data == nullptr) return;

  // Kopier og null-terminer
  char buf[200];
  int len = data_len;
  if (len > (int)sizeof(buf) - 1) len = sizeof(buf) - 1;
  memcpy(buf, data, len);
  buf[len] = '\0';

  latestLine = String(buf);

  float p0,p1,p2,p3,p4,r0,r1,r2,r3,r4;

  int parsed = sscanf(
    buf,
    "p0:%f p1:%f p2:%f p3:%f p4:%f r0:%f r1:%f r2:%f r3:%f r4:%f",
    &p0,&p1,&p2,&p3,&p4,
    &r0,&r1,&r2,&r3,&r4
  );

  // Debug til PC (og til OpenLog, se under)
  // Serial.print("RX: ");
  // Serial.print(buf);
  // Serial.print("  parsed = ");
  // Serial.println(parsed);

  if (parsed == 10) {
    p[0]=p0; p[1]=p1; p[2]=p2; p[3]=p3; p[4]=p4;
    r[0]=r0; r[1]=r1; r[2]=r2; r[3]=r3; r[4]=r4;
  }

  // Si fra til /data at vi faktisk har fått noe
  hasData = true;

  // --------- LOGG TIL OPENLOG ----------
  // Serial går både til PC (USB) og til OpenLog RXI (TX0-pin).
  // Vi skriver bare selve linja fra senderen:
  Serial.print(buf);  // buf inkluderer '\n' fra sender
}


// --------- HTTP handlers ----------

const char MAIN_page[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>SkiLogger Live</title>
<style>
  body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:0; padding:1rem; }
  h1 { font-size:1.4rem; margin-bottom:0.5rem; }
  .card { background:#222; padding:1rem; border-radius:8px; max-width:600px; }
  table { width:100%; border-collapse:collapse; margin-top:0.5rem; }
  th, td { border:1px solid #444; padding:0.3rem 0.5rem; text-align:right; }
  th { background:#333; }
  .small { font-size:0.8rem; color:#aaa; }
</style>
</head>
<body>
<h1>SkiLogger – Live data</h1>
<div class="card">
  <div class="small">Koble til dette WiFi-nettet: <b>SkiLogger</b>, åpne <b>http://192.168.4.1/</b></div>
  <p id="status">Venter på data...</p>
  <table>
    <thead>
      <tr>
        <th></th>
        <th>0</th><th>1</th><th>2</th><th>3</th><th>4</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <th>pitch</th>
        <td id="p0">-</td><td id="p1">-</td><td id="p2">-</td><td id="p3">-</td><td id="p4">-</td>
      </tr>
      <tr>
        <th>roll</th>
        <td id="r0">-</td><td id="r1">-</td><td id="r2">-</td><td id="r3">-</td><td id="r4">-</td>
      </tr>
    </tbody>
  </table>
  <p class="small">Siste rålinje: <span id="raw">-</span></p>
</div>

<script>
async function fetchData() {
  try {
    const res = await fetch('/data');
    if (!res.ok) {
      document.getElementById('status').textContent =
        'Feil ved henting av /data (' + res.status + ')';
      return;
    }
    const d = await res.json();

    if (!d || !d.p || !d.r) {
      document.getElementById('status').textContent = 'Ingen gyldig data ennå...';
      return;
    }

    document.getElementById('status').textContent =
      'Oppdatert: ' + d.time_ms + ' ms';

    for (let i = 0; i < 5; i++) {
      document.getElementById('p'+i).textContent = d.p[i].toFixed(1);
      document.getElementById('r'+i).textContent = d.r[i].toFixed(1);
    }

    document.getElementById('raw').textContent = 'OK';
  } catch (e) {
    document.getElementById('status').textContent = 'Feil: ' + e;
  }
}

setInterval(fetchData, 200);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", MAIN_page);
}

void handleData() {
  String json = "{";

  // tidsstempel
  json += "\"time_ms\":" + String(millis()) + ",";

  // pitch-array
  json += "\"p\":[";
  for (int i = 0; i < 5; i++) {
    if (i) json += ",";
    json += String(p[i], 3);
  }
  json += "],";

  // roll-array
  json += "\"r\":[";
  for (int i = 0; i < 5; i++) {
    if (i) json += ",";
    json += String(r[i], 3);
  }
  json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

// --------- SETUP / LOOP ----------

void setup() {
  Serial.begin(9600);   // <- matcher OpenLog sitt 9600-baud
  delay(200);

  // WiFi AP + ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (apOk) {
    Serial.print("AP startet. SSID: ");
    Serial.print(AP_SSID);
    Serial.print("  IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Klarte ikke å starte AP!");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init feilet");
  } else {
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW mottaker klar.");
  }

  // HTTP routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Webserver startet på port 80.");
}

void loop() {
  server.handleClient();
}


