/****************************************************
 *  Water Quality Monitoring - TDS + EchoStar Satellite
 *  Mode sélectionné par jumper :
 *    - Jumper vers GND → BOOT MODE
 *    - Pas de jumper   → RUN MODE (AT+SEND)
 ****************************************************/

#define DEBUG_SERIAL     Serial
#define ECHOSTAR_SERIAL  Serial2

// --- EchoStar Pins ---
#define ECHOSTAR_nRST_PIN        PB5
#define ECHOSTAR_BOOT_PIN        PA0
#define ECHOSTAR_RTS_PIN         PA10
#define ECHOSTAR_PWR_ENABLE_PIN  PA8

// --- Mode Select Jumper ---
#define MODE_SELECT_PIN false   // true = BOOT, false = RUN

// --- TDS Sensor ---
#define TDS_PIN PA0
#define ADC_RESOLUTION 4095.0
#define VREF 5

// --- TDS Filter ---
#define SCOUNT 30

// --- Intervalle d'envoi ---
const unsigned long SEND_INTERVAL_MS = 10000;
unsigned long last_send_time = 0;


/****************************************************
 *  EchoStar GPIO Init
 ****************************************************/
void echoStar_gpio_init() {
  pinMode(ECHOSTAR_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_PWR_ENABLE_PIN, HIGH);

  pinMode(ECHOSTAR_RTS_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_RTS_PIN, LOW);

  pinMode(ECHOSTAR_nRST_PIN, OUTPUT);
  pinMode(ECHOSTAR_BOOT_PIN, OUTPUT);
}

void reset_to_run(void) {
  digitalWrite(ECHOSTAR_nRST_PIN, LOW);
  digitalWrite(ECHOSTAR_BOOT_PIN, HIGH);
  delay(200);
  digitalWrite(ECHOSTAR_nRST_PIN, HIGH);
  delay(2000);
}

/****************************************************
 *  EchoStar BOOT MODE
 ****************************************************/
void echoStar_enterBootMode() {
  digitalWrite(ECHOSTAR_BOOT_PIN, LOW);
  digitalWrite(ECHOSTAR_nRST_PIN, LOW);
  delay(200);
  digitalWrite(ECHOSTAR_nRST_PIN, HIGH);
  delay(2000);
}

/****************************************************
 *  EchoStar RUN MODE
 ****************************************************/
void echoStar_enterRunMode() {
  digitalWrite(ECHOSTAR_BOOT_PIN, HIGH);
  digitalWrite(ECHOSTAR_nRST_PIN, LOW);
  delay(200);
  digitalWrite(ECHOSTAR_nRST_PIN, HIGH);
  delay(5000);
}

/****************************************************
 *  Envoi Satellite
 ****************************************************/
void sendSatelliteText(const String& msg) {
  ECHOSTAR_SERIAL.print("AT+SEND=1,0,8,0,");
  ECHOSTAR_SERIAL.print(msg);
  ECHOSTAR_SERIAL.print("\r\n");
  delay(300);
}

void sendTDSviaSatellite(float tds, uint8_t quality) {
  String msg = "TDS," + String(tds, 1) + "," + String(quality);
  sendSatelliteText(msg);
}

/****************************************************
 *  Setup
 ****************************************************/
void setup() {

  DEBUG_SERIAL.begin(115200);
  ECHOSTAR_SERIAL.begin(115200);

  ECHOSTAR_SERIAL.print("AT+JOIN\r\n");

  analogReadResolution(12);

  DEBUG_SERIAL.println("=== Water Quality Monitoring (TDS + Satellite) ===");

  echoStar_gpio_init();
  echoStar_enterRunMode();
}

/****************************************************
 *  Loop
 ****************************************************/
void loop() {
  if (millis() - last_send_time > SEND_INTERVAL_MS) {

    float tdsValue = readTDS();
    uint8_t waterQuality = evaluateWaterQuality(tdsValue);

    DEBUG_SERIAL.print("TDS Value : ");
    DEBUG_SERIAL.print(tdsValue);
    DEBUG_SERIAL.println(" ppm");

    DEBUG_SERIAL.print("Water Quality Index : ");
    DEBUG_SERIAL.println(waterQuality);

    sendTDSviaSatellite(tdsValue, waterQuality);

    last_send_time = millis();
  }
}

/****************************************************
 *  Lecture TDS (avec filtrage médian)
 ****************************************************/
float readTDS() {
  int analogBuffer[SCOUNT];
  int analogBufferTemp[SCOUNT];
  
  // Prise de 30 échantillons
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(TDS_PIN);
    delay(40);
  }
  // Copie pour filtrage
  for (int i = 0; i < SCOUNT; i++) {
    analogBufferTemp[i] = analogBuffer[i];
  }
  // Filtrage médian
  int medianValue = getMedianNum(analogBufferTemp, SCOUNT);

  float voltage = medianValue * VREF / ADC_RESOLUTION;
  float tds = (133.42 * voltage * voltage * voltage
             - 255.86 * voltage * voltage
             + 857.39 * voltage) * 0.5;
  return tds;
}

/****************************************************
 *  Filtre Médian
 ****************************************************/
int getMedianNum(int bArray[], int iFilterLen) 
{
  int bTab[iFilterLen];

  for (int i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];

  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  if ((iFilterLen & 1) > 0)
    return bTab[(iFilterLen - 1) / 2];
  else
    return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

/****************************************************
 *  Qualité de l'eau
 ****************************************************/
uint8_t evaluateWaterQuality(float tds) {
  if (tds < 300) return 1;
  else if (tds < 600) return 2;
  else if (tds < 1000) return 3;
  else return 4;
}
