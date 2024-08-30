#include <TFT_eSPI.h>
#include <SPI.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <HardwareSerial.h>
#include <XPT2046_Touchscreen.h>
#include <SD.h>

TFT_eSPI tft = TFT_eSPI();
#define TFT_GREY 0x5AEB
#define LOOP_PERIOD 300

#define BARO_MIN 980
#define BARO_MAX 1020
#define BARO_SCALE_START_ANGLE -225
#define BARO_SCALE_END_ANGLE 45

#define BARO_X0 4
#define BARO_Y0 4
#define BARO_WIDTH 154
#define BARO_HEIGHT 114


#define BARO_CENTER_X (BARO_X0 + BARO_WIDTH / 2)
#define BARO_CENTER_Y (BARO_Y0 + BARO_HEIGHT / 2)
#define BARO_RADIUS (BARO_HEIGHT - 60)

// RTC CLOCK PINS
const int IO = 27;    // DAT
const int SCLK = 14;  // CLK
const int CE = 26;    // RST

// SIM800L PINS
#define SIM800L_RX 16  // RX modułu SIM800L
#define SIM800L_TX 17  // TX modułu SIM800L

// Dotyk PINS
#define TOUCH_CS 5
#define TOUCH_IRQ 0

HardwareSerial sim800l(1);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

void drawClockSection(RtcDateTime now);
#define CALIBRATION_POINTS 4
int16_t calData[CALIBRATION_POINTS] = { 0, 4095, 0, 4095 };
int showDots = 0;
int updateTime = 0;
float lastPressure = -1;

Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
ThreeWire myWire(IO, SCLK, CE);
RtcDS1302<ThreeWire> rtc(myWire);

void setup(void) {
  tft.init();
  tft.setRotation(3);
  tft.setTextSize(2);
  if (!touch.begin(tft.getSPIinstance())) {
    Serial.println("Touch not detected");
    while (1)
      ;
  } else {
    Serial.println("Touchscreen detected");
  }
  calibrateTouch();
  Serial.begin(115200);
  tft.fillScreen(TFT_GREY);

  Wire.begin(33, 32);

  tft.fillRect(4, 4, 312, 232, TFT_BLACK);

  rtc.Begin();
  RtcDateTime now = rtc.GetDateTime();

  tft.fillRect(0, 117, 320, 4, TFT_GREY);
  tft.fillRect(157, 0, 4, 240, TFT_GREY);

  sim800l.begin(9600, SERIAL_8N1, SIM800L_RX, SIM800L_TX);
  delay(1000);

  sim800l.println("AT");
  delay(500);
  if (sim800l.available()) {
    String response = sim800l.readString();
    Serial.println("SIM800L Response: " + response);
  } else {
    Serial.println("Sim800L not working");
  }

  // Sprawdzenie dostępności czujników
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1)
      ;
  }

  if (!aht.begin()) {
    Serial.println("Could not find a valid AHT10 sensor, check wiring!");
    while (1)
      ;
  }
  drawBarometer();
  updateNeedle(1000);

  tft.setTextSize(2);
  tft.drawCentreString("Wyslij SMS ", 237, 169, 1);

  updateTime = millis();
}

void loop() {
  if (updateTime <= millis()) {
    updateTime = millis() + LOOP_PERIOD;

    RtcDateTime now = rtc.GetDateTime();
    drawClockSection(now);

    sensors_event_t ahtHumidity, ahtTemp;
    aht.getEvent(&ahtHumidity, &ahtTemp);

    float bmpTemp = bmp.readTemperature();
    float bmpPressure = bmp.readPressure() / 100.0F;

    float temp = (bmpTemp + ahtTemp.temperature) / 2;

    updateNeedle(bmpPressure);

    drawTempSection(temp, ahtHumidity.relative_humidity);

    if (touch.touched()) {
      TS_Point p = touch.getPoint();

      int x = map(p.x, calData[0], calData[1], 0, tft.width());
      int y = map(p.y, calData[2], calData[3], 0, tft.height());


      if (y < 120) {
        if (x < 160) {
          Serial.println("Top-Left");
        } else {
          Serial.println("Top-Right");
        }
      } else {
        if (x < 160) {
          Serial.println("Bottom-Left");
        } else {
          Serial.println("Bottom-Right");
          sendWeatherReport(bmpPressure, temp, ahtHumidity.relative_humidity);
        }
      }
    }


    if (Serial.available()) {
      String command = Serial.readString();
      if (command == "s") {
        sendWeatherReport(bmpPressure, temp, ahtHumidity.relative_humidity);
      }
    }
  }
}


// Funkcja do wysyłania SMS-a
void sendWeatherReport(float pressure, float temperature, float humidity) {
  char buffer[160];
  snprintf(buffer, sizeof(buffer), "Raport pogody:\nCiśnienie: %.2f hPa\nTemperatura: %.2f C\nWilgotność: %.2f%%", pressure, temperature, humidity);
  String message = String(buffer);
  Serial.println(buffer);
  sim800l.println("AT+CMGF=1");
  delay(500);
  sim800l.println("AT+CMGS=\"+48601160554\"");
  delay(500);
  sim800l.print(message);
  delay(500);
  sim800l.write(26);
  delay(500);
}

// Funkcja do obsługi dotyku
void handleTouch(TS_Point p) {
}

// Sekcja zegara
void drawClockSection(RtcDateTime now) {
  const int roundings[2][2] = {
    { 162, 0 },
    { 319, 117 }
  };
  int centerX = (roundings[0][0] + roundings[1][0]) / 2;
  int beginY = roundings[0][1] + 13;

  tft.setTextSize(2);
  char buffer[11];
  snprintf(buffer, 11, "%02d.%02d.%u", now.Day(), now.Month(), now.Year());

  String message = String(buffer);
  tft.drawCentreString(message, centerX, beginY, 2);

  beginY += 38;
  snprintf(buffer, 9, "%02d %02d %02d", now.Hour(), now.Minute(), now.Second());
  message = String(buffer);
  tft.drawCentreString(message, centerX, beginY, 2);

  int dotsY = beginY + 16;
  if (showDots > 0) {
    tft.fillCircle(centerX - 23, dotsY + 5, 2, TFT_WHITE);
    tft.fillCircle(centerX - 23, dotsY - 5, 2, TFT_WHITE);
    tft.fillCircle(centerX + 21, dotsY + 5, 2, TFT_WHITE);
    tft.fillCircle(centerX + 21, dotsY - 5, 2, TFT_WHITE);
  }
  showDots++;
  if (showDots > 2) showDots = -1;

  switch (now.DayOfWeek()) {
    case 0: message = "Niedziela"; break;
    case 1: message = "Poniedzialek"; break;
    case 2: message = "Wtorek"; break;
    case 3: message = "Sroda"; break;
    case 4: message = "Czwartek"; break;
    case 5: message = "Piatek"; break;
    case 6: message = "Sobota"; break;
    default: message = "";
  }
  beginY += 38;
  tft.setTextSize(1);

  tft.drawCentreString(message, centerX, beginY, 2);
}

// Sekcja temperatury
void drawTempSection(float temp, float humidity) {
  const int roundings[2][2] = {
    { 4, 123 },
    { 158, 316 }
  };
  int centerX = (roundings[0][0] + roundings[1][0]) / 2;
  int beginY = roundings[0][1] + 16;

  tft.setTextSize(1);
  tft.drawCentreString("Temperatura: ", centerX, beginY, 1);
  beginY += 16;

  tft.setTextSize(2);
  char bufferTemp[7];
  snprintf(bufferTemp, 10, "%.2f°C", temp);

  String message = String(bufferTemp);
  tft.drawCentreString(message, centerX, beginY, 1);

  beginY += 32;
  tft.setTextSize(1);
  tft.drawCentreString("Wilgotnosc: ", centerX, beginY, 1);
  beginY += 16;

  tft.setTextSize(2);
  char bufferHum[7];
  snprintf(bufferHum, 7, "%.2f%%", humidity);
  message = String(bufferHum);
  tft.drawCentreString(message, centerX, beginY, 1);
}

void drawBarometer() {
  tft.fillRect(BARO_X0, BARO_Y0, BARO_WIDTH, BARO_HEIGHT, TFT_BLACK);
  tft.drawRect(BARO_X0, BARO_Y0, BARO_WIDTH, BARO_HEIGHT, TFT_WHITE);

  for (float i = BARO_MIN; i <= BARO_MAX; i += 2.5) {
    float angle = mapFloat(i, BARO_MIN, BARO_MAX, BARO_SCALE_START_ANGLE, BARO_SCALE_END_ANGLE);
    drawTick(angle, String(int(i)), i);
  }
  tft.setCursor(8, 112);
  tft.print("hPa");
}

void drawTick(float angle, String label, float baroVal) {
  float angleRad = radians(angle);
  int x1 = BARO_CENTER_X + cos(angleRad) * (BARO_RADIUS - 10);
  int y1 = BARO_CENTER_Y + sin(angleRad) * (BARO_RADIUS - 10);
  int x2 = BARO_CENTER_X + cos(angleRad) * BARO_RADIUS;
  int y2 = BARO_CENTER_Y + sin(angleRad) * BARO_RADIUS;

  tft.drawLine(x1, y1, x2, y2, TFT_WHITE);
  int xLabel = BARO_CENTER_X + cos(angleRad) * (BARO_RADIUS - 20);
  int yLabel = BARO_CENTER_Y + sin(angleRad) * (BARO_RADIUS - 20);

  Serial.println(angle);
  Serial.println(baroVal);
  if (int(baroVal - 980) % 10 == 0) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(xLabel - 10, yLabel - 5);
    tft.print(label);
  }
}

void updateNeedle(float pressure) {
  if (pressure == lastPressure) return;
  drawNeedle(lastPressure, TFT_BLACK);
  lastPressure = pressure;
  drawNeedle(pressure, TFT_RED);
}

void drawNeedle(float pressure, uint16_t color) {
  float angle = map(pressure, BARO_MIN, BARO_MAX, BARO_SCALE_START_ANGLE, BARO_SCALE_END_ANGLE);
  float angleRad = radians(angle);
  int x = BARO_CENTER_X + cos(angleRad) * (BARO_RADIUS - 20);
  int y = BARO_CENTER_Y + sin(angleRad) * (BARO_RADIUS - 20);

  tft.drawLine(BARO_CENTER_X, BARO_CENTER_Y, x, y, color);
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void calibrateTouch() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t x, y;

  // Punkt 1: Lewy Górny Róg
  tft.drawString("Touch the top left", 10, 10);
  while (!touch.touched()) {}
  TS_Point p = touch.getPoint();
  calData[0] = p.x;
  calData[2] = p.y;
  delay(500);

  // Punkt 2: Prawy Dolny Róg
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Touch the bottom right", tft.width() - 160, tft.height() - 20);
  while (!touch.touched()) {}
  p = touch.getPoint();
  calData[1] = p.x;
  calData[3] = p.y;
  delay(500);

  tft.fillScreen(TFT_BLACK);
  tft.drawString("Calibration complete", tft.width() / 2 - 80, tft.height() / 2);
  delay(1000);
}
