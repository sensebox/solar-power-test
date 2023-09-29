#include <Adafruit_DPS310.h>  // http://librarymanager/All#Adafruit_DPS310
#include <Adafruit_GFX.h>     // http://librarymanager/All#Adafruit_GFX_Library
#include <Adafruit_HDC1000.h>  // http://librarymanager/All#Adafruit_HDC1000_Library
#include <Adafruit_SSD1306.h>  // http://librarymanager/All#Adafruit_SSD1306
#include <ArduinoLowPower.h>
#include <CayenneLPP.h>  // http://librarymanager/All#CayenneLPP
#include <LTR329.h>
#include <SPI.h>
#include <SdsDustSensor.h>  // http://librarymanager/All#Nova_Fitness_Sds_dust_sensors_library
#include <VEML6070.h>
#include <Wire.h>
#include <lmic.h>
#include <senseBoxIO.h>

// absolute path necessary. dont know why #include <hal/hal.h>
#include "C:\Users\bjoer\AppData\Local\Arduino15\packages\sensebox\hardware\samd\1.6.0\libraries\arduino-lmic\src\hal\hal.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define DISPLAY_INTERVAL 5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void printOnDisplay(String title1, String measurement1, String unit1,
                    String title2, String measurement2, String unit2) {
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.println(title1);
  display.setCursor(0, 10);
  display.setTextSize(2);
  display.print(measurement1);
  display.print(" ");
  display.setTextSize(1);
  display.println(unit1);
  display.setCursor(0, 30);
  display.setTextSize(1);
  display.println(title2);
  display.setCursor(0, 40);
  display.setTextSize(2);
  display.print(measurement2);
  display.print(" ");
  display.setTextSize(1);
  display.println(unit2);
}

float pressure;
float pm2_5;
float pm10;
float temperature;
float humidity;

CayenneLPP lpp(51);

static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00,
                                       0x01, 0x00, 0x01, 0x00};
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = {0x00, 0x00, 0x00, 0x00,
                                       0x10, 0x00, 0x10, 0x00};
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {"TODO"};
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;
static osjob_t displayjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = PIN_XB1_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {PIN_XB1_INT, PIN_XB1_INT, LMIC_UNUSED_PIN},
};
Adafruit_DPS310 dps;
SdsDustSensor sds(Serial1);
Adafruit_HDC1000 hdc = Adafruit_HDC1000();

bool lightsensortype = 0;  // 0 for tsl - 1 for ltr
// settings for LTR sensor
LTR329 LTR;
unsigned char gain = 1;
unsigned char integrationTime = 0;
unsigned char measurementRate = 3;

VEML6070 veml;

void initLora() {
  delay(2000);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL),
                          do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

int read_reg(byte address, uint8_t reg) {
  int i = 0;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address, (uint8_t)1);
  delay(1);
  if (Wire.available()) i = Wire.read();
  return i;
}

void write_reg(byte address, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void Lightsensor_begin() {
  Wire.begin();
  unsigned int u = 0;
  u = read_reg(0x29, 0x80 | 0x0A);  // id register
  if ((u & 0xF0) == 0xA0)           // TSL45315
  {
    write_reg(0x29, 0x80 | 0x00, 0x03);  // control: power on
    write_reg(0x29, 0x80 | 0x01, 0x02);  // config: M=4 T=100ms
    delay(120);
    lightsensortype = 0;  // TSL45315
  } else {
    LTR.begin();
    LTR.setControl(gain, false, false);
    LTR.setMeasurementRate(integrationTime, measurementRate);
    LTR.setPowerUp();     // power on with default settings
    delay(10);            // Wait 10 ms (max) - wakeup time from standby
    lightsensortype = 1;  //
  }
}

unsigned int Lightsensor_getIlluminance() {
  unsigned int lux = 0;
  if (lightsensortype == 0)  // TSL45315
  {
    unsigned int u = (read_reg(0x29, 0x80 | 0x04) << 0);  // data low
    u |= (read_reg(0x29, 0x80 | 0x05) << 8);              // data high
    lux = u * 4;                    // calc lux with M=4 and T=100ms
  } else if (lightsensortype == 1)  // LTR-329ALS-01
  {
    delay(100);
    unsigned int data0, data1;
    for (int i = 0; i < 5; i++) {
      if (LTR.getData(data0, data1)) {
        if (LTR.getLux(gain, integrationTime, data0, data1, lux))
          ;
        if (lux > 0)
          break;
        else
          delay(10);
      } else {
        byte error = LTR.getError();
      }
    }
  }
  return lux;
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // updates
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);
    PmResult pm = sds.queryPm();
    pm2_5 = pm.pm25;
    pm10 = pm.pm10;
    pressure = pressure_event.pressure;
    temperature = hdc.readTemperature();
    humidity = hdc.readHumidity();

    // send
    lpp.reset();
    lpp.addTemperature(1, temperature);
    lpp.addRelativeHumidity(1, humidity);
    lpp.addBarometricPressure(1, pressure);
    lpp.addLuminosity(1, Lightsensor_getIlluminance());
    lpp.addLuminosity(2, veml.getUV());
    lpp.addTemperature(2, pm2_5);
    lpp.addTemperature(3, pm10);

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void update_display(osjob_t* t) {
  printOnDisplay("Temperatur", String(temperature), "C",
                 "rel. Luftfeuchtigkeit ", String(humidity), "%");
  display.display();
  os_setTimedCallback(&displayjob, os_getTime() + sec2osticks(DISPLAY_INTERVAL),
                      update_display);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  dps.begin_I2C(0x76);

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  sds.begin();
  sds.setQueryReportingMode();
  hdc.begin();
  Lightsensor_begin();
  veml.begin();

  // display setup
  senseBoxIO.powerI2C(true);
  delay(2000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.display();
  delay(100);
  display.clearDisplay();

  initLora();

  update_display(&displayjob);
}

void loop() { os_runloop_once(); }
