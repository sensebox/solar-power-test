
#include <Adafruit_BMP280.h>  // http://librarymanager/All#Adafruit_BMP280_Library
#include <Adafruit_HDC1000.h>  // http://librarymanager/All#Adafruit_HDC1000_Library
#include <CayenneLPP.h>        // http://librarymanager/All#CayenneLPP
#include <LTR329.h>
// #include <SDS011-select-serial.h>
#include <SPI.h>
#include <SdsDustSensor.h>  // http://librarymanager/All#Nova_Fitness_Sds_dust_sensors_library
#include <VEML6070.h>
#include <Wire.h>
#include <lmic.h>  // http://librarymanager/All#IBM_LMIC_framework
#include <senseBoxIO.h>

// absolute path necessary. dont know why #include <hal/hal.h>
#include "C:\Users\bjoer\AppData\Local\Arduino15\packages\sensebox\hardware\samd\1.6.0\libraries\arduino-lmic\src\hal\hal.h"

// battery stuff
float solar_voltage;
float battery_voltage;
int battery_level;
float battery_temp;
String battery_status_info;

void solar_update() {
  /*
   * I2C i/f with following info on address 0x32:
   * - Register 0: cell voltage, 20mV/LSB
   * - Register 1: input voltage, 100mV/LSB
   * - Register 2: status bits: [B,I,L3,L2,L1,L0,F,C]
   *    B=battery present >2.8V
   *    I=Input voltage present >4.5V
   *    L0-L3=battery status LEDs
   *    F=Fast charge enabled
   *    C=Charging
   * - Register 3: temperature in C, signed 8-bit
   * Thresholds: L0: 3.2V, L1: 3.6V, L2: 3.7V, L3: 3.9V <- somewhere the order
   * is wrong
   *
   * i Switch to slow charge at 8*C Switch to fast charge at 12*C
   */
  byte address = 0x32;
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Wire.requestFrom((uint8_t)address, (uint8_t)4);
    uint vbat = Wire.read();
    uint vin = Wire.read();
    uint flags = Wire.read();
    int temp = (int8_t)(Wire.read());
    battery_voltage = 0.02 * vbat;
    solar_voltage = 0.1 * vin;
    if (flags & 32)
      battery_level = 4;
    else if (flags & 16)
      battery_level = 3;
    else if (flags & 8)
      battery_level = 2;
    else if (flags & 4)
      battery_level = 1;
    else
      battery_level = 0;
    if (flags & 1)
      battery_status_info = "charging\n" + String(flags);
    else
      battery_status_info = "not charging\n";
    if (flags & 64)
      battery_status_info += "good input voltage\n";
    else
      battery_status_info += "low input voltage\n";
    if (flags & 128)
      battery_status_info += "battery present";
    else
      battery_status_info += "battery missing";
  } else if (error == 2) {
    battery_status_info = "Solar module not connected";
  } else if (error) {
    battery_status_info = "unknown error ";
    battery_status_info += String(error);
  }
}

float pm2_5 = -2;
float pm10 = -2;

CayenneLPP lpp(51);

static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x01, 0x01,
                                       0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = {0x01, 0x08, 0x06, 0x05,
                                       0x04, 0x03, 0x02, 0x01};
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {"TODO"};
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;
static osjob_t restartParticleSensorJob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 600;  // 10 minutes

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = PIN_XB1_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {PIN_XB1_INT, PIN_XB1_INT, LMIC_UNUSED_PIN},
};
Adafruit_HDC1000 hdc = Adafruit_HDC1000();
Adafruit_BMP280 bmp;

bool lightsensortype = 0;  // 0 for tsl - 1 for ltr
// settings for LTR sensor
LTR329 LTR;
unsigned char gain = 1;
unsigned char integrationTime = 0;
unsigned char measurementRate = 3;

VEML6070 veml;
// SDS011 SDS(Serial1);
SdsDustSensor sds(Serial1);

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
      // Schedule dustsensor restart 30 seconds before
      os_setTimedCallback(&restartParticleSensorJob,
                          os_getTime() + sec2osticks(TX_INTERVAL - 30),
                          do_restartParticleSensor);
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

float getSoundValue() {
  float v = analogRead(1) * (3.3 / 1024.0);
  float decibel = v * 50;
  return decibel;
}

void do_restartParticleSensor(osjob_t* j) {
  senseBoxIO.powerUART(true);
  Serial.println(F("UARTs on"));
  delay(500);
  sds.begin();
  sds.setQueryReportingMode();
  Serial.println(F("particle sensor restarted"));
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // updates
    solar_update();
    PmResult pm = sds.queryPm();
    pm2_5 = pm.pm25;
    pm10 = pm.pm10;
    // uint8_t attempt = 0;
    // while (attempt < 5) {
    //   bool error = SDS.read(&pm2_5, &pm10);
    //   if (!error) break;
    //   attempt++;
    // }

    // send
    lpp.reset();
    lpp.addTemperature(1, hdc.readTemperature());
    lpp.addRelativeHumidity(2, hdc.readHumidity());
    lpp.addBarometricPressure(3, bmp.readPressure() / 100);
    lpp.addLuminosity(4, Lightsensor_getIlluminance());
    lpp.addLuminosity(5, veml.getUV());
    lpp.addTemperature(6, pm10);
    lpp.addTemperature(7, pm2_5);
    lpp.addTemperature(8, getSoundValue());
    lpp.addTemperature(9, battery_voltage);

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
    senseBoxIO.powerUART(false);
    Serial.println(F("UARTs off"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  hdc.begin();
  bmp.begin(0x76);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  Lightsensor_begin();
  veml.begin();
  sds.begin();
  sds.setQueryReportingMode();

  initLora();
}

void loop() { os_runloop_once(); }
