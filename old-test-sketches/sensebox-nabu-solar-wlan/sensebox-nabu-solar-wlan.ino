#include <Adafruit_DPS310.h>  // http://librarymanager/All#Adafruit_DPS310
#include <Adafruit_HDC1000.h>  // http://librarymanager/All#Adafruit_HDC1000_Library
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoLowPower.h>
#include <LTR329.h>
#include <SdsDustSensor.h>  // http://librarymanager/All#Nova_Fitness_Sds_dust_sensors_library
#include <VEML6070.h>
#include <WiFi101.h>
#include <Wire.h>
#include <senseBoxIO.h>

// deep sleep
void sleep_and_restart(uint32_t sleep_time) {
  senseBoxIO.powerNone();
  LowPower.deepSleep(max(0, sleep_time - 1000));
  delay(1000);
  noInterrupts();
  NVIC_SystemReset();
  while (1)
    ;
}

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

char ssid[] = "TODO";
char pass[] = "TODO";
int status = WL_IDLE_STATUS;

const long intervalInterval = 10000;
long time_startInterval = 0;
long time_actualInterval = 0;

Adafruit_HDC1000 hdc = Adafruit_HDC1000();
const char SENSOR_ID444[] PROGMEM = "60eecb0e4fb91e001cac8444";
const char SENSOR_ID443[] PROGMEM = "60eecb0e4fb91e001cac8443";
Adafruit_DPS310 dps;
const char SENSOR_ID442[] PROGMEM = "60eecb0e4fb91e001cac8442";

bool lightsensortype = 0;  // 0 for tsl - 1 for ltr
// settings for LTR sensor
LTR329 LTR;
unsigned char gain = 1;
unsigned char integrationTime = 0;
unsigned char measurementRate = 3;

const char SENSOR_ID441[] PROGMEM = "60eecb0e4fb91e001cac8441";
VEML6070 veml;
const char SENSOR_ID440[] PROGMEM = "60eecb0e4fb91e001cac8440";
SdsDustSensor sds(Serial1);
const char SENSOR_ID1C4[] PROGMEM = "64bfc454c56c220007b9c1c4";
const char SENSOR_ID1C5[] PROGMEM = "64bfc454c56c220007b9c1c5";
const char SENSOR_ID1C6[] PROGMEM = "64bfc454c56c220007b9c1c6";
const char SENSOR_ID1C7[] PROGMEM = "64bfc454c56c220007b9c1c7";
const char SENSOR_ID1C8[] PROGMEM = "64bfc454c56c220007b9c1c8";
static const uint8_t NUM_SENSORS = 10;
const char SENSEBOX_ID[] PROGMEM = "60eecb0e4fb91e001cac843c";
const char server[] PROGMEM = "ingress.opensensemap.org";
WiFiClient wifiClient;
BearSSLClient client(wifiClient);
typedef struct measurement {
  const char *sensorId;
  float value;
} measurement;
char buffer[750];
measurement measurements[NUM_SENSORS];
uint8_t num_measurements = 0;
const int lengthMultiplikator = 35;

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

unsigned long getTime() { return WiFi.getTime(); }

void addMeasurement(const char *sensorId, float value) {
  measurements[num_measurements].sensorId = sensorId;
  measurements[num_measurements].value = value;
  num_measurements++;
}

void writeMeasurementsToClient() {
  // iterate throug the measurements array
  for (uint8_t i = 0; i < num_measurements; i++) {
    sprintf_P(buffer, PSTR("%s,%9.2f\n"), measurements[i].sensorId,
              measurements[i].value);
    // transmit buffer to client
    client.print(buffer);
  }
  // reset num_measurements
  num_measurements = 0;
}

void submitValues() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    delay(1000);  // wait 1s
    WiFi.begin(ssid, pass);
    delay(5000);  // wait 5s
  }
  if (client.connected()) {
    client.stop();
    delay(1000);
  }
  bool connected = false;
  char _server[strlen_P(server)];
  strcpy_P(_server, server);
  for (uint8_t timeout = 2; timeout != 0; timeout--) {
    Serial.println(F("connecting..."));
    connected = client.connect(_server, 443);
    if (connected == true) {
      // construct the HTTP POST request:
      sprintf_P(buffer,
                PSTR("POST /boxes/%s/data HTTP/1.1\nAuthorization: "
                     "TODO\nHost: %s\nContent-Type: "
                     "text/csv\nConnection: close\nContent-Length: %i\n\n"),
                SENSEBOX_ID, server, num_measurements * lengthMultiplikator);
      // send the HTTP POST request:
      client.print(buffer);
      // send measurements
      writeMeasurementsToClient();
      // send empty line to end the request
      client.println();
      uint16_t timeout = 0;
      // allow the response to be computed
      while (timeout <= 5000) {
        delay(10);
        timeout = timeout + 10;
        if (client.available()) {
          break;
        }
      }

      while (client.available()) {
        char c = client.read();
        // if the server's disconnected, stop the client:
        if (!client.connected()) {
          client.stop();
          break;
        }
      }

      num_measurements = 0;
      break;
    }
    delay(1000);
  }

  if (connected == false) {
    delay(5000);
    noInterrupts();
    NVIC_SystemReset();
    while (1)
      ;
  }
}

void setup() {
  if (WiFi.status() == WL_NO_SHIELD) {
    while (true)
      ;
  }
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }

  hdc.begin();
  dps.begin_I2C(0x76);

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  Lightsensor_begin();
  veml.begin();
  sds.begin();
  sds.setQueryReportingMode();
  ArduinoBearSSL.onGetTime(getTime);
}

void loop() {
  // ensure particle sensor has started
  while (millis() < 30000)
    ;

  solar_update();
  sensors_event_t temp_event, pressure_event;
  dps.getEvents(&temp_event, &pressure_event);
  PmResult pm = sds.queryPm();

  addMeasurement(SENSOR_ID444, hdc.readTemperature());
  addMeasurement(SENSOR_ID443, hdc.readHumidity());
  addMeasurement(SENSOR_ID442, pressure_event.pressure);
  addMeasurement(SENSOR_ID441, Lightsensor_getIlluminance());
  addMeasurement(SENSOR_ID440, veml.getUV());
  addMeasurement(SENSOR_ID1C4, pm.pm25);
  addMeasurement(SENSOR_ID1C5, pm.pm10);
  addMeasurement(SENSOR_ID1C6, battery_voltage);
  addMeasurement(SENSOR_ID1C7, battery_level);
  addMeasurement(SENSOR_ID1C8, solar_voltage);
  submitValues();

  // sleep
  sleep_and_restart(600000);
}
