#include <Adafruit_DPS310.h>  // http://librarymanager/All#Adafruit_DPS310
#include <Adafruit_HDC1000.h>  // http://librarymanager/All#Adafruit_HDC1000_Library
#include <Adafruit_NeoPixel.h>
#include <ArduinoLowPower.h>
#include <LTR329.h>
#include <SdsDustSensor.h>  // http://librarymanager/All#Nova_Fitness_Sds_dust_sensors_library
#include <VEML6070.h>
#include <WiFi101.h>
#include <Wire.h>
#include <arduino_lmic.h>
#include <arduino_lmic_hal_boards.h>
#include <arduino_lmic_hal_configuration.h>
#include <arduino_lmic_lorawan_compliance.h>
#include <arduino_lmic_user_configuration.h>
#include <lmic.h>
#include <senseBoxIO.h>
Adafruit_NeoPixel rgb_led_1 = Adafruit_NeoPixel(1, 1, NEO_GRB + NEO_KHZ800);

char ssid[] = "TODO";
char pass[] = "TODO";
int status = WL_IDLE_STATUS;

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

Adafruit_HDC1000 hdc = Adafruit_HDC1000();
const char SENSOR_ID8D0[] PROGMEM = "6436ab0031e4f900082458d0";
const char SENSOR_ID8CF[] PROGMEM = "6436ab0031e4f900082458cf";

bool lightsensortype = 0;  // 0 for tsl - 1 for ltr
// settings for LTR sensor
LTR329 LTR;
unsigned char gain = 1;
unsigned char integrationTime = 0;
unsigned char measurementRate = 3;

const char SENSOR_ID8CE[] PROGMEM = "6436ab0031e4f900082458ce";
VEML6070 veml;
const char SENSOR_ID8CD[] PROGMEM = "6436ab0031e4f900082458cd";
SdsDustSensor sds(Serial1);
const char SENSOR_ID8CC[] PROGMEM = "6436ab0031e4f900082458cc";
const char SENSOR_ID8CB[] PROGMEM = "6436ab0031e4f900082458cb";
Adafruit_DPS310 dps;
const char SENSOR_ID8CA[] PROGMEM = "6436ab0031e4f900082458ca";
const char SENSOR_ID738[] PROGMEM = "645a06225edac10008d98738";
const char SENSOR_ID739[] PROGMEM = "645a06225edac10008d98739";
const char SENSOR_ID7E7[] PROGMEM = "647778d104d18600084a46e7";
static const uint8_t NUM_SENSORS = 10;
const char SENSEBOX_ID[] PROGMEM = "6436ab0031e4f900082458c9";
const char server[] PROGMEM = "ingress.opensensemap.org";
WiFiClient client;
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
    connected = client.connect(_server, 80);
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
  // led
  rgb_led_1.begin();
  rgb_led_1.setBrightness(30);
  rgb_led_1.setPixelColor(0, rgb_led_1.Color(0, 0, 255));
  rgb_led_1.show();

  if (WiFi.status() == WL_NO_SHIELD) {
    while (true)
      ;
  }
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }

  // init I2C/Wire library
  Wire.begin();

  SerialUSB.begin(9600);

  hdc.begin();
  Lightsensor_begin();
  veml.begin();
  sds.begin();
  sds.setQueryReportingMode();
  dps.begin_I2C(0x76);
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  rgb_led_1.setPixelColor(0, rgb_led_1.Color(0, 255, 0));
  rgb_led_1.show();
}

void loop() {
  while (millis() < 30000)
    ;  // enshure SDS011 was on for at least 30 seconds
  PmResult pm = sds.queryPm();
  sensors_event_t temp_event, pressure_event;
  dps.getEvents(&temp_event, &pressure_event);

  // battery
  solar_update();
  Serial.println(String("solar_voltage: ") + solar_voltage + " V");
  Serial.println(String("battery_voltage: ") + battery_voltage + " V");
  Serial.println(String("battery_level: ") + battery_level + "/4");
  Serial.println(String("temp: ") + (battery_temp) + " *C");

  addMeasurement(SENSOR_ID8D0, hdc.readTemperature());
  addMeasurement(SENSOR_ID8CF, hdc.readHumidity());
  addMeasurement(SENSOR_ID8CE, Lightsensor_getIlluminance());
  addMeasurement(SENSOR_ID8CD, veml.getUV());
  addMeasurement(SENSOR_ID8CC, pm.pm10);
  addMeasurement(SENSOR_ID8CB, pm.pm25);
  addMeasurement(SENSOR_ID8CA, pressure_event.pressure);
  addMeasurement(SENSOR_ID738, battery_voltage);
  addMeasurement(SENSOR_ID739, battery_level);
  addMeasurement(SENSOR_ID7E7, solar_voltage);
  submitValues();

  // battery saving mode
  Serial.print("go to sleep for ");
  int sleep_time = 1000 * 60 * 10;
  Serial.print(sleep_time);
  Serial.println("ms");
  rgb_led_1.setPixelColor(0, rgb_led_1.Color(255, 0, 0));
  rgb_led_1.show();

  delay(2000);
  sleep_and_restart(sleep_time);
}
