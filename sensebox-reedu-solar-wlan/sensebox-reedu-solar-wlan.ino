#include <senseBoxIO.h>
#include <WiFi101.h>
#include <Wire.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoLowPower.h>

char ssid[] = "ReeduNet";
char pass[] = "werockschools!";
int status = WL_IDLE_STATUS;

const long intervalInterval = 10000;
long time_startInterval = 0;
long time_actualInterval = 0;
float solar_voltage;
float battery_voltage;
int battery_level = -1;
float battery_temp;
String battery_status_info;

const char SENSOR_ID39B[] PROGMEM = "66e7fb0c22e674000705f39b";
const char SENSOR_ID39C[] PROGMEM = "66e7fb0c22e674000705f39c";
const char SENSOR_ID39D[] PROGMEM = "66e7fb0c22e674000705f39d";
static const uint8_t NUM_SENSORS = 3;
const char SENSEBOX_ID[] PROGMEM = "66e7fb0c22e674000705f39a";
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


void solar_update() {
  /*
   * I2C i/f with following info on address 0x32:
   * - Register 0: cell voltage, 20mV/LSB
   * - Register 1: input voltage, 100mV/LSB
   * - Register 2: status bits: [B,I,L3,L2,L1,L0,F,C]
   *    B=battery present >2.8V
   *    I=Input voltage present > 4.5V
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
    battery_status_info = "solar module not connected";
  } else if (error) {
    battery_status_info = "unknown error ";
    battery_status_info += String(error);
  }
}

// deep sleep
int sleep_time = 4000;
void sleep_and_restart(uint32_t sleep_time) {
  if (Serial.dtr()) {  // serial monitor is open -> intention to upload new code with arduino ide
    Serial.print("sleep for ");
    Serial.print(sleep_time);
    Serial.println(" ms and restart");
    delay(sleep_time);
  } else {
    senseBoxIO.powerNone();
    LowPower.deepSleep(max(0, sleep_time - 1000));
    delay(1000);
    noInterrupts();
    NVIC_SystemReset();
    while (1)
      ;
  }
}

unsigned long getTime() {
  return WiFi.getTime();
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
    connected = client.connect(_server, 443);
    if (connected == true) {
      // construct the HTTP POST request:
      sprintf_P(buffer,
                PSTR("POST /boxes/%s/data HTTP/1.1\nAuthorization: 28ed4c651a0ef5cb5f575e44e07109e5e3de492caf24b0987f9e3f1b1ad292da\nHost: %s\nContent-Type: "
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
  Serial.begin(9600);
  Serial.dtr();
  delay(1000);
  // if (WiFi.status() == WL_NO_SHIELD) {
  //   while (true)
  //     ;
  // }
  // while (status != WL_CONNECTED) {
  //   status = WiFi.begin(ssid, pass);
  //   delay(5000);
  // }

  // Wire.begin();
  // solar_update();
  // ArduinoBearSSL.onGetTime(getTime);
}


void loop() {
  Serial.dtr();
  Serial.println(battery_voltage);
  Serial.println(battery_level);
  Serial.println(solar_voltage);
  // solar_update();
  // addMeasurement(SENSOR_ID39B, battery_voltage);
  // addMeasurement(SENSOR_ID39C, battery_level);
  // addMeasurement(SENSOR_ID39D, solar_voltage);
  // submitValues();

  // battery saving mode
  sleep_and_restart(sleep_time);


  delay(2000);
}
