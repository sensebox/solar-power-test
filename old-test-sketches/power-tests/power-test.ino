#include <ArduinoLowPower.h>
#include <senseBoxIO.h>

void setup() {}

void loop() {
  // normal
  delay(10000);

  // port power off
  senseBoxIO.powerNone();
  delay(5000);
  senseBoxIO.powerAll();

  // sleep
  LowPower.sleep(5000);

  // deep sleep
  LowPower.deepSleep(5000);

  // deep sleep + port power off
  senseBoxIO.powerNone();
  LowPower.deepSleep(5000);
  senseBoxIO.powerAll();
}