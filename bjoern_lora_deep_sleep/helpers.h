#ifndef DFA_HELPERS_H
#define DFA_HELPERS_H

#include <CayenneLPP.h>  // http://librarymanager/All#CayenneLPP

CayenneLPP lpp(51);


void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


void create_cayenne_message(ei_impulse_result_classification_t *cl)
{
  lpp.reset();
  // lpp.addDigitalOutput(0, classificationResult.timing.dsp);
  // lpp.addDigitalOutput(1, classificationResult.timing.classification);
  // lpp.addDigitalOutput(2, classificationResult.timing.anomaly);
  // lpp.addDigitalOutput(0, 0);
  // lpp.addDigitalOutput(1, 0);
  // lpp.addDigitalOutput(2, 0);
  Serial.println(cl[0].value);
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      lpp.addAnalogInput(ix + 3, cl[ix].value);
  }
}

// Hexadecimal number conversion 
void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

#endif