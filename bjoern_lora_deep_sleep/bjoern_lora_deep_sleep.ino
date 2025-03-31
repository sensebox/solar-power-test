// These sketches are tested with 2.0.15 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <CayenneLPP.h>
#include <SPI.h>
#include <hal/hal.h>
#include <lmic.h>  //wichtig Lora
#include <plant-detection-large_clone_kjell_inferencing.h>

#include "dfa_camera.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "helpers.h"
#include "pin_mappings.h"
#include "ttn_config.h"

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 160
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 160

#define uS_TO_S_FACTOR \
  1000000ULL             /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 15 /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR lmic_t RTC_LMIC;
bool GOTO_DEEPSLEEP = false;

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

void SaveLMICToRTC(int deepsleep_sec) {
  RTC_LMIC = LMIC;
  // EU Like Bands

  // System time is resetted after sleep. So we need to calculate the dutycycle
  // with a resetted system time
  unsigned long now = millis();
#if defined(CFG_LMIC_EU_like)
  for (int i = 0; i < MAX_BANDS; i++) {
    ostime_t correctedAvail =
        RTC_LMIC.bands[i].avail -
        ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (correctedAvail < 0) {
      correctedAvail = 0;
    }
    RTC_LMIC.bands[i].avail = correctedAvail;
  }

  RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail -
                             ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
  if (RTC_LMIC.globalDutyAvail < 0) {
    RTC_LMIC.globalDutyAvail = 0;
  }
#else
  Serial.println("No DutyCycle recalculation function!")
#endif
}

void LoadLMICFromRTC() { LMIC = RTC_LMIC; }

void onEvent(ev_t ev) {
  // Serial.print(os_getTime());
  // Serial.print(": ");
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
      {
        // u4_t netid = 0;
        // devaddr_t devaddr = 0;
        // u1_t nwkKey[16];
        // u1_t artKey[16];
        // LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        // Serial.print("netid: ");
        // Serial.println(netid, DEC);
        // Serial.print("devaddr: ");
        // Serial.println(devaddr, HEX);
        // Serial.print("AppSKey: ");
        // for (size_t i = 0; i < sizeof(artKey); ++i) {
        //   if (i != 0)
        //     Serial.print("-");
        //   printHex2(artKey[i]);
        // }
        // Serial.println("");
        // Serial.print("NwkSKey: ");
        // for (size_t i = 0; i < sizeof(nwkKey); ++i) {
        //   if (i != 0)
        //     Serial.print("-");
        //   printHex2(nwkKey[i]);
        // }
        // Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // esp_restart();
      GOTO_DEEPSLEEP = true;
      // Schedule next transmission
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL),
      // do_send);
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
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      // esp_restart();
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t *j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("PQ"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

/**
 * @brief      Arduino setup function
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // comment out the below line to start inference immediately after upload
  while (!Serial);
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  ++bootCount;

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Load the LoRa information from RTC
  if (RTC_LMIC.seqnoUp != 0) {
    LoadLMICFromRTC();
  }

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

/**
 * @brief      Get data and run inferencing
 *
 * @param[in]  debug  Get debug info if true
 */
void loop() {
  static unsigned long lastPrintTime = 0;

  os_runloop_once();

  const bool timeCriticalJobs =
      os_queryTimeCriticalJobs(ms2osticksRound((TX_INTERVAL * 1000)));
  if (!timeCriticalJobs && GOTO_DEEPSLEEP == true &&
      !(LMIC.opmode & OP_TXRXPEND)) {
    LoraWANPrintLMICOpmode();
    SaveLMICToRTC(TIME_TO_SLEEP);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                   " Seconds");
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
  // else if (lastPrintTime + 2000 < millis())
  // {
  //     Serial.print(F("Cannot sleep "));
  //     Serial.print(F("TimeCriticalJobs: "));
  //     Serial.print(timeCriticalJobs);
  //     Serial.print(" ");

  //     LoraWANPrintLMICOpmode();
  //     PrintRuntime();
  //     lastPrintTime = millis();
  // }
  // Serial.println("I can't sleep :(");
}
