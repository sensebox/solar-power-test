# Ein paar Notizen und Sketche zum Thema senseBox Solar Modul SB-041 und Energiesparmodus

## Solar Modul

Die Werte des Solarmoduls lassen sich mithilfe der Library [SolarChargerSB041](<[test](https://github.com/sensebox/SolarChargerSB041)>)

## Energiesparen

Am meisten Strom verbraucht der Feinstaubsensor SDS011. Danach kommen Display und das Wlan Modul. Weiter unten sind einpaar unsortierte Messungen.

### Wlan auf MCU

Die einfachste Methode bei der Nutzung von Solar Modul und Wlan ist ein Deep Sleep der kompletten senseBox, bei dem alle Anschlüsse ausgeschaltet werden. Nach dem Wake up startet die senseBox neu, um das WLAN Modul zu reinitialisieren:

```arduino
#include <ArduinoLowPower.h>

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
```

mit

```arduino
  while (millis() < 30000)
    ;  // enshure SDS011 was on for at least 30 seconds
```

am Anfang und

```arduino
  sleep_and_restart(sleep_time);
```

am Ende von loop().

### LoRa auf MCU (neue Variante)

Ein sehr detailirtes Beispielskript zur LoRa und energisparien auf SAMD architektur findet sich [hier](https://stuartsprojects.github.io/2022/12/14/Easy-Build-TTN-and-LoRaWAN-Nodes.html). Folgedes muss im TX COMPLETE Abschnitt des Lora Codes ergänzt werden

```arduino
    ...
    case EV_TXCOMPLETE:
        ...
        LowPower.sleep(seconds * 1000);


        hal_sleep_lowpower(DUTY_CYCLE_INTERVAL);         //adjusts lmic_ticks to in effect bypass dutycycle
        hal_sleep_lowpower(25);                          //add a small bit extra to avoid wrap round issues

        do_send(&sendjob);
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(DUTY_CYCLE_INTERVAL), do_send);
        break;

```

### LoRa auf MCU (alte Variante)

Bei LoRa ist das schwieriger, da die Verbindung nach Neustart sehr lange dauert. Man kann aber nur den Feinstaubsensor über die UARTs auschalten und 30 Sekunden vor dem nächsten send_job() wieder einschalten, um das meiste an Strom zu sparen:

```arduino
static osjob_t restartParticleSensorJob;

void do_restartParticleSensor(osjob_t* j) {
  senseBoxIO.powerUART(true);
  Serial.println(F("UARTs on"));
  delay(500);
  sds.begin();
  sds.setQueryReportingMode();
  Serial.println(F("particle sensor restarted"));
}
```

mit einem zusätzlichen os_setTimedCallback(...) 30 Sekunden früher in onEvent()

```arduino
    ...
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
    ...
```

und dem Auschalten der UARTs am Ende von send_job():

```arduino
    ...
    senseBoxIO.powerUART(false);
    Serial.println(F("UARTs off"));
```

Es ist auch sinnvoll das Upload Intervall zu erhöhen:

```arduino
const unsigned TX_INTERVAL = 600;  // 10 minutes
```

Für den Feinstaubsensor wurde die Bibliothek

```arduino
#include <SdsDustSensor.h>
```

verwendet, weil

```arduino
#include <SDS011-select-serial.h>
```

bei den ersten Tests nicht funktioniert hat. Das kann aber ach andere Gründe haben.

## Beispiel Skripte (alt)

Es gibt ein paar Beispiel Skripte zu den senseBoxen beim Nabu Münster und dem MexLab der Uni Münster in den umliegenden Ordnern. Wlan Passwörter und andere Credentials sind durch "TODO" ausgetauscht worden. Die LoRa Daten haben Jan oder Paul.

## unsortierte Notizen

from https://sensebox.github.io/books-v2/home/de/Stromverbrauch.html

senseBox:home mit Feinstaubsensor 100mA 200-300mA
senseBox:home ohne Feinstaubsensor 30-35mA 140mA
senseBox:home ohne einen Sensor angeschlossen 30mA keine Peaks
senseBox:home mit I2C aus 26-30mA keine Peaks
senseBox:home mit UART(Serielle Ports) aus 26-30mA keine Peaks
senseBox:home mit XB2 aus 26-30mA keine Peaks
senseBox:home mit jedem Port aus(XB1,XB2,UART,I2C) 21mA keine Peaks

own tests, mA mA(restart)

default 28 29

sleep 13 28

deep-sleep 13 28

deep-sleep-io-off 5 28

programming mode 15

power off 17

solar board alone 5

nur wlan 113

wlan +few I2C 114

Solarplatte: 20W (oder 10W) 12V -> 4000mA (oder 2000mA)

Akku: https://www.akkuteile.de/keeppower-18650-3500mah-3-6v-3-7v-li-ionen-akku-geschaeuetzt-pluspol-erhaeoeht_12049_1053

Typische Kapazität 3500mAh
Min. Kapazität 3350mAh
Nennspannung 3,6V - 3,7V
Ladeschlussspannung 4,2V ± 0,05V
Entladeschlussspannung 2,5V
Ladestrom/Ladezeit 0 - 1500mA/3,5h

# Energy Consumption

# Current at 5V

| Component                        | Usage              | Current in mA |
| -------------------------------- | ------------------ | ------------- |
| Display                          | full black         | 0.0           |
| Display                          | test text          | 4.3           |
| Display                          | full white         | 30.7          |
| Temperature and Humidity (DPS31) | measurement        | 2.5           |
| RGB LED (WS2814)                 | black              | 0.5           |
| RGB LED (WS2814)                 | 30/255 brightness  | 3.1           |
| RGB LED (WS2814)                 | 255/255 brightness | 34.5          |
| Servo Motor 9g (9g)              | (moving)           | 500           |
| Servo Motor 9g (9g)              | (moving with load) | 500           |
