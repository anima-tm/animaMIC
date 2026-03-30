/*
 * animaMIC.ino — punct de intrare Arduino Nano R4.
 *
 * STEM: aici se leagă obiectele globale (actuatoare, senzor IR, procesor IR, animator) și se pornește
 * bucla cooperativă (Thread::setup în setup(), Thread::run în loop()). Nu există multitasking preemptiv:
 * fiecare modul își primește rândul în loop().
 */

#include "src/Threads.h"
#include "src/IRSense.h"
#include "src/IRProc.h"
#include "src/ServoAct.h"
#include "src/i2c.h"
#include "src/anima.h"

// Patru plăci ServoAct pe I2C: adrese fixe; numele scurte folosite în comanda „act” și în posturi
ServoAct actuator_ox("ox", 0x44);     // ox: ochi stânga/dreapta pe X
ServoAct actuator_oyup("oyup", 0x43); // oyup: ochi Y, urechi, pleoape
ServoAct actuator_mma("mma", 0x42);   // mma: umăr și palmă (mână mare)
ServoAct actuator_mm("mm", 0x41);     // mm: mână mică

IRSense ir_sense;                          // driver UART pentru plăci IRsense
Animator anima;                            // mașină de stări + urmărire țintă
IRProcessor ir_processor(&ir_sense);       // flux de procesare IR pe cadre de la ir_sense

/**
 * Inițializare: UART senzor înainte de USB; apoi Serial USB; apoi Thread::setup() (interpretor + begin() pe toate modulele).
 */
void setup() {
  // UART senzor IR înainte de USB (ordine importantă la inițializare)
  Serial1.begin(115200);
  Serial.begin(115200);
  for (auto startNow = millis() + 2000; !Serial && millis() < startNow; delay(250));

  Serial.println("animaMIC is alive.");

  Thread::setup();
}

/**
 * Bucla principală: citește comenzi pe USB și execută toate modulele Thread (interval 0 sau periodic).
 */
void loop() {
  Thread::run();
}
