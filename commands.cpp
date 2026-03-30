/*
 * commands.cpp
 *
 * animaMIC — Implementare handler-e pentru comenzi serial (alive, reset, ir, teleplot, get, act delegat, anima, state delegat).
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "commands.h"
#include "src/Threads.h"
#include "src/ServoAct.h"
#include "src/IRSense.h"
#include "src/IRProc.h"
#include "src/anima.h"
#include "src/postures.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

extern Animator anima;
extern ServoAct actuator_ox;
extern ServoAct actuator_oyup;
extern ServoAct actuator_mma;
extern ServoAct actuator_mm;

/**
 * Comanda alive — răspuns „Alive!” pentru verificare conexiune.
 * @param arg ignorat
 * @return mereu true
 */
bool cmd_alive(const char* arg) {
    (void)arg;
    THREADS_SERIAL.println("Alive!");
    return true;
}

/**
 * Tipărește o poziție servo sau „X” dacă valoarea e marcatorul de „nu schimba” (în afara intervalului normal).
 * @param v poziție set sau constantă X din ServoAct
 */
static void printPostureValue(float v) {
    if (v > 2.0f || v < -2.0f) {
        THREADS_SERIAL.print("X");
        return;
    }
    char buf[24];
    snprintf(buf, sizeof(buf), "%.2ff", (double)v);
    THREADS_SERIAL.print(buf);
}

/**
 * Subcomanda get posture — afișează pozițiile țintă curente pentru ox, oyup, mma, mm (structură imbricată).
 * @param arg nefolosit
 * @return true
 */
bool cmd_get_posture(const char* arg) {
    (void)arg;
    ServoAct* actuators[] = { &actuator_ox, &actuator_oyup, &actuator_mma, &actuator_mm };
    THREADS_SERIAL.print("{ ");
    for (unsigned int a = 0; a < NUM_ACTUATORS; a++) {
        if (a > 0) THREADS_SERIAL.print(", ");
        THREADS_SERIAL.print("{ ");
        for (uint8_t s = 0; s < MAX_SERVOS_PER_ACTUATOR; s++) {
            if (s > 0) THREADS_SERIAL.print(", ");
            if (s >= actuators[a]->numServos())
                THREADS_SERIAL.print("X");
            else
                printPostureValue(actuators[a]->getSetPosition(s));
        }
        THREADS_SERIAL.print(" }");
    }
    THREADS_SERIAL.println(" }");
    return true;
}

/**
 * Comanda get — în prezent doar „get posture”.
 * @param arg text după „get” (ex.: „posture”)
 * @return true dacă subcomanda e recunoscută
 */
bool cmd_get(const char* arg) {
    if (!arg) {
        THREADS_SERIAL.println("get: usage: get posture");
        return false;
    }
    while (*arg == ' ') arg++;
    if (strcmp(arg, "posture") == 0)
        return cmd_get_posture(nullptr);
    THREADS_SERIAL.println("get: unknown subcommand (posture)");
    return false;
}

/**
 * Comanda ir — trimite restul liniei pe UART către plăcile IRsense (newline adăugat în driver).
 * @param arg conținut de trimis (poate fi gol)
 * @return true
 */
bool cmd_ir_forward(const char* arg) {
    ir_forward_to_uart(arg ? arg : "");
    return true;
}

/**
 * Comanda teleplot — comută canale de vizualizare (urmărire țintă, ir, irf, actors N).
 * @param arg subcomandă și parametri
 * @return true dacă sintaxa e recunoscută
 */
bool cmd_teleplot(const char* arg) {
    const char* p = arg ? arg : "";
    while (*p == ' ') p++;
    if (strncmp(p, "tracker", 7) == 0 && (p[7] == '\0' || p[7] == ' ')) {
        anima.setTeleplotTrackerEnabled(!anima.isTeleplotTrackerEnabled());
        THREADS_SERIAL.print("teleplot tracker ");
        THREADS_SERIAL.println(anima.isTeleplotTrackerEnabled() ? "on" : "off");
        return true;
    }
    if (strncmp(p, "actors", 6) == 0) {
        p += 6;
        while (*p == ' ') p++;
        if (*p == '\0') {
            THREADS_SERIAL.println("teleplot: usage: teleplot actors <N>");
            return false;
        }
        int n = atoi(p);
        if (n < 0) {
            THREADS_SERIAL.println("teleplot: actors N must be >= 0");
            return false;
        }
        ir_processor.setTeleplotActorsCount(n);
        THREADS_SERIAL.print("teleplot actors ");
        THREADS_SERIAL.println(n);
        return true;
    }
    if (strncmp(p, "irf", 3) == 0) {
        p += 3;
        while (*p == ' ') p++;
        if (*p != '\0') {
            THREADS_SERIAL.println("teleplot: usage: teleplot tracker|ir|irf|actors [N]");
            return false;
        }
        ir_processor.setTeleplotIrfEnabled(!ir_processor.getTeleplotIrfEnabled());
        THREADS_SERIAL.print("teleplot irf ");
        THREADS_SERIAL.println(ir_processor.getTeleplotIrfEnabled() ? "on" : "off");
        return true;
    }
    if (strncmp(p, "ir", 2) != 0) {
        THREADS_SERIAL.println("teleplot: usage: teleplot tracker|ir|irf|actors [N]");
        return false;
    }
    p += 2;
    while (*p == ' ') p++;
    if (*p == '\0') {
        ir_processor.setTeleplotIrEnabled(!ir_processor.getTeleplotIrEnabled());
        THREADS_SERIAL.print("teleplot ir ");
        THREADS_SERIAL.println(ir_processor.getTeleplotIrEnabled() ? "on" : "off");
        return true;
    }
    if (isdigit((unsigned char)*p)) {
        int n = atoi(p);
        if (n < 1) {
            THREADS_SERIAL.println("teleplot: sensor index must be >= 1");
            return false;
        }
        size_t max_s = ir_sense.getNumSensors();
        if (max_s > 0 && (size_t)n > max_s) {
            THREADS_SERIAL.println("teleplot: sensor index out of range");
            return false;
        }
        ir_processor.setTeleplotSensorEnabled(n, !ir_processor.getTeleplotSensorEnabled(n));
        THREADS_SERIAL.print("teleplot ir");
        THREADS_SERIAL.print(n);
        THREADS_SERIAL.print(" ");
        THREADS_SERIAL.println(ir_processor.getTeleplotSensorEnabled(n) ? "on" : "off");
        return true;
    }
    THREADS_SERIAL.println("teleplot: usage: teleplot tracker|ir|irf|actors [N]");
    return false;
}

/**
 * Reset software al microcontrollerului după mesaj pe serial; nu revine (în practică).
 * @param arg ignorat
 */
bool cmd_reset(const char* arg) {
    (void)arg;
    THREADS_SERIAL.println("Resetting...");
    THREADS_SERIAL.flush();
    delay(200);

#if defined(__AVR__)
        wdt_enable(WDTO_15MS);
        while(1) {}
#elif defined(__ARM_ARCH) || defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_RENESAS_UNO)
        #define SCB_AIRCR (*(volatile uint32_t*)0xE000ED0C)
        SCB_AIRCR = 0x05FA0004;
        while(1) {}
#else
        void(* resetFunc) (void) = 0;
        resetFunc();
#endif

    return true;
}

bool getIntArg(const char* arg, uint8_t index, int* value) {
    if (!arg || !value) {
        return false;
    }
    const char* p = arg;
    uint8_t current_index = 0;
    while (*p == ' ') {
        p++;
    }
    while (current_index < index) {
        while (*p != ' ' && *p != '\0') {
            p++;
        }
        if (*p == '\0') {
            return false;
        }
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            return false;
        }
        current_index++;
    }
    *value = atoi(p);
    return true;
}

bool getFloatArg(const char* arg, uint8_t index, float* value) {
    if (!arg || !value) {
        return false;
    }
    const char* p = arg;
    uint8_t current_index = 0;
    while (*p == ' ') {
        p++;
    }
    while (current_index < index) {
        while (*p != ' ' && *p != '\0') {
            p++;
        }
        if (*p == '\0') {
            return false;
        }
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            return false;
        }
        current_index++;
    }
    char* endptr;
    float result = strtof(p, &endptr);
    if (endptr == p) {
        return false;
    }
    if (*endptr != ' ' && *endptr != '\0') {
        return false;
    }
    *value = result;
    return true;
}

const CommandEntry commands[] = {
    {"alive", cmd_alive},
    {"reset", cmd_reset},
    {"ir", cmd_ir_forward},
    {"teleplot", cmd_teleplot},
    {"get", cmd_get},
    {"act", ServoAct::cmd_act},
    {"anima", Animator::cmd_anima},
    {"state", Animator::cmd_state}
};

const uint8_t commands_count = sizeof(commands) / sizeof(commands[0]);
