/*
 * postures.h
 *
 * animaMIC — Posturi statice: nume + matrice de poziții normalizate [-1,1] per actuator și per servo.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: ordinea actuatoarelor e aceeași ca în Animator (ox, oyup, mma, mm); simbolul X (ServoAct) = servo lăsat neschimbat la takePosture.
 * Comanda: anima take <name>.
 */

#ifndef POSTURES_H
#define POSTURES_H

#include <stdint.h>
#include <string.h>
#include "ServoAct.h"

/** Număr de actuatoare (ox, oyup, mma, mm); aliniat la Animator și la dimensiunea Posture::positions. */
static const unsigned int NUM_ACTUATORS = 4;

/**
 * Postură: identificator text și poziții țintă per actuator [servo0, servo1, ...] până la MAX_SERVOS_PER_ACTUATOR.
 */
struct Posture {
    const char* name;  /* identificator pentru anima take */
    float positions[NUM_ACTUATORS][MAX_SERVOS_PER_ACTUATOR];
};


static const Posture POSTURES[] = {
    /* nume   | ox (stânga, dreapta) | oyup (ochi X, ureche) | mma (umeri)        | mm                  */
    { "mic",         { {  0.20f,     0.20f },   {  -0.30f,     1.00f },    {  -0.35f,    -0.20f  },   {  0.00f,  X } }    },
    { "melt",        { { -1.00f,     1.00f },   {  -0.50f,    -1.00f },    {   1.00f,     1.00f  },   { -1.00f,  X } }    },
    { "sleepy",      { {  0.25f,    -0.25f },   {  -0.50f,    -0.60f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },
    { "surrender",   { {  0.00f,     0.00f },   {   0.60f,     0.30f },    {  -0.60f,    -0.25f  },   {  1.00f,  X } }    },
    { "showdoor",    { {  0.50f,     0.50f },   {   0.00f,     0.7f },    {   1.00f,    -0.50f  },   {  0.70f,  X } }    },
    { "hello",       { {  0.00f,     0.00f },   {   0.50f,     0.70f },    {  -0.60f,     0.50f  },   { -0.30f,  X } }    },
    { "hug",         { { -0.25f,     0.25f },   {   0.00f,     0.60f },    {   0.30f,    -0.20f  },   {  0.20f,  X } }    },
    { "angry",       { {  0.80f,    -0.80f },   {  -0.60f,    -0.75f },    {  -0.80f,    -0.80f  },   {  0.00f,  X } }    },
    // { "",      { {  0.00f,     0.00f },   {  -0.50f,    -0.50f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },
    // { "",      { {  0.00f,     0.00f },   {  -0.50f,    -0.50f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },
    // { "",      { {  0.00f,     0.00f },   {  -0.50f,    -0.50f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },
    // { "",      { {  0.00f,     0.00f },   {  -0.50f,    -0.50f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },
    // { "",      { {  0.00f,     0.00f },   {  -0.50f,    -0.50f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },
    // { "",      { {  0.00f,     0.00f },   {  -0.50f,    -0.50f },    {   0.00f,     0.60f  },   { -0.40f,  X } }    },

    
    


};

static const uint8_t POSTURE_COUNT = sizeof(POSTURES) / sizeof(POSTURES[0]);

/** Caută în POSTURES după name; nullptr dacă nu există. */
inline const Posture* getPostureByName(const char* name) {
    if (!name) return nullptr;
    for (uint8_t i = 0; i < POSTURE_COUNT; i++) {
        if (strcmp(name, POSTURES[i].name) == 0) return &POSTURES[i];
    }
    return nullptr;
}

#endif // POSTURES_H
