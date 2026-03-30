/*
 * gestures.cpp
 *
 * animaMIC — Registru static de gesturi (max MAX_GESTURES) și implementări concrete (subclase anonime globale).
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: fiecare obiect global (ex. wave1_left) construiește Gesture → registerGesture la load; run() programează cozi pe actuator_mm / mma / ox / oyup.
 */

#include "gestures.h"
#include "ServoAct.h"
#include "postures.h"
#include "anima_tools.h"
#include <string.h>
#include <math.h>

extern ServoAct actuator_mm;
extern ServoAct actuator_ox;
extern ServoAct actuator_oyup;
extern ServoAct actuator_mma;

static const unsigned int MAX_GESTURES = 16;
static Gesture* s_gestures[MAX_GESTURES];
static unsigned int s_count = 0;

/** La construirea oricărui gest derivat: înregistrare în s_gestures[]. */
Gesture::Gesture() {
    registerGesture(this);
}

void registerGesture(Gesture* g) {
    if (!g || s_count >= MAX_GESTURES)
        return;
    s_gestures[s_count++] = g;
}

/** Căutare liniară după name(). */
Gesture* findGesture(const char* name) {
    if (!name)
        return nullptr;
    for (unsigned int i = 0; i < s_count; i++) {
        if (strcmp(s_gestures[i]->name(), name) == 0)
            return s_gestures[i];
    }
    return nullptr;
}

unsigned int gestureCount() {
    return s_count;
}

/** Acces pentru enumerare (ex. comanda anima fără argument). */
Gesture* gestureAt(unsigned int index) {
    if (index >= s_count)
        return nullptr;
    return s_gestures[index];
}

// --- Valuri braț stânga (actuator mm, un servo) ---

/** Val în trei timpi în jurul poziției curente (sus/jos după semnul p0). */
class Wave1Left : public Gesture {
public:
    const char* name() const override { return "wave1_left"; }
    uint32_t duration() const override { return 600U + 1000U + 600U; }
    bool run() override {
        float p0 = actuator_mm.getSetPosition(0);
        float peak = (p0 + 0.5f) > 1.0f ? 1.0f : (p0 + 0.5f);
        float low = (p0 - 0.5f) < -1.0f ? -1.0f : (p0 - 0.5f);
        if (p0 >= 0.5f) {
            actuator_mm.go_to(low, (uint16_t)600);
            actuator_mm.then_go_to(peak, (uint16_t)1000);
            actuator_mm.then_go_to(p0, (uint16_t)600);
        } else {
            actuator_mm.go_to(peak, (uint16_t)600);
            actuator_mm.then_go_to(low, (uint16_t)1000);
            actuator_mm.then_go_to(p0, (uint16_t)600);
        }
        return true;
    }
} wave1_left;

/** Val scurt (două segmente) pe mm. */
class Wave2Left : public Gesture {
public:
    const char* name() const override { return "wave2_left"; }
    uint32_t duration() const override { return 300U + 300U; }
    bool run() override {
        float p0 = actuator_mm.getSetPosition(0);
        float peak = (p0 + 0.7f) > 1.0f ? 1.0f : (p0 + 0.7f);
        float low = (p0 - 0.7f) < -1.0f ? -1.0f : (p0 - 0.7f);
        if (p0 >= 0.0f) {
            actuator_mm.go_to(low, (uint16_t)300);
            actuator_mm.then_go_to(p0, (uint16_t)300);
        } else {
            actuator_mm.go_to(peak, (uint16_t)300);
            actuator_mm.then_go_to(p0, (uint16_t)300);
        }
        return true;
    }
} wave2_left;

// --- Valuri braț dreapta (mma: servo 1 ținut la X dacă e cazul) ---

/** Val în trei timpi pe mma (servo index 1), X pe celălalt canal. */
class Wave1Right : public Gesture {
public:
    const char* name() const override { return "wave1_right"; }
    uint32_t duration() const override { return 600U + 1000U + 600U; }
    bool run() override {
        float p0 = actuator_mma.getSetPosition(1);
        float peak = (p0 + 0.8f) > 1.0f ? 1.0f : (p0 + 0.8f);
        float low = (p0 - 0.8f) < -1.0f ? -1.0f : (p0 - 0.8f);
        if (p0 >= 0.5f) {
            actuator_mma.go_to(X, low, (uint16_t)600);
            actuator_mma.then_go_to(X, peak, (uint16_t)1000);
            actuator_mma.then_go_to(X, p0, (uint16_t)600);
        } else {
            actuator_mma.go_to(X, peak, (uint16_t)600);
            actuator_mma.then_go_to(X, low, (uint16_t)1000);
            actuator_mma.then_go_to(X, p0, (uint16_t)600);
        }
        return true;
    }
} wave1_right;

/** Val scurt pe mma. */
class Wave2Right : public Gesture {
public:
    const char* name() const override { return "wave2_right"; }
    uint32_t duration() const override { return 300U + 300U; }
    bool run() override {
        float p0 = actuator_mma.getSetPosition(1);
        float peak = (p0 + 0.7f) > 1.0f ? 1.0f : (p0 + 0.7f);
        float low = (p0 - 0.7f) < -1.0f ? -1.0f : (p0 - 0.7f);
        if (p0 >= 0.0f) {
            actuator_mma.go_to(X, low, (uint16_t)300);
            actuator_mma.then_go_to(X, p0, (uint16_t)300);
        } else {
            actuator_mma.go_to(X, peak, (uint16_t)300);
            actuator_mma.then_go_to(X, p0, (uint16_t)300);
        }
        return true;
    }
} wave2_right;

/** Mișcare scurtă ambele axe mma; direcție după poziția servo 0. */
class Wave3Right : public Gesture {
public:
    const char* name() const override { return "wave3_right"; }
    uint32_t duration() const override { return 200U + 400U + 500U; }
    bool run() override {
        /* Ambele servomotoare mma; direcție: dacă mma servo0 < -0.6, delta +, altfel -. */
        float p0_0 = actuator_mma.getSetPosition(0);
        float p0_1 = actuator_mma.getSetPosition(1);
        float delta = (p0_0 < -0.6f) ? 0.5f : -0.5f;

        float t0 = p0_0 + delta;
        float t1 = p0_1 + delta;
        if (t0 > 1.0f) t0 = 1.0f;
        if (t0 < -1.0f) t0 = -1.0f;
        if (t1 > 1.0f) t1 = 1.0f;
        if (t1 < -1.0f) t1 = -1.0f;

        actuator_mma.go_to(t0, t1, (uint16_t)400);
        actuator_mma.delay_next_goto(200);
        actuator_mma.then_go_to(p0_0, p0_1, (uint16_t)500);
        return true;
    }
} wave3_right;

/** Val compus pe mma: mai mulți pași + întoarcere la poziția inițială. */
class Wave4Right : public Gesture {
public:
    const char* name() const override { return "wave4_right"; }
    uint32_t duration() const override { return 600U + 200U + 400U + 400U + 200U + 500U; }
    bool run() override {
        float p0_0 = actuator_mma.getSetPosition(0);
        float p0_1 = actuator_mma.getSetPosition(1);
        float dir = (p0_0 > 0.0f) ? -1.0f : 1.0f;

        float t0_0 = p0_0 + dir * 0.8f;
        float t0_1 = p0_1 + dir * 0.5f;
        if (t0_0 > 1.0f) t0_0 = 1.0f;
        if (t0_0 < -1.0f) t0_0 = -1.0f;
        if (t0_1 > 1.0f) t0_1 = 1.0f;
        if (t0_1 < -1.0f) t0_1 = -1.0f;
        actuator_mma.go_to(t0_0, t0_1, (uint16_t)600);

        float t1_1 = t0_1 + dir * (-0.5f);
        if (t1_1 > 1.0f) t1_1 = 1.0f;
        if (t1_1 < -1.0f) t1_1 = -1.0f;
        actuator_mma.delay_next_goto(200);
        actuator_mma.then_go_to(X, t1_1, (uint16_t)400);

        float t2_1 = t1_1 + dir * 0.5f;
        if (t2_1 > 1.0f) t2_1 = 1.0f;
        if (t2_1 < -1.0f) t2_1 = -1.0f;
        actuator_mma.then_go_to(X, t2_1, (uint16_t)400);

        actuator_mma.delay_next_goto(200);
        actuator_mma.then_go_to(p0_0, p0_1, (uint16_t)500);
        return true;
    }
} wave4_right;

// --- Invite / scare (mm + mma sau oyup) ---

/** Secvență sincronă mm / mma: poziții extinse apoi revenire la setpoint salvat. */
class Invite1 : public Gesture {
public:
    const char* name() const override { return "invite1"; }
    uint32_t duration() const override { return 1000U + 600U + 1000U + 1000U; }
    bool run() override {
        const float mm0 = actuator_mm.getSetPosition(0);
        const float mma0 = actuator_mma.getSetPosition(0);
        const float mma1 = actuator_mma.getSetPosition(1);

        actuator_mm.go_to(-0.7f, (uint16_t)1000);
        actuator_mma.go_to(-0.7f, -0.5f, (uint16_t)1000);

        actuator_mm.then_go_to(1.0f, (uint16_t)600);
        actuator_mma.then_go_to(1.0f, -0.5f, (uint16_t)600);

        actuator_mm.delay_next_goto(1000);
        actuator_mm.then_go_to(mm0, (uint16_t)1000);
        actuator_mma.delay_next_goto(1000);
        actuator_mma.then_go_to(mma0, mma1, (uint16_t)1000);
        return true;
    }
} invite1;

/**
 * „Scara” pe mma servo0 până la +1 cu pași intermediari; durata depinde de stepCount(p0).
 */
class Invite2 : public Gesture {
public:
    const char* name() const override { return "invite2"; }
    uint32_t duration() const override {
        const float p0 = actuator_mma.getSetPosition(0);
        const uint32_t n = stepCount(p0);
        const uint32_t between = (n > 1U) ? (n - 1U) * (500U + 600U) : 0U;
        /* Durată aliniată buclei run(): primul go_to, pași then, pauză, întoarcere. */
        return 600U + n * 600U + between + 1000U + 1500U;
    }
    bool run() override {
        const float p0_saved = actuator_mma.getSetPosition(0);
        const float p1_saved = actuator_mma.getSetPosition(1);

        actuator_mma.go_to(X, -0.9f, (uint16_t)600);

        float p0_track = p0_saved;
        for (;;) {
            float next = p0_track + 0.5f;
            if (next > 1.0f)
                next = 1.0f;
            actuator_mma.then_go_to(next, 0.5f, (uint16_t)600);
            p0_track = next;
            if (p0_track >= 1.0f)
                break;
            actuator_mma.then_go_to(X, -0.9f, (uint16_t)600);
        }

        actuator_mma.delay_next_goto(1500);
        actuator_mma.then_go_to(p0_saved, p1_saved, (uint16_t)1500);
        return true;
    }

private:
    /** Număr de pași +0.5 pe servo0 până la 1; aliniat cu bucla din run(). */
    static uint32_t stepCount(float p0_start);
} invite2;

uint32_t Invite2::stepCount(float p0_start) {
    uint32_t n = 0;
    float p0_track = p0_start;
    do {
        float next = p0_track + 0.5f;
        if (next > 1.0f)
            next = 1.0f;
        n++;
        p0_track = next;
    } while (p0_track < 1.0f);
    return n;
}

/** mma apoi mm: pauze lungi, revenire la poziții salvate. */
class Invite3 : public Gesture {
public:
    const char* name() const override { return "invite3"; }
    uint32_t duration() const override { return 4000U + 1000U; }
    bool run() override {
        const float mm0 = actuator_mm.getSetPosition(0);
        const float mma0 = actuator_mma.getSetPosition(0);
        const float mma1 = actuator_mma.getSetPosition(1);

        actuator_mma.go_to(1.0f, -0.5f, (uint16_t)500);
        actuator_mma.delay_next_goto(3500);
        actuator_mma.then_go_to(mma0, mma1, (uint16_t)1000);

        actuator_mm.delay_next_goto(1500);
        actuator_mm.go_to(-0.8f, (uint16_t)500);
        actuator_mm.then_go_to(1.0f, (uint16_t)500);
        actuator_mm.delay_next_goto(4000);
        actuator_mm.then_go_to(mm0, (uint16_t)1000);
        return true;
    }
} invite3;

/** Clipire / sperietură: oyup.2 (ureche) jos apoi sus lent. */
class Scare : public Gesture {
public:
    const char* name() const override { return "scare"; }
    uint32_t duration() const override { return 300U + 500U + 2500U; }
    bool run() override {
        actuator_oyup.go_to(X, -0.5f, (uint16_t)300);
        actuator_oyup.delay_next_goto(500);
        actuator_oyup.then_go_to(X, 0.75f, (uint16_t)2500);
        return true;
    }
} scare;

/** Rulare ochi: oyup (poziție apoi neutral) + ox extrem apoi centru; durată estimată 2500 ms. */
class RollEyes : public Gesture {
public:
    const char* name() const override { return "roll_eyes"; }
    uint32_t duration() const override { return 2500U; }
    bool run() override {
        actuator_oyup.go_to(1.0f, X, (uint16_t)500);
        actuator_oyup.then_go_to(0.0f, X, (uint16_t)1500);
        actuator_ox.go_to(-1.0f, 1.0f, (uint16_t)1000);
        actuator_ox.then_go_to(0.0f, 0.0f, (uint16_t)1500);
        return true;
    }
} roll_eyes;

/**
 * Intrare approach→focus: ridică urechea (oyup.2); dacă ochii prea jos, aduce și oyup.1 într-un singur go_to.
 */
class FocusGesture : public Gesture {
public:
    const char* name() const override { return "focus"; }
    uint32_t duration() const override { return 200U; }
    bool run() override {
        float p0 = actuator_oyup.getSetPosition(0);
        if (p0 < -0.3f)
            actuator_oyup.go_to(0.0f, 1.0f, (uint16_t)200);
        else
            actuator_oyup.go_to(X, 1.0f, (uint16_t)0);
        return true;
    }
} focus;

/**
 * Aliniere la postura „mic”: pentru oyup.2, mma, mm — dacă abatere > 0.3, țintă aleatoare în jurul valorii „mic”; ox netratat; pornire simultană 1200 ms.
 */
class StraightenGesture : public Gesture {
public:
    const char* name() const override { return "straighten"; }
    uint32_t duration() const override { return 1200U; }
    bool run() override {
        const Posture* post = getPostureByName("mic");
        if (!post) return false;
        const uint16_t time_ms = 1200;

        /* oyup.2 = positions[1][1]; oyup.1 rămâne la curent */
        const float mic_oyup2 = post->positions[1][1];
        float cur_oyup1 = actuator_oyup.getSetPosition(0);
        float cur_oyup2 = actuator_oyup.getSetPosition(1);
        float t_oyup2 = (fabsf(cur_oyup2 - mic_oyup2) > 0.3f) ? clampMicRandom(mic_oyup2) : cur_oyup2;

        /* mma: positions[2][0], [2][1] */
        const float mic_mma1 = post->positions[2][0];
        const float mic_mma2 = post->positions[2][1];
        float cur_mma1 = actuator_mma.getSetPosition(0);
        float cur_mma2 = actuator_mma.getSetPosition(1);
        float t_mma1 = (fabsf(cur_mma1 - mic_mma1) > 0.3f) ? clampMicRandom(mic_mma1) : cur_mma1;
        float t_mma2 = (fabsf(cur_mma2 - mic_mma2) > 0.3f) ? clampMicRandom(mic_mma2) : cur_mma2;

        /* mm: positions[3][0] */
        const float mic_mm1 = post->positions[3][0];
        float cur_mm1 = actuator_mm.getSetPosition(0);
        float t_mm1 = (fabsf(cur_mm1 - mic_mm1) > 0.3f) ? clampMicRandom(mic_mm1) : cur_mm1;

        actuator_oyup.go_to(cur_oyup1, t_oyup2, time_ms);
        actuator_mma.go_to(t_mma1, t_mma2, time_ms);
        actuator_mm.go_to(t_mm1, time_ms);
        return true;
    }

private:
    /** Offset aleator mic în jurul țintei „mic”, tăiat la [-1,1]. */
    static float clampMicRandom(float mic) {
        float offset = ((float)td_random(0, 600) / 300.0f - 1.0f) * 0.2f;
        float t = mic + offset;
        if (t < -1.0f) return -1.0f;
        if (t > 1.0f) return 1.0f;
        return t;
    }
} straighten;

/** Test latență I2C: ox ambele axe, trei segmente (depanare). */
class LatencyTest : public Gesture {
public:
    const char* name() const override { return "latency_test"; }
    uint32_t duration() const override { return 300U + 600U + 300U; }
    bool run() override {
        ServoAct* act = &actuator_ox;
        float p0 = act->getSetPosition(0);
        float peak = (p0 + 0.5f) > 1.0f ? 1.0f : (p0 + 0.5f);
        float low = (peak - 1.0f) < -1.0f ? -1.0f : (peak - 1.0f);
        act->go_to(peak, peak, (uint16_t)300);
        act->then_go_to(low, low, (uint16_t)600);
        act->then_go_to(p0, p0, (uint16_t)300);
        return true;
    }
} latency_test;
