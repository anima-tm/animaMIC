/*
 * anima_tools.h
 *
 * animaMIC — Unelte comune: LED RGB de stare, buton cu evenimente, mașină de stări AnimaState, td_random.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: thread-uri cu interval fix pentru LED și buton; AnimaState = registru global + tranziții
 * orchestrate de Animator::loop prin runStateMachine.
 */

#ifndef ANIMA_TOOLS_H
#define ANIMA_TOOLS_H

#include "Threads.h"
#include <stdint.h>
#include <vector>

/** Durate min/max între comutări în modul random_blink (ms). */
#define BLINK_MIN_DURATION 50
#define BLINK_MAX_DURATION 300

/** Pin buton (GND + pull-up intern); A7 pe Nano R4. */
#define BUTTON_PIN 21
/** Praguri timp pentru debounce, click, apăsare lungă, secret double-tap (ms). */
#define BUTTON_DEBOUNCE_TIME 100
#define BUTTON_CLICK_MAX_TIME 800
#define BUTTON_PRESS_MIN_TIME 2500
#define BUTTON_SECRETCLICK_TIME 1500

#define DIAGNOSE_STATES 1

/** Evită auto-activarea ServoAct în boot când anima conduce secvența. */
bool isAnimatorInBootState();

/**
 * Generator pseudo-aleator ușor (16 biți din micros + stare); interval [min_val, max_val] inclusiv.
 */
uint16_t td_random(uint16_t min_val, uint16_t max_val);

/** RGB 8 biți per canal. */
struct Color {
    uint8_t r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
};

/** boot / activ / activ cu erori / focus IR / eroare — vezi anima_tools.cpp pentru valori RGB. */
extern const Color COLOR_BOOT;
extern const Color COLOR_ACTIVE;
extern const Color COLOR_ACTIVE_WITH_ERRORS;
extern const Color COLOR_FOCUS;
extern const Color COLOR_ERROR;

/**
 * LED RGB (Nano R4: LEDR/G/B), active-low. Moduri: set fix, blink, random blink, puls.
 */
class ColorLed : public Thread {
public:
    ColorLed();
    void begin() override;
    void loop() override;

    /** Memorează culoarea logică (fără a schimba modul). */
    void setColor(Color c);
    Color getColor() const { return color_; }
    /** Lumină continuă; oprește blink/pulse. */
    void set(Color c);
    /** Dreptunghi periodic: jumătate perioadă on, jumătate off. */
    void blink(Color c, uint32_t period_ms);
    /** Intervale aleatoare între toggle (BLINK_MIN/MAX). */
    void random_blink(Color c);
    /** Variație pseudo-sinusoidală a intensității. */
    void pulse(Color c, uint32_t period_ms);

private:
    enum class Mode { SET, BLINK, RANDOM_BLINK, PULSE };
    void writeRgb(uint8_t r, uint8_t g, uint8_t b);

    Color color_;
    Mode mode_;
    uint8_t r_, g_, b_;
    uint32_t pulse_period_ms_;
    bool blink_on_;
    uint32_t blink_until_ms_;
    uint32_t blink_period_half_ms_;
};

/**
 * Buton la GND: debounce, click scurt, apăsare lungă, secret_click (două eliberări apropiate).
 */
class PushButton : public Thread {
public:
    enum class Event { none = 0, click, press, secret_click };

    explicit PushButton(uint8_t pin);
    void begin() override;
    void loop() override;

    bool get_state() const { return debounced_pressed_; }
    /** Returnează ultimul eveniment și îl resetează la none. */
    Event get_event();

private:
    uint8_t pin_;
    bool raw_last_;
    bool debounced_pressed_;
    uint32_t debounce_start_ms_;
    uint32_t pressed_since_ms_;
    uint32_t press_released_at_ms_;
    Event event_;
    bool fired_press_event_;
};

class Animator;
struct Posture;
class Gesture;

/**
 * Stare nominală anima: nume, timpi intrare/ieșire, înregistrare în vector static.
 * Tranziții: changeState setează pending; runStateMachine aplică end/begin și șterge stări marcate.
 */
class AnimaState {
    friend class Animator;
public:
    explicit AnimaState(const char* name);
    virtual ~AnimaState();

    virtual void begin(Animator* a);
    virtual void end();
    virtual void loop(Animator* a) { (void)a; }

    uint32_t getLastEntryTime() const { return last_entry_ms_; }
    uint32_t getLastExitTime() const { return last_exit_ms_; }

    const char* getName() const { return name_ ? name_ : ""; }

    static void changeState(Animator* a, AnimaState* next) { (void)a; pending_state_ = next; }

    static AnimaState* getCurrentState() { return current_state_; }
    static AnimaState* getPreviousState() { return previous_state_; }
    static void setCurrentState(AnimaState* s) { current_state_ = s; }

    void markReadyForDeletion() { ready_for_deletion_ = true; }
    bool readyForDeletion() const { return ready_for_deletion_; }

    static void runStateMachine(Animator* a);

    static size_t getRegistryCount() { return registry_.size(); }
    static AnimaState* getStateFromRegistry(size_t i);
    static AnimaState* findByName(const char* name);

    /** Delegă Animator::takePosture; dacă DIAGNOSE_STATES, mesaj serial. */
    bool takePosture(Animator* a, const Posture* post, uint16_t time_ms = 1000, bool eyes_too = true);
    /** Delegă Animator::runGesture; dacă DIAGNOSE_STATES, mesaj serial. */
    bool runGesture(Animator* a, Gesture* g);

private:
    static void deleteStatesMarkedForDeletion();

    const char* name_;
    bool ready_for_deletion_ = false;
    uint32_t last_entry_ms_ = 0;
    uint32_t last_exit_ms_ = 0;
    static std::vector<AnimaState*> registry_;
    static AnimaState* current_state_;
    static AnimaState* pending_state_;
    static AnimaState* previous_state_;
};

#endif /* ANIMA_TOOLS_H */
