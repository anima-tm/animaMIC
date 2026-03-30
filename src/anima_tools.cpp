/*
 * anima_tools.cpp
 *
 * animaMIC — Implementare: td_random, AnimaState (registru, runStateMachine), ColorLed, PushButton.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "anima_tools.h"
#include "anima.h"
#include <Arduino.h>
#include <string.h>

/** Pseudo-aleator 16 biți: micros, stare statică, mapare în [min_val, max_val]. */
uint16_t td_random(uint16_t min_val, uint16_t max_val) {
    static uint16_t state = 0;
    uint16_t raw = (uint16_t)(micros() & 0xFFFFU);
    uint32_t x = (uint32_t)raw ^ ((uint32_t)state << 7);
    x ^= x << 7;
    x ^= x >> 9;
    x ^= x << 8;
    uint16_t mixed = (uint16_t)(x & 0xFFFFU);
    state = (uint16_t)((state + raw) ^ (mixed >> 1));
    if (min_val >= max_val)
        return min_val;
    uint32_t range = (uint32_t)(max_val - min_val + 1);
    return (uint16_t)(min_val + (mixed % range));
}

/* Monitor stare: boot / activ / erori / focus IR / eroare */
const Color COLOR_BOOT(0, 0, 255);
const Color COLOR_ACTIVE(0, 255, 0);
const Color COLOR_ACTIVE_WITH_ERRORS(255, 165, 0);
const Color COLOR_FOCUS(0, 255, 255);
const Color COLOR_ERROR(255, 0, 0);

/** Starea curentă nume „boot”. */
bool isAnimatorInBootState() {
    AnimaState* c = AnimaState::getCurrentState();
    return c && strcmp(c->getName(), "boot") == 0;
}

std::vector<AnimaState*> AnimaState::registry_;
AnimaState* AnimaState::current_state_ = nullptr;
AnimaState* AnimaState::pending_state_ = nullptr;
AnimaState* AnimaState::previous_state_ = nullptr;

/** Înregistrare automată în registry_. */
AnimaState::AnimaState(const char* name) : name_(name) {
    registry_.push_back(this);
}

/** Elimină this din registry_ (swap cu ultimul). */
AnimaState::~AnimaState() {
    for (size_t i = 0; i < registry_.size(); i++) {
        if (registry_[i] == this) {
            registry_[i] = registry_.back();
            registry_.pop_back();
            break;
        }
    }
}

/** last_entry_ms_ = millis(). */
void AnimaState::begin(Animator* a) {
    (void)a;
    last_entry_ms_ = millis();
}

/** Înregistrează momentul ieșirii din stare. */
void AnimaState::end() {
    last_exit_ms_ = millis();
}

/** Acces indexat la registry_. */
AnimaState* AnimaState::getStateFromRegistry(size_t i) {
    return (i < registry_.size()) ? registry_[i] : nullptr;
}

/** Căutare liniară după getName(). */
AnimaState* AnimaState::findByName(const char* name) {
    if (!name) return nullptr;
    for (AnimaState* s : registry_) {
        if (strcmp(s->getName(), name) == 0)
            return s;
    }
    return nullptr;
}

/** Șterge din registry stările marcate, cu excepția stării curente. */
void AnimaState::deleteStatesMarkedForDeletion() {
    AnimaState* cur = getCurrentState();
    size_t i = registry_.size();
    while (i > 0) {
        i--;
        AnimaState* s = registry_[i];
        if (s == cur || !s->readyForDeletion())
            continue;
        delete s;
        /* Destructorul scurtează vectorul; reiau de la noul size. */
        i = registry_.size();
    }
}

/**
 * Aplică pending: end stare veche, begin nouă; opțional diagnostic; șterge stări marcate.
 */
void AnimaState::runStateMachine(Animator* a) {
    if (!a || !pending_state_)
        return;
    previous_state_ = current_state_;
    if (current_state_)
        current_state_->end();
    pending_state_->begin(a);
    current_state_ = pending_state_;
    pending_state_ = nullptr;
#if DIAGNOSE_STATES
    Serial.print("State switch: ");
    Serial.print(previous_state_ ? previous_state_->getName() : "");
    Serial.print(" -> ");
    Serial.println(current_state_->getName());
#endif
    deleteStatesMarkedForDeletion();
}

/** Thread 10 ms; mod SET implicit. */
ColorLed::ColorLed()
    : Thread(10),
      mode_(Mode::SET),
      r_(0), g_(0), b_(0),
      pulse_period_ms_(1000),
      blink_on_(true),
      blink_until_ms_(0),
      blink_period_half_ms_(0) {
    color_.r = 0;
    color_.g = 0;
    color_.b = 0;
}

/** Pini LEDR/G/B ca ieșiri analogice. */
void ColorLed::begin() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    writeRgb(0, 0, 0);
}

void ColorLed::setColor(Color c) {
    color_ = c;
}

void ColorLed::set(Color c) {
    color_ = c;
    mode_ = Mode::SET;
    r_ = c.r;
    g_ = c.g;
    b_ = c.b;
}

void ColorLed::blink(Color c, uint32_t period_ms) {
    color_ = c;
    mode_ = Mode::BLINK;
    r_ = c.r;
    g_ = c.g;
    b_ = c.b;
    blink_on_ = true;
    const uint32_t half = (period_ms > 0) ? (period_ms / 2) : 1;
    blink_period_half_ms_ = half;
    blink_until_ms_ = millis() + half;
}

void ColorLed::random_blink(Color c) {
    color_ = c;
    mode_ = Mode::RANDOM_BLINK;
    r_ = c.r;
    g_ = c.g;
    b_ = c.b;
    blink_on_ = true;
    blink_until_ms_ = millis() + td_random(BLINK_MIN_DURATION, BLINK_MAX_DURATION);
}

void ColorLed::pulse(Color c, uint32_t period_ms) {
    color_ = c;
    mode_ = Mode::PULSE;
    r_ = c.r;
    g_ = c.g;
    b_ = c.b;
    pulse_period_ms_ = (period_ms > 0) ? period_ms : 1;
}

/** Inversare 255-x pentru active-low. */
void ColorLed::writeRgb(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(LEDR, 255 - r);
    analogWrite(LEDG, 255 - g);
    analogWrite(LEDB, 255 - b);
}

/** O iterație: SET constant sau blink / random / puls după mode_. */
void ColorLed::loop() {
    switch (mode_) {
    case Mode::SET:
        writeRgb(r_, g_, b_);
        break;
    case Mode::BLINK: {
        const uint32_t now = millis();
        if (now >= blink_until_ms_) {
            blink_on_ = !blink_on_;
            blink_until_ms_ = now + blink_period_half_ms_;
        }
        if (blink_on_)
            writeRgb(r_, g_, b_);
        else
            writeRgb(0, 0, 0);
        break;
    }
    case Mode::RANDOM_BLINK: {
        const uint32_t now = millis();
        if (now >= blink_until_ms_) {
            blink_on_ = !blink_on_;
            blink_until_ms_ = now + td_random(BLINK_MIN_DURATION, BLINK_MAX_DURATION);
        }
        if (blink_on_)
            writeRgb(r_, g_, b_);
        else
            writeRgb(0, 0, 0);
        break;
    }
    case Mode::PULSE: {
        const uint32_t t_ms = millis() % pulse_period_ms_;
        const float t = (float)t_ms / (float)pulse_period_ms_;
        float phase;
        if (t < 0.25f)
            phase = 1.0f - 4.0f * t;
        else if (t < 0.75f)
            phase = 0.0f;
        else
            phase = 4.0f * (t - 0.75f);
        writeRgb((uint8_t)((float)r_ * phase), (uint8_t)((float)g_ * phase), (uint8_t)((float)b_ * phase));
        break;
    }
    }
}

/** Fir 10 ms; evenimente inițial none. */
PushButton::PushButton(uint8_t pin)
    : Thread(10),
      pin_(pin),
      raw_last_(false),
      debounced_pressed_(false),
      debounce_start_ms_(0),
      pressed_since_ms_(0),
      press_released_at_ms_(0),
      event_(Event::none),
      fired_press_event_(false) {}

void PushButton::begin() {
    pinMode(pin_, INPUT_PULLUP);
    raw_last_ = (digitalRead(pin_) == LOW);
    debounced_pressed_ = raw_last_;
    debounce_start_ms_ = millis();
}

PushButton::Event PushButton::get_event() {
    Event e = event_;
    event_ = Event::none;
    return e;
}

/** Debounce, detectare click / press / secret_click după praguri. */
void PushButton::loop() {
    const bool raw = (digitalRead(pin_) == LOW);
    const uint32_t now = millis();

    if (raw != raw_last_) {
        raw_last_ = raw;
        debounce_start_ms_ = now;
    }

    if (press_released_at_ms_ != 0 && (now - press_released_at_ms_) > (uint32_t)BUTTON_SECRETCLICK_TIME)
        press_released_at_ms_ = 0;

    if ((now - debounce_start_ms_) >= (uint32_t)BUTTON_DEBOUNCE_TIME && raw != debounced_pressed_) {
        debounced_pressed_ = raw;

        if (debounced_pressed_) {
            pressed_since_ms_ = now;
            fired_press_event_ = false;
        } else {
            if (fired_press_event_)
                press_released_at_ms_ = now;
            else if ((now - pressed_since_ms_) <= BUTTON_CLICK_MAX_TIME) {
                if (press_released_at_ms_ != 0 && (now - press_released_at_ms_) <= (uint32_t)BUTTON_SECRETCLICK_TIME) {
                    event_ = Event::secret_click;
                    press_released_at_ms_ = 0;
                } else {
                    event_ = Event::click;
                }
            }
        }
    }

    if (debounced_pressed_ && !fired_press_event_ && (now - pressed_since_ms_) >= (uint32_t)BUTTON_PRESS_MIN_TIME) {
        event_ = Event::press;
        fired_press_event_ = true;
    }
}
