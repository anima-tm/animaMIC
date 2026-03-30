/*
 * anima.cpp
 *
 * animaMIC — Implementare Animator: automat de stări, boot, standby/approach/focus, pauză, erori,
 * urmărire țintă (ochi), comenzi serială anima/state, test actuator opțional.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "anima.h"
#include "gestures.h"
#include "IRProc.h"
#include "Threads.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

extern IRSense ir_sense;
extern IRProcessor ir_processor;
extern ServoAct actuator_ox;
extern ServoAct actuator_oyup;
extern ServoAct actuator_mma;
extern ServoAct actuator_mm;
extern Animator anima;

static ServoAct* const ACTUATORS[] = { &actuator_ox, &actuator_oyup, &actuator_mma, &actuator_mm };

/**
 * Dacă toate actuatoarele sunt offline: setError și true (oprește starea curentă).
 */
bool abortIfAllActuatorsOffline(Animator* a) {
    unsigned int offline = 0;
    for (unsigned int i = 0; i < NUM_ACTUATORS; i++) {
        if (!ACTUATORS[i]->isOnline())
            offline++;
    }
    if (offline >= NUM_ACTUATORS) {
        a->setError("all actuators offline");
        return true;
    }
    return false;
}

/** Reset hard al microcontrolerului (WDT sau registru AIRCR / vector null); nu revine. */
static void resetArduino() {
#if defined(__AVR__)
    wdt_enable(WDTO_15MS);
    while (1) {}
#elif defined(__ARM_ARCH) || defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_RENESAS_UNO)
    (*(volatile uint32_t*)0xE000ED0C) = 0x05FA0004;
    while (1) {}
#else
    void (*resetFunc)(void) = 0;
    resetFunc();
#endif
}

/* Număr de servomotoare per actuator (ox, oyup, mma, mm); dimensiune vectori în Posture. */
static const uint8_t ACTUATOR_SERVOS[] = { 2, 2, 2, 1 };

/** Inițializare membri tracker, buton, fără înregistrare stări (vezi begin()). */
Animator::Animator()
    : Thread(0),
      active_since_ms_(0),
      reset_condition_start_ms_(0),
      error_since_ms_(0),
      tracker_active_after_ms_(0),
      tracker_pos_filt_(0.5f),
      tracker_dist_filt_(0.5f),
      eye_tracker_enabled_(true),
      teleplot_tracker_enabled_(false),
      eye_tracker_has_target_(false),
      tracker_last_pos_(0.5f),
      tracker_last_pos_filt_(0.5f),
      tracker_last_os_(0.0f),
      tracker_last_od_(0.0f),
      tracker_last_actuator_pos_(0.5f),
      tracker_last_oy_(0.0f),
      tracker_last_actuator_oy_(0.0f),
      ir_sense_prev_state_(IRSenseState::IDLE),
      boot_led_done_(false),
      press_state_(PressState::None),
      press_step_start_ms_(0),
      button_(BUTTON_PIN),
      paused_from_state_name_(nullptr)
#if TEST_ACTUATOR
      , last_test_goto_ms_(0),
      test_burst_in_progress_(false),
      test_actuator_index_(0)
#endif
{
}

/** Construiește obiectele de stare, setCurrentState(boot), StateBoot::begin (LED, IRProcessor off). */
void Animator::begin() {
    reset_condition_start_ms_ = 0;
    (void)new StatePaused();
    (void)new StateError();
    (void)new StateStandby();
    (void)new StateApproach();
    (void)new StateFocus();
    AnimaState* boot = new StateBoot();
    AnimaState::setCurrentState(boot);
    boot->begin(this);  /* StateBoot: LED albastru random blink la inițializare */
}

// ========== Inițializare (StateBoot) ==========

/** La intrare: LED albastru random blink, sub-stare CheckHw; IRProcessor dezactivat până la „alive”. */
void StateBoot::begin(Animator* a) {
    AnimaState::begin(a);
    // În timpul inițializării păstrăm IRProcessor dezactivat ca să nu apară o fantomă tranzitorie
    // din încălzirea inițială a fluxului de procesare IR; îl activăm doar când animatorul afișează
    // "--- The animator is alive! ---".
    ir_processor.setEnabled(false);
    a->color_led_.random_blink(COLOR_BOOT);
    boot_sub_state_ = BootSubState::CheckHw;
    activation_start_ms_ = 0;
    boot_wait_ir_start_ms_ = 0;
}

/** La ieșire: marchează starea pentru ștergere (tranziție la standby). */
void StateBoot::end() {
    markReadyForDeletion();
    AnimaState::end();
}

/** Mesaj serial „alive”, IRProcessor on, tracker delay, schimbare la standby. */
void StateBoot::finishToStandby(Animator* a) {
    Serial.println("--- The animator is alive! ---");
    ir_processor.setEnabled(true);
    a->boot_led_done_ = true;
    a->active_since_ms_ = millis();
    a->tracker_active_after_ms_ = millis() + TRACKER_STARTTIME;
#if TEST_ACTUATOR
    a->last_test_goto_ms_ = 0;
#endif
    AnimaState* standby = AnimaState::findByName("standby");
    if (standby) {
        markReadyForDeletion();
        AnimaState::changeState(a, standby);
    }
}

/**
 * Sub-stări: CheckHw → activare în perechi (mm/mma, ox/oyup) → postură „mic” → WaitIr (ACTIVE sau timeout) → finishToStandby.
 */
void StateBoot::loop(Animator* a) {
    switch (boot_sub_state_) {
    case BootSubState::CheckHw: {
        unsigned int offline = 0;
        for (unsigned int i = 0; i < NUM_ACTUATORS; i++) {
            if (!ACTUATORS[i]->isOnline()) {
                offline++;
                Serial.print("anima: Actuator '");
                Serial.print(ACTUATORS[i]->name());
                Serial.println("' is not online");
            }
        }
        if (offline >= NUM_ACTUATORS) {
            a->setError("all actuators offline");
            break;
        }
        boot_sub_state_ = BootSubState::ActivateMmMma;
        activation_start_ms_ = 0;
        break;
    }

    case BootSubState::ActivateMmMma:
    case BootSubState::ActivateOxoyup: {
        const unsigned int idx0 = (boot_sub_state_ == BootSubState::ActivateMmMma) ? 3 : 0;
        const unsigned int idx1 = (boot_sub_state_ == BootSubState::ActivateMmMma) ? 2 : 1;
        ServoAct* s0 = ACTUATORS[idx0];
        ServoAct* s1 = ACTUATORS[idx1];
        if (activation_start_ms_ == 0) {
            bool started = false;
            if (s0->isOnline() && !s0->allServosActive()) {
                s0->activate();
                started = true;
            } else if (s0->isOnline()) {
                Serial.print("anima: Skipping activation of actuator '");
                Serial.print(ACTUATORS[idx0]->name());
                Serial.println("' (all servos already active)");
            } else {
                Serial.print("anima: Skipping actuator '");
                Serial.print(ACTUATORS[idx0]->name());
                Serial.println("' (not online)");
            }
            if (s1->isOnline() && !s1->allServosActive()) {
                s1->activate();
                started = true;
            } else if (s1->isOnline()) {
                Serial.print("anima: Skipping activation of actuator '");
                Serial.print(ACTUATORS[idx1]->name());
                Serial.println("' (all servos already active)");
            } else {
                Serial.print("anima: Skipping actuator '");
                Serial.print(ACTUATORS[idx1]->name());
                Serial.println("' (not online)");
            }
            activation_start_ms_ = started ? millis() : 0;
            if (!started) {
                boot_sub_state_ = (boot_sub_state_ == BootSubState::ActivateMmMma) ? BootSubState::ActivateOxoyup : BootSubState::DefaultPosture;
            }
            break;
        }
        if (s0->isActivating() || s1->isActivating()) {
            if ((uint32_t)(millis() - activation_start_ms_) >= (uint32_t)SERVO_ACTIVATION_TIMEOUT) {
                if (s0->isActivating()) {
                    Serial.print("anima: Actuator '");
                    Serial.print(ACTUATORS[idx0]->name());
                    Serial.println("' activation timeout");
                }
                if (s1->isActivating()) {
                    Serial.print("anima: Actuator '");
                    Serial.print(ACTUATORS[idx1]->name());
                    Serial.println("' activation timeout");
                }
                activation_start_ms_ = 0;
                boot_sub_state_ = (boot_sub_state_ == BootSubState::ActivateMmMma) ? BootSubState::ActivateOxoyup : BootSubState::DefaultPosture;
            }
            break;
        }
        activation_start_ms_ = 0;
        boot_sub_state_ = (boot_sub_state_ == BootSubState::ActivateMmMma) ? BootSubState::ActivateOxoyup : BootSubState::DefaultPosture;
        break;
    }

    case BootSubState::DefaultPosture: {
        takePosture(a, getPostureByName("mic"), 600);
        boot_wait_ir_start_ms_ = millis();
        boot_sub_state_ = BootSubState::WaitIr;
        break;
    }

    case BootSubState::WaitIr: {
        IRSenseState ir_state = ir_sense.getState();
        if (ir_state == IRSenseState::ACTIVE) {
            finishToStandby(a);
            return;
        }
        if ((uint32_t)(millis() - boot_wait_ir_start_ms_) >= (uint32_t)BOOT_WAIT_IR_TIMEOUT_MS) {
            finishToStandby(a);
            return;
        }
        break;
    }
    }
}

// ========== StateStandby ==========

static const uint32_t STANDBY_MIN_POSTURE_TIME = 120;    // 2 minute
static const uint32_t STANDBY_MAX_POSTURE_TIME = 900;   // 15 minute
static const uint32_t STANDBY_MIN_GESTURE_TIME = 30;     // 30 secunde
static const uint32_t STANDBY_MAX_GESTURE_TIME = 300;   // 5 minute
static const uint8_t STANDBY_MAX_GESTURE_TRIALS = 5;

static const char* const STANDBY_POSTURE_NAMES[] = { "melt", "sleepy", "surrender", "showdoor", "hello", "hug", "angry" };
static const uint8_t STANDBY_POSTURE_COUNT = sizeof(STANDBY_POSTURE_NAMES) / sizeof(STANDBY_POSTURE_NAMES[0]);

static const char* const STANDBY_GESTURE_NAMES[] = { "wave1_left", "wave2_left", "wave1_right", "wave2_right", "wave3_right", "wave4_right", "roll_eyes" };
static const uint8_t STANDBY_GESTURE_COUNT = sizeof(STANDBY_GESTURE_NAMES) / sizeof(STANDBY_GESTURE_NAMES[0]);

/** Încarcă pointeri la posturi și gesturi din tabelele statice (max STANDBY_MAX_LIST). */
StateStandby::StateStandby() : AnimaState("standby"), next_posture_ms_(0), next_gesture_ms_(0), current_posture_index_(0xFF) {
    for (uint8_t i = 0; i < STANDBY_POSTURE_COUNT && i < STANDBY_MAX_LIST; i++) {
        const Posture* p = getPostureByName(STANDBY_POSTURE_NAMES[i]);
        standby_postures[i] = p;
#if DIAGNOSE_STATES
        if (!p) {
            Serial.print("standby: posture not found: ");
            Serial.println(STANDBY_POSTURE_NAMES[i]);
        }
#endif
    }
    for (uint8_t i = 0; i < STANDBY_GESTURE_COUNT && i < STANDBY_MAX_LIST; i++) {
        Gesture* g = findGesture(STANDBY_GESTURE_NAMES[i]);
        standby_gestures[i] = g;
#if DIAGNOSE_STATES
        if (!g) {
            Serial.print("standby: gesture not found: ");
            Serial.println(STANDBY_GESTURE_NAMES[i]);
        }
#endif
    }
}

/** La intrare: LED verde sau portocaliu (IR/actuatoare offline); programează intervale postură/gest. */
void StateStandby::begin(Animator* a) {
    AnimaState::begin(a);
    if (a->hasActuatorOrIrOffline())
        a->color_led_.set(COLOR_ACTIVE_WITH_ERRORS);
    else
        a->color_led_.set(COLOR_ACTIVE);
    const uint32_t now = millis();
    /* Rezoluție 0,1 s: constante în secunde, aleator în zecimi apoi *100 → ms; constantă max 6553 s (uint16_t). */
    next_posture_ms_ = now + (uint32_t)td_random((uint16_t)(STANDBY_MIN_POSTURE_TIME * 10), (uint16_t)(STANDBY_MAX_POSTURE_TIME * 10)) * 100U;
    next_gesture_ms_ = now + (uint32_t)td_random((uint16_t)(STANDBY_MIN_GESTURE_TIME * 10), (uint16_t)(STANDBY_MAX_GESTURE_TIME * 10)) * 100U;
}

/*
 * La fiecare tick:
 * - Verifică actuatoare/IR; tranziții la error, focus sau approach.
 * - La intervale (POSTURE_TIME): postură aleatoare; la schimbare postură resetează și timer-ul de gest.
 * - La intervale (GESTURE_TIME): alege la întâmplare un gest din listă; dacă run() returnează false, încearcă altul (max STANDBY_MAX_GESTURE_TRIALS); dacă toate refuză, printează mesaj.
 * - Actualizează LED.
 */
void StateStandby::loop(Animator* a) {
    if (abortIfAllActuatorsOffline(a))
        return;

    const IRProcessorState ir_state = ir_processor.getState();
    if (ir_state == IRProcessorState::TRACKING) {
        AnimaState* focus = AnimaState::findByName("focus");
        if (focus) AnimaState::changeState(a, focus);
        return;
    }
    if (ir_state == IRProcessorState::CATCHING_GHOSTS) {
        AnimaState* approach = AnimaState::findByName("approach");
        if (approach) AnimaState::changeState(a, approach);
        return;
    }

    const uint32_t now = millis();

    if (STANDBY_POSTURE_COUNT > 0 && now >= next_posture_ms_) {
        uint8_t idx;
        do {
            idx = (uint8_t)td_random(0, (uint16_t)(STANDBY_POSTURE_COUNT - 1));
        } while (STANDBY_POSTURE_COUNT > 1 && current_posture_index_ != 0xFF && idx == current_posture_index_);
        current_posture_index_ = idx;
        if (standby_postures[idx] != nullptr)
            takePosture(a, standby_postures[idx]);
        next_posture_ms_ = now + (uint32_t)td_random((uint16_t)(STANDBY_MIN_POSTURE_TIME * 10), (uint16_t)(STANDBY_MAX_POSTURE_TIME * 10)) * 100U;
        next_gesture_ms_ = now + (uint32_t)td_random((uint16_t)(STANDBY_MIN_GESTURE_TIME * 10), (uint16_t)(STANDBY_MAX_GESTURE_TIME * 10)) * 100U;
    }

    if (STANDBY_GESTURE_COUNT > 0 && now >= next_gesture_ms_) {
        bool done = false;
        for (uint8_t t = 0; t < STANDBY_MAX_GESTURE_TRIALS && !done; t++) {
            uint8_t g_idx = (uint8_t)td_random(0, (uint16_t)(STANDBY_GESTURE_COUNT - 1));
            if (standby_gestures[g_idx] != nullptr && runGesture(a, standby_gestures[g_idx]))
                done = true;
        }
        if (!done)
            Serial.println("State standby: all gestures refused");
        next_gesture_ms_ = now + (uint32_t)td_random((uint16_t)(STANDBY_MIN_GESTURE_TIME * 10), (uint16_t)(STANDBY_MAX_GESTURE_TIME * 10)) * 100U;
    }

    /*
     * --- LED: intenția după stare ---
     * Verde solid dacă totul e ok; blink aleatoriu dacă un actuator se activează;
     * verde cu erori (solid) dacă actuator sau IR e offline.
     */
    {
        Animator::LedIntent want = Animator::LedIntent::ActiveSolid;
        if (a->hasActuatorActivating())
            want = Animator::LedIntent::ActuatorActivatingRandomBlink;
        else if (a->hasActuatorOrIrOffline())
            want = Animator::LedIntent::ActiveWithErrorsSolid;
        static Animator::LedIntent last = Animator::LedIntent::Off;
        if (want != last) {
            last = want;
            if (want == Animator::LedIntent::ActuatorActivatingRandomBlink)
                a->color_led_.random_blink(a->color_led_.getColor());
            else if (want == Animator::LedIntent::ActiveWithErrorsSolid)
                a->color_led_.set(COLOR_ACTIVE_WITH_ERRORS);
            else
                a->color_led_.set(COLOR_ACTIVE);
        }
    }

#if TEST_ACTUATOR
    /* Test actuator: burst aleator pe un actuator la intervale. */
    a->runTestLoop();
#endif
}

/** La ieșire: apel de bază AnimaState (last_exit_ms_). */
void StateStandby::end() {
    AnimaState::end();
}

// ========== StateApproach ==========

static const uint32_t APPROACH_MIN_ACTION_TIME = 5;   // secunde
static const uint32_t APPROACH_MAX_ACTION_TIME = 15;  // secunde
static const uint8_t APPROACH_MAX_GESTURE_TRIALS = 5;

static const char* const APPROACH_GESTURE_NAMES[] = { "wave1_left", "wave2_left", "wave1_right", "wave2_right" };
static const uint8_t APPROACH_GESTURE_NAME_COUNT = sizeof(APPROACH_GESTURE_NAMES) / sizeof(APPROACH_GESTURE_NAMES[0]);

/** Încarcă gesturile approach din APPROACH_GESTURE_NAMES. */
StateApproach::StateApproach() : AnimaState("approach"), next_action_ms_(0), last_focus_gesture_ms_(0) {
    for (uint8_t i = 0; i < APPROACH_GESTURE_NAME_COUNT && i < APPROACH_GESTURE_COUNT; i++) {
        Gesture* g = findGesture(APPROACH_GESTURE_NAMES[i]);
        approach_gestures[i] = g;
#if DIAGNOSE_STATES
        if (!g) {
            Serial.print("approach: gesture not found: ");
            Serial.println(APPROACH_GESTURE_NAMES[i]);
        }
#endif
    }
}

/**
 * La intrare: LED turcoaz. Din standby: gest „focus” dacă nu a rulat recent; programează interval wave.
 */
void StateApproach::begin(Animator* a) {
    AnimaState::begin(a);
    a->color_led_.set(COLOR_FOCUS);
    const uint32_t now = millis();
    AnimaState* prev = AnimaState::getPreviousState();
    if (prev && strcmp(prev->getName(), "standby") == 0) {
        if (last_focus_gesture_ms_ == 0 || (now - last_focus_gesture_ms_) >= (APPROACH_MIN_ACTION_TIME * 1000U)) {
            Gesture* focus_g = findGesture("focus");
            if (focus_g && runGesture(a, focus_g))
                last_focus_gesture_ms_ = now;
        }
    }
    next_action_ms_ = now + (uint32_t)td_random((uint16_t)(APPROACH_MIN_ACTION_TIME * 10), (uint16_t)(APPROACH_MAX_ACTION_TIME * 10)) * 100U;
}

/**
 * Tranziții: IDLE→standby, TRACKING→focus; altfel gesturi wave la intervale aleatoare.
 */
void StateApproach::loop(Animator* a) {
    if (abortIfAllActuatorsOffline(a))
        return;
    const IRProcessorState ir_state = ir_processor.getState();
    const uint32_t now = millis();

    if (ir_state == IRProcessorState::IDLE) {
        AnimaState* standby = AnimaState::findByName("standby");
        if (standby) AnimaState::changeState(a, standby);
        return;
    }
    if (ir_state == IRProcessorState::TRACKING) {
        AnimaState* focus = AnimaState::findByName("focus");
        if (focus) AnimaState::changeState(a, focus);
        return;
    }

    if (APPROACH_GESTURE_COUNT > 0 && now >= next_action_ms_) {
        bool done = false;
        for (uint8_t t = 0; t < APPROACH_MAX_GESTURE_TRIALS && !done; t++) {
            uint8_t g_idx = (uint8_t)td_random(0, (uint16_t)(APPROACH_GESTURE_COUNT - 1));
            if (approach_gestures[g_idx] != nullptr && runGesture(a, approach_gestures[g_idx]))
                done = true;
        }
        next_action_ms_ = now + (uint32_t)td_random((uint16_t)(APPROACH_MIN_ACTION_TIME * 10), (uint16_t)(APPROACH_MAX_ACTION_TIME * 10)) * 100U;
    }

    a->color_led_.set(COLOR_FOCUS);
#if TEST_ACTUATOR
    a->runTestLoop();
#endif
}

/** La ieșire: apel de bază AnimaState. */
void StateApproach::end() {
    AnimaState::end();
}

// ========== StateFocus ==========

static const uint32_t FOCUS_STILL_TIME = 2;         // secunde fără mișcare pentru a intra în invite
static const uint32_t FOCUS_MIN_SESSION_TIME = 20;  // secunde min în invite pentru a face goodbye la pierdere țintă
static const uint32_t FOCUS_MIN_INVITE_TIME = 5;   // secunde
static const uint32_t FOCUS_MAX_INVITE_TIME = 10;   // secunde
static const char* const FOCUS_INVITE_GESTURE_NAMES[] = { "invite1", "invite2", "invite3", "scare" };
static const char* const FOCUS_GOODBYE_GESTURE_NAMES[] = { "wave1_right", "wave2_right", "wave3_right", "wave4_right" };

/** La intrare: LED turcoaz intermitent; gest „straighten”; încarcă tabele invite/goodbye; Follow. */
void StateFocus::begin(Animator* a) {
    AnimaState::begin(a);
    a->color_led_.blink(COLOR_FOCUS, 100);
    const uint32_t now = millis();
    if (last_straighten_gesture_ms_ == 0 || (now - last_straighten_gesture_ms_) >= (APPROACH_MIN_ACTION_TIME * 1000U)) {
        Gesture* straighten_g = findGesture("straighten");
        if (straighten_g && runGesture(a, straighten_g))
            last_straighten_gesture_ms_ = now;
    }
    for (uint8_t i = 0; i < FOCUS_INVITE_COUNT; i++)
        focus_invite_gestures[i] = findGesture(FOCUS_INVITE_GESTURE_NAMES[i]);
    for (uint8_t i = 0; i < FOCUS_GOODBYE_COUNT; i++)
        focus_goodbye_gestures[i] = findGesture(FOCUS_GOODBYE_GESTURE_NAMES[i]);
    focus_sub_ = FocusSub::Follow;
    focus_still_since_ms_ = 0;
    focus_invite_first_enter_ms_ = 0;
    focus_distance_on_ = false;
    focus_next_invite_ms_ = 0;
    focus_gesture_ends_ms_ = 0;
    focus_goodbye_ends_ms_ = 0;
}

/**
 * Sub-stări Follow / Invite / Goodbye: runEyeTracker pe cadru nou; tranziții spre standby/approach după pierdere țintă.
 */
void StateFocus::loop(Animator* a) {
    if (abortIfAllActuatorsOffline(a))
        return;
    const IRProcessorState ir_state = ir_processor.getState();
    const uint32_t now = millis();

    /* Goodbye: așteptăm durata gestului + 500 ms chiar dacă nu mai avem țintă (IDLE). */
    if (focus_sub_ == FocusSub::Goodbye) {
        if (focus_goodbye_ends_ms_ != 0 && now >= focus_goodbye_ends_ms_) {
            focus_goodbye_ends_ms_ = 0;
            uint16_t r = td_random(0, 2);
            if (r == 0) {
                AnimaState* standby = AnimaState::findByName("standby");
                if (standby) AnimaState::changeState(a, standby);
            } else if (r == 1) {
                AnimaState* approach = AnimaState::findByName("approach");
                if (approach) AnimaState::changeState(a, approach);
            } else {
                focus_sub_ = FocusSub::Follow;
                focus_distance_on_ = false;
                focus_still_since_ms_ = 0;
                focus_gesture_ends_ms_ = 0;
                focus_next_invite_ms_ = 0;
                focus_invite_first_enter_ms_ = 0;
            }
        }
#if TEST_ACTUATOR
        a->runTestLoop();
#endif
        return;
    }

    if (ir_state == IRProcessorState::IDLE) {
        if ((focus_sub_ == FocusSub::Invite || focus_sub_ == FocusSub::Follow) && focus_invite_first_enter_ms_ != 0 && (now - focus_invite_first_enter_ms_) >= (FOCUS_MIN_SESSION_TIME * 1000U) && ir_processor.getCachedExitCause() == 2) {
            focus_sub_ = FocusSub::Goodbye;
            uint8_t idx = (uint8_t)td_random(0, (uint16_t)(FOCUS_GOODBYE_COUNT - 1));
            if (focus_goodbye_gestures[idx] != nullptr) {
                runGesture(a, focus_goodbye_gestures[idx]);
                focus_goodbye_ends_ms_ = now + focus_goodbye_gestures[idx]->duration() + 500U;
            } else {
                focus_goodbye_ends_ms_ = now + 500U;
            }
        } else {
            AnimaState* standby = AnimaState::findByName("standby");
            if (standby) AnimaState::changeState(a, standby);
        }
        return;
    }
    if (ir_state == IRProcessorState::CATCHING_GHOSTS) {
        if ((focus_sub_ == FocusSub::Invite || focus_sub_ == FocusSub::Follow) && focus_invite_first_enter_ms_ != 0 && (now - focus_invite_first_enter_ms_) >= (FOCUS_MIN_SESSION_TIME * 1000U) && ir_processor.getCachedExitCause() == 2) {
            focus_sub_ = FocusSub::Goodbye;
            uint8_t idx = (uint8_t)td_random(0, (uint16_t)(FOCUS_GOODBYE_COUNT - 1));
            if (focus_goodbye_gestures[idx] != nullptr) {
                runGesture(a, focus_goodbye_gestures[idx]);
                focus_goodbye_ends_ms_ = now + focus_goodbye_gestures[idx]->duration() + 500U;
            } else {
                focus_goodbye_ends_ms_ = now + 500U;
            }
        } else {
            AnimaState* approach = AnimaState::findByName("approach");
            if (approach) AnimaState::changeState(a, approach);
        }
        return;
    }

    /* TRACKING */
    bool tracker_sent = false;
    if (a->eye_tracker_enabled_ && ir_processor.hadNewFrameLastLoop()) {
        tracker_sent = a->runEyeTracker(focus_distance_on_);
        ir_processor.clearHadNewFrame();
    }

    if (focus_sub_ == FocusSub::Invite) {
        if (focus_gesture_ends_ms_ != 0 && now >= focus_gesture_ends_ms_)
            focus_gesture_ends_ms_ = 0;
        if (focus_gesture_ends_ms_ == 0 && now >= focus_next_invite_ms_) {
            uint8_t idx = (uint8_t)td_random(0, (uint16_t)(FOCUS_INVITE_COUNT - 1));
            if (focus_invite_gestures[idx] != nullptr) {
                runGesture(a, focus_invite_gestures[idx]);
                focus_gesture_ends_ms_ = now + focus_invite_gestures[idx]->duration() + 500U;
                // Intervalul dintre gesturi: end-to-start.
                focus_next_invite_ms_ = focus_gesture_ends_ms_ + (uint32_t)td_random((uint16_t)(FOCUS_MIN_INVITE_TIME * 10), (uint16_t)(FOCUS_MAX_INVITE_TIME * 10)) * 100U;
                focus_distance_on_ = true;
                if (focus_invite_first_enter_ms_ == 0)
                    focus_invite_first_enter_ms_ = now;
            }
        }
#if TEST_ACTUATOR
        a->runTestLoop();
#endif
        return;
    }

    /* Follow: urmărire a țintei doar pe poziție; după FOCUS_STILL_TIME fără mișcare trecem în Invite. */
    if (tracker_sent)
        focus_still_since_ms_ = 0;
    else if (focus_still_since_ms_ == 0)
        focus_still_since_ms_ = now;
    if (focus_still_since_ms_ != 0 && (now - focus_still_since_ms_) >= (FOCUS_STILL_TIME * 1000U)) {
        focus_sub_ = FocusSub::Invite;
        focus_still_since_ms_ = 0;
        focus_invite_first_enter_ms_ = now;
        uint8_t idx = (uint8_t)td_random(0, (uint16_t)(FOCUS_INVITE_COUNT - 1));
        if (focus_invite_gestures[idx] != nullptr) {
            runGesture(a, focus_invite_gestures[idx]);
            focus_gesture_ends_ms_ = now + focus_invite_gestures[idx]->duration() + 500U;
            // Intervalul dintre gesturi: end-to-start.
            focus_next_invite_ms_ = focus_gesture_ends_ms_ + (uint32_t)td_random((uint16_t)(FOCUS_MIN_INVITE_TIME * 10), (uint16_t)(FOCUS_MAX_INVITE_TIME * 10)) * 100U;
        } else {
            focus_gesture_ends_ms_ = 0;
            focus_next_invite_ms_ = now + (uint32_t)td_random((uint16_t)(FOCUS_MIN_INVITE_TIME * 10), (uint16_t)(FOCUS_MAX_INVITE_TIME * 10)) * 100U;
        }
        focus_distance_on_ = true;
    }
#if TEST_ACTUATOR
    a->runTestLoop();
#endif
}

/** La ieșire: eye_tracker_has_target_ false pentru intrare curată ulterioară. */
void StateFocus::end() {
    AnimaState::end();
    anima.eye_tracker_has_target_ = false;
}

bool Animator::isActive() const {
    AnimaState* c = AnimaState::getCurrentState();
    if (!c || !c->getName()) return false;
    const char* n = c->getName();
    return strcmp(n, "standby") == 0 || strcmp(n, "approach") == 0 || strcmp(n, "focus") == 0;
}

/** Delegă la Animator::takePosture; diagnostic opțional DIAGNOSE_STATES. */
bool AnimaState::takePosture(Animator* a, const Posture* post, uint16_t time_ms, bool eyes_too) {
#if DIAGNOSE_STATES
    if (post) {
        Serial.print("state ");
        Serial.print(getName());
        Serial.print(": take ");
        Serial.println(post->name);
    }
#endif
    return post ? a->takePosture(post, time_ms, eyes_too) : false;
}

/** Delegă la Animator::runGesture; diagnostic opțional. */
bool AnimaState::runGesture(Animator* a, Gesture* g) {
#if DIAGNOSE_STATES
    if (g) {
        Serial.print("state ");
        Serial.print(getName());
        Serial.print(": do ");
        Serial.println(g->name());
    }
#endif
    return g ? a->runGesture(g) : false;
}

// ========== StatePaused ==========

/** La intrare: LED puls după starea IR/actuatoare. */
void StatePaused::begin(Animator* a) {
    AnimaState::begin(a);
    if (a->hasActuatorOrIrOffline())
        a->color_led_.pulse(COLOR_ACTIVE_WITH_ERRORS, 3000);
    else
        a->color_led_.pulse(COLOR_ACTIVE, 3000);
}

/** Menține LED conform activării / erori; abort dacă toate actuatoarele offline. */
void StatePaused::loop(Animator* a) {
    if (abortIfAllActuatorsOffline(a))
        return;
    enum { Pulse, PulseErrors, RandomBlink } want = a->hasActuatorActivating() ? RandomBlink : (a->hasActuatorOrIrOffline() ? PulseErrors : Pulse);
    static int last = -1;
    if ((int)want != last) {
        last = (int)want;
        if (want == RandomBlink)
            a->color_led_.random_blink(a->color_led_.getColor());
        else
            a->color_led_.pulse(want == PulseErrors ? COLOR_ACTIVE_WITH_ERRORS : COLOR_ACTIVE, 3000);
    }
}

/** La ieșire: apel de bază AnimaState. */
void StatePaused::end() {
    AnimaState::end();
}

// ========== StateError ==========

/** La intrare: LED roșu solid. */
void StateError::begin(Animator* a) {
    AnimaState::begin(a);
    a->color_led_.set(COLOR_ERROR);
}

/** După ANIMA_ERROR_RESET_TIME: resetArduino(). */
void StateError::loop(Animator* a) {
    if ((millis() - a->error_since_ms_) >= (uint32_t)ANIMA_ERROR_RESET_TIME)
        resetArduino();
}

/** La ieșire: apel de bază AnimaState. */
void StateError::end() {
    AnimaState::end();
}

/**
 * Rulează automatul de stări; buton (poate întuneca LED); watchdog activare; tick stare curentă;
 * la revenire ACTIVE din IDLE/FAILED reset ușor IRProcessor și amânare tracker.
 */
void Animator::loop() {
    AnimaState::runStateMachine(this);
    processButton();
    if (press_state_ != PressState::None) {
        color_led_.set(Color(0, 0, 0));
        return;
    }
    AnimaState* cur = AnimaState::getCurrentState();
    if (!cur) {
        color_led_.set(Color(0, 0, 0));
        return;
    }
    for (unsigned int i = 0; i < NUM_ACTUATORS; i++) {
        if (ACTUATORS[i]->getAndClearActivationWatchdogTrigger()) {
            char buf[48];
            strcpy(buf, "activation watchdog (actuator ");
            strncat(buf, ACTUATORS[i]->name(), 4);
            strcat(buf, ")");
            setError(buf);
            return;
        }
    }
    cur->loop(this);

    /* IRSense poate alterna periodic între ACTIVE și GET_PENDING.
     * Nu vrem să resetăm IRProcessor la fiecare ciclu normal; refacem doar la intrarea ACTIVE
     * după o reconectare/relansare (IDLE/FAILED). */
    {
        AnimaState* after = AnimaState::getCurrentState();
        IRSenseState s = ir_sense.getState();
        if (after && after->getName() && strcmp(after->getName(), "boot") != 0) {
            const bool entered_active_from_idle_or_failed =
                (s == IRSenseState::ACTIVE) &&
                (ir_sense_prev_state_ == IRSenseState::IDLE || ir_sense_prev_state_ == IRSenseState::FAILED);
            if (entered_active_from_idle_or_failed) {
                tracker_active_after_ms_ = millis() + TRACKER_STARTTIME;
                if (ir_processor.isEnabled())
                    ir_processor.begin();
            }
        }
        ir_sense_prev_state_ = s;
    }
}

/** Golește registru, reset actuatoare și IRSense; oprește LED. */
void Animator::end() {
    AnimaState* cur = AnimaState::getCurrentState();
    if (cur)
        cur->end();
    AnimaState::current_state_ = nullptr;
    AnimaState::pending_state_ = nullptr;
    AnimaState::previous_state_ = nullptr;
    paused_from_state_name_ = nullptr;
    while (AnimaState::getRegistryCount() > 0)
        delete AnimaState::getStateFromRegistry(0);
    color_led_.set(Color(0, 0, 0));
    active_since_ms_ = 0;
    ir_sense_prev_state_ = IRSenseState::IDLE;
#if TEST_ACTUATOR
    last_test_goto_ms_ = 0;
    test_burst_in_progress_ = false;
#endif
    actuator_ox.reset();
    actuator_oyup.reset();
    actuator_mma.reset();
    actuator_mm.reset();
    ir_sense.init();
}

/** end(); begin(); — ciclul complet de repornire. */
void Animator::reset() {
    end();
    begin();
}

/** Tranziție forțată la „error”; afișează cauza pe serial. */
void Animator::setError(const char* cause) {
    error_since_ms_ = millis();
    boot_led_done_ = true;
    Serial.print("Animator error: ");
    Serial.println(cause);
    AnimaState* err = AnimaState::findByName("error");
    if (err) AnimaState::changeState(this, err);
}

/** true dacă ir1≈1000 și ceilalți senzori ≈0 (condiție reset software / depanare). */
bool Animator::checkResetCondition() const {
    const IRFrame* frame = ir_sense.getReadings();
    size_t n = ir_sense.getNumSensors();
    if (n == 0)
        return false;
    if (frame->at(0) < 999.0f)
        return false;
    for (size_t i = 1; i < n; i++) {
        if (frame->at(i) > 0.5f)
            return false;
    }
    return true;
}

/**
 * pause true: din stări ACTIVE salvează numele, postură „mic”, trece la paused; din altele (non-boot) idem unde e cazul.
 * pause false: revine la starea memorată în paused_from_state_name_.
 */
void Animator::setPaused(bool pause) {
    AnimaState* cur = AnimaState::getCurrentState();
    if (cur && cur->getName() && strcmp(cur->getName(), "boot") == 0)
        return;
    if (pause) {
        if (cur && isActive()) {
            takePosture("mic", 1000);
            size_t len = strlen(cur->getName());
            if (len >= PAUSED_FROM_NAME_BUF) len = PAUSED_FROM_NAME_BUF - 1;
            memcpy(paused_from_state_name_buf_, cur->getName(), len);
            paused_from_state_name_buf_[len] = '\0';
            paused_from_state_name_ = paused_from_state_name_buf_;
            boot_led_done_ = true;
            AnimaState* ps = AnimaState::findByName("paused");
            if (ps) AnimaState::changeState(this, ps);
        } else if (cur && strcmp(cur->getName(), "paused") != 0 && strcmp(cur->getName(), "error") != 0) {
            size_t len = strlen(cur->getName());
            if (len >= PAUSED_FROM_NAME_BUF) len = PAUSED_FROM_NAME_BUF - 1;
            memcpy(paused_from_state_name_buf_, cur->getName(), len);
            paused_from_state_name_buf_[len] = '\0';
            paused_from_state_name_ = paused_from_state_name_buf_;
            boot_led_done_ = true;
            AnimaState* ps = AnimaState::findByName("paused");
            if (ps) AnimaState::changeState(this, ps);
        }
    } else {
        if (cur && strcmp(cur->getName(), "paused") == 0 && paused_from_state_name_) {
            AnimaState* back = AnimaState::findByName(paused_from_state_name_);
            if (back) AnimaState::changeState(this, back);
        }
    }
}

/** Pornește/oprește apelurile runEyeTracker; la activare reprogramează amânarea TRACKER_STARTTIME. */
void Animator::setEyeTrackerEnabled(bool on) {
    bool was = eye_tracker_enabled_;
    eye_tracker_enabled_ = on;
    if (on && !was) {
        tracker_active_after_ms_ = millis() + TRACKER_STARTTIME;
    }
}

/**
 * Secvență apăsare lungă: LED off → timeout → reset actuatoare + IRsense → timeout → reset Arduino.
 * click: comută pauză (în afară de boot). secret_click: reset imediat.
 */
void Animator::processButton() {
    PushButton::Event ev = button_.get_event();
    (void)button_.get_state();

    /* Apăsare lungă: LED off, așteptare 2.5 s, trimite reset la actuatoare+IR, așteptare 2 s (non-blocking ca I2C/UART să ruleze), apoi reset Arduino. Secret_click = doar reset Arduino. */
    if (press_state_ != PressState::None) {
        if (ev == PushButton::Event::secret_click) {
            Serial.println("Initiating reset...");
            resetArduino();
        }
        const uint32_t now = millis();
        const uint32_t elapsed = now - press_step_start_ms_;
        if (press_state_ == PressState::WaitSecretClick && elapsed >= 1500) {
            press_state_ = PressState::WaitOneSec;
            press_step_start_ms_ = now;
        } else if (press_state_ == PressState::WaitOneSec && elapsed >= 1000) {
            Serial.println("Initiating system reset...");
            ir_sense.resetForSystemReset();
            actuator_ox.reset();
            actuator_oyup.reset();
            actuator_mma.reset();
            actuator_mm.reset();
            press_state_ = PressState::WaitAfterReset;
            press_step_start_ms_ = now;
        } else if (press_state_ == PressState::WaitAfterReset && elapsed >= 2000) {
            resetArduino();
        }
        return;
    }
    if (ev == PushButton::Event::press) {
        color_led_.set(Color(0, 0, 0));
        press_state_ = PressState::WaitSecretClick;
        press_step_start_ms_ = millis();
        return;
    }
    if (ev == PushButton::Event::secret_click) {
        Serial.println("Initiating reset...");
        resetArduino();
    }
    if (ev == PushButton::Event::click) {
        AnimaState* st = AnimaState::getCurrentState();
        if (st && st->getName() && strcmp(st->getName(), "boot") == 0)
            return;
        setPaused(!isPaused());
    }
}

bool Animator::hasActuatorOrIrOffline() const {
    for (unsigned int i = 0; i < NUM_ACTUATORS; i++) {
        if (!ACTUATORS[i]->isOnline())
            return true;
    }
    /* GET_PENDING e normal (așteptare răspuns get); doar IDLE/INIT/FAILED contează ca offline. */
    const IRSenseState s = ir_sense.getState();
    return (s != IRSenseState::ACTIVE && s != IRSenseState::GET_PENDING);
}

bool Animator::hasActuatorActivating() const {
    for (unsigned int i = 0; i < NUM_ACTUATORS; i++) {
        if (ACTUATORS[i]->isActivating())
            return true;
    }
    return false;
}

/** Caută postura după nume; delegă la overload cu pointer. */
bool Animator::takePosture(const char* posture_name, uint16_t time_ms, bool eyes_too) {
    const Posture* post = getPostureByName(posture_name);
    return takePosture(post, time_ms, eyes_too);
}

/**
 * Trimite go_to pe fiecare actuator conform post->positions; ox poate fi sărit; oyup folosește tracker_last_actuator_oy_ pentru Y.
 */
bool Animator::takePosture(const Posture* post, uint16_t time_ms, bool eyes_too) {
    if (!post) return false;
    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
        ServoAct* a = ACTUATORS[i];
        if (!eyes_too && strcmp(a->name(), "ox") == 0)
            continue;
        uint8_t n = ACTUATOR_SERVOS[i];
        const float* p = post->positions[i];
        if (!eyes_too && strcmp(a->name(), "oyup") == 0) {
            if (n >= 2)
                a->go_to(tracker_last_actuator_oy_, p[1], time_ms);
            else
                a->go_to(p[0], time_ms);
        } else {
            switch (n) {
            case 1: a->go_to(p[0], time_ms); break;
            case 2: a->go_to(p[0], p[1], time_ms); break;
            default: break;
            }
        }
    }
    return true;
}

/** findGesture după nume; delegă la runGesture(Gesture*). */
bool Animator::runGesture(const char* gesture_name) {
    Gesture* g = findGesture(gesture_name);
    return runGesture(g);
}

/** Rulează secvența gestului (g->run()). */
bool Animator::runGesture(Gesture* g) {
    if (!g) return false;
    return g->run();
}

/**
 * Interpretor subcomenzi: listă posturi/gesturi, „take”, „do”, „track”, „on”/„off” (pauză).
 */
bool Animator::cmd_anima(const char* arg) {
    const char* sub = arg ? arg : "";
    while (*sub == ' ') sub++;
    if (!*sub) {
        for (uint8_t i = 0; i < POSTURE_COUNT; i++) {
            Serial.print(F("  - "));
            Serial.println(POSTURES[i].name);
        }
        Serial.println(F("anima take <posture> [in <millis>]"));
        Serial.println(F("anima do <gesture>"));
        Serial.println(F("anima track"));
        Serial.println(F("anima on|off"));
        for (unsigned int i = 0; i < gestureCount(); i++) {
            Gesture* g = gestureAt(i);
            if (g) {
                Serial.print(F("  - "));
                Serial.println(g->name());
            }
        }
        return true;
    }
    if (strncmp(sub, "do ", 3) == 0) {
        sub += 3;
        while (*sub == ' ') sub++;
        const char* name_start = sub;
        while (*sub && *sub != ' ') sub++;
        if (name_start == sub) {
            for (unsigned int i = 0; i < gestureCount(); i++) {
                Gesture* g = gestureAt(i);
                if (g) {
                    Serial.print(F("  - "));
                    Serial.println(g->name());
                }
            }
            Serial.println(F("anima do <gesture>"));
            return true;
        }
        char name[24];
        size_t len = (size_t)(sub - name_start);
        if (len >= sizeof(name)) len = sizeof(name) - 1;
        memcpy(name, name_start, len);
        name[len] = '\0';
        return anima.runGesture(name);
    }
    if (strncmp(sub, "take ", 5) == 0) {
        sub += 5;
        while (*sub == ' ') sub++;
        const char* name_start = sub;
        while (*sub && *sub != ' ') sub++;
        if (name_start == sub) {
            for (uint8_t i = 0; i < POSTURE_COUNT; i++) {
                Serial.print(F("  - "));
                Serial.println(POSTURES[i].name);
            }
            Serial.println(F("anima take <posture> [in <millis>]"));
            return true;
        }
        char name[16];
        size_t len = (size_t)(sub - name_start);
        if (len >= sizeof(name)) len = sizeof(name) - 1;
        memcpy(name, name_start, len);
        name[len] = '\0';
        uint16_t time_ms = 1000;
        while (*sub == ' ') sub++;
        if (*sub) {
            if (strncmp(sub, "in", 2) != 0 || (sub[2] != '\0' && sub[2] != ' ')) {
                Serial.println(F("anima take <posture> [in <millis>]"));
                return true;
            }
            sub += 2;
            while (*sub == ' ') sub++;
            if (!*sub) {
                Serial.println(F("anima take <posture> [in <millis>]"));
                return true;
            }
            time_ms = (uint16_t)atoi(sub);
        }
        return anima.takePosture(name, time_ms);
    }
    if (strncmp(sub, "track", 5) == 0 && (sub[5] == '\0' || sub[5] == ' ')) {
        sub += 5;
        while (*sub == ' ') sub++;
        if (!*sub) {
            bool was = anima.isEyeTrackerEnabled();
            anima.setEyeTrackerEnabled(!was);
            THREADS_SERIAL.print(F("anima track "));
            THREADS_SERIAL.println(anima.isEyeTrackerEnabled() ? "on" : "off");
            return true;
        }
    }
    if (strncmp(sub, "off", 3) == 0 && (sub[3] == '\0' || sub[3] == ' ')) {
        AnimaState* cur = anima.getCurrentState();
        if (cur && cur->getName() && strcmp(cur->getName(), "boot") == 0)
            return true;
        anima.setPaused(true);
        THREADS_SERIAL.println(F("anima off"));
        return true;
    }
    if (strncmp(sub, "on", 2) == 0 && (sub[2] == '\0' || sub[2] == ' ')) {
        anima.setPaused(false);
        THREADS_SERIAL.println(F("anima on"));
        return true;
    }
    Serial.println(F("anima take <posture> [in <millis>]"));
    Serial.println(F("anima do <gesture>"));
    Serial.println(F("anima track"));
    Serial.println(F("anima on|off"));
    return true;
}

/**
 * Fără argument: nume stare curentă; „list”: registru; altfel nume stare sau „boot” (instanță nouă).
 */
bool Animator::cmd_state(const char* arg) {
    const char* sub = arg ? arg : "";
    while (*sub == ' ') sub++;

    if (*sub == '\0') {
        THREADS_SERIAL.println(anima.getCurrentStateName());
        return true;
    }
    if (strncmp(sub, "list", 4) == 0 && (sub[4] == '\0' || sub[4] == ' ')) {
        AnimaState* cur = anima.getCurrentState();
        for (size_t i = 0; i < AnimaState::getRegistryCount(); i++) {
            AnimaState* s = AnimaState::getStateFromRegistry(i);
            if (!s) continue;
            if (s == cur) THREADS_SERIAL.print("[A] ");
            else THREADS_SERIAL.print("    ");
            if (s->readyForDeletion()) THREADS_SERIAL.print("[D] ");
            THREADS_SERIAL.println(s->getName());
        }
        return true;
    }
    /* state <nume>: forțează tranziția la starea cu numele dat */
    const char* name_start = sub;
    while (*sub && *sub != ' ' && *sub != '\t') sub++;
    char name_buf[16];
    size_t len = (size_t)(sub - name_start);
    if (len >= sizeof(name_buf)) len = sizeof(name_buf) - 1;
    memcpy(name_buf, name_start, len);
    name_buf[len] = '\0';
    if (len == 0) {
        THREADS_SERIAL.println(F("state [list|<name>]"));
        return true;
    }
    AnimaState* target = nullptr;
    if (strcmp(name_buf, "boot") == 0)
        target = new StateBoot();
    else
        target = AnimaState::findByName(name_buf);
    if (target) {
        AnimaState::changeState(&anima, target);
        THREADS_SERIAL.print(F("state -> "));
        THREADS_SERIAL.println(target->getName());
        return true;
    }
    THREADS_SERIAL.print(F("state: unknown '"));
    THREADS_SERIAL.print(name_buf);
    THREADS_SERIAL.println(F("' (standby|approach|focus|paused|error|boot)"));
    return false;
}

/**
 * Mapează poziția actorului 0..1 la comanda ochi [-1,1]: center_pos e punctul care se aliniază la 0 pe servo.
 */
static float mapEyePosition(float pos, float center_pos) {
    if (center_pos <= 0.0f)
        return (pos >= 1.0f ? 1.0f : -1.0f + 2.0f * pos);
    if (center_pos >= 1.0f)
        return (pos <= 0.0f ? -1.0f : -1.0f + 2.0f * pos);
    if (pos <= center_pos)
        return -1.0f + (pos / center_pos);
    return (pos - center_pos) / (1.0f - center_pos);
}

/**
 * Urmărire țintă: filtre EMA, mapEyePosition la ox, opțional oyup.1 după distanță dacă track_distance.
 * @return true dacă s-a trimis cel puțin un go_to la ox sau oyup.
 */
bool Animator::runEyeTracker(bool track_distance) {
    const uint32_t now = millis();
    /* Nu rulăm urmărirea țintei înainte de TRACKER_STARTTIME după ACTIVE/reset. */
    if (now < tracker_active_after_ms_)
        return false;

    bool sent_any = false;
    float os_val = -1.0f;
    float od_val = -1.0f;
    float pos_val = -1.0f;
    float pos_filt_val = -1.0f;
    const bool have_target = (ir_processor.getState() == IRProcessorState::TRACKING);
    const float pos = have_target ? ir_processor.getTrackedActorPos() : 0.0f;

    float oy_val = 0.0f;
    if (have_target) {
        /* Avem țintă: actualizăm filtrele EMA pentru poziție și distanță, mapăm la os/od (X) și oy (distanță → oyup.1). */
        tracker_pos_filt_ = TRACKER_EMA_ALPHA_POS * pos + (1.0f - TRACKER_EMA_ALPHA_POS) * tracker_pos_filt_;
        const float dist = ir_processor.getTrackedActorDist();
        tracker_dist_filt_ = TRACKER_EMA_ALPHA_DIST * dist + (1.0f - TRACKER_EMA_ALPHA_DIST) * tracker_dist_filt_;
        pos_val = pos;
        pos_filt_val = tracker_pos_filt_;
        os_val = mapEyePosition(tracker_pos_filt_, TRACKER_XPOS_EYES);
        od_val = mapEyePosition(tracker_pos_filt_, TRACKER_XPOS_EYES);
        float df = tracker_dist_filt_;
        if (df < TRACKER_MIN_DISTANCE) df = TRACKER_MIN_DISTANCE;
        if (df > 1.0f) df = 1.0f;
        oy_val = TRACKER_YPOS_MIN + (df - TRACKER_MIN_DISTANCE) / (1.0f - TRACKER_MIN_DISTANCE) * (TRACKER_YPOS_MAX - TRACKER_YPOS_MIN);
        tracker_last_pos_ = pos_val;
        tracker_last_pos_filt_ = pos_filt_val;
        tracker_last_os_ = os_val;
        tracker_last_od_ = od_val;
        tracker_last_oy_ = oy_val;
    } else {
        /* Fără țintă: folosim ultimele valori (pentru vizualizare, modulele următoare și pentru oy când readucem ținta). */
        pos_val = tracker_last_pos_;
        pos_filt_val = tracker_last_pos_filt_;
        os_val = tracker_last_os_;
        od_val = tracker_last_od_;
        oy_val = tracker_last_oy_;
    }
    bool just_found = have_target && !eye_tracker_has_target_;
    bool pos_changed_enough = have_target && (fabsf(pos_filt_val - tracker_last_actuator_pos_) > TRACKER_MIN_REFRESH_POS);
    bool oy_changed_enough = have_target && (fabsf(oy_val - tracker_last_actuator_oy_) > TRACKER_MIN_REFRESH_DIST);
    /* Trimitem go_to la ox (poziție X pe ochi) când tocmai am găsit ținta sau poziția s-a schimbat suficient; durată ca la Y (delta * TRACKER_X_SPEED_INVERSE). */
    if (just_found || pos_changed_enough) {
        const float cur_os = actuator_ox.getPosition(0);
        const float cur_od = actuator_ox.getPosition(1);
        const float dist_x = fmaxf(fabsf(os_val - cur_os), fabsf(od_val - cur_od));
        const uint16_t time_ms_ox = (uint16_t)(dist_x * (float)TRACKER_X_SPEED_INVERSE);
        actuator_ox.go_to(os_val, od_val, time_ms_ox);
        tracker_last_actuator_pos_ = pos_filt_val;
        sent_any = true;
    }
    /* Trimitem go_to la oyup (distanță → oyup.1) doar dacă track_distance=true; altfel doar filtrăm (fără mișcare oy). */
    if (track_distance && (just_found || oy_changed_enough)) {
        if (actuator_oyup.getGotoState() == ServoAct::GotoState::IDLE) {
            const float current_oy = actuator_oyup.getPosition(0);
            const uint16_t time_ms = (uint16_t)(fabsf(oy_val - current_oy) * (float)TRACKER_Y_SPEED_INVERSE);
            actuator_oyup.go_to(oy_val, X, time_ms);
            tracker_last_actuator_oy_ = oy_val;
            sent_any = true;
        }
    }
    eye_tracker_has_target_ = have_target;

    /* Opțional: date pentru vizualizare — țintă, poziție, os/od, oy. */
    if (teleplot_tracker_enabled_) {
        THREADS_SERIAL.print(">target:");
        THREADS_SERIAL.print(have_target ? "1" : "0");
        THREADS_SERIAL.print("\n");
        THREADS_SERIAL.print(">pos:");
        THREADS_SERIAL.print(pos_val, 4);
        THREADS_SERIAL.print("\n");
        THREADS_SERIAL.print(">pos_filt:");
        THREADS_SERIAL.print(pos_filt_val, 4);
        THREADS_SERIAL.print("\n");
        THREADS_SERIAL.print(">os:");
        THREADS_SERIAL.print(os_val, 4);
        THREADS_SERIAL.print("\n");
        THREADS_SERIAL.print(">od:");
        THREADS_SERIAL.print(od_val, 4);
        THREADS_SERIAL.print("\n");
        THREADS_SERIAL.print(">oy:");
        THREADS_SERIAL.print(oy_val, 4);
        THREADS_SERIAL.print("\n");
    }
    return sent_any;
}

#if TEST_ACTUATOR
/* Întârziere după ACTIVE înainte de primul burst (postura implicită ~1000 ms). */
static const uint32_t TEST_START_DELAY_MS = 1200;
static uint32_t s_last_test_debug_ms = 0;

/** Buclă de test: burst aleator pe un actuator la intervale (doar când TEST_ACTUATOR). */
void Animator::runTestLoop() {
    const uint32_t now = millis();
    const uint32_t active_runtime = now - active_since_ms_;
    // Secvența de test rulează doar când anima e ACTIVE (intrare după "The animator is alive!").

    if (test_burst_in_progress_) {
        ServoAct* act = ACTUATORS[test_actuator_index_];
        if (act->isGotoSequenceComplete()) {
            test_burst_in_progress_ = false;
            last_test_goto_ms_ = now;
            Serial.println("anima test: burst complete.");
        }
        return;
    }
    if (last_test_goto_ms_ != 0 && (now - last_test_goto_ms_) < TEST_REPEAT_MS) {
        if (now - s_last_test_debug_ms >= 5000) {
            s_last_test_debug_ms = now;
            // Serial.print("anima test: waiting repeat, next in ");
            // Serial.print(TEST_REPEAT_MS - (now - last_test_goto_ms_));
            // Serial.println(" ms");
        }
        return;
    }
    // Așteptăm postura implicită (1000 ms) să se termine înainte de primul burst de test
    if (active_runtime < TEST_START_DELAY_MS) {
        if (now - s_last_test_debug_ms >= 500) {
            s_last_test_debug_ms = now;
            // Serial.print("anima test: waiting delay, runtime=");
            // Serial.print(active_runtime);
            // Serial.print("/");
            // Serial.print(TEST_START_DELAY_MS);
            // Serial.println(" ms");
        }
        return;
    }

    test_actuator_index_ = (uint8_t)random(0, NUM_ACTUATORS);
    ServoAct* act = ACTUATORS[test_actuator_index_];
    uint8_t n_servos = ACTUATOR_SERVOS[test_actuator_index_];
    Serial.print("anima test: starting burst on ");
    Serial.println(ACTUATORS[test_actuator_index_]->name());
    test_burst_in_progress_ = true;
    if (n_servos == 1) {
        act->go_to(1.0f, (uint16_t)1000);
        act->then_go_to(0.5f, (uint16_t)500, TEST_DELAY_MS);
        act->then_go_to(-1.0f, (uint16_t)3000, TEST_DELAY_MS);
        act->then_go_to(0.0f, (uint16_t)500, TEST_DELAY_MS);
    } else {
        act->go_to(1.0f, 1.0f, (uint16_t)1000);
        act->then_go_to(0.5f, 0.5f, (uint16_t)500, TEST_DELAY_MS);
        act->then_go_to(-1.0f, -1.0f, (uint16_t)3000, TEST_DELAY_MS);
        act->then_go_to(0.0f, 0.0f, (uint16_t)500, TEST_DELAY_MS);
    }
}
#endif
