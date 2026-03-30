/*
 * anima.h
 *
 * animaMIC — Animator: coordonează senzorul IR, fluxul de procesare IR (IRProcessor) și actuatoarele servo pentru MIC.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: thread principal care rulează automatul de stări (boot → standby / approach / focus / paused / error),
 * urmărirea țintei pe ochi (ox, oyup) după regulile TRACKER_*, LED de stare, buton și secvență de reset.
 * Posturi și gesturi: repertoriu în postures / gestures.
 */

#ifndef ANIMA_H
#define ANIMA_H

#include "Threads.h"
#include "ServoAct.h"
#include "IRSense.h"
#include "postures.h"
#include "gestures.h"
#include "anima_tools.h"
#include <string.h>

class StateBoot;
class StateStandby;
class StateApproach;
class StateFocus;
class StatePaused;
class StateError;


/** Timp (ms) în starea ERROR după care monitorul de status declanșează reset Arduino. */
#define ANIMA_ERROR_RESET_TIME 3000

/** Max wait în WaitIr (inițializare) pentru IRSense ACTIVE; aliniat la timeout inițializare + retry UART (vezi IRSense). */
#ifndef BOOT_WAIT_IR_TIMEOUT_MS
#define BOOT_WAIT_IR_TIMEOUT_MS (IR_SENSE_BOOT_TIMEOUT_MS + IR_SENSE_UART_TIMEOUT_MS * 4 + IR_SENSE_RETRY_MS)
#endif

/** Comutator compilare: 1 = activează bucla de test (actuator aleator per burst), 0 = dezactivat. */
#ifndef TEST_ACTUATOR
#define TEST_ACTUATOR 0
#endif

/**
 * Animator: bucla cooperativă a personajului — stări, posturi, gesturi, urmărire țintă, pauză, erori.
 * Stările concrete sunt în acest fișier; logica de tranziție și tick în anima.cpp.
 */
class Animator : public Thread {
public:
    Animator();

    /** Înregistrează stările în registru, pornește din StateBoot (LED inițializare, IRProcessor oprit până la „alive”). */
    void begin() override;
    /** Rulează automatul de stări, butonul, watchdog activare servomotoare; resincronizare urmărire după IR ACTIVE din IDLE/FAILED. */
    void loop() override;
    /** Golește registru stări, oprește LED, reset actuatoare, ir_sense.init(). */
    void end() override;

    /** Apel end() apoi begin() — repornire curată a animatorului. */
    void reset();

    /** true când starea curentă e standby, approach sau focus (operativ, nu inițializare/paused/error). */
    bool isActive() const;
    bool isError() const { AnimaState* c = AnimaState::getCurrentState(); return c && c->getName() && strcmp(c->getName(), "error") == 0; }

    /** Trimite go_to la toate actuatoarele cu pozițiile posturii; time_ms aplicat tuturor. Dacă eyes_too e false, sare peste ox. Returnează false dacă postura nu e găsită. */
    bool takePosture(const char* posture_name, uint16_t time_ms = 1000, bool eyes_too = true);
    /** Idem, dar primește pointer la postură (fără căutare după nume). Returnează false dacă post e nullptr. */
    bool takePosture(const Posture* post, uint16_t time_ms = 1000, bool eyes_too = true);

    /** Rulează gestul după nume. Returnează false dacă gestul nu e găsit. */
    bool runGesture(const char* gesture_name);
    /** Idem, dar primește pointer la gest. Returnează false dacă g e nullptr. */
    bool runGesture(Gesture* g);

    bool isEyeTrackerEnabled() const { return eye_tracker_enabled_; }
    /** La activare (false->true), resetează starea urmăririi țintei (TRACKER_STARTTIME, țintă ștearsă). */
    void setEyeTrackerEnabled(bool on);
    /** true când urmărirea țintei (ochi) are țintă curentă (actor); când false, pos/pos_filt/os/od rămân la ultimele valori. */
    bool hasEyeTrackerTarget() const { return eye_tracker_has_target_; }
    bool isTeleplotTrackerEnabled() const { return teleplot_tracker_enabled_; }
    void setTeleplotTrackerEnabled(bool on) { teleplot_tracker_enabled_ = on; }

    /** Pauză: din ACTIVE ia postura implicită apoi pauză; din init: pauză directă. Reluare cu setPaused(false). */
    void setPaused(bool pause);
    bool isPaused() const { AnimaState* c = AnimaState::getCurrentState(); return c && c->getName() && strcmp(c->getName(), "paused") == 0; }

    /** Handler pentru comanda "anima" (interpretor local pentru subcomenzi). */
    static bool cmd_anima(const char* arg);
    /** Handler pentru comanda "state" (raport/listă/schimbare). */
    static bool cmd_state(const char* arg);

    /** Starea curentă (delegare la mașina de stări). */
    AnimaState* getCurrentState() const { return AnimaState::getCurrentState(); }
    /** Numele stării curente sau "(none)" dacă nu e setată. */
    const char* getCurrentStateName() const { AnimaState* c = AnimaState::getCurrentState(); return c ? c->getName() : "(none)"; }

    friend class AnimaState;
    friend class StateBoot;
    friend class StateStandby;
    friend class StateApproach;
    friend class StateFocus;
    friend class StatePaused;
    friend class StateError;
    friend bool abortIfAllActuatorsOffline(Animator* a);

private:
    /** Trece în StateError, memorează momentul pentru auto-reset după ANIMA_ERROR_RESET_TIME. */
    void setError(const char* cause);
    /** Buton: apăsare lungă / secret_click → secvență reset actuatoare+IR sau reset Arduino. Un apel per loop(). */
    void processButton();

    enum class PressState { None, WaitSecretClick, WaitOneSec, WaitAfterReset };

    enum class LedIntent {
        Off,
        Error,
        InitRandomBlink,
        ActuatorActivatingRandomBlink,  /* actuator în curs de reactivare */
        ActiveSolid,
        ActiveWithErrorsSolid,
        FocusSolid,      /* actori detectați, neurmăriți */
        FocusBlink,   /* actori urmăriți */
        PausedPulse,
        PausedPulseErrors
    };
    /** true dacă cel puțin un actuator sau senzorul IR e offline. */
    bool hasActuatorOrIrOffline() const;
    /** true dacă cel puțin un actuator e în curs de activare (reactivare). */
    bool hasActuatorActivating() const;
    /** true când ir1=1000 și toți ceilalți senzori IR citesc 0. */
    bool checkResetCondition() const;
    /** Urmărire a țintei (ochi): rulează o dată per cadru IR când ACTIVE și după TRACKER_STARTTIME; opțional afișare os/od/oy pentru depanare. Dacă track_distance=false nu trimite go_to la oyup. Returnează true dacă a trimis vreun go_to la ox sau oyup, false altfel. */
    bool runEyeTracker(bool track_distance = false);
#if TEST_ACTUATOR
    void runTestLoop();
    static const unsigned int TEST_REPEAT_MS = 20000;
    static const unsigned int TEST_DELAY_MS = 3000;
    /** Index în ACTUATORS[] pentru burst-ul de test curent; se schimbă la fiecare iterație (aleator). */
    uint8_t test_actuator_index_;
#endif

    /** Numele stării de revenit la reluare din PAUSED (ex.: "standby"); pointer în paused_from_state_name_buf_. */
    const char* paused_from_state_name_;
    static const unsigned int PAUSED_FROM_NAME_BUF = 16;
    char paused_from_state_name_buf_[PAUSED_FROM_NAME_BUF];
    uint32_t active_since_ms_;
    /** Când condiția de reset a devenit true (0 = nu e îndeplinită în prezent). */
    uint32_t reset_condition_start_ms_;
    /** Când am intrat în starea ERROR (pentru auto-reset ANIMA_ERROR_RESET_TIME). */
    uint32_t error_since_ms_;
    /** Tracker activ după acest timp (millis()); înainte nu rulează urmărirea. */
    uint32_t tracker_active_after_ms_;
    /** Poziție actor filtrată EMA (0–1) pentru urmărirea țintei (ochi). */
    float tracker_pos_filt_;
    /** Distanță actor filtrată EMA (0–1) pentru urmărirea țintei (Y = oyup.1). */
    float tracker_dist_filt_;
    /** Dacă false, runEyeTracker() nu e apelat. Implicit on. */
    bool eye_tracker_enabled_;
    /** Dacă true, runEyeTracker() trimite date pentru vizualizare (comanda serială teleplot tracker). */
    bool teleplot_tracker_enabled_;
    /** true când urmărirea țintei are în prezent țintă; expus pentru restul codului anima. */
    bool eye_tracker_has_target_;
    /** Ultimul pos/pos_filt/os/od când ținta se pierde (menținut pentru vizualizare și pentru modulele următoare). */
    float tracker_last_pos_;
    float tracker_last_pos_filt_;
    float tracker_last_os_;
    float tracker_last_od_;
    /** Poziție filtrată (0–1) când am trimis ultima dată go_to la ox; folosit pentru TRACKER_MIN_REFRESH_POS. */
    float tracker_last_actuator_pos_;
    /** Ultimul oy (oyup.1) când ținta se pierde; valoare trimisă la oyup pentru Y (distanță). */
    float tracker_last_oy_;
    /** Valoare oyup.1 ultima dată trimisă; folosit pentru refresh. */
    float tracker_last_actuator_oy_;
    /** Ultima stare IRSense văzută (pentru recuperare IR → ACTIVE în runtime). */
    IRSenseState ir_sense_prev_state_;
    /** true după prima ieșire din init (LED-ul de inițializare nu se mai afișează la reinițializări). */
    bool boot_led_done_;
    /** Secvență press: LED off -> 1500 ms (secret_click = reset) -> reset actuatoare+IR -> 1 s -> reset Arduino. */
    PressState press_state_;
    uint32_t press_step_start_ms_;
    ColorLed color_led_;
    PushButton button_;
#if TEST_ACTUATOR
    uint32_t last_test_goto_ms_;
    bool test_burst_in_progress_;
#endif
};

/** Inițializare: verificare HW, activare actuatoare în perechi, postură „mic”, așteptare IRSense ACTIVE sau timeout. */
class StateBoot : public AnimaState {
public:
    StateBoot() : AnimaState("boot") {}
    void begin(Animator* a) override;
    void end() override;
    void loop(Animator* a) override;
private:
    enum class BootSubState { CheckHw, ActivateMmMma, ActivateOxoyup, DefaultPosture, WaitIr };
    BootSubState boot_sub_state_ = BootSubState::CheckHw;
    uint32_t activation_start_ms_ = 0;
    /** millis la intrare în WaitIr (după DefaultPosture). */
    uint32_t boot_wait_ir_start_ms_ = 0;
    /** Activează IRProcessor, mesaj „alive”, tranziție la standby. */
    void finishToStandby(Animator* a);
};

/** Așteptare vizitator: posturi și gesturi la intervale aleatoare; tranziții spre approach/focus după flux IR. */
class StateStandby : public AnimaState {
public:
    StateStandby();
    void begin(Animator* a) override;
    void end() override;
    void loop(Animator* a) override;
private:
    static const uint8_t STANDBY_MAX_LIST = 16;
    const Posture* standby_postures[STANDBY_MAX_LIST];
    Gesture* standby_gestures[STANDBY_MAX_LIST];
    uint32_t next_posture_ms_;
    uint32_t next_gesture_ms_;
    uint8_t current_posture_index_;
};

/** Fantome / actori neurmăriți: gesturi wave, LED turcoaz; spre focus când apare urmărire țintă. */
class StateApproach : public AnimaState {
public:
    StateApproach();
    void begin(Animator* a) override;
    void end() override;
    void loop(Animator* a) override;
private:
    static const uint8_t APPROACH_GESTURE_COUNT = 4;
    Gesture* approach_gestures[APPROACH_GESTURE_COUNT];
    uint32_t next_action_ms_;
    /** Timestamp ultimei executări a gestului "focus"; 0 = niciodată (pentru condiția la intrare din standby). */
    uint32_t last_focus_gesture_ms_;
};

/** Urmărire țintă: sub-stări Follow / Invite / Goodbye; runEyeTracker pe cadre IR noi. */
class StateFocus : public AnimaState {
public:
    StateFocus() : AnimaState("focus"), last_straighten_gesture_ms_(0) {}
    void begin(Animator* a) override;
    void end() override;
    void loop(Animator* a) override;
private:
    enum class FocusSub { Follow, Invite, Goodbye };
    /** Timestamp ultimei executări a gestului "straighten"; 0 = niciodată. */
    uint32_t last_straighten_gesture_ms_;
    FocusSub focus_sub_;
    uint32_t focus_still_since_ms_;
    uint32_t focus_invite_first_enter_ms_;
    bool focus_distance_on_;
    uint32_t focus_next_invite_ms_;
    uint32_t focus_gesture_ends_ms_;
    uint32_t focus_goodbye_ends_ms_;
    static const uint8_t FOCUS_INVITE_COUNT = 4;
    static const uint8_t FOCUS_GOODBYE_COUNT = 2;
    Gesture* focus_invite_gestures[FOCUS_INVITE_COUNT];
    Gesture* focus_goodbye_gestures[FOCUS_GOODBYE_COUNT];
};

/** Pauză: LED puls; la reluare revine la starea memorată în paused_from_state_name_. */
class StatePaused : public AnimaState {
public:
    StatePaused() : AnimaState("paused") {}
    void begin(Animator* a) override;
    void end() override;
    void loop(Animator* a) override;
};

/** Eroare fatală: LED roșu; după ANIMA_ERROR_RESET_TIME reset Arduino. */
class StateError : public AnimaState {
public:
    StateError() : AnimaState("error") {}
    void begin(Animator* a) override;
    void end() override;
    void loop(Animator* a) override;
};

#endif /* ANIMA_H */
