/*
 * ServoAct.h
 *
 * animaMIC — Driver I2C pentru plăcile ServoAct: citire STATUS/POS, activare servomotoare, cozi go_to asincrone
 * prin thread-ul I2C partajat (i2c.h).
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: fiecare actuator = adresă I2C, până la MAX_SERVOS_PER_ACTUATOR servomotoare; în animaMIC posturile/urmărirea folosesc de obicei [-1,+1];
 * domeniul pe I2C urmează protocolul plăcii. Simbolul X = „nu schimba această coordonată” la go_to. Automat go_to: POS_SET → GOTO →
 * așteptare MOV clar / durată minimă / timeout. Recuperare la erori: reset, sondare online, reactivare.
 */

#ifndef SERVOACT_H
#define SERVOACT_H

#include <Arduino.h>
#include <Wire.h>
#include <queue>
#include <deque>
#include "i2c.h"
#include "Threads.h"

/** Registre I2C (protocol placă ServoAct). */
#define I2C_REG_RESET 0x00
#define I2C_REG_START 0x01
#define I2C_REG_STATUS 0x02
#define I2C_REG_POS1 0x04
#define I2C_REG_POS_SET1 0x0C
#define I2C_REG_POS_SET2 0x0E
#define I2C_REG_POS_SET3 0x10
#define I2C_REG_POS_SET4 0x12
#define I2C_REG_GOTO 0x14

#define MAX_SERVOS_PER_ACTUATOR 2

/** Interval între citiri STATUS/POS periodice (ms). */
#define ACTUATOR_REFRESH_TIME 100
/** Durată maximă de așteptare pentru finalizarea activării servomotoarelor (ms). */
#define SERVO_ACTIVATION_TIMEOUT 20000
/** Două eșecuri de activare la interval ≤ acest timp → semnal watchdog pentru anima. */
#define SERVO_ACTIVATION_WATCHDOG_TIME 60000
/** Durată minimă considerată pentru o mișcare în curs (ms). */
#define SERVO_MIN_MOTION_DURATION 300
/** Marjă suplimentară la timeout mișcare față de time_ms comandat (ms). */
#define SERVO_MAX_MOTION_EXTRATIME 2000
/** Limită elemente în coada go_to / then_go_to (gesturi lungi). */
#define MAX_GOTO_QUEUE_SIZE 32

/**
 * Valoare sentinelă: „nu modifica poziția” pe canalul respectiv (comparare float).
 * Folosită în tabele de posturi și în cozi goto.
 */
#define X 10.0f
// #define DIAGNOSE_GOTO

/** O intrare per servo: biți din STATUS și poziție citită sau ultima comandată. */
struct ServoData {
    bool active;     /* bit ACT în STATUS */
    bool error;      /* bit ERR */
    float position;  /* [-2,+2]: măsurată dacă există feedback, altfel țintă */
};

/**
 * Thread per actuator: refresh STATUS/POS, automat go_to pe coadă I2C, activare și recuperare.
 */
class ServoAct : public Thread {
public:
    ServoAct(const char* name, uint8_t i2c_address);
    ~ServoAct();

    /** Înregistrare în lista statică; sondă I2C directă (setup) pentru STATUS inițial. */
    void begin() override;
    /** Buclă: recuperare, activare, automat goto, refresh, Teleplot opțional. */
    void loop() override;
    /** Scoate din listă, golește cozi, oprește stări. */
    void end() override;

    /** Scrie START: inițializare servomotoare pe placă. */
    void activate();

    /** Scrie RESET pe placă. */
    void reset();

    /** Golește cozile goto și delayed; fără I2C; mișcarea pe placă poate continua. */
    void clearGotoQueues();

    /**
     * Întârziere aplicată la următorul go_to / then_go_to (consum next_goto_delay_ms_).
     * Apel înainte de segmentul care trebuie amânat.
     */
    void delay_next_goto(uint16_t delay_ms);

    /**
     * Comandă mișcare: poziții în [-2,+2] sau X pe canal nefolosit; time_ms=0 = viteză maximă.
     * Dacă alt go_to e în curs, intră în coadă (cu filtrare); vezi then_go_to pentru lanț.
     */
    void go_to(float pos1, uint16_t time_ms = 0);
    void go_to(float pos1, float pos2, uint16_t time_ms = 0);
    void go_to(float pos1, float pos2, float pos3, uint16_t time_ms = 0);
    void go_to(float pos1, float pos2, float pos3, float pos4, uint16_t time_ms = 0);

    void then_go_to(float pos1, uint16_t time_ms = 0);
    void then_go_to(float pos1, float pos2, uint16_t time_ms = 0);
    void then_go_to(float pos1, float pos2, float pos3, uint16_t time_ms = 0);
    void then_go_to(float pos1, float pos2, float pos3, float pos4, uint16_t time_ms = 0);

    /** Stări interne ale automatului go_to (POS_SET, GOTO, așteptări). */
    enum class GotoState {
        IDLE,
        WAITING_POS,
        WAITING_GOTO,
        WAITING_MIN_DURATION,
        WAITING_MOV_CLEAR,
        ERROR
    };
    GotoState getGotoState() const { return goto_data_.state; }
    /** Numele actuatorului (max 4 caractere + nul). */
    const char* name() const { return name_; }
    bool isOnline() const { return online_; }
    bool isActivating() const { return activating_; }
    /** Număr servomotoare instalate (1 sau 2 pe acest actuator). */
    uint8_t numServos() const { return num_servos_; }
    /** Toate servomotoarele instalate au ACT setat. */
    bool allServosActive() const;
    /** IDLE, cozi goale, fără întârziere pending — secvența s-a terminat. */
    bool isGotoSequenceComplete() const;

    /**
     * Returnează true o dată dacă s-a declanșat watchdog (2 eșecuri activare în fereastra definită);
     * apoi șterge flag-ul.
     */
    bool getAndClearActivationWatchdogTrigger();

    /** Poziție curentă servo index 0-based; 0 dacă invalid. */
    float getPosition(uint8_t servo_index) const;
    /** Ultima țintă după un goto completat; altfel ca getPosition. */
    float getSetPosition(uint8_t servo_index) const;

    /** Interpretor comanda serială „act” (subcomenzi plot, activate, reset, get, info, goto, status). */
    static bool cmd_act(const char* arg);

private:
    char name_[5];
    uint8_t i2c_address_;
    ServoData servos_[MAX_SERVOS_PER_ACTUATOR];

    uint8_t read_buffer_[2 + (MAX_SERVOS_PER_ACTUATOR * 2)];
    uint8_t start_buffer_[1];
    uint8_t num_servos_;
    uint8_t last_servo_;
    uint16_t trans_id_;
    uint32_t next_refresh_ms_;
    bool online_;
    uint8_t consecutive_failures_;
    bool plot_enabled_;
    bool activating_;
    uint32_t activation_start_ms_;
    uint16_t start_trans_id_;
    uint32_t next_start_read_ms_;
    bool saw_initializing_;
    bool wait_for_status_after_activation_;
    uint32_t last_activation_failure_ms_;
    uint32_t previous_activation_failure_ms_;
    bool activation_watchdog_triggered_;
    float last_set_position_[MAX_SERVOS_PER_ACTUATOR];
    bool last_set_valid_;

    /**
     * După timeout mișcare sau ERR: golește cozi, marchează offline, traseu reset → online → activate.
     */
    enum class RecoveryState {
        RECOVERY_IDLE,
        RECOVERY_RESET,
        RECOVERY_WAIT_ONLINE,
        RECOVERY_SETTLE,
        RECOVERY_ACTIVATE,
        RECOVERY_ACTIVATE_WAIT
    };
    bool recovering_;
    RecoveryState recovery_state_;
    uint32_t recovery_start_ms_;
    uint32_t recovery_next_probe_ms_;
    uint16_t recovery_probe_trans_id_;

    /** O comandă goto în coadă: poziții, durată, întârziere după segmentul anterior. */
    struct GotoCoordinates {
        float positions[MAX_SERVOS_PER_ACTUATOR];
        uint16_t time_ms;
        uint16_t delay_after_previous_ms;
    };

    /** Goto planificat la trigger_at_ms (sortat în deque). */
    struct DelayedGoto {
        uint32_t trigger_at_ms;
        GotoCoordinates coords;
    };

    /** Stare + buffere tranzacții I2C pentru POS_SET/GOTO + cozi. */
    struct GotoData {
        GotoState state;
        uint16_t pos_trans_ids[2];
        uint8_t pos_trans_count;
        uint8_t pos_data[2][8];
        uint8_t pos_data_len[2];
        uint8_t pos_reg_addr[2];
        uint16_t goto_trans_id;
        uint8_t goto_data_bytes[2];
        uint32_t start_time_ms;
        uint16_t time_ms;
        float goto_positions[MAX_SERVOS_PER_ACTUATOR];
        bool queued_status_printed_;
        bool sequence_started_;
        std::queue<GotoCoordinates> queue_;
        std::deque<DelayedGoto> delayed_queue_;
    };
    GotoData goto_data_;
    /** Consum la următorul go_to/then_go_to după delay_next_goto. */
    uint16_t next_goto_delay_ms_ = 0;

    void processGotoStateMachine();

    void startRecovery();
    void processRecovery();
    void recordActivationFailureAndCheckWatchdog();

    static ServoAct* first_;
    ServoAct* next_;

    static bool isActuatorName(const char* name);

    void updateServoDataFromBuffer();

    static bool cmd_act_plot(const char* name_arg, const char* onoff_arg);
    static bool cmd_act_activate(const char* name_arg);
    static bool cmd_act_reset(const char* name_arg);
    static bool cmd_act_get(const char* name_arg);
    static bool cmd_act_info(const char* name_arg);
    static bool cmd_act_goto(const char* name_arg, const char* args);
    static bool cmd_act_status(const char* name_arg);

    static void printActuatorStatus(ServoAct* actuator);
    static void teleplotPositions();

    void go_to_impl(const float positions[MAX_SERVOS_PER_ACTUATOR], uint16_t time_ms);
    void then_go_to_impl(float positions[MAX_SERVOS_PER_ACTUATOR], uint16_t time_ms, uint16_t delay_after_previous_ms = 0, bool executing_delayed = false);
    void pushDelayedGoto(uint32_t trigger_at_ms, const GotoCoordinates& coords);
    bool enqueueGotoCoordinates(const GotoCoordinates& coords);
    void executeNextQueuedGoto();
    /** Pentru fiecare element din coadă: pune X unde comanda nouă suprascrie canalul; elimină dacă devine redundant. */
    void filterQueueByIncomingPositions(const float positions[MAX_SERVOS_PER_ACTUATOR]);
    void printGotoStatus(const char* status, bool final = false);
    void printGotoStatusWithPositions(float positions[MAX_SERVOS_PER_ACTUATOR], uint16_t time_ms, const char* status, bool final = false);
    void formatGotoPositions(float positions[MAX_SERVOS_PER_ACTUATOR], char* buffer, size_t buffer_size);
};

#endif // SERVOACT_H
