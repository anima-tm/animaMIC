/*
 * IRSense.h
 *
 * animaMIC — Driver UART pentru plăcile IRsense și punte spre USB (ecou / depanare).
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: un singur fir UART (IR_SENSE_UART) leagă microcontrollerul de plăcile IRsense; citirile sunt
 * vectori de valori numerice (IRFrame). După inițializare (id → get), starea ACTIVE cere „get” periodic;
 * IRProcessor consumă cadrele pentru fluxul de procesare IR.
 */

#ifndef IRSENSE_H
#define IRSENSE_H

#include <Arduino.h>
#include "Threads.h"
#include <vector>

/** UART hardware către IRsense; trebuie Serial1.begin(...) în setup() înainte de Serial USB. */
#define IR_SENSE_UART Serial1

#define IR_SENSE_BOOT_TIMEOUT_MS 2000    // ms fără mesaj „alive” în IDLE → declanșare init()
#define IR_SENSE_UART_TIMEOUT_MS 100     // ms așteptare răspuns la o comandă trimisă
#define IR_SENSE_RETRY_MS 5000           // ms în FAILED înainte de reîncercare (reset UART)
#define IR_SENSE_GET_INTERVAL_MS 100     // ms între două cereri „get” în ACTIVE

/**
 * Vector de citiri IR: un element = un senzor (float). Raw tipic 0–1000.
 * teleplot_mask_: câte un bit per senzor (1-based în API-ul setTeleplotBit); canal opțional pentru vizualizare.
 */
class IRFrame {
public:
    size_t size() const { return data_.size(); }
    float at(size_t i) const { return data_.at(i); }
    float& operator[](size_t i) { return data_[i]; }
    void clear() { data_.clear(); }
    void push_back(float v) { data_.push_back(v); }
    void resize(size_t n) { data_.resize(n); }

    void setTeleplotBit(int one_based_index, bool on);
    bool getTeleplotBit(int one_based_index) const;
    void setTeleplotAll(size_t n_sensors, bool on);
    bool getTeleplotAll(size_t n_sensors) const;

private:
    std::vector<float> data_;
    uint32_t teleplot_mask_ = 0;  // biți 0..31 ↔ senzori 1..32
};

/** Stări automat UART + inițializare IRsense. */
enum class IRSenseState {
    IDLE,         // neinițializat; ecou linii pe USB (prefix „ir: ”), linii Teleplot „>” ignorate
    INIT_ID,      // trimis „id”; așteptare răspuns sau timeout
    INIT_GET,     // trimis „get” la init; așteptare vector senzori
    ACTIVE,       // periodic „get” + GET_PENDING
    GET_PENDING,  // așteptare răspuns la ultimul „get”
    FAILED        // eroare; retry după IR_SENSE_RETRY_MS
};

/**
 * Thread (interval 0): citește UART, menține automatul de stări, umple readings_.
 * Comanda serială „ir …” trimite text pe UART prin forwardToUart.
 */
class IRSense : public Thread {
public:
    IRSense() : Thread(0), line_len_(0), state_(IRSenseState::IDLE), init_wait_start_ms_(0), boot_idle_start_ms_(0), failed_start_ms_(0), num_boards_(0), last_get_ms_(0), get_wait_start_ms_(0), get_timeout_count_(0), get_wrong_count_(0), expected_sensor_count_(0), expecting_reply_to_forward_(false), new_frame_available_(false), system_reset_pending_(false) { line_buf_[0] = '\0'; pending_forward_[0] = '\0'; }

    /** Golește buffer linie și citiri; pornește cronometrul pentru timeout boot. */
    void begin() override;

    /** O iterație: timeout-uri pe stare, apoi citire caractere UART și linii complete. */
    void loop() override;

    /**
     * Trimite text pe UART (comanda „ir”). În INIT_* / GET_PENDING mesajul utilizator intră în coadă (un slot).
     * @param arg text de trimis (fără \n; se adaugă în flush)
     * @param from_user true = de la utilizator (poate seta err:busy); false = trimitere internă (ex. reset)
     */
    void forwardToUart(const char* arg, bool from_user = true);

    /** Trimite „id”, trece în INIT_ID; răspunsul valid declanșează „get” și INIT_GET. */
    void init();

    /**
     * Înainte de reset Arduino: trimite „reset” pe UART, IDLE, fără re-init până la reboot.
     * Evită mesaje „Found N sensors” în timpul așteptării resetului plăcii.
     */
    void resetForSystemReset();

    IRSenseState getState() const { return state_; }

    const IRFrame* getReadings() const { return &readings_; }
    IRFrame* getReadingsFrame() { return &readings_; }
    size_t getNumSensors() const { return readings_.size(); }

    /** După fiecare „get” valid: true până la clearNewFrame() (consumat de IRProcessor). */
    bool hasNewFrame() const { return new_frame_available_; }
    void clearNewFrame() { new_frame_available_ = false; }

private:
    static int parseIdReplyLine(const char* line);
    bool parseGetReply(const char* line, IRFrame& readings_out, int& num_boards_out);
    void flushPendingForward();

    static const int LINE_BUF = 256;
    char line_buf_[LINE_BUF];       // linie UART în construire
    int line_len_;                   // lungime curentă în line_buf_
    IRSenseState state_;
    uint32_t init_wait_start_ms_;    // deadline timeout INIT_ID
    uint32_t boot_idle_start_ms_;    // început IDLE (pentru IR_SENSE_BOOT_TIMEOUT_MS)
    uint32_t failed_start_ms_;       // început FAILED (pentru retry)
    uint8_t num_boards_;             // din răspuns „id”
    IRFrame readings_;               // ultimul cadru „get” valid
    size_t expected_sensor_count_;   // număr senzori după primul get reușit
    uint32_t last_get_ms_;           // ultima trimitere „get” (ACTIVE)
    uint32_t get_wait_start_ms_;     // început așteptare răspuns GET_PENDING
    uint8_t get_timeout_count_;    // timeout-uri consecutive la „get”
    uint8_t get_wrong_count_;        // răspunsuri invalide consecutive
    char pending_forward_[LINE_BUF]; // coadă un mesaj „ir” cât UART e ocupat cu init
    bool expecting_reply_to_forward_;
    bool new_frame_available_;
    bool system_reset_pending_;      // după resetForSystemReset(); blocare init
};

extern IRSense ir_sense;

/** Apelat din handler-ul comenzii „ir”; delegă la ir_sense.forwardToUart. */
void ir_forward_to_uart(const char* arg);

#endif // IRSENSE_H
