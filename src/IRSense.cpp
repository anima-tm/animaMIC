/*
 * IRSense.cpp
 *
 * animaMIC — Implementare driver UART pentru plăcile IRsense și punte serială USB.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: automatul de stări (IDLE → INIT_* → ACTIVE / FAILED), parsare linii „id”/„get”,
 * inversare ordine senzori (convenție hardware), coadă pentru comanda „ir” cât UART e ocupat.
 */

#include "IRSense.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/** Inițializare thread: golește bufferul de linie și citirile (evită actor fantomă la pornire). */
void IRSense::begin() {
    line_len_ = 0;
    line_buf_[0] = '\0';
    readings_.clear();
    boot_idle_start_ms_ = millis();
    system_reset_pending_ = false;
}

/**
 * Interpretează răspunsul la „id”: id-uri separate de „; ” (ex. „1; 2”).
 * @return număr de plăci sau 0 dacă formatul e invalid
 */
int IRSense::parseIdReplyLine(const char* line) {
    int n = 0;
    const char* p = line;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        if (n > 0) {
            if (p[0] != ';' || p[1] != ' ') return 0;
            p += 2;
        }
        if (!isdigit((unsigned char)*p)) return 0;
        while (isdigit((unsigned char)*p)) p++;
        n++;
    }
    if (*p != '\0' && *p != '\r') return 0;
    return n;
}

/**
 * O iterație: timeout-uri pe stare, apoi citire UART și tratare linii complete (\n / \r).
 * Linii care încep cu „>” sunt ignorate la ecou (Teleplot de pe placă).
 */
void IRSense::loop() {
    // Când UART e liber (nu așteptăm id/get), trimite eventualul mesaj ir din coadă.
    if (state_ != IRSenseState::INIT_ID && state_ != IRSenseState::INIT_GET && state_ != IRSenseState::GET_PENDING && pending_forward_[0] != '\0')
        flushPendingForward();

    // Verificări per stare (timeout etc.)
    switch (state_) {
        case IRSenseState::IDLE:
            // Când resetul de sistem e în așteptare, nu porni init (evită "Found N sensors" înainte de reset Arduino).
            if (system_reset_pending_)
                break;
            // După IR_SENSE_BOOT_TIMEOUT_MS fără "IRsense is alive." declanșăm init (ex.: Arduino reset, IRSense nu).
            if ((uint32_t)(millis() - boot_idle_start_ms_) >= (uint32_t)IR_SENSE_BOOT_TIMEOUT_MS)
                init();
            break;
        case IRSenseState::INIT_ID:
            if ((uint32_t)(millis() - init_wait_start_ms_) >= (uint32_t)IR_SENSE_UART_TIMEOUT_MS) {
                state_ = IRSenseState::FAILED;
                failed_start_ms_ = millis();
                THREADS_SERIAL.println("Error: IRsense no response (timeout).");
                line_len_ = 0;
                line_buf_[0] = '\0';
                return;
            }
            break;
        case IRSenseState::INIT_GET:
            if ((uint32_t)(millis() - get_wait_start_ms_) >= (uint32_t)IR_SENSE_UART_TIMEOUT_MS) {
                state_ = IRSenseState::FAILED;
                failed_start_ms_ = millis();
                THREADS_SERIAL.println("Error: IRsense get timeout at init.");
                line_len_ = 0;
                line_buf_[0] = '\0';
                return;
            }
            break;
        case IRSenseState::ACTIVE:
            // La fiecare IR_SENSE_GET_INTERVAL_MS trimitem "get" și trecem în GET_PENDING.
            if ((uint32_t)(millis() - last_get_ms_) >= (uint32_t)IR_SENSE_GET_INTERVAL_MS) {
                line_len_ = 0;
                line_buf_[0] = '\0';
                IR_SENSE_UART.print("get\n");
                state_ = IRSenseState::GET_PENDING;
                get_wait_start_ms_ = millis();
                last_get_ms_ = millis();
            }
            break;
        case IRSenseState::GET_PENDING:
            // Timeout la răspuns la "get": o dată doar mesaj, la două consecutive trecem în FAILED.
            if ((uint32_t)(millis() - get_wait_start_ms_) >= (uint32_t)IR_SENSE_UART_TIMEOUT_MS) {
                get_timeout_count_++;
                THREADS_SERIAL.println("IRsense timeout");
                if (get_timeout_count_ >= 2) {
                    state_ = IRSenseState::FAILED;
                    failed_start_ms_ = millis();
                } else {
                    state_ = IRSenseState::ACTIVE;
                }
                line_len_ = 0;
                line_buf_[0] = '\0';
                return;
            }
            break;
        case IRSenseState::FAILED:
            // La fiecare IR_SENSE_RETRY_MS trimitem "reset" pe UART; după reboot IRSense trimite "alive" și reinițializăm.
            if ((uint32_t)(millis() - failed_start_ms_) >= (uint32_t)IR_SENSE_RETRY_MS) {
                forwardToUart("reset", false);
                failed_start_ms_ = millis();
            }
            break;
    }

    // Citire caractere de pe UART și formare linii (terminate cu \n sau \r).
    while (IR_SENSE_UART.available()) {
        int c = IR_SENSE_UART.read();
        if (c < 0) break;
        if (c == '\n' || c == '\r') {
            if (line_len_ > 0) {
                line_buf_[line_len_] = '\0';

                switch (state_) {
                    case IRSenseState::INIT_ID: {
                        if (expecting_reply_to_forward_) {
                            THREADS_SERIAL.print("ir: ");
                            THREADS_SERIAL.println(line_buf_);
                            expecting_reply_to_forward_ = false;
                            line_len_ = 0;
                            line_buf_[0] = '\0';
                            continue;
                        }
                        int n = parseIdReplyLine(line_buf_);
                        if (n >= 1) {
                            num_boards_ = (uint8_t)n;
                            get_timeout_count_ = 0;
                            get_wrong_count_ = 0;
                            line_len_ = 0;
                            line_buf_[0] = '\0';
                            IR_SENSE_UART.print("get\n");
                            state_ = IRSenseState::INIT_GET;
                            get_wait_start_ms_ = millis();
                        } else {
                            state_ = IRSenseState::FAILED;
                            failed_start_ms_ = millis();
                            if (line_buf_[0] != '>') {
                                THREADS_SERIAL.print("ir: ");
                                THREADS_SERIAL.println(line_buf_);
                            }
                            THREADS_SERIAL.println("Error: IRsense invalid id reply.");
                        }
                        break;
                    }
                    case IRSenseState::INIT_GET: {
                        int num_boards = 0;
                        bool ok = parseGetReply(line_buf_, readings_, num_boards) &&
                                  num_boards == (int)num_boards_ &&
                                  readings_.size() > 0;
                        if (!ok) {
                            if (expecting_reply_to_forward_) {
                                const char* trim = line_buf_;
                                while (*trim == ' ') trim++;
                                bool looks_like_truncated_get = (line_len_ >= (LINE_BUF - 1)) && isdigit((unsigned char)*trim);
                                if (!looks_like_truncated_get) {
                                    THREADS_SERIAL.print("ir: ");
                                    THREADS_SERIAL.println(line_buf_);
                                    expecting_reply_to_forward_ = false;
                                    get_wait_start_ms_ = millis();
                                    line_len_ = 0;
                                    line_buf_[0] = '\0';
                                    continue;
                                }
                            }
                            if (line_buf_[0] != '>') {
                                THREADS_SERIAL.print("ir: ");
                                THREADS_SERIAL.println(line_buf_);
                            }
                            state_ = IRSenseState::FAILED;
                            failed_start_ms_ = millis();
                            THREADS_SERIAL.println("Error: IRsense invalid get reply at init.");
                            break;
                        }
                        for (size_t i = 0; i < readings_.size() / 2; i++) {
                            size_t j = readings_.size() - 1 - i;
                            uint16_t t = readings_[i];
                            readings_[i] = readings_[j];
                            readings_[j] = t;
                        }
                        expected_sensor_count_ = readings_.size();
                        THREADS_SERIAL.print("Found ");
                        THREADS_SERIAL.print((int)readings_.size());
                        THREADS_SERIAL.print(" sensors on ");
                        THREADS_SERIAL.print((int)num_boards_);
                        THREADS_SERIAL.println(" IR Sense boards");
                        new_frame_available_ = true;
                        state_ = IRSenseState::ACTIVE;
                        last_get_ms_ = millis();
                        break;
                    }
                    case IRSenseState::GET_PENDING: {
                        // Întâi încercăm să interpretăm linia ca răspuns la "get"; altfel (dacă așteptăm răspuns la ir)
                        // o tratăm ca răspuns la comanda manuală. Astfel, dacă placa răspunde la get înainte de răspunsul
                        // la ir, nu consumăm greșit răspunsul get ca reply la forward.
                        int num_boards = 0;
                        bool ok = parseGetReply(line_buf_, readings_, num_boards) &&
                                  num_boards == (int)num_boards_ &&
                                  (expected_sensor_count_ == 0 || readings_.size() == expected_sensor_count_);
                        if (!ok) {
                            // Nu e răspuns get valid: fie e răspunsul la comanda ir trimisă manual, fie eroare.
                            // Nu consumăm ca forwarded reply o linie care ar putea fi get truncat (lungă, începe cu cifră).
                            const char* trim = line_buf_;
                            while (*trim == ' ') trim++;
                            bool looks_like_truncated_get = (line_len_ >= (LINE_BUF - 1)) && isdigit((unsigned char)*trim);
                            if (expecting_reply_to_forward_ && !looks_like_truncated_get) {
                                THREADS_SERIAL.print("ir: ");
                                THREADS_SERIAL.println(line_buf_);
                                expecting_reply_to_forward_ = false;
                                get_wait_start_ms_ = millis();  // prelungește așteptarea pentru răspunsul get
                                line_len_ = 0;
                                line_buf_[0] = '\0';
                                continue;
                            }
                            if (line_buf_[0] != '>') {
                                THREADS_SERIAL.print("ir: ");
                                THREADS_SERIAL.println(line_buf_);
                            }
                            get_wrong_count_++;
                            THREADS_SERIAL.println("IRsense wrong answer");
                            state_ = (get_wrong_count_ >= 2) ? IRSenseState::FAILED : IRSenseState::ACTIVE;
                            if (state_ == IRSenseState::FAILED) failed_start_ms_ = millis();
                            break;
                        }
                        get_timeout_count_ = 0;
                        get_wrong_count_ = 0;
                        // Inversează întreaga listă: ce era ir1 devine irN, ce era irN devine ir1.
                        for (size_t i = 0; i < readings_.size() / 2; i++) {
                            size_t j = readings_.size() - 1 - i;
                            uint16_t t = readings_[i];
                            readings_[i] = readings_[j];
                            readings_[j] = t;
                        }
                        if (expected_sensor_count_ == 0)
                            expected_sensor_count_ = readings_.size();
                        new_frame_available_ = true;
                        state_ = IRSenseState::ACTIVE;
                        break;
                    }
                    case IRSenseState::IDLE:
                    case IRSenseState::ACTIVE:
                    case IRSenseState::FAILED:
                        // Ecou normal pe USB (fără linii Teleplot care încep cu ">").
                        if (line_buf_[0] != '>') {
                            THREADS_SERIAL.print("ir: ");
                            THREADS_SERIAL.println(line_buf_);
                            // Dacă așteptam răspuns la o comandă ir trimisă manual, aceasta e linia (o singură consumată).
                            expecting_reply_to_forward_ = false;
                        }
                        if (strcmp(line_buf_, "IRsense is alive.") == 0) {
                            if (state_ == IRSenseState::ACTIVE || state_ == IRSenseState::FAILED)
                                state_ = IRSenseState::IDLE;
                            if (!system_reset_pending_)
                                init();
                        }
                        break;
                }
            }
            line_len_ = 0;
            line_buf_[0] = '\0';
            continue;
        }
        if (line_len_ + 1 < LINE_BUF) {
            line_buf_[line_len_++] = (char)c;
            line_buf_[line_len_] = '\0';
        }
    }
}

/**
 * Interpretează răspunsul la „get”: „a,b; c,d; …” (plăci separate de „; ”, senzori pe aceeași placă de „,”).
 * Valorile 0–1000 sunt scrise în readings_out; num_boards_out = numărul de segmente.
 * @return false la format invalid sau zero plăci
 */
bool IRSense::parseGetReply(const char* line, IRFrame& readings_out, int& num_boards_out) {
    readings_out.clear();
    num_boards_out = 0;
    const char* p = line;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        const char* start = p;
        while (*p && !(p[0] == ';' && p[1] == ' ')) p++;
        const char* q = start;
        int count = 0;
        while (q < p) {
            while (q < p && (*q == ' ' || *q == ',')) q++;
            if (q >= p) break;
            if (!isdigit((unsigned char)*q)) return false;
            int val = atoi(q);
            if (val < 0 || val > 1000) return false;
            readings_out.push_back((float)val);
            count++;
            while (q < p && isdigit((unsigned char)*q)) q++;
        }
        if (count == 0) return false;
        // Inversează ordinea citirilor în cadrul plăcii: primul din răspuns devine ir2, al doilea ir1 etc.
        size_t seg_start = readings_out.size() - (size_t)count;
        for (size_t i = 0; i < (size_t)count / 2; i++) {
            float t = readings_out[seg_start + i];
            readings_out[seg_start + i] = readings_out[seg_start + (size_t)count - 1 - i];
            readings_out[seg_start + (size_t)count - 1 - i] = t;
        }
        num_boards_out++;
        if (*p) p += 2;
    }
    return num_boards_out >= 1;
}

/**
 * Trimite text pe UART (plus newline). În INIT_* / GET_PENDING mesajul utilizator intră în coadă (un slot).
 * @param from_user dacă e true, setează așteptare răspuns la forward; „reset” în FAILED folosește false
 */
void IRSense::forwardToUart(const char* arg, bool from_user) {
    const char* s = arg ? arg : "";
    if (from_user && (state_ == IRSenseState::INIT_ID || state_ == IRSenseState::INIT_GET || state_ == IRSenseState::GET_PENDING)) {
        if (pending_forward_[0] != '\0') {
            THREADS_SERIAL.println("err:busy");
            return;
        }
        size_t i = 0;
        while (i < (size_t)(LINE_BUF - 1) && s[i] != '\0') {
            pending_forward_[i] = s[i];
            i++;
        }
        pending_forward_[i] = '\0';
        return;
    }
    if (s[0] != '\0')
        IR_SENSE_UART.print(s);
    IR_SENSE_UART.print("\n");
    if (from_user)
        expecting_reply_to_forward_ = true;
}

/** Trimite coada pending_forward_ când UART nu e în INIT_* / GET_PENDING. */
void IRSense::flushPendingForward() {
    if (pending_forward_[0] == '\0')
        return;
    IR_SENSE_UART.print(pending_forward_);
    IR_SENSE_UART.print("\n");
    pending_forward_[0] = '\0';
    expecting_reply_to_forward_ = true;
}

/**
 * Pornește inițializarea: trimite „id”, stare INIT_ID; la răspuns valid urmează „get” și INIT_GET, apoi ACTIVE.
 */
void IRSense::init() {
    line_len_ = 0;
    line_buf_[0] = '\0';
    readings_.clear();
    IR_SENSE_UART.print("id\n");
    state_ = IRSenseState::INIT_ID;
    init_wait_start_ms_ = millis();
}

/**
 * Înainte de reset Arduino: trimite „reset” pe UART, trece în IDLE, setează system_reset_pending_
 * (nu se mai apelează init() până la reboot; evită mesaje „Found N sensors” în așteptare).
 */
void IRSense::resetForSystemReset() {
    forwardToUart("reset", false);
    state_ = IRSenseState::IDLE;
    boot_idle_start_ms_ = millis();
    line_len_ = 0;
    line_buf_[0] = '\0';
    readings_.clear();
    system_reset_pending_ = true;
}

/** Delegă comanda serială „ir” către singleton-ul ir_sense. */
void ir_forward_to_uart(const char* arg) {
    ir_sense.forwardToUart(arg ? arg : "");
}

// --- IRFrame: mască opțională pentru vizualizare (un bit per senzor, vezi interfața serială) ---

void IRFrame::setTeleplotBit(int one_based_index, bool on) {
    if (one_based_index < 1 || one_based_index > 32) return;
    uint32_t bit = 1u << (unsigned)(one_based_index - 1);
    if (on) teleplot_mask_ |= bit; else teleplot_mask_ &= ~bit;
}

bool IRFrame::getTeleplotBit(int one_based_index) const {
    if (one_based_index < 1 || one_based_index > 32) return false;
    return (teleplot_mask_ & (1u << (unsigned)(one_based_index - 1))) != 0;
}

void IRFrame::setTeleplotAll(size_t n_sensors, bool on) {
    if (n_sensors > 32u) n_sensors = 32u;
    if (n_sensors == 0) { if (!on) teleplot_mask_ = 0; return; }
    uint32_t m = (1u << n_sensors) - 1u;
    if (on) teleplot_mask_ |= m; else teleplot_mask_ &= ~m;
}

bool IRFrame::getTeleplotAll(size_t n_sensors) const {
    if (n_sensors > 32u) n_sensors = 32u;
    if (n_sensors == 0) return false;
    uint32_t m = (1u << n_sensors) - 1u;
    return (teleplot_mask_ & m) == m;
}
