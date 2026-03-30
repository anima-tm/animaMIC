/*
 * ServoAct.cpp
 *
 * animaMIC — Implementare ServoAct: listă înlănțuită de actuatoare, automat go_to pe i2c.write/read,
 * activare (START), recuperare (RESET + sondare), comenzi serial „act”.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "ServoAct.h"
#include "anima_tools.h"
#include "../commands.h"
#include <string.h>

ServoAct* ServoAct::first_ = nullptr;

/** Interval thread 0; inițializează servomotoare, goto_data_, înregistrare automată nu (asta e în begin). */
ServoAct::ServoAct(const char* name, uint8_t i2c_address)
    : Thread(0),
      i2c_address_(i2c_address), next_(nullptr),
      num_servos_(0), last_servo_(0),
      trans_id_(0), next_refresh_ms_(0), online_(false), consecutive_failures_(0), plot_enabled_(false),
      activating_(false), activation_start_ms_(0), start_trans_id_(0), next_start_read_ms_(0), saw_initializing_(false),
      wait_for_status_after_activation_(false),
      last_activation_failure_ms_(0), previous_activation_failure_ms_(0), activation_watchdog_triggered_(false),
      recovering_(false), recovery_state_(RecoveryState::RECOVERY_IDLE), recovery_start_ms_(0), recovery_next_probe_ms_(0), recovery_probe_trans_id_(0),
      last_set_valid_(false) {
    /* Nume scurt pentru serial și Teleplot */
    name_[0] = '\0';
    if (name != nullptr) {
        for (int i = 0; i < 4 && name[i] != '\0'; i++) {
            name_[i] = name[i];
        }
        name_[4] = '\0';
    }

    /* ServoData implicit */
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        servos_[i].active = false;
        servos_[i].error = false;
        servos_[i].position = 0.0f;
    }

    /* Stare inițială goto */
    goto_data_.state = GotoState::IDLE;
    goto_data_.pos_trans_ids[0] = 0;
    goto_data_.pos_trans_ids[1] = 0;
    goto_data_.pos_trans_count = 0;
    goto_data_.pos_data_len[0] = 0;
    goto_data_.pos_data_len[1] = 0;
    goto_data_.pos_reg_addr[0] = 0;
    goto_data_.pos_reg_addr[1] = 0;
    goto_data_.goto_trans_id = 0;
    goto_data_.start_time_ms = 0;
    goto_data_.time_ms = 0;
    goto_data_.queued_status_printed_ = false;
    goto_data_.sequence_started_ = false;
    goto_data_.delayed_queue_.clear();
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        goto_data_.goto_positions[i] = X;
        last_set_position_[i] = 0.0f;
    }
    /* Buffer citire STATUS+POS */
    for (int i = 0; i < sizeof(read_buffer_); i++) {
        read_buffer_[i] = 0;
    }
    start_buffer_[0] = 0;
}

ServoAct::~ServoAct() {
    end();
}

/**
 * Înregistrare în lista first_; sondă I2C sincronă (WIRE) pentru STATUS; număr servomotoare, online_.
 * Apel tipic din setup; refresh ulterior prin coada i2c în loop.
 */
void ServoAct::begin() {
    next_ = first_;
    first_ = this;
    
    // Folosim acces direct la WIRE (nu prin coada I2C, pentru ca suntem in setup())
    WIRE.beginTransmission(i2c_address_);
    WIRE.write(I2C_REG_STATUS);
    uint8_t error = WIRE.endTransmission(false);  // false = repeated start (nu trimite STOP)
    
    if (error != 0) {
        // Recupereaza bus-ul: trimite STOP pentru a elibera bus-ul
        WIRE.beginTransmission(i2c_address_);
        WIRE.endTransmission(true);  // Forțează STOP pentru a elibera bus-ul
        Serial.print("Cannot connect to actuator '");
        Serial.print(name_);
        Serial.print("' at address 0x");
        Serial.print(i2c_address_, HEX);
        Serial.print(": I2C error ");
        Serial.println(error);
        return;
    }
    
    // Citeste 2 octeti (LSB si MSB) cu repeated start
    uint8_t bytesRead = WIRE.requestFrom(i2c_address_, 2, true);  // true = trimite STOP
    
    if (bytesRead != 2) {
        Serial.print("Cannot read from actuator '");
        Serial.print(name_);
        Serial.print("' at address 0x");
        Serial.print(i2c_address_, HEX);
        Serial.print(": read ");
        Serial.print(bytesRead);
        Serial.println(" bytes (expected 2)");
        return;
    }
    
    uint8_t status_lsb = WIRE.read();
    uint8_t status_msb = WIRE.read();
    
    // Analizează STATUS pentru fiecare servomotor și identifică servomotoarele instalate
    // STATUS bits: INST (bit 0), ACT (bit 1), MOV (bit 2), ERR (bit 3)
    num_servos_ = 0;
    last_servo_ = 0;
    uint8_t num_active_servos = 0;
    
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        uint8_t servo_status;
        if (i < 2) {
            // Servo 1-2: din LSB
            servo_status = (status_lsb >> (i * 4)) & 0x0F;
        } else {
            // Servo 3-4: din MSB
            servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
        }
        
        // Verifica daca servo-ul este instalat (INST bit = bit 0)
        bool is_installed = (servo_status & 0x01) != 0;
        
        if (is_installed) {
            num_servos_++;
            last_servo_ = i;
            
            // Actualizeaza datele servomotoarelor
            servos_[i].active = (servo_status & 0x02) != 0;  // ACT bit = bit 1
            servos_[i].error = (servo_status & 0x08) != 0;   // ERR bit = bit 3
            
            if (servos_[i].active) {
                num_active_servos++;
            }
        } else {
            // Servo neinstalat, reseteaza datele
            servos_[i].active = false;
            servos_[i].error = false;
            servos_[i].position = 0.0f;
        }
    }
    
    // Afiseaza statusul actuatorului
    Serial.print("Actuator '");
    Serial.print(name_);
    Serial.print("' is online at address 0x");
    Serial.print(i2c_address_, HEX);
    Serial.print(": ");
    Serial.print(num_servos_);
    Serial.print(" servos, ");
    Serial.print(num_active_servos);
    Serial.println(" active");
    
    // Marcheaza actuatorul ca fiind online
    online_ = true;
    consecutive_failures_ = 0;  // Reseteaza contorul de esecuri

    // Pastreaza STATUS in read_buffer_ astfel incat go_to() sa aiba date valide inainte de primul refresh din loop()
    read_buffer_[0] = status_lsb;
    read_buffer_[1] = status_msb;

    // Programeaza reimprospatarea imediat
    next_refresh_ms_ = millis();
}

void ServoAct::end() {
    /* Eliminare din lista înlănțuită */
    if (first_ == this) {
        first_ = next_;
    } else {
        ServoAct* current = first_;
        while (current != nullptr && current->next_ != this) {
            current = current->next_;
        }
        if (current != nullptr) {
            current->next_ = next_;
        }
    }
    next_ = nullptr;
    online_ = false;
    activating_ = false;
    activation_start_ms_ = 0;
    start_trans_id_ = 0;
    goto_data_.state = GotoState::IDLE;
    goto_data_.pos_trans_ids[0] = 0;
    goto_data_.pos_trans_ids[1] = 0;
    goto_data_.pos_trans_count = 0;
    goto_data_.pos_data_len[0] = 0;
    goto_data_.pos_data_len[1] = 0;
    goto_data_.pos_reg_addr[0] = 0;
    goto_data_.pos_reg_addr[1] = 0;
    goto_data_.goto_trans_id = 0;
    while (!goto_data_.queue_.empty()) {
        goto_data_.queue_.pop();
    }
    goto_data_.delayed_queue_.clear();
    recovering_ = false;
    recovery_state_ = RecoveryState::RECOVERY_IDLE;
    recovery_probe_trans_id_ = 0;
}

/** Scriere START 0xFF; urmărește registrul START până la stabilire + STATUS. */
void ServoAct::activate() {
    if (!online_)
        return;
    
    if (activating_) {
        Serial.print("Actuator ");
        Serial.print(name_);
        Serial.println(" is already activating");
        return;
    }
    
    // Scrie in registrul START pentru a porni initializarea
    uint8_t start_value = 0xFF;
    uint16_t write_id = i2c.write(i2c_address_, I2C_REG_START, &start_value, 1);
    
    if (write_id == 0) {
        Serial.print("Cannot activate actuator '");
        Serial.print(name_);
        Serial.println("': failed to queue START write");
        return;
    }
    
    // Porneste monitorizarea initializarii (nu asteptam finalizarea scrierii, va fi procesata de I2C thread)
    activating_ = true;
    activation_start_ms_ = millis();
    saw_initializing_ = false;
    start_trans_id_ = 0;
    next_start_read_ms_ = millis();  // Citeste imediat
    
    Serial.print("Actuator '");
    Serial.print(name_);
    Serial.println("' activation started");
}

/** Scriere RESET 0xFF pe placă. */
void ServoAct::reset() {
    if (!online_)
        return;
    
    // Scrie in registrul RESET pentru a reseta actuatorul
    uint8_t reset_value = 0xFF;
    uint16_t write_id = i2c.write(i2c_address_, I2C_REG_RESET, &reset_value, 1);
    
    if (write_id == 0) {
        Serial.print("Cannot reset actuator '");
        Serial.print(name_);
        Serial.println("': failed to queue RESET write");
        return;
    }
}

/** Returnează true o dată dacă flag-ul watchdog era setat, apoi îl șterge. */
bool ServoAct::getAndClearActivationWatchdogTrigger() {
    if (!activation_watchdog_triggered_)
        return false;
    activation_watchdog_triggered_ = false;
    return true;
}

/** Două eșecuri la interval scurt → activation_watchdog_triggered_. */
void ServoAct::recordActivationFailureAndCheckWatchdog() {
    previous_activation_failure_ms_ = last_activation_failure_ms_;
    last_activation_failure_ms_ = millis();
    if (previous_activation_failure_ms_ != 0 &&
        (last_activation_failure_ms_ - previous_activation_failure_ms_) <= (uint32_t)SERVO_ACTIVATION_WATCHDOG_TIME)
        activation_watchdog_triggered_ = true;
}

/** Golește queue și delayed_queue; stare IDLE. */
void ServoAct::clearGotoQueues() {
    while (!goto_data_.queue_.empty()) {
        goto_data_.queue_.pop();
    }
    goto_data_.delayed_queue_.clear();
    goto_data_.state = GotoState::IDLE;
}

bool ServoAct::allServosActive() const {
    for (uint8_t i = 0; i < num_servos_; i++) {
        if (!servos_[i].active)
            return false;
    }
    return true;
}

bool ServoAct::isGotoSequenceComplete() const {
    return goto_data_.state == GotoState::IDLE &&
           goto_data_.queue_.empty() &&
           goto_data_.delayed_queue_.empty();
}

float ServoAct::getPosition(uint8_t servo_index) const {
    if (servo_index >= num_servos_)
        return 0.0f;
    return servos_[servo_index].position;
}

float ServoAct::getSetPosition(uint8_t servo_index) const {
    if (servo_index >= num_servos_)
        return 0.0f;
    if (!last_set_valid_)
        return servos_[servo_index].position;
    float v = last_set_position_[servo_index];
    if (v < -2.0f || v > 2.0f)  // X sau invalid
        return servos_[servo_index].position;
    return v;
}

/** Mapare [-2,2] → int16 pentru registrele POS_SET (fără coliziune cu sentinel). */
static int16_t floatToInt16(float pos) {
    if (pos < -2.0f) pos = -2.0f;
    if (pos > 2.0f) pos = 2.0f;
    
    // Converteste: -2.0 la +2.0 -> -32768 la 32766
    // Factor de scalare: 16383.0 (32766 / 2.0)
    int32_t val = (int32_t)(pos * 16383.0f);
    
    // Clamp la int16_t range (max 32766, nu 32767 pentru a evita sentinel)
    if (val < -32768) val = -32768;
    if (val > 32766) val = 32766;
    
    return (int16_t)val;
}

bool ServoAct::enqueueGotoCoordinates(const GotoCoordinates& coords) {
    /* Limită MAX_GOTO_QUEUE_SIZE */
    if (goto_data_.queue_.size() >= MAX_GOTO_QUEUE_SIZE) {
        return false;
    }
    
    goto_data_.queue_.push(coords);
    return true;
}

/** Listează doar canalele ≠ X, separate prin virgulă. */
void ServoAct::formatGotoPositions(float positions[MAX_SERVOS_PER_ACTUATOR], char* buffer, size_t buffer_size) {
    buffer[0] = '\0';
    bool first = true;
    for (int i = 0; i < num_servos_; i++) {
        if (positions[i] != X) {
            if (!first) {
                strncat(buffer, ", ", buffer_size - strlen(buffer) - 1);
            }
            char pos_str[16];
            snprintf(pos_str, sizeof(pos_str), "%.3f", positions[i]);
            strncat(buffer, pos_str, buffer_size - strlen(buffer) - 1);
            first = false;
        }
    }
}

void ServoAct::printGotoStatus(const char* status, bool final) {
    char pos_str[64];
    formatGotoPositions(goto_data_.goto_positions, pos_str, sizeof(pos_str));
    Serial.print("  ");
    Serial.print(status);
    if (strcmp(status, "completed") == 0) {
        Serial.print(": goto [");
        Serial.print(pos_str);
        Serial.print("] in ");
        Serial.print(goto_data_.time_ms);
        Serial.print("ms completed");
    }
    Serial.println();
}

void ServoAct::printGotoStatusWithPositions(float positions[MAX_SERVOS_PER_ACTUATOR], uint16_t time_ms, const char* status, bool final) {
    char pos_str[64];
    formatGotoPositions(positions, pos_str, sizeof(pos_str));
    Serial.print("  ");
    Serial.print(status);
    if (strcmp(status, "queued") == 0) {
        Serial.print(": goto [");
        Serial.print(pos_str);
        Serial.print("] in ");
        Serial.print(time_ms);
        Serial.print("ms");
    }
    Serial.println();
}

void ServoAct::filterQueueByIncomingPositions(const float positions[MAX_SERVOS_PER_ACTUATOR]) {
    std::queue<GotoCoordinates> temp;
    while (!goto_data_.queue_.empty()) {
        temp.push(goto_data_.queue_.front());
        goto_data_.queue_.pop();
    }
    while (!temp.empty()) {
        GotoCoordinates coords = temp.front();
        temp.pop();
        for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
            if (positions[i] != X) {
                coords.positions[i] = X;
            }
        }
        bool all_same = true;
        for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
            if (coords.positions[i] != X) {
                all_same = false;
                break;
            }
        }
        if (!all_same) {
            goto_data_.queue_.push(coords);
        }
    }
}

/** Pop front; dacă delay_after_previous_ms, push în delayed; altfel then_go_to_impl. */
void ServoAct::executeNextQueuedGoto() {
    if (goto_data_.queue_.empty()) {
        return;  // Coada goala
    }
    
    GotoCoordinates coords = goto_data_.queue_.front();
    goto_data_.queue_.pop();
    
    if (coords.delay_after_previous_ms > 0) {
        pushDelayedGoto(millis() + coords.delay_after_previous_ms, coords);
        return;
    }
    then_go_to_impl(coords.positions, coords.time_ms, 0);
}

void ServoAct::pushDelayedGoto(uint32_t trigger_at_ms, const GotoCoordinates& coords) {
    DelayedGoto d;
    d.trigger_at_ms = trigger_at_ms;
    d.coords = coords;
    std::deque<DelayedGoto>& q = goto_data_.delayed_queue_;
    size_t i = 0;
    for (; i < q.size(); i++) {
        if (q[i].trigger_at_ms > trigger_at_ms)
            break;
    }
    q.insert(q.begin() + (ptrdiff_t)i, d);
}

/** Anulează tranzacții I2C pentru adresă; pornește automat RecoveryState. */
void ServoAct::startRecovery() {
    while (!goto_data_.queue_.empty()) {
        goto_data_.queue_.pop();
    }
    i2c.cancelTransactionsForAddress(i2c_address_);
    
    online_ = false;
    Serial.print("Actuator '");
    Serial.print(name_);
    Serial.print("' @0x");
    Serial.print(i2c_address_, HEX);
    Serial.println(" went offline (error/timeout recovery)");
    
    goto_data_.state = GotoState::IDLE;
    goto_data_.sequence_started_ = false;
    goto_data_.delayed_queue_.clear();
    goto_data_.pos_trans_ids[0] = 0;
    goto_data_.pos_trans_ids[1] = 0;
    goto_data_.pos_trans_count = 0;
    goto_data_.goto_trans_id = 0;
    trans_id_ = 0;
    next_refresh_ms_ = 0;
    start_trans_id_ = 0;
    saw_initializing_ = false;
    next_start_read_ms_ = 0;
    activating_ = false;
    wait_for_status_after_activation_ = false;
    
    recovery_state_ = RecoveryState::RECOVERY_RESET;
    recovery_start_ms_ = millis();
    recovery_next_probe_ms_ = 0;
    recovery_probe_trans_id_ = 0;
    recovering_ = true;
}

/** Pas cu pas: RESET → sondare STATUS → settle → activate → așteptare (partajat logic cu loop la activare). */
void ServoAct::processRecovery() {
    switch (recovery_state_) {
        case RecoveryState::RECOVERY_IDLE:
            recovering_ = false;
            return;
            
        case RecoveryState::RECOVERY_RESET: {
            static const uint8_t reset_value = 0xFF;
            uint16_t wid = i2c.write(i2c_address_, I2C_REG_RESET, &reset_value, 1);
            if (wid != 0) {
                recovery_state_ = RecoveryState::RECOVERY_WAIT_ONLINE;
                recovery_next_probe_ms_ = millis() + ACTUATOR_REFRESH_TIME;
            }
            break;
        }
            
        case RecoveryState::RECOVERY_WAIT_ONLINE: {
            if (recovery_probe_trans_id_ != 0) {
                I2CTransactionState state = i2c.getState(recovery_probe_trans_id_);
                if (state == I2CTransactionState::SUCCESS) {
                    recovery_probe_trans_id_ = 0;
                    online_ = true;
                    updateServoDataFromBuffer();
                    Serial.print("Actuator '");
                    Serial.print(name_);
                    Serial.print("' @0x");
                    Serial.print(i2c_address_, HEX);
                    Serial.println(" is back online");
                    recovery_state_ = RecoveryState::RECOVERY_SETTLE;
                    recovery_next_probe_ms_ = millis() + 100;  // 100 ms stabilizare inainte de activare
                } else if (state == I2CTransactionState::FAILED) {
                    recovery_probe_trans_id_ = 0;
                    recovery_next_probe_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                }
                return;
            }
            if (millis() - recovery_start_ms_ >= SERVO_ACTIVATION_TIMEOUT) {
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("' @0x");
                Serial.print(i2c_address_, HEX);
                Serial.println(" did not come back online within timeout");
                recovery_state_ = RecoveryState::RECOVERY_IDLE;
                recovering_ = false;
                return;
            }
            if (millis() >= recovery_next_probe_ms_) {
                uint8_t bytes_to_read = 2;
                if (num_servos_ > 0) {
                    bytes_to_read += (last_servo_ + 1) * 2;
                }
                recovery_probe_trans_id_ = i2c.read(i2c_address_, I2C_REG_STATUS, read_buffer_, bytes_to_read);
                if (recovery_probe_trans_id_ != 0) {
                    recovery_next_probe_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                }
            }
            break;
        }
            
        case RecoveryState::RECOVERY_SETTLE:
            if (millis() >= recovery_next_probe_ms_) {
                recovery_state_ = RecoveryState::RECOVERY_ACTIVATE;
            }
            break;
            
        case RecoveryState::RECOVERY_ACTIVATE:
            activate();
            recovery_state_ = RecoveryState::RECOVERY_ACTIVATE_WAIT;
            break;
            
        case RecoveryState::RECOVERY_ACTIVATE_WAIT: {
            if (!activating_) {
                recovering_ = false;
                recovery_state_ = RecoveryState::RECOVERY_IDLE;
                break;
            }
            // Ruleaza automatul de activare (citiri START, apoi o citire STATUS cand wait_for_status_after_activation_)
            if (millis() - activation_start_ms_ >= SERVO_ACTIVATION_TIMEOUT) {
                activating_ = false;
                start_trans_id_ = 0;
                wait_for_status_after_activation_ = false;
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("' activation timeout after ");
                Serial.print(SERVO_ACTIVATION_TIMEOUT);
                Serial.println(" ms");
                if (online_) {
                    Serial.print("Resetting actuator '");
                    Serial.print(name_);
                    Serial.println("'...");
                    reset();
                    recordActivationFailureAndCheckWatchdog();
                    recovery_state_ = RecoveryState::RECOVERY_WAIT_ONLINE;
                    recovery_next_probe_ms_ = millis() + 500;  // Dă dispozitivului timp să se reseteze
                } else {
                    recovering_ = false;
                    recovery_state_ = RecoveryState::RECOVERY_IDLE;
                }
                break;
            }
            if (start_trans_id_ != 0) {
                I2CTransactionState state = i2c.getState(start_trans_id_);
                if (state == I2CTransactionState::SUCCESS) {
                    uint8_t start_value = start_buffer_[0];
                    if (start_value != 0) {
                        saw_initializing_ = true;
                        start_trans_id_ = 0;
                        next_start_read_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                    } else if (saw_initializing_) {
                        wait_for_status_after_activation_ = true;
                        start_trans_id_ = 0;
                    } else {
                        // START deja 0 (dispozitivul poate fi terminat înainte să vedem non-zero): confirmă cu citire STATUS
                        wait_for_status_after_activation_ = true;
                        start_trans_id_ = 0;
                    }
                } else if (state == I2CTransactionState::FAILED) {
                    start_trans_id_ = 0;
                    next_start_read_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                } else if (state == I2CTransactionState::UNKNOWN) {
                    // Rezultat pierdut (ex. ID eliminat din lista de completare); reîncearcă
                    start_trans_id_ = 0;
                    next_start_read_ms_ = millis();
                }
            } else if (wait_for_status_after_activation_ && trans_id_ != 0) {
                I2CTransactionState state = i2c.getState(trans_id_);
                if (state == I2CTransactionState::SUCCESS) {
                    updateServoDataFromBuffer();
                    wait_for_status_after_activation_ = false;
                    activating_ = false;
                    trans_id_ = 0;
                    next_refresh_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                    last_activation_failure_ms_ = 0;
                    previous_activation_failure_ms_ = 0;
                    uint8_t active_count = 0;
                    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                        if (servos_[i].active) active_count++;
                    }
                    Serial.print("Actuator '");
                    Serial.print(name_);
                    Serial.print("' activated ");
                    Serial.print(active_count);
                    Serial.print(" out of ");
                    Serial.print(num_servos_);
                    Serial.println(" servos");
                    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                        if (servos_[i].error) {
                            Serial.print("  -Servo ");
                            Serial.print(i + 1);
                            Serial.print(" of actuator '");
                            Serial.print(name_);
                            Serial.println("' is in ERROR state");
                        }
                    }
                    recovering_ = false;
                    recovery_state_ = RecoveryState::RECOVERY_IDLE;
                } else if (state == I2CTransactionState::FAILED) {
                    trans_id_ = 0;
                    next_refresh_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                } else if (state == I2CTransactionState::UNKNOWN) {
                    trans_id_ = 0;
                    next_refresh_ms_ = millis();
                }
            } else if (wait_for_status_after_activation_ && trans_id_ == 0) {
                uint8_t bytes_to_read = 2;
                if (num_servos_ > 0) {
                    bytes_to_read += (last_servo_ + 1) * 2;
                }
                trans_id_ = i2c.read(i2c_address_, I2C_REG_STATUS, read_buffer_, bytes_to_read);
                if (trans_id_ == 0) {
                    next_refresh_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                }
            } else if (millis() >= next_start_read_ms_) {
                start_trans_id_ = i2c.read(i2c_address_, I2C_REG_START, start_buffer_, 1);
                if (start_trans_id_ == 0) {
                    next_start_read_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                }
            }
            break;
        }
    }
}

/**
 * Nucleu then_go_to: coadă, delayed, filtrare sau start imediat; vezi go_to_impl pentru trimitere I2C.
 */
void ServoAct::then_go_to_impl(float positions[MAX_SERVOS_PER_ACTUATOR], uint16_t time_ms, uint16_t delay_after_previous_ms, bool executing_delayed) {
    if (recovering_)
        return;
    if (!online_)
        return;
    
    // IDLE + delay: programează această mișcare să înceapă după N ms de acum
    if (goto_data_.state == GotoState::IDLE && delay_after_previous_ms > 0) {
        GotoCoordinates coords;
        for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++)
            coords.positions[i] = positions[i];
        coords.time_ms = time_ms;
        coords.delay_after_previous_ms = 0;
        pushDelayedGoto(millis() + delay_after_previous_ms, coords);
        return;
    }
    
    // IDLE + fără delay dar delayed_queue_ nu e goală: "then goto" rulează după ce ultima mișcare programată se termină (sari când executăm un delayed goto scos din coadă)
    if (goto_data_.state == GotoState::IDLE && delay_after_previous_ms == 0 && !executing_delayed && !goto_data_.delayed_queue_.empty()) {
        GotoCoordinates coords;
        for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++)
            coords.positions[i] = positions[i];
        coords.time_ms = time_ms;
        coords.delay_after_previous_ms = 0;
        const DelayedGoto& last = goto_data_.delayed_queue_.back();
        uint32_t trigger = last.trigger_at_ms + last.coords.time_ms;
        pushDelayedGoto(trigger, coords);
        return;
    }
    
    // Daca un go_to este deja in curs, adauga in coada (doar daca suntem inca online)
    if (goto_data_.state != GotoState::IDLE) {
        if (!online_)
            return;
        GotoCoordinates coords;
        for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
            coords.positions[i] = positions[i];
        }
        coords.time_ms = time_ms;
        coords.delay_after_previous_ms = delay_after_previous_ms;
        if (!enqueueGotoCoordinates(coords)) {
            Serial.print("Actuator '");
            Serial.print(name_);
            Serial.println("': goto queue full, dropping coordinates");
            return;
        }
        // Afiseaza status 'in coada' cu pozitiile puse in coada
        // printGotoStatusWithPositions(positions, time_ms, "queued", true);
        return;
    }
    
    // Daca nu este in curs, incepe executia imediat
    if (!goto_data_.sequence_started_) {
        goto_data_.sequence_started_ = true;
    }
    
    // Ignora pozitiile pentru servomotoare care nu exista
    // Verifica care servomotoare trebuie sa se miste (nu sunt X)
    bool needs_move[MAX_SERVOS_PER_ACTUATOR] = {false};
    uint8_t num_need_move = 0;
    
    for (int i = 0; i < num_servos_; i++) {
        if (positions[i] != X) {
            needs_move[i] = true;
            num_need_move++;
        }
    }
    
    if (num_need_move == 0) {
        Serial.print("Actuator '");
        Serial.print(name_);
        Serial.println("': no servos need to move (all X)");
        return;
    }
    
    // Pas 0: Verifica ca toate servomotoarele care trebuie sa se miste sunt active si fara erori
    // Folosim read_buffer_ pentru STATUS (trebuie sa fie actualizat recent)
    uint8_t status_lsb = read_buffer_[0];
    uint8_t status_msb = read_buffer_[1];
    
    for (int i = 0; i < num_servos_; i++) {
        if (needs_move[i]) {
            uint8_t servo_status;
            if (i < 2) {
                servo_status = (status_lsb >> (i * 4)) & 0x0F;
            } else {
                servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
            }
            
            bool is_installed = (servo_status & 0x01) != 0;
            bool is_active = (servo_status & 0x02) != 0;  // ACT bit
            bool has_error = (servo_status & 0x08) != 0;   // ERR bit
            
            if (!is_installed) {
                Serial.print("ERROR: Actuator '");
                Serial.print(name_);
                Serial.print("': servo ");
                Serial.print(i + 1);
                Serial.println(" is not installed");
                return;
            }
            
            if (!is_active) {
                Serial.print("ERROR: Actuator '");
                Serial.print(name_);
                Serial.print("': servo ");
                Serial.print(i + 1);
                Serial.println(" is not active - cannot execute go_to");
                if (!activating_) {
                    activate();
                }
                return;
            }
            
            if (has_error) {
                Serial.print("ERROR: Actuator '");
                Serial.print(name_);
                Serial.print("': servo ");
                Serial.print(i + 1);
                Serial.println(" is in error state - cannot execute go_to");
                return;
            }
        }
    }
    
    // Stocheaza timpul si pozitiile pentru afisare
    goto_data_.time_ms = time_ms;
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        goto_data_.goto_positions[i] = positions[i];
    }
    
    // Pas 1: Scrie POS_SET registrele care trebuie schimbate
    // Grupeaza scrierile consecutiv pentru eficienta
    goto_data_.pos_trans_count = 0;
    
    int i = 0;
    while (i < num_servos_ && goto_data_.pos_trans_count < 2) {
        if (needs_move[i]) {
            // Gaseste urmatoarea secventa consecutiva de servomotoare care trebuie mutate
            int start_idx = i;
            int count = 1;
            
            // Verifica cati servomotoare consecutivi trebuie mutati
            while (i + count < num_servos_ && needs_move[i + count] && count < MAX_SERVOS_PER_ACTUATOR) {
                count++;
            }
            
            // Pregateste datele pentru scriere in buffer persistent
            uint8_t trans_idx = goto_data_.pos_trans_count;
            int data_idx = 0;
            for (int j = start_idx; j < start_idx + count; j++) {
                int16_t pos_int = floatToInt16(positions[j]);
                goto_data_.pos_data[trans_idx][data_idx++] = pos_int & 0xFF;        // LSB
                goto_data_.pos_data[trans_idx][data_idx++] = (pos_int >> 8) & 0xFF; // MSB
            }
            goto_data_.pos_data_len[trans_idx] = count * 2;
            goto_data_.pos_reg_addr[trans_idx] = I2C_REG_POS_SET1 + (start_idx * 2);
            
            // Scrie tranzactia folosind buffer-ul persistent
            uint16_t trans_id = i2c.write(i2c_address_, goto_data_.pos_reg_addr[trans_idx], 
                                          goto_data_.pos_data[trans_idx], goto_data_.pos_data_len[trans_idx]);
            
            if (trans_id == 0) {
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.println("': failed to queue POS_SET write");
                goto_data_.state = GotoState::IDLE;
                return;
            }
            
            goto_data_.pos_trans_ids[goto_data_.pos_trans_count] = trans_id;
            goto_data_.pos_trans_count++;
            
            i += count;
        } else {
            i++;
        }
    }
    
    // Porneste automatul de stari
    goto_data_.state = GotoState::WAITING_POS;
    goto_data_.start_time_ms = millis();
}

void ServoAct::delay_next_goto(uint16_t delay_ms) {
    next_goto_delay_ms_ = delay_ms;
}

/**
 * Filtrare coadă după pozițiile noi; opțional delayed start dacă delay_next_goto a setat ms; altfel then_go_to_impl.
 */
void ServoAct::go_to_impl(const float positions[MAX_SERVOS_PER_ACTUATOR], uint16_t time_ms) {
    filterQueueByIncomingPositions(positions);
    uint16_t delay_ms = next_goto_delay_ms_;
    next_goto_delay_ms_ = 0;
    if (delay_ms > 0) {
        GotoCoordinates coords;
        for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++)
            coords.positions[i] = positions[i];
        coords.time_ms = time_ms;
        coords.delay_after_previous_ms = 0;
        pushDelayedGoto(millis() + delay_ms, coords);
        return;
    }
    float pos_copy[MAX_SERVOS_PER_ACTUATOR];
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) pos_copy[i] = positions[i];
    then_go_to_impl(pos_copy, time_ms, 0);
}

/* Construiesc vectorul de poziții (rest X) și deleg go_to_impl. */

void ServoAct::go_to(float pos1, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    for (int i = 1; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    go_to_impl(positions, time_ms);
}

void ServoAct::go_to(float pos1, float pos2, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    positions[1] = pos2;
    for (int i = 2; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    go_to_impl(positions, time_ms);
}

void ServoAct::go_to(float pos1, float pos2, float pos3, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    positions[1] = pos2;
    if (MAX_SERVOS_PER_ACTUATOR >= 3) positions[2] = pos3;
    for (int i = 3; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    go_to_impl(positions, time_ms);
}

void ServoAct::go_to(float pos1, float pos2, float pos3, float pos4, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    positions[1] = pos2;
    if (MAX_SERVOS_PER_ACTUATOR >= 3) positions[2] = pos3;
    if (MAX_SERVOS_PER_ACTUATOR >= 4) positions[3] = pos4;
    go_to_impl(positions, time_ms);
}

void ServoAct::then_go_to(float pos1, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    for (int i = 1; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    uint16_t d = next_goto_delay_ms_;
    next_goto_delay_ms_ = 0;
    then_go_to_impl(positions, time_ms, d);
}

void ServoAct::then_go_to(float pos1, float pos2, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    positions[1] = pos2;
    for (int i = 2; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    uint16_t d = next_goto_delay_ms_;
    next_goto_delay_ms_ = 0;
    then_go_to_impl(positions, time_ms, d);
}

void ServoAct::then_go_to(float pos1, float pos2, float pos3, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    positions[1] = pos2;
    if (MAX_SERVOS_PER_ACTUATOR >= 3) positions[2] = pos3;
    for (int i = 3; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    uint16_t d = next_goto_delay_ms_;
    next_goto_delay_ms_ = 0;
    then_go_to_impl(positions, time_ms, d);
}

void ServoAct::then_go_to(float pos1, float pos2, float pos3, float pos4, uint16_t time_ms) {
    float positions[MAX_SERVOS_PER_ACTUATOR];
    positions[0] = pos1;
    positions[1] = pos2;
    if (MAX_SERVOS_PER_ACTUATOR >= 3) positions[2] = pos3;
    if (MAX_SERVOS_PER_ACTUATOR >= 4) positions[3] = pos4;
    uint16_t d = next_goto_delay_ms_;
    next_goto_delay_ms_ = 0;
    then_go_to_impl(positions, time_ms, d);
}

/**
 * Avansare tranzacții POS_SET → GOTO → așteptare durată minimă / MOV clar / timeout; apoi următorul din coadă.
 */
void ServoAct::processGotoStateMachine() {
    switch (goto_data_.state) {
        case GotoState::WAITING_POS: {
            // Verifica starea tuturor tranzactiilor POS_SET
            bool all_success = true;
            bool any_failed = false;
            
            for (uint8_t i = 0; i < goto_data_.pos_trans_count; i++) {
                if (goto_data_.pos_trans_ids[i] != 0) {
                    I2CTransactionState state = i2c.getState(goto_data_.pos_trans_ids[i]);
                    if (state == I2CTransactionState::FAILED) {
                        any_failed = true;
                        break;
                    } else if (state != I2CTransactionState::SUCCESS) {
                        all_success = false;
                    }
                }
            }
            
            if (any_failed) {
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.println("': POS_SET write failed, aborting go_to");
                goto_data_.state = GotoState::ERROR;
                goto_data_.pos_trans_ids[0] = 0;
                goto_data_.pos_trans_ids[1] = 0;
                goto_data_.pos_trans_count = 0;
                goto_data_.pos_data_len[0] = 0;
                goto_data_.pos_data_len[1] = 0;
                startRecovery();
            } else if (all_success) {
                // Toate tranzactiile POS_SET au reusit, scrie GOTO
                // Pregateste datele in buffer persistent
                goto_data_.goto_data_bytes[0] = goto_data_.time_ms & 0xFF;        // LSB
                goto_data_.goto_data_bytes[1] = (goto_data_.time_ms >> 8) & 0xFF; // MSB
                
                goto_data_.goto_trans_id = i2c.write(i2c_address_, I2C_REG_GOTO, goto_data_.goto_data_bytes, 2);
#ifdef DIAGNOSE_GOTO
                {
                    char pos_str[64];
                    formatGotoPositions(goto_data_.goto_positions, pos_str, sizeof(pos_str));
                    Serial.print("Actuator '");
                    Serial.print(name_);
                    Serial.print("': goto sent at ");
                    Serial.print(millis());
                    Serial.print(" [");
                    Serial.print(pos_str);
                    Serial.print("] in ");
                    Serial.print(goto_data_.time_ms);
                    Serial.println("ms");
                }
#endif
                if (goto_data_.goto_trans_id == 0) {
                    Serial.println("  ERROR: failed to queue GOTO");
                    goto_data_.state = GotoState::ERROR;
                    goto_data_.pos_trans_ids[0] = 0;
                    goto_data_.pos_trans_ids[1] = 0;
                    goto_data_.pos_trans_count = 0;
                    goto_data_.pos_data_len[0] = 0;
                    goto_data_.pos_data_len[1] = 0;
                    startRecovery();
                } else {
                    goto_data_.state = GotoState::WAITING_GOTO;
                }
            }
            break;
        }
        
        case GotoState::WAITING_GOTO: {
            I2CTransactionState state = i2c.getState(goto_data_.goto_trans_id);
            
            if (state == I2CTransactionState::FAILED) {
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.println("': GOTO write failed, aborting go_to");
                goto_data_.state = GotoState::ERROR;
                goto_data_.goto_trans_id = 0;
                startRecovery();
            } else if (state == I2CTransactionState::SUCCESS) {
                // GOTO scris cu succes, asteapta durata minima
                goto_data_.state = GotoState::WAITING_MIN_DURATION;
                goto_data_.start_time_ms = millis();
                // Fără afișare în așteptarea duratei min
            }
            break;
        }
        
        case GotoState::WAITING_MIN_DURATION:
            // Asteapta SERVO_MIN_MOTION_DURATION
            if (millis() - goto_data_.start_time_ms >= SERVO_MIN_MOTION_DURATION) {
                goto_data_.state = GotoState::WAITING_MOV_CLEAR;
                goto_data_.start_time_ms = millis();  // Reset pentru timeout
                // Fără afișare în așteptarea MOV clear
            }
            break;
            
        case GotoState::WAITING_MOV_CLEAR: {
            // Verifica timeout
            uint32_t timeout_ms = goto_data_.time_ms + SERVO_MAX_MOTION_EXTRATIME;
            if (millis() - goto_data_.start_time_ms >= timeout_ms) {
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("': go_to timeout after ");
                Serial.print(timeout_ms);
                Serial.println(" ms");
                goto_data_.state = GotoState::ERROR;
                goto_data_.pos_trans_ids[0] = 0;
                goto_data_.pos_trans_ids[1] = 0;
                goto_data_.pos_trans_count = 0;
                goto_data_.pos_data_len[0] = 0;
                goto_data_.pos_data_len[1] = 0;
                goto_data_.goto_trans_id = 0;
                startRecovery();
                break;
            }
            
            uint8_t status_lsb = read_buffer_[0];
            uint8_t status_msb = read_buffer_[1];
            
            // Verifica ERR bit - daca orice servo are eroare, intra in recovery
            for (int i = 0; i < num_servos_; i++) {
                uint8_t servo_status;
                if (i < 2) {
                    servo_status = (status_lsb >> (i * 4)) & 0x0F;
                } else {
                    servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
                }
                bool has_error = (servo_status & 0x08) != 0;
                if (has_error) {
                    Serial.print("Actuator '");
                    Serial.print(name_);
                    Serial.print("': servo ");
                    Serial.print(i + 1);
                    Serial.println(" in error state during motion");
                    goto_data_.state = GotoState::ERROR;
                    goto_data_.pos_trans_ids[0] = 0;
                    goto_data_.pos_trans_ids[1] = 0;
                    goto_data_.pos_trans_count = 0;
                    goto_data_.pos_data_len[0] = 0;
                    goto_data_.pos_data_len[1] = 0;
                    goto_data_.goto_trans_id = 0;
                    startRecovery();
                    break;
                }
            }
            if (goto_data_.state == GotoState::ERROR) {
                break;
            }
            
            // Verifica MOV bits din read_buffer_ (actualizat de refresh normal)
            bool all_clear = true;
            for (int i = 0; i < num_servos_; i++) {
                uint8_t servo_status;
                if (i < 2) {
                    servo_status = (status_lsb >> (i * 4)) & 0x0F;
                } else {
                    servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
                }
                
                bool is_installed = (servo_status & 0x01) != 0;
                bool is_moving = (servo_status & 0x04) != 0;  // MOV bit
                
                if (is_installed && is_moving) {
                    all_clear = false;
                    break;
                }
            }
            
            if (all_clear) {
#ifdef DIAGNOSE_GOTO
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("': MOV clear at ");
                Serial.println(millis());
#endif
                for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++)
                    last_set_position_[i] = goto_data_.goto_positions[i];
                last_set_valid_ = true;
                goto_data_.state = GotoState::IDLE;
                goto_data_.pos_trans_ids[0] = 0;
                goto_data_.pos_trans_ids[1] = 0;
                goto_data_.pos_trans_count = 0;
                goto_data_.pos_data_len[0] = 0;
                goto_data_.pos_data_len[1] = 0;
                goto_data_.goto_trans_id = 0;
                
                if (goto_data_.queue_.empty()) {
                    goto_data_.sequence_started_ = false;
                }
                
                // Verifica coada si executa urmatorul goto
                executeNextQueuedGoto();
                
                // Acelasi iteratie: drena I2C pana se completeaza POS_SET, apoi ruleaza automatul din nou pentru a trimite GOTO
                if (goto_data_.state == GotoState::WAITING_POS && goto_data_.pos_trans_count > 0) {
                    const uint32_t drain_start_ms = millis();
                    const uint32_t drain_timeout_ms = 50;
                    for (;;) {
                        bool all_done = true;
                        for (uint8_t i = 0; i < goto_data_.pos_trans_count; i++) {
                            I2CTransactionState s = i2c.getState(goto_data_.pos_trans_ids[i]);
                            if (s != I2CTransactionState::SUCCESS && s != I2CTransactionState::FAILED) {
                                all_done = false;
                                break;
                            }
                        }
                        if (all_done)
                            break;
                        if (millis() - drain_start_ms >= drain_timeout_ms)
                            break;
                        i2c.loop();
                    }
                    processGotoStateMachine();
                    // Drena si scrierea GOTO daca a fost pusă în coadă
                    if (goto_data_.state == GotoState::WAITING_GOTO && goto_data_.goto_trans_id != 0) {
                        const uint32_t goto_start_ms = millis();
                        while (i2c.getState(goto_data_.goto_trans_id) != I2CTransactionState::SUCCESS &&
                               i2c.getState(goto_data_.goto_trans_id) != I2CTransactionState::FAILED) {
                            if (millis() - goto_start_ms >= drain_timeout_ms)
                                break;
                            i2c.loop();
                        }
                        processGotoStateMachine();
                    }
                }
            }
            break;
        }
        
        case GotoState::IDLE:
            // Nu ar trebui sa ajungem aici
            break;
            
        case GotoState::ERROR:
            // Starea ERROR ramane pana la urmatorul go_to care o va reseta
            // Nu face nimic aici
            break;
    }
}

/**
 * Recuperare prioritară; altfel activare (START), automat goto, delayed queue, citire STATUS/POS periodică.
 */
void ServoAct::loop() {
    if (recovering_) {
        processRecovery();
        return;
    }
    
    // Continuam sa incercam tranzactii I2C chiar daca actuatorul este offline
    // pentru a detecta cand se recupereaza
    
    // Gestioneaza citirea registrului START daca suntem in proces de activare (doar daca online)
    if (activating_ && online_) {
        // Verifica timeout-ul
        if (millis() - activation_start_ms_ >= SERVO_ACTIVATION_TIMEOUT) {
            activating_ = false;
            start_trans_id_ = 0;
            wait_for_status_after_activation_ = false;
            Serial.print("Actuator '");
            Serial.print(name_);
            Serial.print("' activation timeout after ");
            Serial.print(SERVO_ACTIVATION_TIMEOUT);
            Serial.println(" ms");
            if (online_) {
                Serial.print("Resetting actuator '");
                Serial.print(name_);
                Serial.println("'...");
                reset();
                recordActivationFailureAndCheckWatchdog();
                recovering_ = true;
                recovery_state_ = RecoveryState::RECOVERY_WAIT_ONLINE;
                recovery_start_ms_ = millis();
                recovery_next_probe_ms_ = millis() + 500;
            }
            return;
        }
        
        if (start_trans_id_ != 0) {
            // Verifica starea tranzactiei de citire START
            I2CTransactionState state = i2c.getState(start_trans_id_);
            
            if (state == I2CTransactionState::SUCCESS) {
                // Citeste valoarea START din buffer
                uint8_t start_value = start_buffer_[0];
                
                // Verifica daca exista servomotoare care se initializeaza
                if (start_value != 0) {
                    // Exista servomotoare care se initializeaza
                    saw_initializing_ = true;
                    start_trans_id_ = 0;
                    next_start_read_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                } else if (saw_initializing_) {
                    // Nu mai exista servomotoare care se initializeaza si am vazut cel putin unul initializand
                    // Initializarea s-a terminat - asteapta urmatoarea citire STATUS pentru date proaspete
                    wait_for_status_after_activation_ = true;
                    start_trans_id_ = 0;
                    // Nu mai citim START, asteptam STATUS
                } else {
                    // START deja 0 (dispozitivul poate fi terminat înainte să vedem non-zero): confirmă cu citire STATUS
                    wait_for_status_after_activation_ = true;
                    start_trans_id_ = 0;
                }
            } else if (state == I2CTransactionState::FAILED) {
                // Eroare la citirea START, reincearca
                start_trans_id_ = 0;
                next_start_read_ms_ = millis() + ACTUATOR_REFRESH_TIME;
                // Nu incrementam consecutive_failures_ aici pentru ca activarea poate continua
            } else if (state == I2CTransactionState::UNKNOWN) {
                start_trans_id_ = 0;
                next_start_read_ms_ = millis();
            }
        } else if (millis() >= next_start_read_ms_) {
            // E timpul pentru urmatoarea citire START
            start_trans_id_ = i2c.read(i2c_address_, I2C_REG_START, start_buffer_, 1);
            if (start_trans_id_ == 0) {
                // Coada plina, reincearca mai tarziu
                next_start_read_ms_ = millis() + ACTUATOR_REFRESH_TIME;
            }
        }
    }
    
    // Gestioneaza automatul de stari pentru go_to
    if (goto_data_.state != GotoState::IDLE && online_) {
        processGotoStateMachine();
    }
    
    // Ruleaza gotos programate (delayed) cand e timpul
    if (online_ && goto_data_.state == GotoState::IDLE && !goto_data_.delayed_queue_.empty() &&
        millis() >= goto_data_.delayed_queue_.front().trigger_at_ms) {
        DelayedGoto d = goto_data_.delayed_queue_.front();
        goto_data_.delayed_queue_.pop_front();
        then_go_to_impl(d.coords.positions, d.coords.time_ms, 0, true);
    }
    
    // Verifica daca exista o tranzactie in curs pentru STATUS si POS
    if (trans_id_ != 0) {
        I2CTransactionState state = i2c.getState(trans_id_);
        
        if (state == I2CTransactionState::SUCCESS) {
            // Tranzactia s-a completat cu succes, actualizeaza datele
            bool was_offline = !online_;  // Verifica daca era offline
            consecutive_failures_ = 0;  // Reseteaza contorul de esecuri
            
            // Verifica daca actuatorul s-a recuperat dupa ce a fost offline
            if (was_offline) {
                online_ = true;  // Marcheaza ca fiind online din nou
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("' @0x");
                Serial.print(i2c_address_, HEX);
                Serial.println(" is back online");
            }
            
            updateServoDataFromBuffer();
            trans_id_ = 0;
            next_refresh_ms_ = millis() + ACTUATOR_REFRESH_TIME;
            
            // Când revine online sau suntem online dar nu toate servomotoarele active: activează dacă e nevoie (nu în inițializare: aceea activează la timp, câte două)
            if (num_servos_ > 0 && !allServosActive() && !activating_ && !isAnimatorInBootState()) {
                if (was_offline || online_)
                    activate();
            }
            
            // Daca asteptam dupa activare, verifica starea finala
            if (wait_for_status_after_activation_) {
                wait_for_status_after_activation_ = false;
                activating_ = false;
                last_activation_failure_ms_ = 0;
                previous_activation_failure_ms_ = 0;
                
                // Verifica starea finala a servomotoarelor
                uint8_t active_count = 0;
                
                for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                    if (servos_[i].active) {
                        active_count++;
                    }
                }
                
                // Afiseaza rezultatul activarii
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("' activated ");
                Serial.print(active_count);
                Serial.print(" out of ");
                Serial.print(num_servos_);
                Serial.println(" servos");
                
                // Afiseaza erorile daca exista
                for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                    if (servos_[i].error) {
                        Serial.print("  -Servo ");
                        Serial.print(i + 1);
                        Serial.print(" of actuator '");
                        Serial.print(name_);
                        Serial.println("' is in ERROR state");
                    }
                }
            }
            
            // Trimite pozitiile in format Teleplot (doar daca este online)
            if (online_) {
                teleplotPositions();
            }
        } else if (state == I2CTransactionState::FAILED) {
            // Tranzactia a esuat
            consecutive_failures_++;
            trans_id_ = 0;
            next_refresh_ms_ = millis() + ACTUATOR_REFRESH_TIME;
            
            // Verifica daca actuatorul a devenit offline (5 esecuri consecutive)
            if (consecutive_failures_ >= 5 && online_) {
                online_ = false;
                activating_ = false;
                start_trans_id_ = 0;
                wait_for_status_after_activation_ = false;
                saw_initializing_ = false;
                Serial.print("Actuator '");
                Serial.print(name_);
                Serial.print("' @0x");
                Serial.print(i2c_address_, HEX);
                Serial.println(" went offline");
            }
        } else if (state == I2CTransactionState::UNKNOWN) {
            trans_id_ = 0;
            next_refresh_ms_ = millis();
        }
        // Daca e IN_PROGRESS sau PENDING, asteapta
    } else {
        // Nu exista tranzactie in curs, verifica daca e timpul pentru reimprospatare
        if (millis() >= next_refresh_ms_) {
            // Citeste STATUS + POS pentru toate sloturile (necesar si cand actuatorul revine online, cand num_servos_ era 0)
            uint8_t bytes_to_read = 2 + (MAX_SERVOS_PER_ACTUATOR * 2);
            trans_id_ = i2c.read(i2c_address_, I2C_REG_STATUS, read_buffer_, bytes_to_read);
            
            if (trans_id_ == 0) {
                // Coada I2C plina, reincearca mai tarziu
                next_refresh_ms_ = millis() + ACTUATOR_REFRESH_TIME;
            }
        }
    }
}

/** Parsează read_buffer_: STATUS nibble per servo, poziții int16 → float. */
void ServoAct::updateServoDataFromBuffer() {
    // Buffer-ul contine: STATUS (2 octeti) + POS1-4 (8 octeti) in ordine
    // STATUS bits: INST (bit 0), ACT (bit 1), MOV (bit 2), ERR (bit 3)
    uint8_t status_lsb = read_buffer_[0];
    uint8_t status_msb = read_buffer_[1];
    
    // Actualizeaza num_servos_ si last_servo_ din STATUS (important cand actuatorul revine online)
    num_servos_ = 0;
    last_servo_ = 0;
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        uint8_t servo_status;
        if (i < 2) {
            servo_status = (status_lsb >> (i * 4)) & 0x0F;
        } else {
            servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
        }
        bool is_installed = (servo_status & 0x01) != 0;
        if (is_installed) {
            num_servos_++;
            last_servo_ = i;
        }
    }
    
    // Analizeaza STATUS si actualizeaza datele servomotoarelor
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        uint8_t servo_status;
        if (i < 2) {
            // Servo 1-2: din LSB
            servo_status = (status_lsb >> (i * 4)) & 0x0F;
        } else {
            // Servo 3-4: din MSB
            servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
        }
        
        // Verifica daca servo-ul este instalat (INST bit = bit 0)
        bool is_installed = (servo_status & 0x01) != 0;
        
        if (is_installed) {
            // Actualizeaza statusul pentru servomotoarele instalate
            servos_[i].active = (servo_status & 0x02) != 0;  // ACT bit = bit 1
            servos_[i].error = (servo_status & 0x08) != 0;   // ERR bit = bit 3
            
            // Actualizeaza pozitia (POS-urile sunt in buffer in ordine: POS1 la offset 2, POS2 la 4, etc.)
            // POS contine intotdeauna date valide: pozitie masurata daca exista feedback, sau pozitie comandata daca nu
            int16_t pos_int = read_buffer_[2 + (i * 2)] | (read_buffer_[2 + (i * 2) + 1] << 8);
            // Converteste din int16 (-32768 la 32766) la float (-2.0 la +2.0)
            // Factor de scalare: 16383.0 (32766 / 2.0)
            servos_[i].position = (float)pos_int / 16383.0f;
        } else {
            // Servo neinstalat, reseteaza datele
            servos_[i].active = false;
            servos_[i].error = false;
            servos_[i].position = 0.0f;
        }
    }
    
}

/** Raport text: adresă, număr servomotoare, active / în mișcare / erori. */
void ServoAct::printActuatorStatus(ServoAct* actuator) {
    // Afiseaza status report pentru actuator
    Serial.print("Actuator '");
    Serial.print(actuator->name_);
    Serial.print("' @0x");
    Serial.print(actuator->i2c_address_, HEX);
    Serial.print(": ");
    Serial.print(actuator->num_servos_);
    Serial.print(" servos");
    
    // Numara servomotoarele active, in miscare si cu erori
    // STATUS bits: INST (bit 0), ACT (bit 1), MOV (bit 2), ERR (bit 3)
    uint8_t active_count = 0;
    uint8_t moving_count = 0;
    uint8_t error_count = 0;
    
    // Citeste STATUS din buffer
    uint8_t status_lsb = actuator->read_buffer_[0];
    uint8_t status_msb = actuator->read_buffer_[1];
    
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
        uint8_t servo_status;
        if (i < 2) {
            servo_status = (status_lsb >> (i * 4)) & 0x0F;
        } else {
            servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
        }
        
        // Verifica daca servo-ul este instalat (INST bit = bit 0)
        bool is_installed = (servo_status & 0x01) != 0;
        
        if (is_installed) {
            if (actuator->servos_[i].active) {
                active_count++;
            }
            if ((servo_status & 0x04) != 0) {  // MOV bit = bit 2
                moving_count++;
            }
            if (actuator->servos_[i].error) {
                error_count++;
            }
        }
    }
    
    Serial.print(", ");
    Serial.print(active_count);
    Serial.print(" active");
    
    Serial.print(", ");
    Serial.print(moving_count);
    Serial.print(" moving");
    
    Serial.print(", ");
    Serial.print(error_count);
    Serial.print(" in error state.");
    
    if (actuator->activating_) {
        Serial.print(" Activating now...");
    }
    
    Serial.println();
}

/** act [nume] fără altceva sau „all”: status pentru un actuator sau toate. */
bool ServoAct::cmd_act_status(const char* name_arg) {
    // Proceseaza comanda pentru actuatorul specificat sau pentru toate
    bool found = false;
    ServoAct* current = first_;
    bool is_all = (!name_arg || name_arg[0] == '\0');
    
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        
        if (should_process) {
            printActuatorStatus(current);
            found = true;
            if (!is_all) {
                break;  // Gasit actuatorul specificat, opreste cautarea
            }
        }
        current = current->next_;
    }
    
    if (!found) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    return true;
}

/** act <nume> reset — coadă scriere RESET. */
bool ServoAct::cmd_act_reset(const char* name_arg) {
    // Proceseaza comanda pentru actuatorul specificat sau pentru toate
    bool found = false;
    ServoAct* current = first_;
    bool is_all = (!name_arg || name_arg[0] == '\0');
    
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        
        if (should_process) {
            current->reset();
            found = true;
            if (!is_all) {
                break;  // Gasit actuatorul specificat, opreste cautarea
            }
        }
        current = current->next_;
    }
    
    if (!found) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    THREADS_SERIAL.print("ok:");
    if (!is_all) {
        THREADS_SERIAL.print(name_arg);
    }
    THREADS_SERIAL.println(" reset");
    
    return true;
}

/** act <nume> activate — apel activate() pe instanță. */
bool ServoAct::cmd_act_activate(const char* name_arg) {
    // Proceseaza comanda pentru actuatorul specificat sau pentru toate
    bool found = false;
    ServoAct* current = first_;
    bool is_all = (!name_arg || name_arg[0] == '\0');
    
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        
        if (should_process) {
            current->activate();
            found = true;
            if (!is_all) {
                break;  // Gasit actuatorul specificat, opreste cautarea
            }
        }
        current = current->next_;
    }
    
    if (!found) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    THREADS_SERIAL.print("ok:");
    if (!is_all) {
        THREADS_SERIAL.print(name_arg);
    }
    THREADS_SERIAL.println(" activate");
    
    return true;
}


/** act <nume> get — poziții curente pe serial. */
bool ServoAct::cmd_act_get(const char* name_arg) {
    // Proceseaza comanda pentru actuatorul specificat sau pentru toate
    bool found = false;
    ServoAct* current = first_;
    bool is_all = (!name_arg || name_arg[0] == '\0');
    int actuator_count = 0;
    
    // Numara cate actuatoare vor fi procesate
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        if (should_process) {
            actuator_count++;
        }
        current = current->next_;
    }
    
    current = first_;
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        
        if (should_process) {
            // Verifica STATUS pentru a determina servomotoarele instalate
            uint8_t status_lsb = current->read_buffer_[0];
            uint8_t status_msb = current->read_buffer_[1];
            
            // Frame response daca comanda se aplica la toate actuatoarele (chiar daca e doar unul)
            if (is_all) {
                Serial.print(current->name_);
                Serial.print(": ");
            }
            
            bool first_pos = true;
            for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                uint8_t servo_status;
                if (i < 2) {
                    servo_status = (status_lsb >> (i * 4)) & 0x0F;
                } else {
                    servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
                }
                
                // Verifica daca servo-ul este instalat (INST bit = bit 0)
                bool is_installed = (servo_status & 0x01) != 0;
                
                if (is_installed) {
                    if (!first_pos) {
                        Serial.print(", ");
                    }
                    Serial.print(current->servos_[i].position, 3);
                    first_pos = false;
                }
            }
            
            Serial.println();
            found = true;
            
            if (!is_all) {
                break;  // Gasit actuatorul specificat, opreste cautarea
            }
        }
        current = current->next_;
    }
    
    if (!found) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    return true;
}

/** act <nume> goto … — parsează poziții și durată, apoi go_to. */
bool ServoAct::cmd_act_goto(const char* name_arg, const char* args) {
    // Verifica ca numele actuatorului este specificat (nu permite "all")
    if (!name_arg || name_arg[0] == '\0') {
        THREADS_SERIAL.println("err:usage");
        return false;
    }
    
    // Gaseste actuatorul
    ServoAct* actuator = nullptr;
    ServoAct* current = first_;
    while (current != nullptr) {
        if (strcmp(current->name_, name_arg) == 0 && current->online_) {
            actuator = current;
            break;
        }
        current = current->next_;
    }
    
    if (actuator == nullptr) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    // Verifica ca actuatorul are servomotoare instalate
    if (actuator->num_servos_ == 0) {
        Serial.print("Actuator '");
        Serial.print(name_arg);
        Serial.println("': no servos installed");
        return false;
    }
    
    // Helper: interpretează argumentul ca string (pentru verificarea „X”)
    auto getArgString = [](const char* arg, uint8_t index, char* buf, size_t buf_size) -> bool {
        if (!arg || !buf || buf_size == 0) {
            return false;
        }
        
        const char* p = arg;
        uint8_t current_index = 0;
        
        // Sare peste spatiile de la inceput
        while (*p == ' ') {
            p++;
        }
        
        // Parcurge argumentele pana la index-ul dorit
        while (current_index < index) {
            // Gaseste sfarsitul argumentului curent
            while (*p != ' ' && *p != '\0') {
                p++;
            }
            
            // Daca am ajuns la sfarsit, nu mai exista argumente
            if (*p == '\0') {
                return false;
            }
            
            // Sare peste spatiile dintre argumente
            while (*p == ' ') {
                p++;
            }
            
            // Daca am ajuns la sfarsit dupa spatiu, nu mai exista argumente
            if (*p == '\0') {
                return false;
            }
            
            current_index++;
        }
        
        // Acum p este la inceputul argumentului dorit
        // Copiaza argumentul in buffer
        const char* arg_start = p;
        while (*p != ' ' && *p != '\0' && (size_t)(p - arg_start) < buf_size - 1) {
            p++;
        }
        size_t len = p - arg_start;
        if (len >= buf_size) {
            len = buf_size - 1;
        }
        strncpy(buf, arg_start, len);
        buf[len] = '\0';
        return true;
    };
    
    // Interpretează pozițiile folosind getFloatArg
    float positions[MAX_SERVOS_PER_ACTUATOR];
    for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) positions[i] = X;
    
    // Gasim unde incepe " in " pentru a limita numarul de argumente de pozitii
    const char* in_pos = strstr(args, " in ");
    int max_pos_args;
    if (in_pos != nullptr) {
        // Numara cate argumente sunt inainte de " in "
        const char* p = args;
        max_pos_args = 0;
        while (p < in_pos && max_pos_args < MAX_SERVOS_PER_ACTUATOR) {
            while (*p == ' ' && p < in_pos) {
                p++;
            }
            if (p >= in_pos) {
                break;
            }
            while (*p != ' ' && *p != '\0' && p < in_pos) {
                p++;
            }
            max_pos_args++;
            while (*p == ' ' && p < in_pos) {
                p++;
            }
        }
    } else {
        // Fara " in "; numara toate argumentele (sunt doar pozitii)
        const char* p = args;
        max_pos_args = 0;
        while (*p != '\0' && max_pos_args < MAX_SERVOS_PER_ACTUATOR) {
            while (*p == ' ') {
                p++;
            }
            if (*p == '\0') {
                break;
            }
            while (*p != ' ' && *p != '\0') {
                p++;
            }
            max_pos_args++;
            while (*p == ' ') {
                p++;
            }
        }
    }
    
    // Verifica ca numarul de pozitii se potriveste cu numarul de servomotoare
    if (max_pos_args != actuator->num_servos_) {
        Serial.print("Actuator '");
        Serial.print(name_arg);
        Serial.print("': expected ");
        Serial.print(actuator->num_servos_);
        Serial.print(" position value(s), got ");
        Serial.print(max_pos_args);
        Serial.println();
        return false;
    }
    
    // Interpretează fiecare poziție
    for (int i = 0; i < actuator->num_servos_; i++) {
        char arg_buf[32];
        if (getArgString(args, i, arg_buf, sizeof(arg_buf))) {
            // Verifica daca este "X"
            if (strcmp(arg_buf, "X") == 0) {
                positions[i] = X;
            } else {
                // Interpretează ca float folosind getFloatArg
                float pos;
                if (!getFloatArg(args, i, &pos)) {
                    THREADS_SERIAL.println("err:usage");
                    return false;
                }
                positions[i] = pos;
            }
        } else {
            THREADS_SERIAL.println("err:usage");
            return false;
        }
    }
    
    // Gaseste index-ul pentru "in" (daca exista) pentru a parsea timpul
    int in_index = -1;
    const char* p = args;
    int arg_count = 0;
    while (*p != '\0' && arg_count < 10) {
        // Sare peste spatiile initiale
        while (*p == ' ') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        
        // Verifica daca este "in"
        if (strncmp(p, "in", 2) == 0 && (p[2] == ' ' || p[2] == '\0')) {
            in_index = arg_count;
            break;
        }
        
        // Gaseste sfarsitul argumentului
        while (*p != ' ' && *p != '\0') {
            p++;
        }
        arg_count++;
        
        // Sare peste spatiile dintre argumente
        while (*p == ' ') {
            p++;
        }
    }
    
    // Interpretează timpul (opțional) — getIntArg pe argumentul de după „in”
    uint16_t time_ms = 0;
    if (in_index >= 0) {
        // "in" a fost gasit la index-ul in_index, timpul este la in_index + 1
        int time_val;
        if (!getIntArg(args, in_index + 1, &time_val)) {
            THREADS_SERIAL.println("err:usage");
            return false;
        }
        if (time_val < 0 || time_val > 65535) {
            THREADS_SERIAL.println("err:usage");
            return false;
        }
        time_ms = (uint16_t)time_val;
    }
    
    // Apeleaza go_to cu pozitiile parseate
    switch (actuator->num_servos_) {
        case 1:
            actuator->go_to(positions[0], time_ms);
            break;
        case 2:
            actuator->go_to(positions[0], positions[1], time_ms);
            break;
        case 3:
            actuator->go_to(positions[0], positions[1], positions[2], time_ms);
            break;
        case 4:
            actuator->go_to(positions[0], positions[1], positions[2], positions[3], time_ms);
            break;
        default:
            return false;
    }
    
    Serial.print("ok:");
    Serial.print(name_arg);
    Serial.println(" goto");
    return true;
}

/** act <nume> info — detalii actuator/servomotoare. */
bool ServoAct::cmd_act_info(const char* name_arg) {
    // Proceseaza comanda pentru actuatorul specificat sau pentru toate
    bool found = false;
    ServoAct* current = first_;
    bool is_all = (!name_arg || name_arg[0] == '\0');
    int actuator_count = 0;
    
    // Numara cate actuatoare vor fi procesate
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        if (should_process) {
            actuator_count++;
        }
        current = current->next_;
    }
    
    current = first_;
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        
        if (should_process) {
            // Verifica STATUS pentru a determina servomotoarele instalate
            uint8_t status_lsb = current->read_buffer_[0];
            uint8_t status_msb = current->read_buffer_[1];
            
            // Frame response pentru mai multe actuatoare
            if (actuator_count > 1) {
                Serial.print("Actuator '");
                Serial.print(current->name_);
                Serial.print("' @0x");
                Serial.print(current->i2c_address_, HEX);
                Serial.print(": ");
            } else {
                Serial.print("Actuator '");
                Serial.print(current->name_);
                Serial.print("' @0x");
                Serial.print(current->i2c_address_, HEX);
                Serial.print(": ");
            }
            Serial.print(current->num_servos_);
            Serial.println(" servos installed");
            
            for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                uint8_t servo_status;
                if (i < 2) {
                    servo_status = (status_lsb >> (i * 4)) & 0x0F;
                } else {
                    servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
                }
                
                // Verifica daca servo-ul este instalat (INST bit = bit 0)
                bool is_installed = (servo_status & 0x01) != 0;
                
                if (is_installed) {
                    Serial.print("  Servo ");
                    Serial.print(i + 1);
                    Serial.print(": ");
                    
                    // INST bit
                    Serial.print("installed");
                    
                    // ACT bit
                    if ((servo_status & 0x02) != 0) {
                        Serial.print(", active");
                    } else {
                        Serial.print(", inactive");
                    }
                    
                    // MOV bit
                    if ((servo_status & 0x04) != 0) {
                        Serial.print(", moving");
                    } else {
                        Serial.print(", stationary");
                    }
                    
                    // ERR bit
                    if ((servo_status & 0x08) != 0) {
                        Serial.print(", ERROR");
                    }
                    
                    // Position
                    Serial.print(", position=");
                    Serial.print(current->servos_[i].position, 3);
                    
                    Serial.println();
                }
            }
            
            found = true;
            
            if (!is_all) {
                break;  // Gasit actuatorul specificat, opreste cautarea
            }
        }
        current = current->next_;
    }
    
    if (!found) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    return true;
}

/** act [nume] plot on|off — Teleplot poziții. */
bool ServoAct::cmd_act_plot(const char* name_arg, const char* onoff_arg) {
    if (!onoff_arg || *onoff_arg == '\0') {
        THREADS_SERIAL.println("err:usage");
        return false;
    }
    
    // Interpretează valoarea on/off
    const char* p = onoff_arg;
    while (*p == ' ') {
        p++;
    }
    
    bool enable = false;
    if (strncmp(p, "on", 2) == 0 && (p[2] == ' ' || p[2] == '\0')) {
        enable = true;
    } else if (strncmp(p, "off", 3) == 0 && (p[3] == ' ' || p[3] == '\0')) {
        enable = false;
    } else {
        THREADS_SERIAL.println("err:usage");
        return false;
    }
    
    // Proceseaza comanda pentru actuatorul specificat sau pentru toate
    bool found = false;
    ServoAct* current = first_;
    bool is_all = (!name_arg || name_arg[0] == '\0');
    
    while (current != nullptr) {
        bool should_process = false;
        if (is_all) {
            should_process = current->online_;
        } else {
            should_process = (strcmp(current->name_, name_arg) == 0 && current->online_);
        }
        
        if (should_process) {
            current->plot_enabled_ = enable;
            found = true;
            if (!is_all) {
                break;  // Gasit actuatorul specificat, opreste cautarea
            }
        }
        current = current->next_;
    }
    
    if (!found) {
        THREADS_SERIAL.println("err:notfound");
        return false;
    }
    
    THREADS_SERIAL.print("ok:");
    if (!is_all) {
        THREADS_SERIAL.print(name_arg);
    }
    THREADS_SERIAL.print(" plot ");
    THREADS_SERIAL.println(enable ? "on" : "off");
    
    return true;
}

/** true dacă name se potrivește unui actuator online din listă. */
bool ServoAct::isActuatorName(const char* name) {
    ServoAct* current = first_;
    while (current != nullptr) {
        if (strcmp(current->name_, name) == 0 && current->online_) {
            return true;
        }
        current = current->next_;
    }
    return false;
}

/** Rută principală: listă actuatoare sau nume + subcomandă. */
bool ServoAct::cmd_act(const char* arg) {
    // Daca nu exista argumente, listeaza actuatoarele
    if (!arg || *arg == '\0') {
        ServoAct* current = first_;
        bool found = false;
        
        while (current != nullptr) {
            if (current->online_) {
                Serial.print("'");
                Serial.print(current->name_);
                Serial.print("' @0x");
                Serial.print(current->i2c_address_, HEX);
                Serial.println();
                found = true;
            }
            current = current->next_;
        }
        
        if (!found) {
            THREADS_SERIAL.println("no actuator installed");
            return false;
        }
        
        return true;
    }
    
    // Interpretează primul argument
    const char* p = arg;
    
    // Sare peste spatiile de la inceput
    while (*p == ' ') {
        p++;
    }
    
    const char* first_word_start = p;
    while (*p != ' ' && *p != '\0') {
        p++;
    }
    
    if (first_word_start == p) {
        THREADS_SERIAL.println("err:usage");
        return false;
    }
    
    // Copiaza primul cuvant intr-un buffer temporar
    char first_word_buf[10] = {0};  // Suficient pentru subcomenzi (activate, etc.)
    int first_word_len = p - first_word_start;
    if (first_word_len > 9) first_word_len = 9;
    strncpy(first_word_buf, first_word_start, first_word_len);
    first_word_buf[first_word_len] = '\0';
    
    // Verifica daca primul cuvant este un nume de actuator cunoscut
    char name_buf[5] = {0};
    const char* subcommand_start = p;
    
    if (isActuatorName(first_word_buf)) {
        // Primul cuvant este un nume de actuator
        strncpy(name_buf, first_word_buf, 4);
        name_buf[4] = '\0';
        
        // Sare peste spatiile dintre argumente
        while (*p == ' ') {
            p++;
        }
        subcommand_start = p;
    } else {
        // Primul cuvant nu este un nume de actuator, deci este o subcomanda
        // Aplica la toate actuatoarele (nume gol)
        name_buf[0] = '\0';
        subcommand_start = first_word_start;
    }
    
    // Verifica daca exista o subcomanda
    if (*subcommand_start == '\0') {
        // Nu exista subcomanda - afiseaza status report
        return cmd_act_status(name_buf);
    }
    
    // Verifica daca urmatorul argument este "plot"
    if (strncmp(subcommand_start, "plot", 4) == 0) {
        p = subcommand_start + 4;  // Sare peste "plot"
        // Sare peste spatiile dintre "plot" si on/off
        while (*p == ' ') {
            p++;
        }
        return cmd_act_plot(name_buf, p);
    } else if (strncmp(subcommand_start, "activate", 8) == 0) {
        // Comanda "activate" nu are argumente suplimentare
        return cmd_act_activate(name_buf);
    } else if (strncmp(subcommand_start, "reset", 5) == 0) {
        // Comanda "reset" nu are argumente suplimentare
        return cmd_act_reset(name_buf);
    } else if (strncmp(subcommand_start, "get", 3) == 0) {
        // Comanda "get" nu are argumente suplimentare
        return cmd_act_get(name_buf);
    } else if (strncmp(subcommand_start, "info", 4) == 0) {
        // Comanda "info" nu are argumente suplimentare
        return cmd_act_info(name_buf);
    } else if (strncmp(subcommand_start, "goto", 4) == 0) {
        p = subcommand_start + 4;  // Sare peste "goto"
        // Sare peste spatiile dintre "goto" si argumente
        while (*p == ' ') {
            p++;
        }
        return cmd_act_goto(name_buf, p);
    } else {
        THREADS_SERIAL.println("err:usage");
        return false;
    }
}

/** >nameN: poziție și >nameN-M: MOV pentru actuatoare cu plot on. */
void ServoAct::teleplotPositions() {
    ServoAct* current = first_;
    
    // Parcurge toate actuatoarele online cu plot activat
    while (current != nullptr) {
        if (current->online_ && current->plot_enabled_ && current->num_servos_ > 0) {
            // Verifica STATUS pentru a determina servomotoarele instalate
            uint8_t status_lsb = current->read_buffer_[0];
            uint8_t status_msb = current->read_buffer_[1];
            
            // Afiseaza pozitiile pentru fiecare servo instalat
            for (int i = 0; i < MAX_SERVOS_PER_ACTUATOR; i++) {
                uint8_t servo_status;
                if (i < 2) {
                    servo_status = (status_lsb >> (i * 4)) & 0x0F;
                } else {
                    servo_status = (status_msb >> ((i - 2) * 4)) & 0x0F;
                }
                
                // Verifica daca servo-ul este instalat (INST bit = bit 0)
                bool is_installed = (servo_status & 0x01) != 0;
                
                if (is_installed) {
                    // Format Teleplot: >label:value (fiecare pe linie separata)
                    // Ex: >gen1:0.123
                    Serial.print(">");
                    Serial.print(current->name_);
                    Serial.print(i + 1);  // Servo index (1-4)
                    Serial.print(":");
                    Serial.print(current->servos_[i].position, 3);
                    Serial.print("\n");
                    
                    // Afiseaza si bitul MOV (moving) ca 0 sau 1
                    // Format: >label-M:value
                    // Ex: >gen1-M:1
                    bool is_moving = (servo_status & 0x04) != 0;  // MOV bit = bit 2
                    Serial.print(">");
                    Serial.print(current->name_);
                    Serial.print(i + 1);
                    Serial.print("-M:");
                    Serial.print(is_moving ? 1 : 0);
                    Serial.print("\n");
                }
            }
        }
        current = current->next_;
    }
}
