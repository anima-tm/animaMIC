/*
 * i2c.cpp
 *
 * animaMIC — Implementare I2CManager: coadă FIFO, execuție sincronă în loop, liste ID-uri SUCCESS/FAILED.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "i2c.h"

I2CManager i2c;

/** Interval I2C_LOOP_INTERVAL_MS; current_transaction_.id=0 = inactiv. */
I2CManager::I2CManager()
    : Thread(I2C_LOOP_INTERVAL_MS),
      transaction_start_ms_(0),
      next_transaction_id_(0) {
    current_transaction_.id = 0;
    current_transaction_.type = I2CTransactionType::WRITE;
}

I2CManager::~I2CManager() {
}

/** WIRE 100 kHz. */
void I2CManager::begin() {
    WIRE.begin();
    WIRE.setClock(100000);
}

/**
 * Finalizează tranzacția curentă (timeout → FAILED), apoi pornește următoarea din coadă.
 */
void I2CManager::loop() {
    if (current_transaction_.id != 0) {
        if (current_transaction_.state == I2CTransactionState::SUCCESS ||
            current_transaction_.state == I2CTransactionState::FAILED) {
            endTransaction(current_transaction_);
            current_transaction_.id = 0;
        } else if (current_transaction_.state == I2CTransactionState::IN_PROGRESS) {
            if (millis() - transaction_start_ms_ > I2C_TRANSACTION_TIMEOUT_MS) {
                current_transaction_.state = I2CTransactionState::FAILED;
                endTransaction(current_transaction_);
                current_transaction_.id = 0;
            }
        }
    }

    if (!queue_.empty() && current_transaction_.id == 0) {
        current_transaction_ = queue_.front();
        queue_.pop();
        executeTransaction(current_transaction_);
    }
}

uint16_t I2CManager::write(uint8_t address, uint8_t reg_addr,
                           const uint8_t* data, uint8_t data_len) {
    if (queue_.size() >= I2C_MAX_QUEUE_SIZE) {
        return 0;
    }

    if (data == nullptr || data_len == 0) {
        return 0;
    }

    I2CTransaction tx;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.id = next_transaction_id_++;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.address = address;
    tx.reg_addr = reg_addr;
    tx.type = I2CTransactionType::WRITE;
    tx.data = const_cast<uint8_t*>(data);
    tx.data_len = data_len;
    tx.state = I2CTransactionState::PENDING;

    queue_.push(tx);

    return tx.id;
}

uint16_t I2CManager::read(uint8_t address, uint8_t reg_addr,
                          uint8_t* data, uint8_t data_len) {
    if (queue_.size() >= I2C_MAX_QUEUE_SIZE) {
        return 0;
    }

    if (data == nullptr || data_len == 0) {
        return 0;
    }

    I2CTransaction tx;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.id = next_transaction_id_++;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.address = address;
    tx.reg_addr = reg_addr;
    tx.type = I2CTransactionType::READ;
    tx.data = data;
    tx.data_len = data_len;
    tx.state = I2CTransactionState::PENDING;

    queue_.push(tx);

    return tx.id;
}

uint16_t I2CManager::setReadAddress(uint8_t address, uint8_t reg_addr) {
    if (queue_.size() >= I2C_MAX_QUEUE_SIZE) {
        return 0;
    }

    I2CTransaction tx;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.id = next_transaction_id_++;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.address = address;
    tx.reg_addr = reg_addr;
    tx.type = I2CTransactionType::SET_READ;
    tx.data = nullptr;
    tx.data_len = 0;
    tx.state = I2CTransactionState::PENDING;

    queue_.push(tx);

    return tx.id;
}

uint16_t I2CManager::continueRead(uint8_t address, uint8_t* data, uint8_t data_len) {
    if (queue_.size() >= I2C_MAX_QUEUE_SIZE) {
        return 0;
    }

    if (data == nullptr || data_len == 0) {
        return 0;
    }

    I2CTransaction tx;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.id = next_transaction_id_++;
    if (next_transaction_id_ == 0) next_transaction_id_ = 1;
    tx.address = address;
    tx.reg_addr = 0;
    tx.type = I2CTransactionType::CONTINUE_READ;
    tx.data = data;
    tx.data_len = data_len;
    tx.state = I2CTransactionState::PENDING;

    queue_.push(tx);

    return tx.id;
}

I2CTransactionState I2CManager::getState(uint16_t transaction_id) {
    if (transaction_id == 0) {
        return I2CTransactionState::UNKNOWN;
    }

    if (current_transaction_.id == transaction_id) {
        return current_transaction_.state;
    }

    queue<I2CTransaction> temp_queue = queue_;
    while (!temp_queue.empty()) {
        if (temp_queue.front().id == transaction_id) {
            return temp_queue.front().state;
        }
        temp_queue.pop();
    }

    for (uint16_t id : succeeded_ids_) {
        if (id == transaction_id) {
            return I2CTransactionState::SUCCESS;
        }
    }

    for (uint16_t id : failed_ids_) {
        if (id == transaction_id) {
            return I2CTransactionState::FAILED;
        }
    }

    return I2CTransactionState::UNKNOWN;
}

void I2CManager::cancelTransactionsForAddress(uint8_t address) {
    queue<I2CTransaction> kept;
    while (!queue_.empty()) {
        I2CTransaction tx = queue_.front();
        queue_.pop();
        if (tx.address != address) {
            kept.push(tx);
        }
    }
    queue_ = kept;
}

/** Execută sincron pe WIRE; setează SUCCESS/FAILED imediat (stare vizibilă în următorul loop). */
void I2CManager::executeTransaction(I2CTransaction& tx) {
    tx.state = I2CTransactionState::IN_PROGRESS;
    transaction_start_ms_ = millis();

    bool success = false;
    switch (tx.type) {
        case I2CTransactionType::WRITE:
            success = performWrite(tx);
            break;

        case I2CTransactionType::READ:
            success = performRead(tx);
            break;

        case I2CTransactionType::SET_READ:
            success = performSetReadAddress(tx);
            break;

        case I2CTransactionType::CONTINUE_READ:
            success = performContinueRead(tx);
            break;
    }

    if (success) {
        tx.state = I2CTransactionState::SUCCESS;
    } else {
        tx.state = I2CTransactionState::FAILED;
    }
}

bool I2CManager::performWrite(I2CTransaction& tx) {
    WIRE.beginTransmission(tx.address);
    WIRE.write(tx.reg_addr);
    for (uint8_t i = 0; i < tx.data_len; i++) {
        WIRE.write(tx.data[i]);
    }
    uint8_t error = WIRE.endTransmission(true);

    return (error == 0);
}

bool I2CManager::performRead(I2CTransaction& tx) {
    WIRE.beginTransmission(tx.address);
    WIRE.write(tx.reg_addr);
    uint8_t error = WIRE.endTransmission(false);

    if (error != 0) {
        return false;
    }

    uint8_t bytesRead = WIRE.requestFrom(tx.address, tx.data_len, true);

    if (bytesRead != tx.data_len) {
        while (WIRE.available()) {
            WIRE.read();
        }
        return false;
    }

    for (uint8_t i = 0; i < tx.data_len; i++) {
        tx.data[i] = WIRE.read();
    }

    while (WIRE.available()) {
        WIRE.read();
    }

    return true;
}

bool I2CManager::performSetReadAddress(I2CTransaction& tx) {
    WIRE.beginTransmission(tx.address);
    WIRE.write(tx.reg_addr);
    uint8_t error = WIRE.endTransmission(true);

    return (error == 0);
}

bool I2CManager::performContinueRead(I2CTransaction& tx) {
    uint8_t bytesRead = WIRE.requestFrom(tx.address, tx.data_len, true);

    if (bytesRead != tx.data_len) {
        while (WIRE.available()) {
            WIRE.read();
        }
        return false;
    }

    for (uint8_t i = 0; i < tx.data_len; i++) {
        tx.data[i] = WIRE.read();
    }

    while (WIRE.available()) {
        WIRE.read();
    }

    return true;
}

/** Mută ID în succeeded_ids_ sau failed_ids_ (FIFO, cap la I2C_MAX_COMPLETION_LIST_SIZE). */
void I2CManager::endTransaction(I2CTransaction& tx) {
    if (tx.state == I2CTransactionState::SUCCESS) {
        if (succeeded_ids_.size() >= I2C_MAX_COMPLETION_LIST_SIZE) {
            succeeded_ids_.erase(succeeded_ids_.begin());
        }
        succeeded_ids_.push_back(tx.id);
    } else {
        if (failed_ids_.size() >= I2C_MAX_COMPLETION_LIST_SIZE) {
            failed_ids_.erase(failed_ids_.begin());
        }
        failed_ids_.push_back(tx.id);
    }
}
