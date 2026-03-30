/*
 * i2c.h
 *
 * animaMIC — Manager I2C: coadă de tranzacții, execuție serială pe bus, interogare stare după ID.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: un singur transfer activ; restul în coadă (FIFO). ServoAct și alți clienți trimit write/read
 * neblocante și așteaptă SUCCESS/FAILED prin getState(id). Timeout scurt per tranzacție în loop.
 */

#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <queue>
#include "Threads.h"

using namespace std;

/** Obiect TwoWire: implicit Wire; proiectul poate defini WIRE ca Wire1 etc. */
#ifndef WIRE
#define WIRE Wire
#endif

/** Număr maxim de tranzacții în așteptare. */
#define I2C_MAX_QUEUE_SIZE 16
/** Lungime maximă listelor de ID-uri finalizate (evită creștere nelimitată). */
#define I2C_MAX_COMPLETION_LIST_SIZE 32
/** Dacă IN_PROGRESS depășește acest timp (ms), tranzacția devine FAILED. */
#define I2C_TRANSACTION_TIMEOUT_MS 10
/** Interval apel loop() thread I2C (ms). */
#define I2C_LOOP_INTERVAL_MS 50

/** Ciclu de viață al unei tranzacții după ID. */
enum class I2CTransactionState {
    PENDING,
    IN_PROGRESS,
    SUCCESS,
    FAILED,
    UNKNOWN
};

/**
 * Tip transfer: WRITE/READ atomice; SET_READ + CONTINUE_READ pentru protocoale în două faze (rar).
 */
enum class I2CTransactionType {
    WRITE,
    READ,
    SET_READ,
    CONTINUE_READ
};

/** Înregistrare coadă: adresă 7 biți, registru, tip, buffer partajat cu apelantul până la SUCCESS/FAILED. */
struct I2CTransaction {
    uint16_t id;
    uint8_t address;
    uint8_t reg_addr;
    I2CTransactionType type;
    uint8_t* data;
    uint8_t data_len;
    I2CTransactionState state;
};

/**
 * Thread dedicat: scoate din coadă, execută sincron pe WIRE, păstrează istoric scurt de ID-uri.
 */
class I2CManager : public Thread {
public:
    I2CManager();
    ~I2CManager();

    void begin() override;
    void loop() override;

    /**
     * Enqueue scriere: reg + payload; returnează ID sau 0 dacă coadă plină / parametri invalizi.
     * Bufferul trebuie să rămână valid până la finalizare.
     */
    uint16_t write(uint8_t address, uint8_t reg_addr,
                   const uint8_t* data, uint8_t data_len);

    /**
     * Enqueue citire atomică: write reg + repeated start + read N octeți.
     */
    uint16_t read(uint8_t address, uint8_t reg_addr,
                  uint8_t* data, uint8_t data_len);

    /** Enqueue: setează pointer de citire pe sclav (WRITE reg + STOP). */
    uint16_t setReadAddress(uint8_t address, uint8_t reg_addr);

    /** Enqueue: citește N octeți după setReadAddress. */
    uint16_t continueRead(uint8_t address, uint8_t* data, uint8_t data_len);

    /** Stare curentă, din coadă sau din listele de completare; UNKNOWN dacă ID necunoscut. */
    I2CTransactionState getState(uint16_t transaction_id);

    /** Elimină din coadă (nu din execuție curentă) toate tranzacțiile către adresă. */
    void cancelTransactionsForAddress(uint8_t address);

private:
    queue<I2CTransaction> queue_;
    I2CTransaction current_transaction_;

    vector<uint16_t> succeeded_ids_;
    vector<uint16_t> failed_ids_;

    uint32_t transaction_start_ms_;
    uint16_t next_transaction_id_;

    void executeTransaction(I2CTransaction& tx);
    bool performWrite(I2CTransaction& tx);
    bool performRead(I2CTransaction& tx);
    bool performSetReadAddress(I2CTransaction& tx);
    bool performContinueRead(I2CTransaction& tx);
    void endTransaction(I2CTransaction& tx);
};

extern I2CManager i2c;

#endif // I2C_H
