/*
 * Threads.cpp
 *
 * animaMIC — Înregistrare module, interpretor comenzi pe linie, parcurgere listă.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "Threads.h"
#include "../commands.h"
#include <string.h>

Thread* Thread::first_ = nullptr;

Thread::Thread(uint32_t interval_ms)
    : interval_ms_(interval_ms), last_run_ms_(0), next_(nullptr), resume_point_(nullptr) {
    registerThread(this);
}

Thread::~Thread() {
    end();
    unregisterThread(this);
}

void Thread::registerThread(Thread* thread) {
    if (thread == nullptr) return;
    // Inserare la începutul listei înlănțuite (ultimul creat = primul parcurs în runThreads dacă ordinea depinde de asta)
    thread->next_ = first_;
    first_ = thread;
}

void Thread::unregisterThread(Thread* thread) {
    if (thread == nullptr || first_ == nullptr) return;
    if (first_ == thread) {
        first_ = thread->next_;
        thread->next_ = nullptr;
        return;
    }
    Thread* current = first_;
    while (current->next_ != nullptr) {
        if (current->next_ == thread) {
            current->next_ = thread->next_;
            thread->next_ = nullptr;
            return;
        }
        current = current->next_;
    }
}

// --- Interpretor de comenzi (linie în buffer până la \n/\r) ---
static const int CMD_BUFFER_SIZE = 64;
static const CommandEntry* parser_commands_ = nullptr;  // pointer la commands[] din commands.cpp
static uint8_t parser_num_commands_ = 0;                // număr de intrări în registru
static char parser_cmd_buffer_[CMD_BUFFER_SIZE];        // linie curentă (fără \n), terminată cu '\0'
static int parser_len_ = 0;                             // număr de caractere utile în buffer

/**
 * Interpretează o linie completă: primul cuvânt = comandă, restul = argument (opțional).
 * Caută comanda în parser_commands_; dacă există, apelează handler(arg). Altfel: err:cmd pe serial.
 * Nu returnează valoare; efect lateral: I/O pe THREADS_SERIAL.
 */
static void parseLine() {
    if (parser_len_ == 0) {
        return;
    }
    while (parser_len_ > 0 && (parser_cmd_buffer_[parser_len_ - 1] == '\r' || parser_cmd_buffer_[parser_len_ - 1] == ' ')) {
        parser_len_--;
        parser_cmd_buffer_[parser_len_] = '\0';
    }
    if (parser_cmd_buffer_[0] == '#') {
        return;
    }
    char* cmd = parser_cmd_buffer_;
    while (*cmd == ' ') {
        cmd++;
    }
    char* e = cmd;
    while (*e != ' ' && *e != '\0') {
        e++;
    }
    const char* arg = nullptr;
    if (*e != '\0') {
        *e = '\0';
        e++;
        while (*e == ' ') {
            e++;
        }
        if (*e != '\0') {
            arg = e;
        }
    }
    for (uint8_t i = 0; i < parser_num_commands_; i++) {
        if (strcmp(cmd, parser_commands_[i].name) == 0) {
            parser_commands_[i].handler(arg);
            return;
        }
    }
    THREADS_SERIAL.print("err:cmd\n");
}

void Thread::initParser() {
    parser_commands_ = commands;
    parser_num_commands_ = commands_count;
    parser_len_ = 0;
    parser_cmd_buffer_[0] = '\0';
}

void Thread::runParser() {
    while (THREADS_SERIAL.available()) {
        int c = THREADS_SERIAL.read();
        if (c < 0) {
            break;
        }
        if (c == '\n' || c == '\r') {
            if (parser_len_ > 0) {
                parseLine();
            }
            parser_len_ = 0;
            parser_cmd_buffer_[0] = '\0';
            continue;
        }
        if (parser_len_ + 1 >= CMD_BUFFER_SIZE) {
            parser_len_ = 0;
            parser_cmd_buffer_[0] = '\0';
            continue;
        }
        parser_cmd_buffer_[parser_len_] = (char)c;
        parser_cmd_buffer_[parser_len_ + 1] = '\0';
        parser_len_++;
    }
}

void Thread::runThreads() {
    uint32_t current_ms = millis();
    Thread* thread = first_;
    while (thread != nullptr) {
        bool should_run = false;
        if (thread->interval_ms_ == 0) {
            should_run = true;
        } else {
            uint32_t elapsed = current_ms - thread->last_run_ms_;
            if (elapsed >= thread->interval_ms_) {
                should_run = true;
            }
        }
        if (should_run) {
            thread->last_run_ms_ = current_ms;
            thread->loop();
        }
        thread = thread->next_;
    }
}

void Thread::setup() {
    initParser();
    Thread* thread = first_;
    while (thread != nullptr) {
        thread->begin();
        thread = thread->next_;
    }
}

void Thread::run() {
    runParser();
    runThreads();
}
