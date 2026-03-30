/*
 * commands.h
 *
 * animaMIC — Registru de comenzi serial și prototipuri pentru handler-e.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: fiecare linie de text pe USB este împărțită în nume comandă + rest (argumente).
 * Handler-ul primește doar șirul de argumente (fără numele comenzii).
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

/** Tip pentru funcția care execută o comandă; arg poate fi nullptr dacă nu există text după comandă. */
typedef bool (*CommandHandler)(const char* arg);

/** O intrare în tabelul static commands[]: nume scurt și funcția apelată de interpretor. */
struct CommandEntry {
    const char* name;        // identificator comandă (fără spații)
    CommandHandler handler;  // pointer la funcția care procesează linia de argumente
};

/**
 * Obține al n-lea câmp din șirul de argumente (separare prin spații).
 * @param arg restul liniei după numele comenzii (poate fi nullptr)
 * @param index 0 = primul cuvânt din arg, 1 = al doilea, etc.
 * @param value unde se scrie întregul citit cu atoi
 * @return true dacă există cel puțin index+1 cuvinte
 */
bool getIntArg(const char* arg, uint8_t index, int* value);

/**
 * La fel ca getIntArg, dar citește un float cu strtof; verifică că numărul se termină la sfârșitul câmpului.
 * @return true dacă parsarea reușește conform regulilor de cuvânt
 */
bool getFloatArg(const char* arg, uint8_t index, float* value);

bool cmd_alive(const char* arg);
bool cmd_reset(const char* arg);
bool cmd_ir_forward(const char* arg);  /** ir — trimite restul liniei pe UART către IRsense */
bool cmd_teleplot(const char* arg);    /** teleplot — comută canale de vizualizare (vezi doc comenzi) */

/** Tabel citit de Thread::initParser(); ultima intrare determină commands_count. */
extern const CommandEntry commands[];
extern const uint8_t commands_count;

#endif // COMMANDS_H
