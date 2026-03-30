/*
 * Threads.h
 *
 * animaMIC — Module care rulează pe rând în bucla principală și interpretor de comenzi pe serial.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * UTILIZARE (STEM):
 * 1. Creezi o clasă care moștenește Thread, suprascrii begin(), end() și loop().
 * 2. În sketch: Thread::setup() din setup() Arduino, Thread::run() din loop() Arduino.
 * 3. În constructor: Thread(interval_ms). Dacă interval_ms = 0, loop() se apelează la fiecare trecere;
 *    dacă > 0, doar după ce au trecut cel puțin interval_ms milisecunde (execuție periodică).
 *
 * COMENZI NOI:
 * 1. Handler în commands.cpp: bool cmd_nume(const char* arg)
 * 2. arg = restul liniei după numele comenzii; răspuns prin THREADS_SERIAL
 * 3. Pentru numere întregi: getIntArg(arg, index, &valoare) — index de la 0
 * 4. Return true dacă sintaxa e acceptată, false dacă nu (mesajul de eroare e afișat de apelant unde e cazul)
 * 5. În commands[]: {"nume", cmd_nume}
 * 6. Declarație în commands.h
 */

#ifndef THREADS_H
#define THREADS_H

#include <stdint.h>
#include <Arduino.h>

// Portul serial folosit de interpretorul de comenzi (implicit USB Serial)
#ifndef THREADS_SERIAL
#define THREADS_SERIAL Serial
#endif

struct CommandEntry;

/**
 * Modul periodic sau „cât mai des” în bucla principală.
 * Nu există sistem de operare: toate modulele moștenite sunt chemate pe rând (buclă cooperativă) de Thread::run().
 */
class Thread {
public:
    /**
     * Înregistrează modulul în lista globală (inserare la început).
     * @param interval_ms 0 = loop() la fiecare apel run(); >0 = interval minim între apeluri (ms).
     */
    Thread(uint32_t interval_ms = 0);
    /** Distrugere: apelează end(), scoate modulul din listă. */
    virtual ~Thread();

    /** Apelat o dată din Thread::setup() după ce toate obiectele Thread au fost construite. */
    virtual void begin() {}
    /** Apelat la distrugere sau explicit; suprascrie pentru eliberare de resurse. */
    virtual void end() {}
    /** Logică periodică a modulului; suprascrie în subclasă. */
    virtual void loop() {}
    // Dacă folosești YIELD() (cedare control) în loop(), pune YIELD_START() la începutul loop()

    /**
     * Inițializează interpretorul de comenzi și apelează begin() pe fiecare modul înregistrat.
     * Apel unic din setup() Arduino, înainte de loop().
     */
    static void setup();
    /**
     * O iterație a buclei cooperativă: citește linii pe serial (interpretor de comenzi), apoi
     * apelează loop() pe fiecare modul conform intervalului. Apel din loop() Arduino.
     */
    static void run();

protected:
    void* resume_point_ = nullptr;  // adresă de reluare după YIELD() (cedare control din loop)

private:
    uint32_t interval_ms_;   // pausă minimă între două apeluri loop() (0 = fără restricție de timp)
    uint32_t last_run_ms_;   // millis() la ultimul apel reușit al loop()
    Thread* next_;           // următorul modul în lista înlănțuită

    static Thread* first_;   // primul modul din listă
    static void registerThread(Thread* thread);    // adaugă la începutul listei
    static void unregisterThread(Thread* thread);  // elimină din listă
    static void initParser();   // leagă tabelul commands[]; golește bufferul liniei
    static void runParser();    // citește THREADS_SERIAL, construiește linii, le trimite la interpretare
    static void runThreads();   // pentru fiecare modul: dacă a expirat intervalul, apelează loop()
};

// Reluare după cedare control (YIELD): obligatoriu la începutul lui loop() dacă folosești YIELD()
#define YIELD_START() \
    do { \
        if (this->resume_point_) { \
            void* __resume = this->resume_point_; \
            this->resume_point_ = nullptr; \
            goto *__resume; \
        } \
    } while (0)

// Cedare control din loop(); la următorul run() execuția continuă după YIELD()
#define YIELD() \
    do { \
        __label__ __resume_here; \
        this->resume_point_ = &&__resume_here; \
        return; \
        __resume_here:; \
    } while (0)

#endif // THREADS_H
