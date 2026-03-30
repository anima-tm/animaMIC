/*
 * gestures.h
 *
 * animaMIC — Gesturi: clasă abstractă Gesture, înregistrare automată la construire, acces prin nume.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: un gest = secvență de go_to / then_go_to pe ServoAct; subclase anonime în gestures.cpp se auto-registrează.
 * Fără modificări în anima/commands la adăugarea unui gest nou (constructor Gesture() apelează registerGesture).
 */

#ifndef GESTURES_H
#define GESTURES_H

#include <stdint.h>

/**
 * Gest: mișcare predefinită pe actuatoare; subclase implementează run().
 */
class Gesture {
public:
    virtual ~Gesture() {}

    /** Numele gestului — comanda: anima do <name>. */
    virtual const char* name() const = 0;

    /**
     * Rulează secvența pe coada ServoAct (go_to, then_go_to).
     * @return false dacă gestul refuză (ex. poziții incompatibile — rare)
     */
    virtual bool run() = 0;

    /** Durată estimată în ms (sumă pași); 0 dacă neoverride-at. */
    virtual uint32_t duration() const { return 0; }

protected:
    Gesture();
};

/** Adaugă gest la sfârșitul tabloului static; ignoră dacă plin sau nullptr. */
void registerGesture(Gesture* g);

/** Caută după name(); nullptr dacă nu există. */
Gesture* findGesture(const char* name);

/** Număr de gesturi înregistrate curent. */
unsigned int gestureCount();

/** Gest la index 0 .. gestureCount()-1; nullptr dacă index invalid. */
Gesture* gestureAt(unsigned int index);

#endif /* GESTURES_H */
