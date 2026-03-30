/*
 * IRProc.h
 *
 * animaMIC — Procesor IR: flux de procesare (filtrare, urmărire plajă, alipire, scanare) și actori.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 *
 * STEM: la fiecare cadru nou de la IRSense, IRProcessor rulează etapele în ordine, actualizează lista
 * de actori și starea (IDLE / CATCHING_GHOSTS / TRACKING) pentru anima. Actor = obiect detectat,
 * creat în etapa scan; urmărirea țintei folosește actorul ales după regulile TRACKER_*.
 */

#ifndef IRPROC_H
#define IRPROC_H

#include "IRSense.h"
#include "Threads.h"
#include <utility>
#include <vector>

/* Filtru EMA: variantă adaptivă (ADAPTIVE_LPF==1) separă semnalul util de zgomot / fantomă. */
#define ADAPTIVE_LPF 1  // 1 = filter EMA adaptiv; 0 = filter EMA fix

#if ADAPTIVE_LPF == 1
#define IR_LPF_EMA_FASTALPHA 0.8f   // alpha EMA când semnalul e considerat sigur (răspuns rapid)
#define IR_LPF_EMA_SLOWALPHA 0.1f   // alpha EMA când semnalul e incert (zgomot)
#define IR_LPF_EMA_ALPHASTEPS 5     // număr de pași pentru tranziția certitudine 0..1
#define LPF_GHOST_THRESHOLD 150.0f  // prag sub care două ieșiri consecutive cresc certitudinea (posibil real)
#define LPF_REAL_THRESHOLD 250.0f   // prag peste care două ieșiri consecutive pun certitudinea la 1
#define IR_GHOST_RESET_TIME 3000    // ms de zerouri după care certitudinea revine la 1
#else
#define IR_LPF_EMA_ALPHA 0.1f       // alpha unic pentru EMA fix (fără adaptare)
#endif

// #define TELEPLOT_CERT_ON_IRX 1   // trimite certitudinea filterului pe canalul irX la Teleplot
// #define TELEPLOT_INDEXES_ON_ACTOR 1  // trimite aX_min și aX_max la Teleplot

#define ACTOR_DETECTION_THRESHOLD 200.0f  // valoare minimă filtrată (0–1000) pentru a considera senzorul „activ”
#define ACTOR_MAXIMUM_DEPTH 500.0f         // diferența max admisă între min și max într-un range (evită două surse)
#define ACTOR_MIN_SPAN 1                   // număr minim de senzori consecutivi pentru un actor
#define ACTOR_LIFETIME_MS 1000             // ms de la deathtime după care actorul pierdut e șters
#define ACTOR_VICINITY 2                   // număr de senzori în plus în fiecare direcție la căutarea range-ului (TrackMotion)

#define TRACKER_MIN_LIFETIME 1500    // ms min de viață al actorului pentru a fi ales la urmărirea țintei (ochi)
#define TRACKER_MIN_DISTANCE 0.3f    // distanță min (0–1) pentru a începe urmărirea; sub ea se ignoră
#define TRACKER_STARTTIME 2000       // ms după ACTIVE sau reset înainte de a rula urmărirea țintei
#define TRACKER_EMA_ALPHA_POS 0.5f   // alpha pentru netezirea poziției (0–1) în urmărirea țintei
#define TRACKER_EMA_ALPHA_DIST 0.3f  // alpha pentru netezirea distanței (0–1) în urmărirea țintei
#define TRACKER_XPOS_EYES 0.4f       // poziția X care se mapează la centru ochi (0.5 = mijloc)
#define TRACKER_YPOS_MIN (-0.6f)     // oyup.1 la distanță min
#define TRACKER_YPOS_MAX 0.8f        // oyup.1 la distanță 1
#define TRACKER_MIN_REFRESH_POS 0.1f // variație min (0–1) față de ultimul go_to pentru a trimite o nouă comandă ox
#define TRACKER_MIN_REFRESH_DIST 0.1f // variație min pentru oyup.1 (distanță) pentru a trimite o nouă comandă
#define TRACKER_X_SPEED_INVERSE 300  // ms pentru o schimbare de poziție de 1.0 pe os/od (ox); același model ca TRACKER_Y pe oyup.1
#define TRACKER_Y_SPEED_INVERSE 800  // Timp (ms) pentru o schimbare de poziție de 1.0 pe oyup.1; mișcarea full range (-1..1) durează 2*TRACKER_Y_SPEED_INVERSE. 
#define TRACKER_LEFTEXIT_MAX_POSITION  0.25f  // poziție sub care considerăm ieșire la stânga (dacă distanța >= TRACKER_MIN_DISTANCE)
#define TRACKER_RIGHTEXIT_MIN_POSITION 0.75f  // poziție peste care considerăm ieșire la dreapta (dacă distanța >= TRACKER_MIN_DISTANCE)

#ifndef DIAGNOSE_TRACKING
#define DIAGNOSE_TRACKING 1  // 1 = mesaje debug la evenimente (irp: GHOSTS, irp: ACTOR id=1, etc.)
#endif

class IRProcessor;
class Actor;

/** Stări ale procesorului IR (pentru interacțiune cu anima). */
enum class IRProcessorState {
    IDLE,            // niciun actor detectat
    CATCHING_GHOSTS, // cel puțin un actor (viu sau murind), dar niciunul urmărit; pot apărea fantome înainte de țintă stabilă
    TRACKING         // un actor este urmărit
};

/** Evenimente emise la schimbarea stării (pentru interacțiune cu anima). */
enum class IRProcessorEvent {
    NONE,
    GHOSTS,             // IDLE -> CATCHING_GHOSTS
    ACTOR,              // IDLE sau CATCHING_GHOSTS -> TRACKING
    LOST_ACTOR_AWAY,    // era TRACKING, actorul s-a pierdut (nu LEFT/RIGHT)
    LOST_ACTOR_LEFT,    // era TRACKING, actorul a ieșit la stânga
    LOST_ACTOR_RIGHT,   // era TRACKING, actorul a ieșit la dreapta
    SWITCH_ACTOR_AWAY,  // era TRACKING, am trecut la alt actor (cel vechi nu LEFT/RIGHT)
    SWITCH_ACTOR_LEFT,  // era TRACKING, am trecut la alt actor (cel vechi a ieșit la stânga)
    SWITCH_ACTOR_RIGHT  // era TRACKING, am trecut la alt actor (cel vechi a ieșit la dreapta)
};

/**
 * Procesor IR: consumă cadre de la IRSense, rulează etapele fluxului, emite evenimente pentru anima.
 */
class IRProcessor : public Thread {
public:
    /** @param driver sursa de cadre (trebuie în ACTIVE cu senzori validați) */
    explicit IRProcessor(IRSense* driver);
    void begin() override;
    /** Dacă există cadru nou: filtrare → TrackMotion → alipire → scanare → ciclu de viață → stare → Teleplot. */
    void loop() override;
    /**
     * Activează/dezactivează fluxul de procesare IR.
     * Dezactivat: cadrele primite se aruncă (clearNewFrame) ca după reactivare să nu se proceseze date vechi,
     * aliniat cu momentul activării animatorului.
     */
    void setEnabled(bool en);
    bool isEnabled() const { return enabled_; }
    /** Adaugă actor creat de scan (proprietate IRProcessor până la removeActor). */
    void addActor(Actor* a);
    /** Elimină din listă și eliberează obiectul. */
    void removeActor(Actor* a);
    /** Comenzi de vizualizare (ex. teleplot): ir, irf, irN, actors — vezi interfața serială. */
    void setTeleplotIrEnabled(bool on);
    bool getTeleplotIrEnabled() const;
    void setTeleplotIrfEnabled(bool on);
    bool getTeleplotIrfEnabled() const;
    void setTeleplotSensorEnabled(int one_based_index, bool on);
    bool getTeleplotSensorEnabled(int one_based_index) const;
    /** N=0: off. N>0: teleplot a1_min/max .. aN_min/max (-1=no actor, 0=lost, else actual index). */
    void setTeleplotActorsCount(int N);
    int getTeleplotActorsCount() const;

#if TELEPLOT_CERT_ON_IRX
    float getFilterCert(int one_based_index) const;
#endif

    /** Stare curentă (IDLE / CATCHING_GHOSTS / TRACKING). */
    IRProcessorState getState() const { return state_; }
    /** Returnează evenimentul curent și îl consumă (la citire devine NONE). */
    IRProcessorEvent getEvent();
    /** Id-ul actorului urmărit (0 dacă niciunul). Valid când getState() == TRACKING. */
    uint8_t getTrackedActorId() const { return tracked_actor_id_; }
    /** Poziția 0–1 a actorului urmărit. Semnificativ doar când TRACKING. */
    float getTrackedActorPos() const;
    /** Distanța 0–1 a actorului urmărit, sau -1. Semnificativ doar când TRACKING. */
    float getTrackedActorDist() const;
    /** Cauza ieșirii (0=AWAY, 1=LEFT, 2=RIGHT). Semnificativ când state != TRACKING după ce am avut țintă. */
    int getCachedExitCause() const { return cached_exit_cause_; }

    /** true dacă ultimul loop() a procesat un cadru IR nou. Șters de clearHadNewFrame() ca Animator să ruleze o dată per cadru. */
    bool hadNewFrameLastLoop() const { return had_new_frame_last_loop_; }
    void clearHadNewFrame() { had_new_frame_last_loop_ = false; }

private:
    std::vector<Actor*> getActors() const;
    uint8_t getActorId(size_t i) const;
    float getActorDist(size_t i) const;
    size_t getActorIndex(uint8_t id) const;
    Actor* getActorById(uint8_t id) const;
    bool isValidActorRange(const IRFrame* frame, int index_min, int index_max) const;
    /** true dacă actorul mai tânăr pare continuarea aceluiași obiect (alipire permisă în ciuda adâncimii). */
    bool isTakeoverAttempt(Actor* older, Actor* younger, const IRFrame* frame) const;
    /** Pentru LOST/SWITCH: cauza ieșirii actorului (0=AWAY, 1=LEFT, 2=RIGHT). */
    int getExitCause(uint8_t actor_id) const;
    /** Actualizează state_, tracked_actor_id_, last_event_ după fluxul de procesare și ciclul de viață. */
    void updateStateAndEvents();
    void runFilter(const IRFrame* readings, IRFrame* filtered_readings);
    void runTrackMotion(const IRFrame* readings, IRFrame* filtered_readings);
    void runMergeActors(const IRFrame* readings, IRFrame* filtered_readings);
    void runScanScene(const IRFrame* readings, IRFrame* filtered_readings);
    /** Găsește un range valid neacoperit și adaugă un actor; returnează true dacă a adăugat. */
    bool scanSceneFindOne(IRFrame* filtered_readings, std::vector<std::pair<int, int>>& new_ranges_this_frame, std::vector<uint8_t>& ids_this_frame);
    void applyPendingLifecycle();
    void sendTeleplot();
    IRSense* driver_;
    bool enabled_ = false;
    std::vector<Actor*> actors_;
    IRFrame ir_filtered_;
    uint32_t last_teleplot_ms_;
    bool had_new_frame_last_loop_;
    int teleplot_actors_count_;
    IRProcessorState state_ = IRProcessorState::IDLE;
    IRProcessorState last_state_ = IRProcessorState::IDLE;
    uint8_t last_tracked_id_ = 0;
    uint8_t tracked_actor_id_ = 0;
    /** Motiv ieșire (0=AWAY, 1=LEFT, 2=RIGHT) memorat cât timp actorul urmărit e pierdut dar încă în listă. */
    int cached_exit_cause_ = 0;
    float cached_exit_pos_ = 0.0f;
    float cached_exit_dist_ = -1.0f;
    IRProcessorEvent last_event_ = IRProcessorEvent::NONE;
    // Stare filter EMA (fix sau adaptiv)
    std::vector<float> filter_prev_;
#if ADAPTIVE_LPF
    std::vector<float> filter_prev_in_;
    std::vector<float> filter_cert_;
    std::vector<uint32_t> filter_zero_since_ms_;
#endif
};

/**
 * Actor: obiect detectat (id, plajă senzori 1-based, timpi). Creat în etapa scan.
 * deathtime_ != 0 după pierdere temporară (TrackMotion); 0 = plajă încă găsită.
 */
class Actor {
public:
    Actor(uint8_t id, int min_index, int max_index);

    uint8_t get_id() const { return id_; }
    int get_min_index() const { return min_index_; }
    int get_max_index() const { return max_index_; }
    float get_pos(size_t num_sensors) const;
    /** Distanță medie pe [min_index, max_index], fără goluri; normalizată 0–1 (senzori 0–1000). -1 dacă nu e senzor valid. */
    float get_dist(const IRFrame* frame) const;
    uint32_t get_lifetime() const;
    uint32_t get_deathtime() const { return deathtime_; }
    uint32_t get_birthtime() const { return birthtime_; }

    void set_range(int min_index, int max_index);
    void set_deathtime(uint32_t t) { deathtime_ = t; }
    /** Motiv ieșire (0=AWAY, 1=LEFT, 2=RIGHT) la prima pierdere a actorului (se setează deathtime). */
    int get_exit_cause() const { return exit_cause_; }
    void set_exit_cause(int c) { exit_cause_ = c; }
    /** Ultima poziție/distanță cunoscută cât timp plaja era validă; folosit la motiv ieșire când cadrul curent dă dist=-1. */
    float get_last_pos() const { return last_pos_; }
    float get_last_dist() const { return last_dist_; }
    void set_last_pos_dist(float pos, float dist) { last_pos_ = pos; last_dist_ = dist; }
    void mark_for_deletion() { marked_for_deletion_ = true; }
    bool is_marked_for_deletion() const { return marked_for_deletion_; }

private:
    uint8_t id_;
    int min_index_;
    int max_index_;
    uint32_t birthtime_;
    uint32_t deathtime_;
    int exit_cause_ = 0;
    float last_pos_ = 0.5f;
    float last_dist_ = -1.0f;
    bool marked_for_deletion_;
};

extern IRProcessor ir_processor;

#endif // IRPROC_H
