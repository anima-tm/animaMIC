/*
 * IRProc.cpp
 *
 * animaMIC — Implementare procesor IR: etape flux (filtrare, urmărire, alipire, scanare), actori, Teleplot.
 * Licență GPLv3. https://www.gnu.org/licenses/
 * Copyright (C) 2025 anima.tm
 */

#include "IRProc.h"

// --- IRProcessor ---

IRProcessor::IRProcessor(IRSense* driver)
    : Thread(0), driver_(driver), last_teleplot_ms_(0), teleplot_actors_count_(0), had_new_frame_last_loop_(false) {
}

/** Setează masca Teleplot pe citirile brute (canale irN); delegă la IRFrame din driver. */
void IRProcessor::setTeleplotIrEnabled(bool on) {
    size_t n = driver_ ? driver_->getNumSensors() : 0;
    if (driver_) driver_->getReadingsFrame()->setTeleplotAll(n, on);
}

bool IRProcessor::getTeleplotIrEnabled() const {
    return driver_ && driver_->getReadings()->getTeleplotAll(driver_->getNumSensors());
}

/** Setează masca Teleplot pe ieșirea filtrată (irfN). */
void IRProcessor::setTeleplotIrfEnabled(bool on) {
    size_t n = driver_ ? driver_->getNumSensors() : 0;
    ir_filtered_.setTeleplotAll(n, on);
}

bool IRProcessor::getTeleplotIrfEnabled() const {
    return driver_ && ir_filtered_.getTeleplotAll(driver_->getNumSensors());
}

/** Pornește/oprește ambele canale (raw + filtrat) pentru senzorul dat. */
void IRProcessor::setTeleplotSensorEnabled(int one_based_index, bool on) {
    if (driver_) driver_->getReadingsFrame()->setTeleplotBit(one_based_index, on);
    ir_filtered_.setTeleplotBit(one_based_index, on);
}

bool IRProcessor::getTeleplotSensorEnabled(int one_based_index) const {
    return ir_filtered_.getTeleplotBit(one_based_index);
}

/** N=0: fără canale a*_dist/pos. N>0: până la N actori (vezi sendTeleplot). */
void IRProcessor::setTeleplotActorsCount(int N) {
    teleplot_actors_count_ = (N < 0) ? 0 : N;
}

int IRProcessor::getTeleplotActorsCount() const {
    return teleplot_actors_count_;
}

#if TELEPLOT_CERT_ON_IRX
float IRProcessor::getFilterCert(int one_based_index) const {
#if ADAPTIVE_LPF
    if (one_based_index < 1) return 0.0f;
    size_t i = (size_t)one_based_index - 1;
    return (i < filter_cert_.size()) ? filter_cert_[i] : 0.0f;
#else
    (void)one_based_index;
    return 0.0f;
#endif
}
#endif

/** Adaugă pointer în actors_; nu verifică duplicate. */
void IRProcessor::addActor(Actor* a) {
    if (a) actors_.push_back(a);
}

/** Șterge primul match și face delete pe Actor. */
void IRProcessor::removeActor(Actor* a) {
    if (!a) return;
    for (size_t i = 0; i < actors_.size(); i++) {
        if (actors_[i] == a) {
            actors_.erase(actors_.begin() + (ptrdiff_t)i);
            delete a;
            return;
        }
    }
}

std::vector<Actor*> IRProcessor::getActors() const {
    std::vector<Actor*> out;
    for (Actor* a : actors_) {
        if (a && !a->is_marked_for_deletion())
            out.push_back(a);
    }
    return out;
}

uint8_t IRProcessor::getActorId(size_t i) const {
    std::vector<Actor*> actors = getActors();
    if (i >= actors.size()) return 0;
    return actors[i]->get_id();
}

float IRProcessor::getActorDist(size_t i) const {
    std::vector<Actor*> actors = getActors();
    if (i >= actors.size()) return -1.0f;
    return actors[i]->get_dist(&ir_filtered_);
}

size_t IRProcessor::getActorIndex(uint8_t id) const {
    std::vector<Actor*> actors = getActors();
    for (size_t i = 0; i < actors.size(); i++) {
        if (actors[i]->get_id() == id) return i;
    }
    return (size_t)-1;
}

Actor* IRProcessor::getActorById(uint8_t id) const {
    size_t i = getActorIndex(id);
    if (i == (size_t)-1) return nullptr;
    std::vector<Actor*> actors = getActors();
    return actors[i];
}

float IRProcessor::getTrackedActorPos() const {
    if (tracked_actor_id_ == 0) return 0.5f;
    Actor* a = getActorById(tracked_actor_id_);
    if (!a) return 0.5f;
    size_t n = driver_ ? driver_->getNumSensors() : 0;
    return a->get_pos(n);
}

float IRProcessor::getTrackedActorDist() const {
    if (tracked_actor_id_ == 0) return -1.0f;
    Actor* a = getActorById(tracked_actor_id_);
    if (!a) return -1.0f;
    return a->get_dist(&ir_filtered_);
}

/**
 * Actualizează tracked_actor_id_, state_ și last_event_ după ciclul de viață al actorilor.
 * Politica: urmărește actorul până la ștergere din listă, nu doar până la deathtime (vezi comentariu în corp).
 */
void IRProcessor::updateStateAndEvents() {
    std::vector<Actor*> actors = getActors();
    const bool had_tracked = (tracked_actor_id_ != 0);

    /*
     * Track until death, not until lost: păstrăm tracked_actor_id_ atâta timp cât actorul
     * există în listă (nu a fost șters din applyPendingLifecycle). Nu renunțăm la el doar
     * pentru că are deathtime != 0 („pierdut” de TrackMotion). Astfel folosim perioada
     * ACTOR_LIFETIME_MS ca „life expander”: scurtături de semnal nu mai provoacă LOST apoi
     * ACTOR cu același id; renunțăm doar când actorul e efectiv distrus sau când trecem
     * la alt actor (SWITCH).
     */
    bool tracked_still_valid = false;
    if (had_tracked) {
        Actor* a = getActorById(tracked_actor_id_);
        if (a) {
            tracked_still_valid = true;
            if (a->get_deathtime() != 0) {
                cached_exit_cause_ = a->get_exit_cause();
                size_t n = driver_ ? driver_->getNumSensors() : 0;
                cached_exit_pos_ = a->get_pos(n);
                cached_exit_dist_ = a->get_dist(&ir_filtered_);
            } else
                cached_exit_cause_ = 0;
        }
    }

    /* Dacă actorul urmărit nu mai există (a fost șters), alegem cel mai bun actor viu. */
    if (!tracked_still_valid) {
        tracked_actor_id_ = 0;
        uint32_t best_lifetime = 0;
        size_t best_i = (size_t)-1;
        for (size_t i = 0; i < actors.size(); i++) {
            Actor* a = actors[i];
            if (a->get_deathtime() != 0)
                continue;
            if (a->get_lifetime() <= TRACKER_MIN_LIFETIME)
                continue;
            float dist = getActorDist(i);
            if (dist < TRACKER_MIN_DISTANCE || dist < 0.0f)
                continue;
            uint32_t lt = a->get_lifetime();
            if (lt > best_lifetime) {
                best_lifetime = lt;
                best_i = i;
            }
        }
        if (best_i != (size_t)-1)
            tracked_actor_id_ = getActorId(best_i);
    }

    last_state_ = state_;
    if (actors.empty())
        state_ = IRProcessorState::IDLE;
    else if (tracked_actor_id_ != 0)
        state_ = IRProcessorState::TRACKING;
    else
        state_ = IRProcessorState::CATCHING_GHOSTS;

    last_event_ = IRProcessorEvent::NONE;
    if (last_state_ == IRProcessorState::IDLE && state_ == IRProcessorState::CATCHING_GHOSTS)
        last_event_ = IRProcessorEvent::GHOSTS;
    else if ((last_state_ == IRProcessorState::IDLE || last_state_ == IRProcessorState::CATCHING_GHOSTS) && state_ == IRProcessorState::TRACKING)
        last_event_ = IRProcessorEvent::ACTOR;
    else if (last_state_ == IRProcessorState::TRACKING && state_ != IRProcessorState::TRACKING) {
        int cause = cached_exit_cause_;
        last_event_ = (cause == 1) ? IRProcessorEvent::LOST_ACTOR_LEFT : (cause == 2) ? IRProcessorEvent::LOST_ACTOR_RIGHT : IRProcessorEvent::LOST_ACTOR_AWAY;
    }
    else if (last_state_ == IRProcessorState::TRACKING && state_ == IRProcessorState::TRACKING && last_tracked_id_ != tracked_actor_id_) {
        int cause = cached_exit_cause_;
        last_event_ = (cause == 1) ? IRProcessorEvent::SWITCH_ACTOR_LEFT : (cause == 2) ? IRProcessorEvent::SWITCH_ACTOR_RIGHT : IRProcessorEvent::SWITCH_ACTOR_AWAY;
    }

#if DIAGNOSE_TRACKING
    if (last_event_ != IRProcessorEvent::NONE) {
        THREADS_SERIAL.print("irp: ");
        switch (last_event_) {
            case IRProcessorEvent::GHOSTS:            THREADS_SERIAL.println("GHOSTS"); break;
            case IRProcessorEvent::ACTOR:             THREADS_SERIAL.print("ACTOR id="); THREADS_SERIAL.println((int)tracked_actor_id_); break;
            case IRProcessorEvent::LOST_ACTOR_AWAY:   THREADS_SERIAL.print("LOST_ACTOR_AWAY id="); THREADS_SERIAL.println((int)last_tracked_id_); break;
            case IRProcessorEvent::LOST_ACTOR_LEFT:   THREADS_SERIAL.print("LOST_ACTOR_LEFT id="); THREADS_SERIAL.println((int)last_tracked_id_); break;
            case IRProcessorEvent::LOST_ACTOR_RIGHT:  THREADS_SERIAL.print("LOST_ACTOR_RIGHT id="); THREADS_SERIAL.println((int)last_tracked_id_); break;
            case IRProcessorEvent::SWITCH_ACTOR_AWAY:  THREADS_SERIAL.print("SWITCH_ACTOR_AWAY id="); THREADS_SERIAL.print((int)last_tracked_id_); THREADS_SERIAL.print(" -> "); THREADS_SERIAL.println((int)tracked_actor_id_); break;
            case IRProcessorEvent::SWITCH_ACTOR_LEFT:  THREADS_SERIAL.print("SWITCH_ACTOR_LEFT id="); THREADS_SERIAL.print((int)last_tracked_id_); THREADS_SERIAL.print(" -> "); THREADS_SERIAL.println((int)tracked_actor_id_); break;
            case IRProcessorEvent::SWITCH_ACTOR_RIGHT: THREADS_SERIAL.print("SWITCH_ACTOR_RIGHT id="); THREADS_SERIAL.print((int)last_tracked_id_); THREADS_SERIAL.print(" -> "); THREADS_SERIAL.println((int)tracked_actor_id_); break;
            default: break;
        }
    }
#endif

    last_tracked_id_ = tracked_actor_id_;
}

/** Motiv ieșire 0=AWAY, 1=LEFT, 2=RIGHT; folosește last pos/dist dacă cadrul curent nu are distanță validă. */
int IRProcessor::getExitCause(uint8_t actor_id) const {
    Actor* a = getActorById(actor_id);
    if (!a) return 0;
    size_t n = driver_ ? driver_->getNumSensors() : 0;
    float pos = a->get_pos(n);
    float dist = a->get_dist(&ir_filtered_);
    /* Când cadrul curent nu dă distanță validă (pierdut), folosește ultima pos/dist cunoscută pentru ieșire laterală. */
    if (dist < 0.0f && a->get_last_dist() >= 0.0f) {
        pos = a->get_last_pos();
        dist = a->get_last_dist();
    }
    /* Ieșire laterală (LEFT/RIGHT) doar dacă distanța e în plaja de urmărire; altfel AWAY. */
    int cause = 0;
    if (dist >= TRACKER_MIN_DISTANCE && dist >= 0.0f) {
        if (pos < TRACKER_LEFTEXIT_MAX_POSITION) cause = 1;
        else if (pos > TRACKER_RIGHTEXIT_MIN_POSITION) cause = 2;
    }
    return cause;
}

/** Returnează ultimul eveniment și îl consumă (devine NONE). */
IRProcessorEvent IRProcessor::getEvent() {
    IRProcessorEvent e = last_event_;
    last_event_ = IRProcessorEvent::NONE;
    return e;
}

/** Resetează stare, filtre și evenimente la pornire thread. */
void IRProcessor::begin() {
    ir_filtered_.clear();
    state_ = IRProcessorState::IDLE;
    last_state_ = IRProcessorState::IDLE;
    last_tracked_id_ = 0;
    tracked_actor_id_ = 0;
    last_event_ = IRProcessorEvent::NONE;
    filter_prev_.clear();
#if ADAPTIVE_LPF
    filter_prev_in_.clear();
    filter_cert_.clear();
    filter_zero_since_ms_.clear();
#endif
}

void IRProcessor::setEnabled(bool en) {
    enabled_ = en;
    if (!enabled_) {
        // Aruncă orice cadru în așteptare ca să nu se proceseze date vechi după reactivare.
        if (driver_ && driver_->hasNewFrame())
            driver_->clearNewFrame();
    }
}

/**
 * Dacă enabled și există cadru nou: rulează etapele, consumă cadrul, aplică viața actorilor, stare, Teleplot.
 */
void IRProcessor::loop() {
    had_new_frame_last_loop_ = false;
    if (!enabled_) {
        // Cât timp e dezactivat, aruncă cadrele ca prima procesare după activare să pornească curat.
        if (driver_ && driver_->hasNewFrame())
            driver_->clearNewFrame();
        return;
    }
    if (!driver_ || !driver_->hasNewFrame())
        return;
    const IRFrame* readings = driver_->getReadings();
    if (!readings || readings->size() == 0)
        return;
    size_t n = readings->size();
    if (ir_filtered_.size() != n) {
        ir_filtered_.resize(n);
        for (size_t i = 0; i < n; i++) ir_filtered_[i] = 0.0f;
    }
    runFilter(readings, &ir_filtered_);
    runTrackMotion(readings, &ir_filtered_);
    runMergeActors(readings, &ir_filtered_);
    runScanScene(readings, &ir_filtered_);
    had_new_frame_last_loop_ = true;
    driver_->clearNewFrame();
    applyPendingLifecycle();
    updateStateAndEvents();
    sendTeleplot();
}

/** La fiecare IR_SENSE_GET_INTERVAL_MS trimite canalele bifate (>irN, >irfN, opțional actori). */
void IRProcessor::sendTeleplot() {
    if ((uint32_t)(millis() - last_teleplot_ms_) < (uint32_t)IR_SENSE_GET_INTERVAL_MS)
        return;
    const IRFrame* readings = driver_->getReadings();
    if (!readings || readings->size() == 0) return;
    if (ir_filtered_.size() != readings->size()) return;
    last_teleplot_ms_ = millis();

    size_t n = readings->size();
    for (size_t i = 0; i < n; i++) {
        int idx = (int)(i + 1);
        if (readings->getTeleplotBit(idx)) {
            THREADS_SERIAL.print(">ir");
            THREADS_SERIAL.print(idx);
            THREADS_SERIAL.print(":");
            THREADS_SERIAL.print((int)(readings->at(i) + 0.5f));
            THREADS_SERIAL.print("\n");
        }
        if (ir_filtered_.getTeleplotBit(idx)) {
            THREADS_SERIAL.print(">irf");
            THREADS_SERIAL.print(idx);
            THREADS_SERIAL.print(":");
            THREADS_SERIAL.print((int)(ir_filtered_.at(i) + 0.5f));
            THREADS_SERIAL.print("\n");
#if TELEPLOT_CERT_ON_IRX
            THREADS_SERIAL.print(">ir");
            THREADS_SERIAL.print(idx);
            THREADS_SERIAL.print("_cert:");
            THREADS_SERIAL.print(getFilterCert(idx), 4);
            THREADS_SERIAL.print("\n");
#endif
        }
    }
    if (teleplot_actors_count_ > 0) {
        std::vector<Actor*> actors = getActors();
        size_t num_sensors = ir_filtered_.size();
        for (int id = 1; id <= teleplot_actors_count_; id++) {
            Actor* a = nullptr;
            for (Actor* x : actors) {
                if (x->get_id() == (uint8_t)id) { a = x; break; }
            }
            float dist_val = -0.5f;
            float pos_val = -0.5f;
            if (a) {
                pos_val = a->get_pos(num_sensors);
                if (a->get_deathtime() != 0) {
                    dist_val = 0.0f;
                } else {
                    dist_val = a->get_dist(&ir_filtered_);
                }
            }
            THREADS_SERIAL.print(">a");
            THREADS_SERIAL.print(id);
            THREADS_SERIAL.print("_dist:");
            THREADS_SERIAL.print(dist_val);
            THREADS_SERIAL.print("\n");
            THREADS_SERIAL.print(">a");
            THREADS_SERIAL.print(id);
            THREADS_SERIAL.print("_pos:");
            THREADS_SERIAL.print(pos_val);
            THREADS_SERIAL.print("\n");
#if TELEPLOT_INDEXES_ON_ACTOR
            int min_val = -1;
            int max_val = -1;
            if (a) {
                if (a->get_deathtime() != 0) {
                    min_val = 0;
                    max_val = 0;
                } else {
                    min_val = a->get_min_index();
                    max_val = a->get_max_index();
                }
            }
            THREADS_SERIAL.print(">a");
            THREADS_SERIAL.print(id);
            THREADS_SERIAL.print("_min:");
            THREADS_SERIAL.print(min_val);
            THREADS_SERIAL.print("\n");
            THREADS_SERIAL.print(">a");
            THREADS_SERIAL.print(id);
            THREADS_SERIAL.print("_max:");
            THREADS_SERIAL.print(max_val);
            THREADS_SERIAL.print("\n");
#endif
        }
    }
}

/** Șterge actori marcați sau pierduți de mai mult de ACTOR_LIFETIME_MS. */
void IRProcessor::applyPendingLifecycle() {
    std::vector<Actor*> to_remove;
    for (Actor* a : actors_) {
        if (!a) continue;
        uint32_t dt = a->get_deathtime();
        if (dt != 0 && (uint32_t)(millis() - dt) >= (uint32_t)ACTOR_LIFETIME_MS) {
            to_remove.push_back(a);
        } else if (a->is_marked_for_deletion()) {
            to_remove.push_back(a);
        }
    }
    for (Actor* a : to_remove) {
        removeActor(a);
    }
}

// --- Etapa: filtrare (EMA fix sau adaptiv) ---

#if ADAPTIVE_LPF == 0
void IRProcessor::runFilter(const IRFrame* readings, IRFrame* filtered_readings) {
    if (!readings || !filtered_readings) return;
    const size_t n = readings->size();
    if (n == 0) return;
    if (filter_prev_.size() != n) {
        filter_prev_.resize(n);
        for (size_t i = 0; i < n; i++)
            filter_prev_[i] = 0.0f;
    }
    const float alpha = IR_LPF_EMA_ALPHA;
    const float one_minus_alpha = 1.0f - alpha;
    for (size_t i = 0; i < n; i++) {
        float y = alpha * readings->at(i) + one_minus_alpha * filter_prev_[i];
        filter_prev_[i] = y;
        (*filtered_readings)[i] = y;
    }
}
#else
void IRProcessor::runFilter(const IRFrame* readings, IRFrame* filtered_readings) {
    if (!readings || !filtered_readings) return;
    const size_t n = readings->size();
    if (n == 0) return;
    const float cert_increment = 1.0f / (float)IR_LPF_EMA_ALPHASTEPS;
    const float alpha_range = IR_LPF_EMA_FASTALPHA - IR_LPF_EMA_SLOWALPHA;
    if (filter_prev_.size() != n) {
        filter_prev_.resize(n);
        filter_prev_in_.resize(n);
        filter_cert_.resize(n);
        filter_zero_since_ms_.resize(n);
        for (size_t i = 0; i < n; i++) {
            filter_prev_[i] = 0.0f;
            filter_prev_in_[i] = 0.0f;
            filter_cert_[i] = 1.0f;
            filter_zero_since_ms_[i] = 0;
        }
    }
    uint32_t now = millis();
    for (size_t i = 0; i < n; i++) {
        float x = readings->at(i);
        if (filter_prev_in_[i] == 0.0f && x == 0.0f && filter_prev_[i] != 0.0f) {
            filter_prev_[i] = 0.0f;
            (*filtered_readings)[i] = 0.0f;
            filter_cert_[i] -= cert_increment;
            if (filter_cert_[i] < 0.0f) filter_cert_[i] = 0.0f;
            filter_zero_since_ms_[i] = now;
        } else if (filter_prev_in_[i] == 0.0f && x == 0.0f) {
            (*filtered_readings)[i] = 0.0f;
            if (filter_zero_since_ms_[i] == 0)
                filter_zero_since_ms_[i] = now;
            else if ((uint32_t)(now - filter_zero_since_ms_[i]) >= (uint32_t)IR_GHOST_RESET_TIME) {
                filter_cert_[i] = 1.0f;
                filter_zero_since_ms_[i] = 0;
            }
        } else {
            float a = IR_LPF_EMA_SLOWALPHA + filter_cert_[i] * alpha_range;
            float y = a * x + (1.0f - a) * filter_prev_[i];
            if (y > LPF_REAL_THRESHOLD && filter_prev_[i] > LPF_REAL_THRESHOLD)
                filter_cert_[i] = 1.0f;
            else if (y > LPF_GHOST_THRESHOLD && filter_prev_[i] > LPF_GHOST_THRESHOLD) {
                filter_cert_[i] += cert_increment;
                if (filter_cert_[i] > 1.0f) filter_cert_[i] = 1.0f;
            }
            filter_prev_[i] = y;
            (*filtered_readings)[i] = y;
            filter_zero_since_ms_[i] = 0;
        }
        filter_prev_in_[i] = x;
    }
}
#endif

// --- Actor ---

Actor::Actor(uint8_t id, int min_index, int max_index)
    : id_(id), min_index_(min_index), max_index_(max_index), birthtime_(millis()), deathtime_(0), marked_for_deletion_(false) {}

/** Poziție normalizată 0–1 pe axa senzorilor (centru plajă). */
float Actor::get_pos(size_t num_sensors) const {
    if (num_sensors <= 1) return 0.5f;
    float center = (min_index_ + max_index_) * 0.5f;
    float p = (center - 1.0f) / (float)(num_sensors - 1);
    if (p < 0.0f) return 0.0f;
    if (p > 1.0f) return 1.0f;
    return p;
}

float Actor::get_dist(const IRFrame* frame) const {
    if (!frame || (size_t)max_index_ > frame->size()) return -1.0f;
    float sum = 0.0f;
    int count = 0;
    for (int k = min_index_; k <= max_index_; k++) {
        if (k < 1) continue;
        float v = frame->at((size_t)(k - 1));
        if (v >= ACTOR_DETECTION_THRESHOLD) {
            sum += v;
            count++;
        }
    }
    if (count == 0) return -1.0f;
    /* Senzorii sunt 0–1000; normalizare la 0–1 */
    return (sum / (float)count) / 1000.0f;
}

uint32_t Actor::get_lifetime() const {
    return (uint32_t)(millis() - birthtime_);
}

void Actor::set_range(int min_index, int max_index) {
    min_index_ = min_index;
    max_index_ = max_index;
    deathtime_ = 0;
    exit_cause_ = 0;
}

// --- Etapa: urmărire plajă (TrackMotion) ---

static bool rangeOverlapsTaken(int lo, int hi, const std::vector<bool>& taken) {
    if (lo < 1 || hi < lo || (size_t)hi >= taken.size()) return true;
    for (int i = lo; i <= hi; i++) {
        if (taken[(size_t)i]) return true;
    }
    return false;
}

/**
 * Pentru fiecare actor: caută plajă validă în vecinătatea plajei anterioare; dacă nu există, setează deathtime și cauză ieșire.
 */
void IRProcessor::runTrackMotion(const IRFrame* readings, IRFrame* filtered_readings) {
    (void)readings;
    if (!filtered_readings) return;
    const size_t n = filtered_readings->size();
    if (n == 0) return;

    std::vector<bool> taken((size_t)n + 1, false);
    std::vector<Actor*> actors = getActors();
    for (Actor* a : actors) {
        int prev_lo = a->get_min_index();
        int prev_hi = a->get_max_index();
        int lo = prev_lo - (int)ACTOR_VICINITY;
        int hi = prev_hi + (int)ACTOR_VICINITY;
        if (lo < 1) lo = 1;
        if (hi > (int)n) hi = (int)n;
        if (lo > hi) {
            if (a->get_deathtime() == 0) {
                a->set_exit_cause(getExitCause(a->get_id()));
                a->set_deathtime(millis());
            }
            continue;
        }
        const int min_span = (int)ACTOR_MIN_SPAN - 1;
        int best_lo = -1, best_hi = -1;
        for (int W = hi - lo; W >= min_span && best_lo < 0; W--) {
            for (int c_lo = lo; c_lo <= hi - W; c_lo++) {
                int c_hi = c_lo + W;
                if (!rangeOverlapsTaken(c_lo, c_hi, taken) && isValidActorRange(filtered_readings, c_lo, c_hi)) {
                    best_lo = c_lo;
                    best_hi = c_hi;
                    break;
                }
            }
        }
        if (best_lo < 0) {
            if (a->get_deathtime() == 0) {
                a->set_exit_cause(getExitCause(a->get_id()));
                a->set_deathtime(millis());
            }
            continue;
        }
        a->set_range(best_lo, best_hi);
        a->set_last_pos_dist(a->get_pos(n), a->get_dist(filtered_readings));
        for (int i = best_lo; i <= best_hi; i++) taken[(size_t)i] = true;
    }
}

// --- Etapa: alipire actori ---
/*
 * Alipiește actorii care au plaje suprapuse/adiacente și o plajă unitară validă.
 * Prioritate: mai întâi doar alipiri care „reactivează” un actor pierdut (deathtime != 0):
 * dacă actorul mai vechi (older) e pierdut și îl putem uni cu un vecin, îi dăm plaja
 * rezultată prin alipire (set_range resetează deathtime) și ștergem cel mai tânăr. Astfel, când TrackMotion
 * nu găsește temporar o plajă și ScanScene creează un actor nou în aceeași zonă, runMergeActors
 * le alipește și păstrăm actorul vechi (reactivat), nu un id nou. După aceste alipiri, facem și restul
 * alipirilor (orice pereche validă).
 */
void IRProcessor::runMergeActors(const IRFrame* readings, IRFrame* filtered_readings) {
    (void)readings;
    if (!filtered_readings) return;

    auto doMergePass = [this, filtered_readings](bool only_reactivate_lost) {
        std::vector<Actor*> actors = getActors();
        if (actors.size() < 2) return false;
        for (size_t i = 0; i + 1 < actors.size(); i++) {
            for (size_t j = i + 1; j < actors.size(); j++) {
                if (actors[j]->get_min_index() < actors[i]->get_min_index()) {
                    Actor* t = actors[i];
                    actors[i] = actors[j];
                    actors[j] = t;
                }
            }
        }
        for (size_t i = 0; i < actors.size(); i++) {
            Actor* left = actors[i];
            if (left->is_marked_for_deletion()) continue;
            Actor* right = nullptr;
            int best_min = 9999;
            for (size_t j = 0; j < actors.size(); j++) {
                if (j == i) continue;
                Actor* b = actors[j];
                if (b->is_marked_for_deletion()) continue;
                if (b->get_min_index() <= left->get_min_index()) continue;
                if (b->get_min_index() < best_min) {
                    best_min = b->get_min_index();
                    right = b;
                }
            }
            if (!right) continue;
            int merged_lo = left->get_min_index();
            int merged_hi = left->get_max_index();
            if (right->get_min_index() < merged_lo) merged_lo = right->get_min_index();
            if (right->get_max_index() > merged_hi) merged_hi = right->get_max_index();
            Actor* older = (left->get_birthtime() <= right->get_birthtime()) ? left : right;
            Actor* younger = (left->get_birthtime() <= right->get_birthtime()) ? right : left;
            if (!isValidActorRange(filtered_readings, merged_lo, merged_hi)) {
                if (only_reactivate_lost && isTakeoverAttempt(older, younger, filtered_readings)) {
                    older->set_range(younger->get_min_index(), younger->get_max_index());
                    {
                        size_t nn = filtered_readings->size();
                        older->set_last_pos_dist(older->get_pos(nn), older->get_dist(filtered_readings));
                    }
                    younger->mark_for_deletion();
                    return true;
                }
                continue;
            }
            if (only_reactivate_lost && older->get_deathtime() == 0)
                continue;
            older->set_range(merged_lo, merged_hi);
            {
                size_t nn = filtered_readings->size();
                older->set_last_pos_dist(older->get_pos(nn), older->get_dist(filtered_readings));
            }
            younger->mark_for_deletion();
            return true;
        }
        return false;
    };

    while (doMergePass(true)) { }
    while (doMergePass(false)) { }
}

// --- Etapa: scanare scenă ---

/** Repetă scanSceneFindOne până nu mai apar plaje noi neacoperite. */
void IRProcessor::runScanScene(const IRFrame* readings, IRFrame* filtered_readings) {
    (void)readings;
    if (!filtered_readings || filtered_readings->size() == 0) return;
    std::vector<std::pair<int, int>> new_ranges_this_frame;
    std::vector<uint8_t> ids_this_frame;
    while (scanSceneFindOne(filtered_readings, new_ranges_this_frame, ids_this_frame)) { }
}

/* Verifică dacă [lo, hi] are amplitudine (max-min) <= ACTOR_MAXIMUM_DEPTH, eventual excluzând un singur senzor (gap). */
static bool checkDepth(const IRFrame* frame, int lo, int hi);

/** Plajă [index_min, index_max] 1-based: praguri capete, adâncime, span minim. */
bool IRProcessor::isValidActorRange(const IRFrame* frame, int index_min, int index_max) const {
    if (!frame || frame->size() == 0) return false;
    if (index_min < 1 || index_max < index_min || (size_t)index_max > frame->size()) return false;
    if (!checkDepth(frame, index_min, index_max)) return false;
    if ((index_max - index_min) < (int)ACTOR_MIN_SPAN - 1) return false;
    if (frame->at((size_t)(index_min - 1)) < ACTOR_DETECTION_THRESHOLD) return false;
    if (frame->at((size_t)(index_max - 1)) < ACTOR_DETECTION_THRESHOLD) return false;
    return true;
}

/** true dacă actorul tânăr pare același obiect ca cel vechi pierdut (alipire în ciuda adâncimii). */
bool IRProcessor::isTakeoverAttempt(Actor* older, Actor* younger, const IRFrame* frame) const {
    if (!older || !younger || !frame) return false;
    if (older->get_deathtime() == 0) return false;
    uint32_t dt_death = older->get_deathtime();
    uint32_t dt_birth = younger->get_birthtime();
    uint32_t gap = (dt_birth >= dt_death) ? (dt_birth - dt_death) : (dt_death - dt_birth);
    if (gap > (uint32_t)(ACTOR_LIFETIME_MS / 2)) return false;
    float center_old = 0.5f * (float)(older->get_min_index() + older->get_max_index());
    float center_new = 0.5f * (float)(younger->get_min_index() + younger->get_max_index());
    if (fabsf(center_old - center_new) > 2.0f * (float)ACTOR_VICINITY) return false;
    float d_old = older->get_dist(frame);
    float d_new = younger->get_dist(frame);
    if (d_old < 0.0f || d_new < 0.0f) return false;
    if (fabsf(d_old - d_new) > 0.2f) return false;
    return true;
}

static bool checkDepth(const IRFrame* frame, int lo, int hi) {
    float rmin = 1e9f, rmax = -1e9f;
    for (int k = lo; k <= hi; k++) {
        float v = frame->at((size_t)(k - 1));
        if (v < rmin) rmin = v;
        if (v > rmax) rmax = v;
    }
    if (rmax - rmin <= ACTOR_MAXIMUM_DEPTH) return true;
    for (int gap = lo; gap <= hi; gap++) {
        float mn = 1e9f, mx = -1e9f;
        for (int k = lo; k <= hi; k++) {
            if (k == gap) continue;
            float v = frame->at((size_t)(k - 1));
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (mx - mn <= ACTOR_MAXIMUM_DEPTH) return true;
    }
    return false;
}

/**
 * Găsește o plajă liberă validă, alocă id nou, creează Actor și îl adaugă; returnează false dacă nu mai e nimic.
 */
bool IRProcessor::scanSceneFindOne(IRFrame* filtered_readings, std::vector<std::pair<int, int>>& new_ranges_this_frame, std::vector<uint8_t>& ids_this_frame) {
    const size_t n = filtered_readings->size();
    if (n == 0) return false;

    std::vector<bool> available((size_t)n + 1, true);
    for (Actor* a : getActors()) {
        int lo = a->get_min_index(), hi = a->get_max_index();
        for (int i = lo; i <= hi && (size_t)i <= n; i++)
            if (i >= 1) available[(size_t)i] = false;
    }
    for (const auto& r : new_ranges_this_frame) {
        for (int i = r.first; i <= r.second && (size_t)i <= n; i++)
            if (i >= 1) available[(size_t)i] = false;
    }

    for (int candidate_start = 1; candidate_start <= (int)n; candidate_start++) {
        if (!available[(size_t)candidate_start]) continue;
        if (filtered_readings->at((size_t)(candidate_start - 1)) < ACTOR_DETECTION_THRESHOLD) continue;
        bool enough_span = true;
        for (int o = 1; o < (int)ACTOR_MIN_SPAN; o++) {
            int idx = candidate_start + o;
            if (idx > (int)n || !available[(size_t)idx]) { enough_span = false; break; }
        }
        if (!enough_span) continue;

        int min_index = candidate_start;
        int max_index = candidate_start;
        float min_dist = filtered_readings->at((size_t)(min_index - 1));
        float max_dist = min_dist;
        bool gap = false;
        int current = candidate_start + 1;

        while (current <= (int)n && available[(size_t)current]) {
            float v = filtered_readings->at((size_t)(current - 1));
            if (v >= ACTOR_DETECTION_THRESHOLD) {
                gap = false;
                float new_min = (min_dist < v) ? min_dist : v;
                float new_max = (max_dist > v) ? max_dist : v;
                if (checkDepth(filtered_readings, min_index, current)) {
                    max_index = current;
                    min_dist = new_min;
                    max_dist = new_max;
                    current++;
                } else {
                    break;
                }
            } else {
                if (gap) {
                    max_index = current - 2;
                    break;
                }
                gap = true;
                current++;
            }
        }

        if (isValidActorRange(filtered_readings, min_index, max_index)) {
            bool used[256] = {false};
            used[0] = true;
            for (Actor* a : getActors()) used[a->get_id()] = true;
            for (uint8_t x : ids_this_frame) used[x] = true;
            uint8_t id = 0;
            for (int i = 1; i <= 255; i++) {
                if (!used[i]) { id = (uint8_t)i; break; }
            }
            if (id == 0) return false;
            ids_this_frame.push_back(id);
            Actor* actor = new Actor(id, min_index, max_index);
            addActor(actor);
            new_ranges_this_frame.push_back({min_index, max_index});
            return true;
        }

        candidate_start = min_index;
    }
    return false;
}
