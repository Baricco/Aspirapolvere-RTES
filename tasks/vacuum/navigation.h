#ifndef TASK_NAVIGATION_H
#define TASK_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "driver/driver.h"

#define CLOSED_CELLS_MAX 16 

typedef struct {
    
    uint32_t num;                                   // numero di celle chiuse
    uint16_t cells[CLOSED_CELLS_MAX];               // lista circolare con gli indici delle celle chiuse
    uint32_t mission_num;                           // numero della pulizia: quando cambia, la mappatura azzera quello che in una pulizia non si aggiorna più
    uint32_t stop_search_num;                       // cresce quando un tasto viene premuto con la posizione persa: la mappatura annulla la ricerca e aspetta che il robot sia sulla base
} navigation_closed_cells_t;


// in base al tipo di obiettivo, solo alcuni campi della struttura 
// avranno un'effettiva utilità
typedef enum {
    OBJ_STOP,           // nessun campo, questo dev'essere il primo valore dell'enum, così se la struttura è azzerata l'obiettivo è OBJ_STOP
    OBJ_ROTATE,         // solo i campi theta (fino a che angolo ruotare) e omega (velocità massima di rotazione espressa in rad/s)
    OBJ_MOVE,           // omega (velocità massima di rotazione, la rotazione la calcola da x e y); x e y (coordinate dell'obiettivo) e v (velocità massima del movimento, se negativa significa andare all'indietro, espressa in m/s)
    OBJ_CONST_SPEED,    // v (velocità da mantenere) e omega (velocità di rotazione da mantenere)
} objective_type_t;

// struttura dati che verrà mandata al task di movimento
// Tutti i dati di questa struttura saranno in coordinate assolute
// dove l'origine delle coordinate si trova nel punto di partenza
// inziale del robot
typedef struct {

    uint32_t num;               // numero progressivo, id dell'obiettivo
    double v;                   // velocità massima alla quale il robot può andare verso l'obiettivo
    double omega;               // velocità massima alla quale il robot può ruotare 
    double x, y;                // coordinate dell'obiettivo rispetto all'origine
    double theta;               // angolo verso il quale il robot deve girarsi rispetto all'asse x nell'origine
    objective_type_t obj_type;  // obiettivo attuale, indica quali campi di questa struttura dati contano
    bool pass_through;          // serve solo se l'obiettivo è OBJ_MOVE: il punto corrente non è un arrivo, il robot lo attraversa senza fermarsi

} navigation_objective_t;


// Enum degli stati del task di navigazione
// Il task di navigazione è una macchina a stati
// ! I valori nuovi vanno aggiunti in fondo: gli strumenti di prova e l'app usano l'ordine di questo enum
typedef enum {
    NAV_STATE_WAIT,             // attendi che la mappa si riempia
    NAV_STATE_EXPLORE,          // cerca la frontiera più vicina
    NAV_STATE_CLEAN,            // prepara il prossimo tratto da pulire
    NAV_STATE_FOLLOW_BORDERS,   // gira lungo i bordi della stanza (muri, attorno agli ostacoli)
    NAV_STATE_RETURN_HOME,      // cerca il percorso per la base di ricarica
    NAV_STATE_HOOK_UP,          // passa il controllo al driver per agganciare il robot alla base
    NAV_STATE_FOLLOW_PATH,      // segui il percorso calcolato, un punto di svolta alla volta
    NAV_STATE_UNLOCK,           // dopo un allarme, ferma il robot, indietreggia leggermente e riparti
    NAV_STATE_SLEEP,            // robot fermo, missione completata
    NAV_STATE_ALIGN,            // robot in allineamento, dopo un arresto, porta il robot al centro della cella di navigazione
    NAV_STATE_FAILURE,          // robot fermo, qualcosa è andato storto
    NAV_STATE_BORDER_LAP,       // il robot segue un ostacolo tenendolo a destra, fino alla fine del giro
    NAV_STATE_LEAVE_BASE,       // il robot esce dalla base di ricarica
    NAV_STATE_CROSS,            // il robot incastrato su un tappeto o un cavo avanza piano ruotando a turno a sinistra e a destra
    NAV_STATE_PAUSE,            // pulizia in pausa (tasto di accensione): robot fermo, riprende alla prossima pressione
    NAV_STATE_REMOTE,           // modalità telecomando: il robot va alla velocità chiesta dall'app
} nav_state_t;

// Costanti del telecomando: gli stessi valori di crociera dei task, ma definiti a parte
#define REMOTE_MAX_V            0.35            // velocità massima in avanti in telecomando, espressa in m/s
#define REMOTE_MAX_V_BACK       0.35            // velocità massima all'indietro in telecomando, espressa in m/s
#define REMOTE_MAX_OMEGA        1.2             // velocità massima di rotazione in telecomando, espressa in rad/s
#define REMOTE_MOVE_TIMEOUT     RT_MS(500)      // un comando di movimento del telecomando vale per questo tempo, poi il robot si ferma
#define REMOTE_IDLE_TIMEOUT     RT_S(30)        // tempo senza comandi del telecomando dopo cui il telecomando viene rilasciato

// Tipi di comando che la navigazione riceve: dai tasti (che diventano comandi) e dall'app
// ! Al massimo 16 valori: nel comando impacchettato il tipo occupa 4 bit
typedef enum {
    NAV_COMMAND_NONE,               // nessun comando
    NAV_COMMAND_START,              // nuova pulizia
    NAV_COMMAND_PAUSE,              // pausa
    NAV_COMMAND_RESUME,             // ripresa dopo la pausa
    NAV_COMMAND_HOME,               // ritorno alla base
    NAV_COMMAND_REMOTE_TAKE,        // telecomando: lo prende
    NAV_COMMAND_REMOTE_MOVE,        // telecomando: v e omega, valgono per REMOTE_MOVE_TIMEOUT
    NAV_COMMAND_REMOTE_CLEANING,    // telecomando: spazzole e aspirazione
    NAV_COMMAND_REMOTE_KEEPALIVE,   // telecomando: segno di vita dell'app
    NAV_COMMAND_REMOTE_RELEASE,     // telecomando: lo rilascia
} nav_command_type_t;

// Esito di un comando, pubblicato dalla navigazione in navigation_status_t
typedef enum {
    NAV_RESULT_OK,                  // eseguito
    NAV_RESULT_BUSY,                // start: c'è già una missione
    NAV_RESULT_NOT_RUNNING,         // pause: nessuna missione in corso
    NAV_RESULT_NOT_PAUSED,          // resume: il robot non è in pausa
    NAV_RESULT_ALREADY_HOME,        // home: il robot è già a riposo sulla base
    NAV_RESULT_POSITION_LOST,       // la posizione è persa: solo home fa qualcosa
    NAV_RESULT_IN_FAULT,            // il driver è ancora in guasto
} nav_command_result_t;

// Un comando per la navigazione
typedef struct {
    uint16_t num;                   // numero progressivo: un numero diverso dall'ultimo visto vuol dire comando nuovo. 0 = nessun comando
    nav_command_type_t type;
    double v;                       // solo NAV_COMMAND_REMOTE_MOVE: velocità, espressa in m/s
    double omega;                   // solo NAV_COMMAND_REMOTE_MOVE: velocità di rotazione, espressa in rad/s
    bool brushes;                   // solo NAV_COMMAND_REMOTE_CLEANING: spazzole accese
    bool suction;                   // solo NAV_COMMAND_REMOTE_CLEANING: aspirazione accesa
} navigation_command_t;

// Il comando viaggia in una sola variabile atomica da 64 bit, senza doppio buffer:
// bit 0-15 numero, 16-19 tipo, 20-35 v in mm/s, 36-51 omega in mrad/s (con il segno), 52 spazzole, 53 aspirazione
#define NAV_COMMAND_NUM_SHIFT       0
#define NAV_COMMAND_TYPE_SHIFT      16
#define NAV_COMMAND_V_SHIFT         20
#define NAV_COMMAND_OMEGA_SHIFT     36
#define NAV_COMMAND_BRUSHES_SHIFT   52
#define NAV_COMMAND_SUCTION_SHIFT   53
#define NAV_COMMAND_NUM_MASK        0xFFFFu
#define NAV_COMMAND_TYPE_MASK       0xFu
#define NAV_COMMAND_SPEED_MASK      0xFFFFu
#define NAV_COMMAND_SPEED_SCALE     1000.0      // da m/s a mm/s e da rad/s a mrad/s
#define NAV_COMMAND_SPEED_MAX       32767       // valore massimo di v e omega impacchettati (int16)

// funzione helper che porta una velocità in millesimi, limitata a quello che sta in 16 bit con il segno
static inline uint64_t pack_navigation_speed(double speed) {
    double scaled = round(speed * NAV_COMMAND_SPEED_SCALE);
    if (!(scaled > -NAV_COMMAND_SPEED_MAX)) scaled = -NAV_COMMAND_SPEED_MAX;    // anche se speed non è un numero
    if (scaled > NAV_COMMAND_SPEED_MAX) scaled = NAV_COMMAND_SPEED_MAX;
    return (uint64_t)(uint16_t)(int16_t)scaled;
}

// funzione helper che impacchetta un comando nei 64 bit
static inline uint64_t pack_navigation_command(const navigation_command_t *command) {
    return ((uint64_t)command->num << NAV_COMMAND_NUM_SHIFT) |
           ((uint64_t)(command->type & NAV_COMMAND_TYPE_MASK) << NAV_COMMAND_TYPE_SHIFT) |
           (pack_navigation_speed(command->v) << NAV_COMMAND_V_SHIFT) |
           (pack_navigation_speed(command->omega) << NAV_COMMAND_OMEGA_SHIFT) |
           ((uint64_t)command->brushes << NAV_COMMAND_BRUSHES_SHIFT) |
           ((uint64_t)command->suction << NAV_COMMAND_SUCTION_SHIFT);
}

// funzione helper che fa l'operazione inversa di pack_navigation_command
static inline navigation_command_t unpack_navigation_command(uint64_t packed) {
    navigation_command_t command = {
        .num = (uint16_t)((packed >> NAV_COMMAND_NUM_SHIFT) & NAV_COMMAND_NUM_MASK),
        .type = (nav_command_type_t)((packed >> NAV_COMMAND_TYPE_SHIFT) & NAV_COMMAND_TYPE_MASK),
        .v = (int16_t)((packed >> NAV_COMMAND_V_SHIFT) & NAV_COMMAND_SPEED_MASK) / NAV_COMMAND_SPEED_SCALE,
        .omega = (int16_t)((packed >> NAV_COMMAND_OMEGA_SHIFT) & NAV_COMMAND_SPEED_MASK) / NAV_COMMAND_SPEED_SCALE,
        .brushes = (packed >> NAV_COMMAND_BRUSHES_SHIFT) & 1,
        .suction = (packed >> NAV_COMMAND_SUCTION_SHIFT) & 1,
    };
    return command;
}

// Stato della navigazione pubblicato a ogni job: lo leggono la sicurezza (is_remote) e i thread dell'app
typedef struct {
    nav_state_t state;                          // stato corrente
    nav_state_t mission;                        // fase della missione in corso, NAV_STATE_SLEEP se non c'è una missione
    nav_state_t interrupted_mission;            // missione interrotta per la batteria scarica, NAV_STATE_WAIT se non c'è
    uint32_t mission_number;                    // numero della pulizia: cresce a ogni pulizia nuova
    int64_t mission_start_t;                    // istante di inizio della missione in corso
    bool is_remote;                             // true se il telecomando è attivo (anche durante un guasto)
    bool remote_brushes;                        // telecomando: spazzole chieste dall'app
    bool remote_suction;                        // telecomando: aspirazione chiesta dall'app
    uint16_t last_command_num;                  // numero dell'ultimo comando dell'app eseguito
    nav_command_result_t last_command_result;   // e il suo esito
} navigation_status_t;

// caratteristiche del task navigation
extern rt_task_t navigation_task;

void init_navigation(void);
navigation_objective_t read_navigation(void);
// funzione per leggere le celle che sono state chiuse dalla navigazione (lo leggerà il task mapping)
navigation_closed_cells_t read_closed_cells(void);
// funzione per leggere lo stato della navigazione (lo leggeranno il task safety e i thread dell'app)
navigation_status_t read_navigation_status(void);
// funzione per mandare un comando alla navigazione: la chiama solo l'interprete dei comandi dell'app, un comando
// alla volta (aspetta l'esito in navigation_status_t prima di mandare il successivo). Non aspetta mai
void send_navigation_command(const navigation_command_t *command);
// funzione per pubblicare il prossimo obiettivo (lo leggerà task movement)
void write_objective(const navigation_objective_t *next_objective);
void job_navigation(void *arg);



#endif
