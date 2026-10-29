/*
 * funzioni-pulizia.c - Prova delle funzioni della pulizia di tasks/vacuum/navigation.c, senza simulatore.
 * Include navigation.c così com'è (le funzioni static si possono chiamare) e lo collega con gli altri task, senza
 * main.c. Il codice in tasks/ non viene toccato. Si lancia con tools/prove-nav/funzioni.sh.
 */
#include <stdio.h>
#include "navigation.c"

static int fallite = 0, fatte = 0;
static void controlla(const char *cosa, int ottenuto, int atteso) {
    fatte++;
    if (ottenuto != atteso) { fallite++; printf("FALLITA: %s: ottenuto %d, atteso %d\n", cosa, ottenuto, atteso); }
}
static int pulite(uint16_t c) { return is_half_cell_clean(c, 0) | is_half_cell_clean(c, 1) << 1; }
/* segna con il robot nella posizione (x, y) partendo dalla maschera vuota e restituisce le metà pulite (bit 0, bit 1) */
static int segna(double x, double y) {
    memset(nav_state.clean_cell_mask, 0, sizeof nav_state.clean_cell_mask);
    movement_position_data_t p = {0}; p.x = x; p.y = y; p.valid = true;
    set_robot_cell_clean(p);
    return pulite(coord_to_nav_index(x, y));
}

/* ---- passo 1: maschera, corsie, segno, pulizia attiva ---- */
static void prova_corsie(void) {
    memset(nav_state.clean_cell_mask, 0, sizeof nav_state.clean_cell_mask);
    set_half_cell_clean(1000, 1); set_half_cell_clean(65535, 0);
    controlla("maschera 1000/1", is_half_cell_clean(1000, 1), 1);
    controlla("maschera 1000/0", is_half_cell_clean(1000, 0), 0);
    controlla("maschera 999/1 e 1001/0", is_half_cell_clean(999, 1) | is_half_cell_clean(1001, 0), 0);
    controlla("maschera 65535/0 (ultima cella)", is_half_cell_clean(65535, 0), 1);
    controlla("maschera 65535/1", is_half_cell_clean(65535, 1), 0);
    int accesi = 0; for (size_t i = 0; i < sizeof nav_state.clean_cell_mask; i++) accesi += __builtin_popcount(nav_state.clean_cell_mask[i]);
    controlla("bit accesi in tutta la maschera", accesi, 2);

    uint16_t c = coord_to_nav_index(0.1, 0.1);      /* cella (128,128): da 0 a 0,2 m, centro (0,1; 0,1) */
    memset(nav_state.map, NAV_CELL_FREE, sizeof nav_state.map);
    nav_state.strip_axis_x = false;                 /* strisce lungo y: asse laterale x */
    controlla("spostamento laterale, strisce lungo y", calc_clean_lateral_shift(), 1);
    controlla("corsia 0, vicine libere (mm)", (int)lround(calc_clean_lane_shift(c, 0) * 1000), -50);
    controlla("corsia 1, vicine libere (mm)", (int)lround(calc_clean_lane_shift(c, 1) * 1000), 50);
    controlla("robot sulla corsia 0", segna(0.05, 0.13), 1);
    controlla("robot sulla corsia 1", segna(0.15, 0.07), 2);
    controlla("robot al centro", segna(0.10, 0.10), 0);
    controlla("robot a 2 cm dalla corsia 0", segna(0.07, 0.10), 1);
    controlla("robot a 2,6 cm dalla corsia 0", segna(0.024, 0.10), 0);
    controlla("lo spostamento in y non conta", segna(0.05, 0.19), 1);
    nav_state.map[c - 1] = NAV_CELL_BLOCKED;
    controlla("corsia 0 con la vicina bloccata (mm)", (int)lround(calc_clean_lane_shift(c, 0) * 1000), 0);
    controlla("robot al centro, vicina 0 bloccata", segna(0.10, 0.10), 1);
    controlla("robot sulla corsia 1, vicina 0 bloccata", segna(0.15, 0.10), 2);
    nav_state.map[c + 1] = NAV_CELL_UNKNOWN;
    controlla("robot al centro, vicine non libere", segna(0.11, 0.10), 3);
    memset(nav_state.map, NAV_CELL_FREE, sizeof nav_state.map);

    nav_state.strip_axis_x = true;                  /* strisce lungo x: asse laterale y */
    controlla("spostamento laterale, strisce lungo x", calc_clean_lateral_shift(), NAV_MAP_CELL_NUMBER);
    controlla("asse x: robot sulla corsia 0 (y bassa)", segna(0.13, 0.05), 1);
    controlla("asse x: robot sulla corsia 1 (y alta)", segna(0.07, 0.15), 2);
    controlla("asse x: lo spostamento in x non conta", segna(0.19, 0.05), 1);
    nav_state.map[c - NAV_MAP_CELL_NUMBER] = NAV_CELL_BLOCKED;
    controlla("asse x: vicina a y bassa bloccata, robot al centro", segna(0.10, 0.10), 1);
    nav_state.map[c - 1] = NAV_CELL_BLOCKED;
    controlla("asse x: la vicina lungo x non conta", (int)lround(calc_clean_lane_shift(c, 1) * 1000), 50);
    memset(nav_state.map, NAV_CELL_FREE, sizeof nav_state.map);

    nav_state.strip_axis_x = false;                 /* cella (127,127), da -0,2 a 0, centro (-0,1; -0,1) */
    controlla("coordinate negative, corsia 0", segna(-0.15, -0.10), 1);
    controlla("coordinate negative, corsia 1", segna(-0.05, -0.12), 2);

    nav_state.mission = NAV_STATE_CLEAN; nav_state.state = NAV_STATE_CLEAN; nav_state.is_axis = false;
    controlla("pulizia senza asse calcolato: spenta", is_active_cleaning(), 0);
    nav_state.is_axis = true;
    nav_state_t attivi[] = { NAV_STATE_CLEAN, NAV_STATE_FOLLOW_PATH, NAV_STATE_ALIGN };
    nav_state_t spenti[] = { NAV_STATE_UNLOCK, NAV_STATE_FAILURE, NAV_STATE_WAIT, NAV_STATE_SLEEP };
    for (unsigned i = 0; i < 3; i++) { nav_state.state = attivi[i]; controlla("pulizia attiva", is_active_cleaning(), 1); }
    for (unsigned i = 0; i < 4; i++) { nav_state.state = spenti[i]; controlla("pulizia spenta", is_active_cleaning(), 0); }
    nav_state.mission = NAV_STATE_EXPLORE; nav_state.state = NAV_STATE_FOLLOW_PATH;
    controlla("esplorazione: pulizia spenta", is_active_cleaning(), 0);
}

/* ---- passo 2: la ricerca della cella non pulita ---- */
static uint16_t I(int x, int y) { return (uint16_t)(y << 8 | x); }
static void stanza(void) {      /* stanza libera da (120,120) a (139,139), tutta pulita; fuori bloccato */
    memset(nav_state.map, NAV_CELL_BLOCKED, sizeof nav_state.map);
    memset(nav_state.clean_cell_mask, 0, sizeof nav_state.clean_cell_mask);
    for (int y = 120; y < 140; y++) for (int x = 120; x < 140; x++) {
        nav_state.map[I(x, y)] = NAV_CELL_FREE; set_half_cell_clean(I(x, y), 0); set_half_cell_clean(I(x, y), 1);
    }
}
static void sporca(int x, int y) { nav_state.clean_cell_mask[I(x, y) / 4] &= (uint8_t)~(3u << (2 * (I(x, y) % 4))); }
/* il percorso parte dal robot, va fra celle vicine e in mezzo passa solo da celle libere e pulite */
static void controlla_percorso(const char *cosa, uint16_t start, uint32_t len) {
    char m[200]; uint16_t *p = nav_state.path.cells;
    snprintf(m, sizeof m, "%s: percorso parte dal robot", cosa); controlla(m, p[0] == start, 1);
    int ok_passi = 1, ok_pulite = 1;
    for (uint32_t i = 1; i < len; i++) {
        int dx = abs((p[i] & 0xFF) - (p[i-1] & 0xFF)), dy = abs((p[i] >> 8) - (p[i-1] >> 8));
        if (dx + dy != 1) ok_passi = 0;
        if (i < len - 1 && !(nav_state.map[p[i]] == NAV_CELL_FREE && is_half_cell_clean(p[i], 0) && is_half_cell_clean(p[i], 1))) ok_pulite = 0;
    }
    snprintf(m, sizeof m, "%s: passi fra celle vicine", cosa); controlla(m, ok_passi, 1);
    snprintf(m, sizeof m, "%s: in mezzo solo celle libere e pulite", cosa); controlla(m, ok_pulite, 1);
}
static void prova_ricerca(void) {
    uint16_t r = I(125, 125); uint32_t len;

    stanza(); sporca(130, 125); sporca(125, 128);   /* la più vicina fra due celle sporche */
    len = search_cell(r, GOAL_DIRTY, 0);
    controlla("vicina: lunghezza", (int)len, 4);
    controlla("vicina: meta (125,128)", len && nav_state.path.cells[len - 1] == I(125, 128), 1);
    controlla_percorso("vicina", r, len);

    /* lungo il percorso vero, non in linea d'aria: muro in x=127 da y=120 a 134 */
    stanza(); for (int y = 120; y <= 134; y++) nav_state.map[I(127, y)] = NAV_CELL_BLOCKED;
    sporca(129, 125);           /* 4 celle in linea d'aria, 24 passi attorno al muro */
    sporca(125, 132);           /* 7 celle in linea d'aria, 7 passi */
    len = search_cell(r, GOAL_DIRTY, 0);
    controlla("muro: meta (125,132)", len && nav_state.path.cells[len - 1] == I(125, 132), 1);
    controlla("muro: lunghezza", (int)len, 8);
    set_half_cell_clean(I(125, 132), 0); set_half_cell_clean(I(125, 132), 1);
    len = search_cell(r, GOAL_DIRTY, 0);
    controlla("muro, solo dietro: meta (129,125)", len && nav_state.path.cells[len - 1] == I(129, 125), 1);
    controlla("muro, solo dietro: lunghezza", (int)len, 25);
    controlla_percorso("muro, solo dietro", r, len);

    for (int h = 0; h < 2; h++) {                   /* una sola metà pulita: è ancora una meta */
        stanza(); sporca(126, 125); set_half_cell_clean(I(126, 125), h);
        len = search_cell(r, GOAL_DIRTY, 0);
        controlla(h ? "metà 1 pulita, 0 sporca: è una meta" : "metà 0 pulita, 1 sporca: è una meta",
                  len == 2 && nav_state.path.cells[1] == I(126, 125), 1);
    }

    stanza();                                       /* tutto pulito (le bloccate hanno la maschera a zero) */
    controlla("tutto pulito: lunghezza 0", (int)search_cell(r, GOAL_DIRTY, 0), 0);

    stanza(); sporca(135, 135);                     /* sporca chiusa da celle bloccate */
    nav_state.map[I(134, 135)] = nav_state.map[I(136, 135)] = NAV_CELL_BLOCKED;
    nav_state.map[I(135, 134)] = nav_state.map[I(135, 136)] = NAV_CELL_BLOCKED;
    controlla("sporca irraggiungibile: lunghezza 0", (int)search_cell(r, GOAL_DIRTY, 0), 0);

    stanza();                                       /* troppo stretta o sconosciuta: non è una meta */
    sporca(126, 125); nav_state.map[I(126, 125)] = NAV_CELL_TOO_NARROW;
    sporca(124, 125); nav_state.map[I(124, 125)] = NAV_CELL_UNKNOWN;
    controlla("troppo stretta e sconosciuta: nessuna meta", (int)search_cell(r, GOAL_DIRTY, 0), 0);
}

/* ---- passo 3, parte 2: l'asse delle strisce dalla lista dell'ultima ricerca dell'esplorazione ---- */
/* rettangolo libero da (x0,y0) di w x h celle, senza celle sconosciute, in una mappa tutta bloccata (se pulisci) */
static void rettangolo(int pulisci, int x0, int y0, int w, int h) {
    if (pulisci) memset(nav_state.map, NAV_CELL_BLOCKED, sizeof nav_state.map);
    for (int y = y0; y < y0 + h; y++) for (int x = x0; x < x0 + w; x++) nav_state.map[I(x, y)] = NAV_CELL_FREE;
}
/* come alla fine dell'esplorazione: la ricerca della frontiera non trova niente, poi l'asse */
static int asse_x_dopo_esplorazione(const char *cosa, uint16_t robot) {
    char m[200];
    nav_state.is_axis = false; nav_state.strip_axis_x = false;
    snprintf(m, sizeof m, "%s: nessuna frontiera", cosa); controlla(m, (int)search_cell(robot, GOAL_BORDER, 0), 0);
    calc_strip_axis();
    snprintf(m, sizeof m, "%s: asse segnato come calcolato", cosa); controlla(m, nav_state.is_axis, 1);
    return nav_state.strip_axis_x;
}
static void prova_asse(void) {
    rettangolo(1, 120, 120, 25, 15);                /* 5 x 3 m, lunga in x */
    controlla("stanza lunga in x: strisce lungo x", asse_x_dopo_esplorazione("lunga in x", I(125, 125)), 1);
    rettangolo(1, 120, 120, 15, 25);                /* lunga in y */
    controlla("stanza lunga in y: strisce lungo y", asse_x_dopo_esplorazione("lunga in y", I(125, 125)), 0);
    rettangolo(1, 120, 120, 20, 20);                /* quadrata: a pari numero resta y */
    controlla("stanza quadrata: strisce lungo y", asse_x_dopo_esplorazione("quadrata", I(125, 125)), 0);
    /* stanza lunga in x raggiungibile + isola lunga in y irraggiungibile: sull'intera mappa vincerebbe y
       (x: 15 + 40 = 55 partenze, y: 25 + 3 = 28), sulla lista conta solo la stanza */
    rettangolo(1, 120, 120, 25, 15); rettangolo(0, 160, 100, 3, 40);
    controlla("isola irraggiungibile: non conta", asse_x_dopo_esplorazione("isola", I(125, 125)), 1);
    /* stanza a L: braccio lungo in x (30 x 6) e braccio corto in y (6 x 12): x 6+12=18 partenze, y 30+... */
    rettangolo(1, 100, 100, 30, 6); rettangolo(0, 100, 106, 6, 12);
    controlla("stanza a L lunga in x: strisce lungo x", asse_x_dopo_esplorazione("a L", I(102, 102)), 1);
    /* la cella di partenza bloccata (uscita da una zona bloccata) non deve rompere il conteggio */
    rettangolo(1, 120, 120, 25, 15); nav_state.map[I(125, 125)] = NAV_CELL_BLOCKED;
    controlla("partenza bloccata: strisce lungo x", asse_x_dopo_esplorazione("partenza bloccata", I(125, 125)), 1);
}

/* ---- passo 3, gruppo A: spostamenti, mezza cella libera e sporca, metà del robot, vicine ---- */
static int vicina(int x, int y, int h, clean_dir_t d, int *vx, int *vy) {   /* restituisce la metà, scrive la cella */
    uint16_t c; bool m; get_half_cell_neighbor(I(x, y), h, d, &c, &m);
    *vx = c & 0xFF; *vy = c >> 8; return m;
}
static void controlla_vicina(const char *cosa, int x, int y, int h, clean_dir_t d, int ex, int ey, int eh) {
    int vx, vy, vh = vicina(x, y, h, d, &vx, &vy); char m[200];
    snprintf(m, sizeof m, "%s: (%d,%d) metà %d invece di (%d,%d) metà %d", cosa, vx, vy, vh, ex, ey, eh);
    controlla(m, vx == ex && vy == ey && vh == eh, 1);
}
static int meta_robot(double x, double y) {
    movement_position_data_t p = {0}; p.x = x; p.y = y; p.valid = true;
    return get_robot_half_cell(p, coord_to_nav_index(x, y));
}
static void prova_gruppo_a(void) {
    nav_state.strip_axis_x = false;
    controlla("lungo le strisce, strisce lungo y", calc_clean_along_shift(), NAV_MAP_CELL_NUMBER);
    nav_state.strip_axis_x = true;
    controlla("lungo le strisce, strisce lungo x", calc_clean_along_shift(), 1);

    stanza(); sporca(125, 125); set_half_cell_clean(I(125, 125), 1);
    controlla("libera e sporca: metà 0 sporca", is_half_cell_free_dirty(I(125, 125), 0), 1);
    controlla("libera e sporca: metà 1 pulita", is_half_cell_free_dirty(I(125, 125), 1), 0);
    controlla("libera e sporca: cella bloccata (maschera a zero)", is_half_cell_free_dirty(I(10, 10), 0), 0);
    nav_state.map[I(125, 125)] = NAV_CELL_TOO_NARROW;
    controlla("libera e sporca: troppo stretta", is_half_cell_free_dirty(I(125, 125), 0), 0);

    /* strisce lungo y: nord = y+1, est = x+1 */
    nav_state.strip_axis_x = false;
    controlla_vicina("y, nord", 125, 125, 0, CLEAN_DIR_NORTH, 125, 126, 0);
    controlla_vicina("y, sud", 125, 125, 1, CLEAN_DIR_SOUTH, 125, 124, 1);
    controlla_vicina("y, est dalla metà 0", 125, 125, 0, CLEAN_DIR_EAST, 125, 125, 1);
    controlla_vicina("y, est dalla metà 1", 125, 125, 1, CLEAN_DIR_EAST, 126, 125, 0);
    controlla_vicina("y, ovest dalla metà 1", 125, 125, 1, CLEAN_DIR_WEST, 125, 125, 0);
    controlla_vicina("y, ovest dalla metà 0", 125, 125, 0, CLEAN_DIR_WEST, 124, 125, 1);
    /* strisce lungo x: nord = x+1, est = y+1 */
    nav_state.strip_axis_x = true;
    controlla_vicina("x, nord", 125, 125, 1, CLEAN_DIR_NORTH, 126, 125, 1);
    controlla_vicina("x, sud", 125, 125, 0, CLEAN_DIR_SOUTH, 124, 125, 0);
    controlla_vicina("x, est dalla metà 0", 125, 125, 0, CLEAN_DIR_EAST, 125, 125, 1);
    controlla_vicina("x, est dalla metà 1", 125, 125, 1, CLEAN_DIR_EAST, 125, 126, 0);
    controlla_vicina("x, ovest dalla metà 1", 125, 125, 1, CLEAN_DIR_WEST, 125, 125, 0);
    controlla_vicina("x, ovest dalla metà 0", 125, 125, 0, CLEAN_DIR_WEST, 125, 124, 1);
    /* quattro passi a est e quattro a ovest tornano al punto di partenza */
    int x = 125, y = 125, h = 0, vx, vy;
    for (int k = 0; k < 4; k++) { h = vicina(x, y, h, CLEAN_DIR_EAST, &vx, &vy); x = vx; y = vy; }
    controlla("x, quattro passi a est: due celle più in là, metà 0", x == 125 && y == 127 && h == 0, 1);
    for (int k = 0; k < 4; k++) { h = vicina(x, y, h, CLEAN_DIR_WEST, &vx, &vy); x = vx; y = vy; }
    controlla("x, e quattro a ovest: di nuovo al punto di partenza", x == 125 && y == 125 && h == 0, 1);

    /* metà del robot: cella (128,128), centro (0,1; 0,1), strisce lungo y (asse laterale x) */
    memset(nav_state.map, NAV_CELL_FREE, sizeof nav_state.map);
    nav_state.strip_axis_x = false;
    controlla("metà del robot: sulla corsia 1", meta_robot(0.14, 0.10), 1);
    controlla("metà del robot: sulla corsia 0", meta_robot(0.07, 0.10), 0);
    controlla("metà del robot: al centro, vince la 0", meta_robot(0.10, 0.10), 0);
    controlla("metà del robot: appena a destra del centro", meta_robot(0.101, 0.10), 1);
    nav_state.map[I(127, 128)] = NAV_CELL_BLOCKED;      /* corsie a 0 e +0,05 */
    controlla("metà del robot: corsia 0 al centro, robot a +1 cm", meta_robot(0.11, 0.10), 0);
    controlla("metà del robot: corsia 0 al centro, robot a +4 cm", meta_robot(0.14, 0.10), 1);
    nav_state.strip_axis_x = true;                       /* asse laterale y: conta la y */
    memset(nav_state.map, NAV_CELL_FREE, sizeof nav_state.map);
    controlla("metà del robot, asse x: conta la y", meta_robot(0.06, 0.14), 1);
    nav_state.strip_axis_x = false;
}

/* ---- passo 3, gruppo B: la striscia ---- */
static void sporca_meta(int x, int y, int h) {
    uint32_t b = 2u * I(x, y) + (unsigned)h;
    nav_state.clean_cell_mask[b / 8] &= (uint8_t)~(1u << (b % 8));
}
/* controlla lunghezza restituita, lunghezza salvata, prima e ultima cella e che le celle siano in fila */
static void controlla_striscia(const char *cosa, int x, int y, int h, uint32_t elen, int ex, int ey) {
    char m[200]; uint32_t len = plan_strip(I(x, y), h); uint16_t *p = nav_state.path.cells;
    snprintf(m, sizeof m, "%s: lunghezza", cosa); controlla(m, (int)len, (int)elen);
    snprintf(m, sizeof m, "%s: lunghezza salvata nel percorso", cosa); controlla(m, (int)nav_state.path.length, (int)elen);
    snprintf(m, sizeof m, "%s: prima cella", cosa); controlla(m, p[0] == I(x, y), 1);
    snprintf(m, sizeof m, "%s: ultima cella (%d,%d)", cosa, ex, ey); controlla(m, len && p[len - 1] == I(ex, ey), 1);
    int fila = 1;
    for (uint32_t i = 1; i < len; i++) if (abs((p[i] & 0xFF) - (p[i-1] & 0xFF)) + abs((p[i] >> 8) - (p[i-1] >> 8)) != 1) fila = 0;
    snprintf(m, sizeof m, "%s: celle una dopo l'altra", cosa); controlla(m, fila, 1);
}
static void prova_gruppo_b(void) {
    nav_state.strip_axis_x = false;                 /* strisce lungo y: nord = y+1 */
    stanza(); for (int y = 122; y <= 130; y++) sporca_meta(125, y, 0);
    controlla_striscia("verso nord fino alla metà pulita", 125, 125, 0, 6, 125, 130);
    controlla("la metà 1, pulita, non c'entra", is_half_cell_free_dirty(I(125, 126), 1), 0);

    stanza(); for (int y = 122; y <= 128; y++) sporca_meta(125, y, 0);
    controlla_striscia("nord preferito a sud", 125, 125, 0, 4, 125, 128);

    stanza(); for (int y = 120; y <= 125; y++) sporca_meta(125, y, 1);
    controlla_striscia("sud quando a nord è pulito", 125, 125, 1, 6, 125, 120);

    stanza(); for (int y = 125; y <= 139; y++) sporca_meta(125, y, 0);
    controlla_striscia("fino al muro", 125, 125, 0, 15, 125, 139);

    stanza(); for (int y = 125; y <= 135; y++) sporca_meta(125, y, 0);
    nav_state.map[I(124, 130)] = NAV_CELL_BLOCKED;  /* da y=130 la corsia della metà 0 va al centro */
    controlla_striscia("si ferma dove la corsia cambia scostamento", 125, 125, 0, 5, 125, 129);
    controlla_striscia("la striscia dopo parte sulla corsia al centro", 125, 130, 0, 1, 125, 130);

    stanza(); sporca_meta(125, 125, 0);
    controlla_striscia("una cella sola", 125, 125, 0, 1, 125, 125);

    stanza(); sporca_meta(125, 125, 0); nav_state.map[I(125, 126)] = NAV_CELL_TOO_NARROW; sporca_meta(125, 124, 1);
    controlla_striscia("vicine non libere o sull'altra metà: una cella", 125, 125, 0, 1, 125, 125);

    nav_state.strip_axis_x = true;                  /* strisce lungo x: nord = x+1 */
    stanza(); for (int x = 125; x <= 133; x++) sporca_meta(x, 125, 0);
    controlla_striscia("strisce lungo x, verso nord (x crescente)", 125, 125, 0, 9, 133, 125);
    nav_state.strip_axis_x = false;
}

int main(void) {
    prova_corsie();
    prova_ricerca();
    prova_asse();
    prova_gruppo_a();
    prova_gruppo_b();
    printf("%d controlli, %d falliti\n", fatte, fallite);
    return fallite != 0;
}
