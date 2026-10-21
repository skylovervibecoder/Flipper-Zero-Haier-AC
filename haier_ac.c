/*
 * Haier AC - telecomando IR completo per Flipper Zero (Momentum)
 * Climatizzatore: Haier AS35RBAHRA-3, telecomando YR-HE2
 *
 * Protocollo ricavato dalle catture: frame da 176 bit (22 byte), 38 kHz,
 * 2 checksum a somma (byte 13 = somma byte 0..12, byte 21 = somma byte 14..20).
 * Ogni trasmissione contiene lo STATO COMPLETO, come il telecomando originale.
 * Il segnale viene inviato SOLO quando si preme OK.
 *
 * Campi (tutti verificati su catture reali):
 *   byte 1  : (temp-16)<<4 | posizione alette verticali (0 se spento):
 *             pos1=A, pos2=8, pos3=6, pos4=4, pos5=2, auto=C  (= 2*(6-pos))
 *   byte 3  : 0x02 Health
 *   byte 4  : 0x40 acceso, 0x00 spento
 *   byte 5  : ventola auto A0 / low 60 / med 40 / high 20, |0x08 se Sleep
 *   byte 6  : 0x40 Turbo, 0x80 Quiet
 *   byte 7  : modo auto 00 / cool 20 / dry 40 / heat 80 / fan C0
 *   byte 8  : 0x80 se Sleep
 *   byte 10 : 0x10 Self clean
 *   byte 12 : codice dell'ULTIMO TASTO premuto sul telecomando:
 *             swing 02, fan 04, health 07, turbo/quiet 08, sleep 0B, self clean 19
 *             (power/mode/temp: 05 nelle catture, ma una cattura in Auto a 26 C da' 00)
 *   byte 14 : 0xB7
 *   byte 15 : 0x40 Self clean
 *   byte 16 : ventola low C0 / med 80 / high 40, auto = 00
 *
 * Regole del telecomando (da te): pos 1 alette solo in Heat; Sleep non in Fan;
 * Turbo/Quiet solo in Cool e Heat (il primo clic porta SwingV a 3, modificabile) e
 * escludono la ventola manuale (premere FAN li disattiva e riporta la ventola ad Auto);
 * in Fan niente ventola Auto; Self clean solo in Cool, Dry e Fan e non si puo'
 * spegnere dal telecomando (qui e' un'azione: OK sulla riga lo invia).
 *
 * Non ancora verificato: tasti Power/Mode/Temp e byte 12, Turbo/Quiet con ventola
 * non auto, ventola low/med/high in Cool/Heat/Dry, swing orizzontale, timer.
 */

#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>
#include <infrared_transmit.h>

#define FRAME_BYTES   22
#define FRAME_BITS    176
#define TIMINGS_COUNT (4 + FRAME_BITS * 2 + 1) /* 357 */

#define T_HDR_1 3050
#define T_HDR_2 3059
#define T_HDR_3 4418
#define T_MARK  555
#define T_ONE   1670
#define T_ZERO  565

/* codici byte 12 (ultimo tasto) */
#define KEY_PWR_MODE_TEMP 0x05
#define KEY_SWING         0x02
#define KEY_FAN           0x04
#define KEY_HEALTH        0x07
#define KEY_TURBO         0x08
#define KEY_SLEEP         0x0B
#define KEY_CLEAN         0x19

typedef enum { ModeAuto, ModeCool, ModeHeat, ModeDry, ModeFan, ModeCount } AcMode;
typedef enum { FanAuto, FanLow, FanMed, FanHigh, FanCount } AcFan;
typedef enum {
    RowPower,
    RowMode,
    RowTemp,
    RowFan,
    RowSwingV,
    RowHealth,
    RowSleep,
    RowTurbo,
    RowClean,
    RowCount
} Row;

#define SWV_AUTO 6 /* swingV: 1..5 = posizioni, 6 = auto */
#define TQ_OFF   0
#define TQ_TURBO 1
#define TQ_QUIET 2

static const uint8_t mode_code[ModeCount] = {0x00, 0x20, 0x80, 0x40, 0xC0};
static const uint8_t fan_code[FanCount] = {0xA0, 0x60, 0x40, 0x20};
static const uint8_t fan_b16_code[FanCount] = {0x00, 0xC0, 0x80, 0x40};
static const char* mode_name[ModeCount] = {"Auto", "Cool", "Heat", "Dry", "Fan"};
static const char* fan_name[FanCount] = {"Auto", "Low", "Med", "High"};
static const char* lbl[RowCount] =
    {"Power", "Mode", "Temp", "Fan", "SwingV", "Health", "Sleep", "Turbo/Quiet", "Self clean"};

typedef struct {
    bool power;
    AcMode mode;
    uint8_t temp; /* 16..30 */
    AcFan fan;
    uint8_t swing_v; /* 1..5, SWV_AUTO */
    bool health;
    bool sleep;
    uint8_t tq; /* TQ_* */
    uint8_t last_key; /* byte 12 */
    Row row;
} AcState;

typedef struct {
    AcState st;
    FuriMutex* mutex;
    FuriMessageQueue* queue;
} App;

static bool sleep_allowed(const AcState* s) {
    return s->mode != ModeFan;
}
static bool tq_allowed(const AcState* s) {
    return s->mode == ModeCool || s->mode == ModeHeat;
}
static bool clean_allowed(const AcState* s) {
    return s->mode == ModeCool || s->mode == ModeDry || s->mode == ModeFan;
}

static void build_frame(const AcState* s, bool clean, uint8_t f[FRAME_BYTES]) {
    memset(f, 0, FRAME_BYTES);
    uint8_t swc = (s->swing_v == SWV_AUTO) ? 0x0C : (uint8_t)(2 * (6 - s->swing_v));
    /* con Turbo/Quiet attivi la ventola manuale e' esclusa: il telecomando manda Auto */
    AcFan fan = s->tq ? FanAuto : s->fan;

    f[0] = 0xA6;
    f[1] = ((s->temp - 16) << 4) | (s->power ? swc : 0);
    f[3] = s->health ? 0x02 : 0x00;
    f[4] = s->power ? 0x40 : 0x00;
    f[5] = fan_code[fan] | (s->sleep ? 0x08 : 0x00);
    f[6] = (s->tq == TQ_TURBO) ? 0x40 : (s->tq == TQ_QUIET) ? 0x80 : 0x00;
    f[7] = mode_code[s->mode];
    f[8] = s->sleep ? 0x80 : 0x00;
    f[10] = clean ? 0x10 : 0x00;
    f[12] = clean ? KEY_CLEAN : s->last_key;
    uint8_t sum = 0;
    for(int i = 0; i < 13; i++) sum += f[i];
    f[13] = sum;
    f[14] = 0xB7;
    f[15] = clean ? 0x40 : 0x00;
    f[16] = fan_b16_code[fan];
    sum = 0;
    for(int i = 14; i < 21; i++) sum += f[i];
    f[21] = sum;
}

static void send_state(const AcState* s, bool clean) {
    uint8_t f[FRAME_BYTES];
    build_frame(s, clean, f);

    uint32_t t[TIMINGS_COUNT];
    uint32_t n = 0;
    t[n++] = T_HDR_1;
    t[n++] = T_HDR_1;
    t[n++] = T_HDR_2;
    t[n++] = T_HDR_3;
    for(int i = 0; i < FRAME_BYTES; i++) {
        for(int b = 7; b >= 0; b--) { /* MSB first */
            t[n++] = T_MARK;
            t[n++] = (f[i] >> b) & 1 ? T_ONE : T_ZERO;
        }
    }
    t[n++] = T_MARK;

    infrared_send_raw_ext(t, n, true, 38000, 0.33f);
}

static void row_value(const AcState* s, Row r, char* out, size_t len) {
    switch(r) {
    case RowPower: snprintf(out, len, "%s", s->power ? "ON" : "OFF"); break;
    case RowMode: snprintf(out, len, "%s", mode_name[s->mode]); break;
    case RowTemp: snprintf(out, len, "%u C", s->temp); break;
    case RowFan:
        if(s->tq == TQ_TURBO) snprintf(out, len, "Turbo");
        else if(s->tq == TQ_QUIET) snprintf(out, len, "Quiet");
        else snprintf(out, len, "%s", fan_name[s->fan]);
        break;
    case RowSwingV:
        if(s->swing_v == SWV_AUTO) snprintf(out, len, "Auto");
        else snprintf(out, len, "%u", s->swing_v);
        break;
    case RowHealth: snprintf(out, len, "%s", s->health ? "ON" : "OFF"); break;
    case RowSleep:
        snprintf(out, len, "%s", !sleep_allowed(s) ? "--" : s->sleep ? "ON" : "OFF");
        break;
    case RowTurbo:
        snprintf(out, len, "%s", !tq_allowed(s) ? "--" : s->tq == TQ_TURBO ? "Turbo" :
                                                          s->tq == TQ_QUIET ? "Quiet" : "OFF");
        break;
    case RowClean: snprintf(out, len, "%s", clean_allowed(s) ? "OK=invia" : "--"); break;
    default: out[0] = 0; break;
    }
}

#define VISIBLE_ROWS 5

static void draw_cb(Canvas* canvas, void* ctx) {
    App* app = ctx;
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    const AcState* s = &app->st;

    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 10, "Haier AC");
    uint16_t tw = canvas_string_width(canvas, "Haier AC");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2 + tw + 5, 10, "by Marcopiiit");

    int top = (int)s->row - 2;
    if(top < 0) top = 0;
    if(top > RowCount - VISIBLE_ROWS) top = RowCount - VISIBLE_ROWS;

    for(int k = 0; k < VISIBLE_ROWS; k++) {
        int i = top + k;
        int y = 22 + k * 10;
        char val[16];
        row_value(s, (Row)i, val, sizeof(val));
        if(i == (int)s->row) {
            canvas_draw_box(canvas, 0, y - 8, 126, 10);
            canvas_set_color(canvas, ColorWhite);
        }
        canvas_draw_str(canvas, 4, y, lbl[i]);
        canvas_draw_str_aligned(canvas, 122, y - 8, AlignRight, AlignTop, val);
        canvas_set_color(canvas, ColorBlack);
    }
    /* barra di scorrimento */
    int bar_h = 50 * VISIBLE_ROWS / RowCount;
    int bar_y = 14 + (50 - bar_h) * top / (RowCount - VISIBLE_ROWS);
    canvas_draw_box(canvas, 126, bar_y, 2, bar_h);
    furi_mutex_release(app->mutex);
}

static void input_cb(InputEvent* ev, void* ctx) {
    App* app = ctx;
    furi_message_queue_put(app->queue, ev, FuriWaitForever);
}

static void change(AcState* s, int dir) {
    switch(s->row) {
    case RowPower:
        s->power = !s->power;
        s->last_key = KEY_PWR_MODE_TEMP;
        break;
    case RowMode:
        s->mode = (s->mode + ModeCount + dir) % ModeCount;
        if(s->mode != ModeHeat && s->swing_v == 1) s->swing_v = 2; /* pos 1 solo in Heat */
        if(s->mode == ModeFan && s->fan == FanAuto) s->fan = FanLow; /* in Fan niente Auto */
        if(!sleep_allowed(s)) s->sleep = false;
        if(!tq_allowed(s)) s->tq = TQ_OFF;
        s->last_key = KEY_PWR_MODE_TEMP;
        break;
    case RowTemp:
        if(dir > 0 && s->temp < 30) s->temp++;
        if(dir < 0 && s->temp > 16) s->temp--;
        s->last_key = KEY_PWR_MODE_TEMP;
        break;
    case RowFan:
        if(s->tq != TQ_OFF) {
            /* FAN disattiva Turbo/Quiet e riporta la ventola ad Auto (come da cattura) */
            s->tq = TQ_OFF;
            s->fan = (s->mode == ModeFan) ? FanLow : FanAuto;
        } else if(s->mode == ModeFan) {
            s->fan = FanLow + ((s->fan - FanLow + 3 + dir) % 3); /* Low/Med/High */
        } else {
            s->fan = (s->fan + FanCount + dir) % FanCount;
        }
        s->last_key = KEY_FAN;
        break;
    case RowSwingV: {
        int lo = (s->mode == ModeHeat) ? 1 : 2; /* pos 1 solo in Heat */
        int n = SWV_AUTO - lo + 1; /* lo..5 + auto */
        int idx = s->swing_v - lo;
        idx = (idx + n + dir) % n;
        s->swing_v = lo + idx;
        s->last_key = KEY_SWING;
        break;
    }
    case RowHealth:
        s->health = !s->health;
        s->last_key = KEY_HEALTH;
        break;
    case RowSleep:
        if(!sleep_allowed(s)) break;
        s->sleep = !s->sleep;
        s->last_key = KEY_SLEEP;
        break;
    case RowTurbo:
        if(!tq_allowed(s)) break;
        {
            uint8_t prev = s->tq;
            s->tq = (s->tq + 3 + dir) % 3; /* Off -> Turbo -> Quiet -> Off */
            if(prev == TQ_OFF && s->tq == TQ_TURBO) s->swing_v = 3; /* primo clic Turbo */
            s->last_key = KEY_TURBO;
        }
        break;
    default: break;
    }
}

int32_t haier_ac_app(void* p) {
    UNUSED(p);
    App* app = malloc(sizeof(App));
    app->st = (AcState){.power = true, .mode = ModeCool, .temp = 24, .fan = FanAuto,
                        .swing_v = 2, .health = false, .sleep = false, .tq = TQ_OFF,
                        .last_key = KEY_PWR_MODE_TEMP, .row = RowPower};
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    ViewPort* vp = view_port_alloc();
    view_port_draw_callback_set(vp, draw_cb, app);
    view_port_input_callback_set(vp, input_cb, app);
    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, vp, GuiLayerFullscreen);

    InputEvent ev;
    bool running = true;
    while(running) {
        if(furi_message_queue_get(app->queue, &ev, FuriWaitForever) != FuriStatusOk) continue;
        if(ev.type != InputTypeShort && ev.type != InputTypeRepeat) continue;

        furi_mutex_acquire(app->mutex, FuriWaitForever);
        AcState* s = &app->st;
        bool tx = false;
        bool clean = false;
        switch(ev.key) {
        case InputKeyUp: s->row = (s->row + RowCount - 1) % RowCount; break;
        case InputKeyDown: s->row = (s->row + 1) % RowCount; break;
        case InputKeyLeft: change(s, -1); break;
        case InputKeyRight: change(s, +1); break;
        case InputKeyOk:
            if(s->row == RowClean) {
                if(clean_allowed(s)) {
                    clean = true;
                    tx = true;
                }
            } else {
                tx = true; /* invia lo stato completo */
            }
            break;
        case InputKeyBack: running = false; break;
        default: break;
        }
        if(tx) send_state(s, clean);
        furi_mutex_release(app->mutex);
        view_port_update(vp);
    }

    gui_remove_view_port(gui, vp);
    furi_record_close(RECORD_GUI);
    view_port_free(vp);
    furi_message_queue_free(app->queue);
    furi_mutex_free(app->mutex);
    free(app);
    return 0;
}
