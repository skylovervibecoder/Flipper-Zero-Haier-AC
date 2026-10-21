/*
 * Haier AC - telecomando IR completo per Flipper Zero (Momentum)
 * Climatizzatore: Haier AS35RBAHRA-3, telecomando YR-HE2
 *
 * Protocollo ricavato dalle catture H.ir: frame da 176 bit (22 byte), 38 kHz,
 * 2 checksum a somma (byte 13 = somma byte 0..12, byte 21 = somma byte 14..20).
 * Ogni trasmissione contiene lo STATO COMPLETO, come il telecomando originale.
 *
 * Campi verificati su 10 catture (tutti i frame ricostruiti coincidono):
 *   byte 1  : (temp-16)<<4 | posizione alette verticali (nibble basso, 0 se spento):
 *             pos1=A, pos2=8, pos3=6, pos4=4, pos5=2, auto=C  (= 2*(6-pos))
 *   byte 4  : 0x40 acceso, 0x00 spento
 *   byte 5  : ventola  auto A0 / low 60 / med 40 / high 20
 *   byte 7  : modo     auto 00 / cool 20 / heat 80 / dry 40 / fan C0
 *   byte 12 : 0x02 fisso. Nelle catture vale 05, 02, 00 o 04 senza una regola
 *             ancora chiara (probabile altro stato del telecomando)
 *   byte 14 : 0xB7
 *   byte 16 : ventola low C0 / med 80 / high 40, auto = 00 (verificato in Auto e Fan;
 *             in Cool/Heat/Dry con ventola non auto e' ancora un'ipotesi)
 *
 * La posizione 1 delle alette e' valida solo in modo Heat.
 *
 * NON ancora verificato: ventola low/med/high in cool/heat/dry (byte 16),
 * swing orizzontale, turbo, sleep, quiet, timer.
 *
 * Il segnale viene inviato SOLO quando si preme OK.
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
#define BYTE12  0x02

typedef enum { ModeAuto, ModeCool, ModeHeat, ModeDry, ModeFan, ModeCount } AcMode;
typedef enum { FanAuto, FanLow, FanMed, FanHigh, FanCount } AcFan;
typedef enum { RowPower, RowMode, RowTemp, RowFan, RowSwingV, RowCount } Row;

#define SWV_AUTO 6 /* swingV: 1..5 = posizioni, 6 = auto */

static const uint8_t mode_code[ModeCount] = {0x00, 0x20, 0x80, 0x40, 0xC0};
static const uint8_t fan_code[FanCount] = {0xA0, 0x60, 0x40, 0x20};
static const uint8_t fan_b16_code[FanCount] = {0x00, 0xC0, 0x80, 0x40};
static const char* mode_name[ModeCount] = {"Auto", "Cool", "Heat", "Dry", "Fan"};
static const char* fan_name[FanCount] = {"Auto", "Low", "Med", "High"};

typedef struct {
    bool power;
    AcMode mode;
    uint8_t temp; /* 16..30 */
    AcFan fan;
    uint8_t swing_v; /* 1..5, SWV_AUTO */
    Row row;
    bool sent;
} AcState;

typedef struct {
    AcState st;
    FuriMutex* mutex;
    FuriMessageQueue* queue;
} App;

static void build_frame(const AcState* s, uint8_t f[FRAME_BYTES]) {
    memset(f, 0, FRAME_BYTES);
    f[0] = 0xA6;
    uint8_t swc = (s->swing_v == SWV_AUTO) ? 0x0C : (uint8_t)(2 * (6 - s->swing_v));
    f[1] = ((s->temp - 16) << 4) | (s->power ? swc : 0);
    f[4] = s->power ? 0x40 : 0x00;
    f[5] = fan_code[s->fan];
    f[7] = mode_code[s->mode];
    f[12] = BYTE12;
    uint8_t sum = 0;
    for(int i = 0; i < 13; i++) sum += f[i];
    f[13] = sum;
    f[14] = 0xB7;
    f[16] = fan_b16_code[s->fan];
    sum = 0;
    for(int i = 14; i < 21; i++) sum += f[i];
    f[21] = sum;
}

static void send_state(const AcState* s) {
    uint8_t f[FRAME_BYTES];
    build_frame(s, f);

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

    char val[5][16];
    snprintf(val[RowPower], sizeof(val[0]), "%s", s->power ? "ON" : "OFF");
    snprintf(val[RowMode], sizeof(val[0]), "%s", mode_name[s->mode]);
    snprintf(val[RowTemp], sizeof(val[0]), "%u C", s->temp);
    snprintf(val[RowFan], sizeof(val[0]), "%s", fan_name[s->fan]);
    if(s->swing_v == SWV_AUTO)
        snprintf(val[RowSwingV], sizeof(val[0]), "Auto");
    else
        snprintf(val[RowSwingV], sizeof(val[0]), "%u", s->swing_v);
    const char* lbl[5] = {"Power", "Mode", "Temp", "Fan", "SwingV"};

    for(int i = 0; i < RowCount; i++) {
        int y = 22 + i * 10;
        if(i == (int)s->row) {
            canvas_draw_box(canvas, 0, y - 8, 128, 10);
            canvas_set_color(canvas, ColorWhite);
        }
        canvas_draw_str(canvas, 4, y, lbl[i]);
        canvas_draw_str_aligned(canvas, 124, y - 8, AlignRight, AlignTop, val[i]);
        canvas_set_color(canvas, ColorBlack);
    }
    furi_mutex_release(app->mutex);
}

static void input_cb(InputEvent* ev, void* ctx) {
    App* app = ctx;
    furi_message_queue_put(app->queue, ev, FuriWaitForever);
}

static void change(AcState* s, int dir) {
    switch(s->row) {
    case RowPower: s->power = !s->power; break;
    case RowMode:
        s->mode = (s->mode + ModeCount + dir) % ModeCount;
        if(s->mode != ModeHeat && s->swing_v == 1) s->swing_v = 2; /* pos 1 solo in Heat */
        break;
    case RowTemp:
        if(dir > 0 && s->temp < 30) s->temp++;
        if(dir < 0 && s->temp > 16) s->temp--;
        break;
    case RowFan: s->fan = (s->fan + FanCount + dir) % FanCount; break;
    case RowSwingV: {
        int lo = (s->mode == ModeHeat) ? 1 : 2; /* pos 1 solo in Heat */
        int n = SWV_AUTO - lo + 1; /* lo..5 + auto */
        int idx = s->swing_v - lo;
        idx = (idx + n + dir) % n;
        s->swing_v = lo + idx;
        break;
    }
    default: break;
    }
}

int32_t haier_ac_app(void* p) {
    UNUSED(p);
    App* app = malloc(sizeof(App));
    app->st = (AcState){.power = true, .mode = ModeCool, .temp = 24, .fan = FanAuto,
                        .swing_v = 2, .row = RowPower, .sent = false};
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
        switch(ev.key) {
        case InputKeyUp: s->row = (s->row + RowCount - 1) % RowCount; break;
        case InputKeyDown: s->row = (s->row + 1) % RowCount; break;
        case InputKeyLeft: change(s, -1); break;
        case InputKeyRight: change(s, +1); break;
        case InputKeyOk: tx = true; break; /* invia lo stato completo */
        case InputKeyBack: running = false; break;
        default: break;
        }
        if(tx) {
            s->sent = true;
            view_port_update(vp);
            send_state(s);
            s->sent = false;
        }
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
