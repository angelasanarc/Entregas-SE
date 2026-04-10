#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_random.h"

// ================= PINES =================
static const int ROW_PINS[6]   = {33, 32, 23, 22, 21, 0};
static const int COL_V_PINS[6] = {13, 14, 26, 19, 5, 16};
static const int COL_R_PINS[6] = {12, 27, 25, 18, 17, 4};

#define BTN_LEFT  2
#define BTN_RIGHT 15

// ================= CONFIG =================
#define GRID_ROWS 6
#define GRID_COLS 6

#define PLAYER_ROW 0   // ajustado por rotación

#define MAX_ENEMIES 12

#define DISPLAY_PERIOD_US 2000
#define LEVEL_DURATION_MS 30000

static const int ENEMY_SPEED_MS[3] = {700, 450, 250};
static const int ENEMY_SPAWN_MS[3] = {1800, 1200, 700};

// ================= FRAMEBUFFER =================
static uint8_t g_fb[GRID_ROWS][GRID_COLS];

static void fb_clear(void) { memset(g_fb, 0, sizeof(g_fb)); }

// ROTACIÓN 90°
static void fb_set(int r, int c, uint8_t color) {
    if (r >= 0 && r < GRID_ROWS && c >= 0 && c < GRID_COLS) {
        int new_r = c;
        int new_c = GRID_ROWS - 1 - r;
        g_fb[new_r][new_c] = color;
    }
}

// ================= DISPLAY ISR =================
static volatile int disp_row = 0;

static void IRAM_ATTR display_isr(void *arg) {
    int prev = (disp_row + GRID_ROWS - 1) % GRID_ROWS;
    gpio_set_level(ROW_PINS[prev], 0);

    for (int c = 0; c < GRID_COLS; c++) {
        gpio_set_level(COL_V_PINS[c], 0);
        gpio_set_level(COL_R_PINS[c], 0);
    }

    int r = disp_row;
    for (int c = 0; c < GRID_COLS; c++) {
        uint8_t pix = g_fb[r][c];
        if (pix == 1) gpio_set_level(COL_V_PINS[c], 1);
        if (pix == 2) gpio_set_level(COL_R_PINS[c], 1);
    }

    gpio_set_level(ROW_PINS[r], 1);
    disp_row = (disp_row + 1) % GRID_ROWS;
}

// ================= GPIO =================
static void gpio_init_all(void) {
    for (int i = 0; i < GRID_ROWS; i++) {
        gpio_set_direction(ROW_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(ROW_PINS[i], 0);
    }

    for (int i = 0; i < GRID_COLS; i++) {
        gpio_set_direction(COL_V_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(COL_R_PINS[i], GPIO_MODE_OUTPUT);
    }

    gpio_set_direction(BTN_LEFT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_LEFT, GPIO_PULLUP_ONLY);

    gpio_set_direction(BTN_RIGHT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_RIGHT, GPIO_PULLUP_ONLY);
}

// ================= DEBOUNCE =================
#define DEBOUNCE_MS 50

typedef struct {
    int pin;
    bool last_stable;
    bool current;
    TickType_t last_change;
} Button;

static Button btn_left  = {BTN_LEFT, true, true, 0};
static Button btn_right = {BTN_RIGHT, true, true, 0};

static bool btn_pressed(Button *b) {
    bool raw = gpio_get_level(b->pin);
    TickType_t now = xTaskGetTickCount();

    if (raw != b->current) {
        b->current = raw;
        b->last_change = now;
    }

    if ((now - b->last_change) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        if (b->last_stable != b->current) {
            b->last_stable = b->current;
            if (!b->last_stable) return true;
        }
    }
    return false;
}

// ================= ENTIDADES =================
typedef struct {
    int row, col;
    bool active;
} Enemy;

static Enemy g_enemies[MAX_ENEMIES];
static int g_player_col = 2;

// ================= GAME STATE =================
typedef enum {
    STATE_PLAYING,
    STATE_GAME_OVER,
    STATE_WIN
} GameState;

static GameState g_state = STATE_PLAYING;
static int g_level = 0;

// ================= LOGICA =================
static void spawn_enemy(void) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!g_enemies[i].active) {
            g_enemies[i].active = true;
            g_enemies[i].row = GRID_ROWS - 1;   // ahora aparecen arriba
            g_enemies[i].col = rand() % GRID_COLS;
            return;
        }
    }
}

static bool check_collision(void) {
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (g_enemies[i].active &&
            g_enemies[i].row == PLAYER_ROW &&
            g_enemies[i].col == g_player_col)
            return true;
    }
    return false;
}

static void render(void) {
    fb_clear();

    fb_set(PLAYER_ROW, g_player_col, 1);

    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (g_enemies[i].active)
            fb_set(g_enemies[i].row, g_enemies[i].col, 2);
    }
}

// ================= GAME TASK =================
static void game_task(void *pvParam) {

    TickType_t last_move = 0;
    TickType_t last_spawn = 0;
    TickType_t start_time = xTaskGetTickCount();

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (g_state == STATE_PLAYING) {

            if (btn_pressed(&btn_left) && g_player_col > 0)
                g_player_col--;

            if (btn_pressed(&btn_right) && g_player_col < GRID_COLS - 1)
                g_player_col++;

            // enemigos ahora CAEN
            if ((now - last_move) >= pdMS_TO_TICKS(ENEMY_SPEED_MS[g_level])) {
                for (int i = 0; i < MAX_ENEMIES; i++) {
                    if (g_enemies[i].active) {
                        g_enemies[i].row--;
                        if (g_enemies[i].row < 0)
                            g_enemies[i].active = false;
                    }
                }
                last_move = now;
            }

            if ((now - last_spawn) >= pdMS_TO_TICKS(ENEMY_SPAWN_MS[g_level])) {
                spawn_enemy();
                last_spawn = now;
            }

            if (check_collision()) {
                g_state = STATE_GAME_OVER;
            }

            if ((now - start_time) >= pdMS_TO_TICKS(LEVEL_DURATION_MS)) {
                g_state = STATE_WIN;
            }

            render();
        }

        else if (g_state == STATE_GAME_OVER) {
            fb_clear();
            for (int i = 0; i < 6; i++) fb_set(i, i, 2);
            vTaskDelay(pdMS_TO_TICKS(2000));

            memset(g_enemies, 0, sizeof(g_enemies));
            g_player_col = 2;
            g_level = 0;
            start_time = xTaskGetTickCount();
            g_state = STATE_PLAYING;
        }

        else if (g_state == STATE_WIN) {
            fb_clear();
            for (int i = 0; i < 6; i++) fb_set(i, i, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));

            memset(g_enemies, 0, sizeof(g_enemies));
            g_player_col = 2;
            g_level++;
            if (g_level > 2) g_level = 2;

            start_time = xTaskGetTickCount();
            g_state = STATE_PLAYING;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ================= DISPLAY TASK =================
static void display_task(void *pvParam) {
    esp_timer_handle_t timer;

    esp_timer_create_args_t args = {
        .callback = display_isr
    };

    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, DISPLAY_PERIOD_US);

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

// ================= MAIN =================
void app_main(void) {

    srand(esp_random());

    gpio_init_all();
    fb_clear();

    xTaskCreatePinnedToCore(display_task, "display", 2048, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(game_task, "game", 4096, NULL, 5, NULL, 0);
}