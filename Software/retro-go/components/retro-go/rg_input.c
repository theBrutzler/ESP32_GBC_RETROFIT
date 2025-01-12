#include "rg_system.h"
#include "rg_input.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef ESP_PLATFORM
#include <driver/gpio.h>
#include <driver/adc.h>
#else
#include <SDL2/SDL.h>
#endif

#if RG_BATTERY_DRIVER == 1
#include <esp_adc_cal.h>
static esp_adc_cal_characteristics_t adc_chars;
#endif

#ifdef RG_GAMEPAD_ADC1_MAP
static rg_keymap_adc1_t keymap_adc1[] = RG_GAMEPAD_ADC1_MAP;
#endif
#ifdef RG_GAMEPAD_GPIO_MAP
static rg_keymap_gpio_t keymap_gpio[] = RG_GAMEPAD_GPIO_MAP;
#endif
#ifdef RG_GAMEPAD_MAP
static rg_keymap_t keymap[] = RG_GAMEPAD_MAP;
#endif
static bool input_task_running = false;
static uint32_t gamepad_state = -1; // _Atomic
static rg_battery_t battery_state = {0};


bool rg_input_read_battery_raw(rg_battery_t *out)
{
    uint32_t raw_value = 0;

#if RG_BATTERY_DRIVER == 1 /* ADC1 */
    for (int i = 0; i < 4; ++i)
        raw_value += esp_adc_cal_raw_to_voltage(adc1_get_raw(RG_BATTERY_ADC_CHANNEL), &adc_chars);
    raw_value /= 4;
#elif RG_BATTERY_DRIVER == 2 /* I2C */
    uint8_t data[5];
    if (!rg_i2c_read(0x20, -1, &data, 5))
        return false;
    raw_value = data[4];
#else
    return false;
#endif
    if (!out)
        return true;

    *out = (rg_battery_t){
        .level = RG_MAX(0.f, RG_MIN(100.f, RG_BATTERY_CALC_PERCENT(raw_value))),
        .volts = RG_BATTERY_CALC_VOLTAGE(raw_value),
        .present = true,
    };
    return true;
}

bool rg_input_read_gamepad_raw(uint32_t *out)
{
    uint32_t state = 0;

#if defined(RG_GAMEPAD_ADC1_MAP)
    for (size_t i = 0; i < RG_COUNT(keymap_adc1); ++i)
    {
        const rg_keymap_adc1_t *mapping = &keymap_adc1[i];
        int value = adc1_get_raw(mapping->channel);
        if (value > mapping->min && value < mapping->max)
            state |= mapping->key;
    }
#endif

#if defined(RG_GAMEPAD_GPIO_MAP)
    for (size_t i = 0; i < RG_COUNT(keymap_gpio); ++i)
    {
        const rg_keymap_gpio_t *mapping = &keymap_gpio[i];
        if (gpio_get_level(mapping->num) == mapping->level)
            state |= mapping->key;
    }
#endif

#if RG_GAMEPAD_DRIVER == 2 // Serial

    gpio_set_level(RG_GPIO_GAMEPAD_LATCH, 0);
    rg_usleep(5);
    gpio_set_level(RG_GPIO_GAMEPAD_LATCH, 1);
    rg_usleep(1);

    uint32_t buttons = 0;
    for (int i = 0; i < 16; i++)
    {
        buttons |= gpio_get_level(RG_GPIO_GAMEPAD_DATA) << (15 - i);
        gpio_set_level(RG_GPIO_GAMEPAD_CLOCK, 0);
        rg_usleep(1);
        gpio_set_level(RG_GPIO_GAMEPAD_CLOCK, 1);
        rg_usleep(1);
    }

    for (size_t i = 0; i < RG_COUNT(keymap); ++i)
    {
        if ((buttons & keymap[i].src) == keymap[i].src)
            state |= keymap[i].key;
    }

#elif RG_GAMEPAD_DRIVER == 3 // I2C

    uint8_t data[5];
    if (rg_i2c_read(0x20, -1, &data, 5))
    {
        uint32_t buttons = ~((data[2] << 8) | data[1]);

        for (size_t i = 0; i < RG_COUNT(keymap); ++i)
        {
            if ((buttons & keymap[i].src) == keymap[i].src)
                state |= keymap[i].key;
        }
    }

#elif RG_GAMEPAD_DRIVER == 4 // I2C via AW9523

    uint32_t buttons = ~(rg_i2c_gpio_read_port(0) | rg_i2c_gpio_read_port(1) << 8);

    for (size_t i = 0; i < RG_COUNT(keymap); ++i)
    {
        if ((buttons & keymap[i].src) == keymap[i].src)
            state |= keymap[i].key;
    }

#elif RG_GAMEPAD_DRIVER == 6

    int numkeys = 0;
    const uint8_t *keys = SDL_GetKeyboardState(&numkeys);

    for (size_t i = 0; i < RG_COUNT(keymap); ++i)
    {
        if (keymap[i].src < 0 || keymap[i].src >= numkeys)
            continue;
        if (keys[keymap[i].src])
            state |= keymap[i].key;
    }

#endif

    // Virtual buttons (combos) to replace essential missing buttons.
#if !RG_GAMEPAD_HAS_MENU_BTN
    if (state == (RG_KEY_SELECT | RG_KEY_START))
        state = RG_KEY_MENU;
#endif
#if !RG_GAMEPAD_HAS_OPTION_BTN
    if (state == (RG_KEY_SELECT | RG_KEY_A))
        state = RG_KEY_OPTION;
#endif

    if (out) *out = state;
    return true;
}

static void input_task(void *arg)
{
    const uint8_t debounce_level = 0x03;
    uint8_t debounce[RG_KEY_COUNT];
    uint32_t local_gamepad_state = 0;
    uint32_t state;
    int64_t next_battery_update = 0;

    memset(debounce, debounce_level, sizeof(debounce));
    input_task_running = true;

    while (input_task_running)
    {
        if (rg_input_read_gamepad_raw(&state))
        {
            for (int i = 0; i < RG_KEY_COUNT; ++i)
            {
                debounce[i] = ((debounce[i] << 1) | ((state >> i) & 1));
                debounce[i] &= debounce_level;

                if (debounce[i] == debounce_level) // Pressed
                {
                    local_gamepad_state |= (1 << i);
                }
                else if (debounce[i] == 0x00) // Released
                {
                    local_gamepad_state &= ~(1 << i);
                }
            }
            gamepad_state = local_gamepad_state;
        }

        if (rg_system_timer() >= next_battery_update)
        {
            rg_battery_t temp = {0};
            if (rg_input_read_battery_raw(&temp))
            {
                if (fabsf(battery_state.level - temp.level) < 1.0f)
                    temp.level = battery_state.level;
                if (fabsf(battery_state.volts - temp.volts) < 0.010f)
                    temp.volts = battery_state.volts;
            }
            battery_state = temp;
            next_battery_update = rg_system_timer() + 2 * 1000000;
        }

        rg_task_delay(10);
    }

    input_task_running = false;
    gamepad_state = -1;
}

void rg_input_init(void)
{
    RG_ASSERT(!input_task_running, "Input already initialized!");

#if defined(RG_GAMEPAD_ADC1_MAP)
    RG_LOGI("Initializing ADC1 gamepad driver...");
    adc1_config_width(ADC_WIDTH_MAX - 1);
    for (size_t i = 0; i < RG_COUNT(keymap_adc1); ++i)
    {
        const rg_keymap_adc1_t *mapping = &keymap_adc1[i];
        adc1_config_channel_atten(mapping->channel, mapping->atten);
    }
#endif

#if defined(RG_GAMEPAD_GPIO_MAP)
    RG_LOGI("Initializing GPIO gamepad driver...");
    for (size_t i = 0; i < RG_COUNT(keymap_gpio); ++i)
    {
        const rg_keymap_gpio_t *mapping = &keymap_gpio[i];
        gpio_set_direction(mapping->num, GPIO_MODE_INPUT);
        gpio_set_pull_mode(mapping->num, mapping->pull);
    }
#endif

#if RG_GAMEPAD_DRIVER == 2 // Serial

    RG_LOGI("Initializing SERIAL gamepad driver...");
    gpio_set_direction(RG_GPIO_GAMEPAD_CLOCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(RG_GPIO_GAMEPAD_LATCH, GPIO_MODE_OUTPUT);
    gpio_set_direction(RG_GPIO_GAMEPAD_DATA, GPIO_MODE_INPUT);
    gpio_set_level(RG_GPIO_GAMEPAD_LATCH, 0);
    gpio_set_level(RG_GPIO_GAMEPAD_CLOCK, 1);

#elif RG_GAMEPAD_DRIVER == 3 // I2C

    RG_LOGI("Initializing I2C gamepad driver...");
    rg_i2c_init();
    // The first read returns bogus data, waste it.
    rg_input_read_gamepad_raw(NULL);

#elif RG_GAMEPAD_DRIVER == 4 // I2C w/AW9523

    RG_LOGI("Initializing I2C-GPIO gamepad driver...");
    rg_i2c_gpio_init();

    // All that below should be moved elsewhere, and possibly specific to the qtpy...
    rg_i2c_gpio_set_direction(AW_TFT_RESET, 0);
    rg_i2c_gpio_set_direction(AW_TFT_BACKLIGHT, 0);
    rg_i2c_gpio_set_direction(AW_HEADPHONE_EN, 0);

    rg_i2c_gpio_set_level(AW_TFT_BACKLIGHT, 1);
    rg_i2c_gpio_set_level(AW_HEADPHONE_EN, 1);

    // tft reset
    rg_i2c_gpio_set_level(AW_TFT_RESET, 0);
    rg_usleep(10 * 1000);
    rg_i2c_gpio_set_level(AW_TFT_RESET, 1);
    rg_usleep(10 * 1000);

#elif RG_GAMEPAD_DRIVER == 6 // SDL2

    RG_LOGI("Initializing SDL2 gamepad driver...");

#endif

#if RG_BATTERY_DRIVER == 1 /* ADC1 */
    RG_LOGI("Initializing ADC1 battery driver...");
    adc1_config_width(ADC_WIDTH_MAX - 1);
    adc1_config_channel_atten(RG_BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_MAX - 1, 1100, &adc_chars);
#endif

    // Start background polling
    rg_task_create("rg_input", &input_task, NULL, 3 * 1024, RG_TASK_PRIORITY_6, 1);
    while (gamepad_state == -1)
        rg_task_yield();
    RG_LOGI("Input ready. state=" PRINTF_BINARY_16 "\n", PRINTF_BINVAL_16(gamepad_state));
}

void rg_input_deinit(void)
{
    input_task_running = false;
    // while (gamepad_state != -1)
    //     rg_task_yield();
    RG_LOGI("Input terminated.\n");
}

uint32_t rg_input_read_gamepad(void)
{
#ifdef RG_TARGET_SDL2
    SDL_PumpEvents();
#endif
    return gamepad_state;
}

bool rg_input_key_is_pressed(rg_key_t mask)
{
    return (bool)(rg_input_read_gamepad() & mask);
}

bool rg_input_wait_for_key(rg_key_t mask, bool pressed, int timeout_ms)
{
    int64_t expiration = timeout_ms < 0 ? INT64_MAX : (rg_system_timer() + timeout_ms * 1000);
    while (rg_input_key_is_pressed(mask) != pressed)
    {
        if (rg_system_timer() > expiration)
            return false;
        rg_task_delay(10);
    }
    return true;
}

rg_battery_t rg_input_read_battery(void)
{
    return battery_state;
}

const char *rg_input_get_key_name(rg_key_t key)
{
    switch (key)
    {
    case RG_KEY_UP: return "Up";
    case RG_KEY_RIGHT: return "Right";
    case RG_KEY_DOWN: return "Down";
    case RG_KEY_LEFT: return "Left";
    case RG_KEY_SELECT: return "Select";
    case RG_KEY_START: return "Start";
    case RG_KEY_MENU: return "Menu";
    case RG_KEY_OPTION: return "Option";
    case RG_KEY_A: return "A";
    case RG_KEY_B: return "B";
    case RG_KEY_X: return "X";
    case RG_KEY_Y: return "Y";
    case RG_KEY_L: return "Left Shoulder";
    case RG_KEY_R: return "Right Shoulder";
    case RG_KEY_NONE: return "None";
    default: return "Unknown";
    }
}

const rg_gui_keyboard_t virtual_map1 = {
    .columns = 10,
    .rows = 4,
    .data = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
        'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
        'U', 'V', 'W', 'X', 'Y', 'Z', ' ', ',', '.', ' ',
    }
};

const rg_gui_keyboard_t virtual_map2 = {
    .columns = 10,
    .rows = 4,
    .data = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',
        'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',
        'u', 'v', 'w', 'x', 'y', 'z', ' ', ',', '.', ' ',
    }
};

const rg_gui_keyboard_t virtual_map3 = {
    .columns = 10,
    .rows = 4,
    .data = {
        '!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
        '`', '~', '-', '+', '=', ' ', ' ', ' ', '<', '>',
        ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '{', '}',
        ' ', ' ', ' ', ' ', ' ', '|','\\', '/', '[', ']',
    }
};

const rg_gui_keyboard_t virtual_map4 = {
    .columns = 4,
    .rows = 4,
    .data = {
        '0', '1', '2', '3',
        '4', '5', '6', '7',
        '8', '9', 'A', 'B',
        'C', 'D', 'E', 'F',
    }
};

const rg_gui_keyboard_t *virtual_maps[] = {
    &virtual_map1,
    &virtual_map2,
    &virtual_map3,
    &virtual_map4,
};

int rg_input_read_keyboard(/* const char *custom_map */)
{
    static size_t selected_map = 0;
    static size_t cursor = 0;

    rg_input_wait_for_key(RG_KEY_ALL, false, 1000);

    while (1)
    {
        uint32_t joystick = rg_input_read_gamepad();

        const rg_gui_keyboard_t *map = virtual_maps[selected_map];

        size_t prev_cursor = cursor;

        if (joystick & RG_KEY_LEFT)
            cursor--;
        if (joystick & RG_KEY_RIGHT)
            cursor++;
        if (joystick & RG_KEY_UP)
            cursor -= map->columns;
        if (joystick & RG_KEY_DOWN)
            cursor += map->columns;

        if (cursor >= map->columns * map->rows)
            cursor = prev_cursor;
        prev_cursor = cursor;

        if (joystick & RG_KEY_SELECT)
            selected_map = (selected_map + 1) % RG_COUNT(virtual_maps);

        if (joystick & RG_KEY_A)
            return map->data[cursor];
        if (joystick & RG_KEY_B)
            break;

        rg_gui_draw_keyboard("[select] to change map", map, cursor);

        rg_input_wait_for_key(~(RG_KEY_UP|RG_KEY_DOWN|RG_KEY_LEFT|RG_KEY_RIGHT), false, 100);
        rg_task_delay(50);
        rg_system_tick(0);
    }

    return -1;
}
