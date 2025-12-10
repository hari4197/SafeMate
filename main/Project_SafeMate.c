// Project_SafeMate.c
// Patient-Room Safety & Environment Monitor
// Target: ESP32-C3-DevKitM-1
//
// Peripherals used:
//  - DHT11 (GPIO2)
//  - PIR sensor (GPIO3)
//  - Vibration sensor (GPIO1)
//  - Silence button (GPIO0, active-low)
//  - Reset button  (GPIO10, active-low)
//  - LEDs: Green (GPIO6), Yellow (GPIO7), Red (GPIO18)
//  - Buzzer (GPIO19)
//  - 16x2 LCD with I2C backpack (PCF8574 @ 0x27) on SDA=GPIO4, SCL=GPIO5

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

// ============================================================================
// Pin mapping
// ============================================================================
#define GPIO_DHT11        2
#define GPIO_PIR          3
#define GPIO_VIB          1

#define GPIO_BTN_SILENCE  0    // active-low
#define GPIO_BTN_RESET    10   // active-low

#define GPIO_LED_GREEN    6
#define GPIO_LED_YELLOW   7
#define GPIO_LED_RED      18

#define GPIO_BUZZER       19

#define I2C_SDA_GPIO      4
#define I2C_SCL_GPIO      5
#define I2C_PORT          I2C_NUM_0
#define I2C_FREQ_HZ       100000

// ============================================================================
// LCD over I2C 
// ============================================================================
#define LCD_ADDR          0x27

#define LCD_BACKLIGHT     0x08
#define LCD_ENABLE        0x04
#define LCD_RW            0x02
#define LCD_RS            0x01

#define MAIN_LOOP_PERIOD_MS  100
static const char *TAG = "SafeMate";

// Low-level I2C write ---------------------------------------------------------
static esp_err_t lcd_i2c_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_i2c_write(data | LCD_ENABLE);
    esp_rom_delay_us(1);
    lcd_i2c_write(data & ~LCD_ENABLE);
    esp_rom_delay_us(50);
}

static void lcd_write4bits(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | LCD_BACKLIGHT | mode;
    lcd_pulse_enable(data);
}

static void lcd_send(uint8_t value, uint8_t mode)
{
    lcd_write4bits(value & 0xF0, mode);          
    lcd_write4bits((value << 4) & 0xF0, mode);   
}

static void lcd_command(uint8_t cmd)
{
    lcd_send(cmd, 0); 
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_write_char(char c)
{
    lcd_send((uint8_t)c, LCD_RS);
}

static void lcd_clear(void)
{
    lcd_command(0x01);           // clear display
    vTaskDelay(pdMS_TO_TICKS(3));
}

static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_command(0x80 | (col + row_offsets[row]));
}

static void lcd_print_fixed16(const char *s)
{
    char buf[17];
    size_t len = strlen(s);
    if (len > 16) len = 16;
    memset(buf, ' ', 16);
    memcpy(buf, s, len);
    buf[16] = '\0';

    for (int i = 0; i < 16; ++i) {
        lcd_write_char(buf[i]);
    }
}

static void lcd_init(void)
{
    // Initial 4-bit sequence
    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_write4bits(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x30, 0);
    esp_rom_delay_us(150);
    lcd_write4bits(0x30, 0);
    esp_rom_delay_us(150);

    lcd_write4bits(0x20, 0);     // 4-bit mode
    esp_rom_delay_us(150);

    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_clear();
}

// ============================================================================
// I2C master init
// ============================================================================
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

// ===============================
// DHT11 
// ===============================
#define SENSOR_PERIOD_MS  2000  

static void dht11_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_DHT11),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_DHT11, 1);   
}

static bool dht11_read_raw(uint8_t data[5])
{
    memset(data, 0, 5);

    // Start signal
    gpio_set_direction(GPIO_DHT11, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_DHT11, 0);
    vTaskDelay(pdMS_TO_TICKS(20));        

    gpio_set_level(GPIO_DHT11, 1);
    esp_rom_delay_us(30);                 

    gpio_set_direction(GPIO_DHT11, GPIO_MODE_INPUT);

    uint32_t timeout;

    timeout = 0;
    while (gpio_get_level(GPIO_DHT11) == 1) {
        if (++timeout > 10000) return false;
        esp_rom_delay_us(1);
    }

    timeout = 0;
    while (gpio_get_level(GPIO_DHT11) == 0) {
        if (++timeout > 10000) return false;
        esp_rom_delay_us(1);
    }

    timeout = 0;
    while (gpio_get_level(GPIO_DHT11) == 1) {
        if (++timeout > 10000) return false;
        esp_rom_delay_us(1);
    }

    for (int i = 0; i < 40; i++) {
        timeout = 0;
        while (gpio_get_level(GPIO_DHT11) == 0) {
            if (++timeout > 10000) return false;
            esp_rom_delay_us(1);
        }

        timeout = 0;
        while (gpio_get_level(GPIO_DHT11) == 1) {
            esp_rom_delay_us(1);
            if (++timeout > 10000) break;
        }

        int byteIndex = i / 8;
        data[byteIndex] <<= 1;

        if (timeout > 40) {
            data[byteIndex] |= 1;
        }
    }

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if ((sum & 0xFF) != data[4]) {
        return false;
    }

    return true;
}

static bool dht11_read(float *temperature, float *humidity)
{
    uint8_t data[5];
    if (!dht11_read_raw(data)) {
        return false;
    }

    *humidity    = (float)data[0] + (float)data[1] / 10.0f;
    *temperature = (float)data[2] + (float)data[3] / 10.0f;
    return true;
}


// ============================================================================
// GPIO init
// ============================================================================
static void gpio_init_all(void)
{
    // LEDs + buzzer
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << GPIO_LED_GREEN) |
            (1ULL << GPIO_LED_YELLOW) |
            (1ULL << GPIO_LED_RED)   |
            (1ULL << GPIO_BUZZER),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Sensors: PIR, vibration
    gpio_config_t in_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_PIR) | (1ULL << GPIO_VIB),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    gpio_config_t btn_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_BTN_SILENCE) | (1ULL << GPIO_BTN_RESET),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    gpio_set_level(GPIO_LED_GREEN, 0);
    gpio_set_level(GPIO_LED_YELLOW, 0);
    gpio_set_level(GPIO_LED_RED, 0);
    gpio_set_level(GPIO_BUZZER, 0);
}

// ============================================================================
// Main application
// ============================================================================
void app_main(void)
{
    i2c_master_init();
    gpio_init_all();
    lcd_init();
    dht11_init();

    lcd_set_cursor(0, 0);
    lcd_print_fixed16("SafeMate Starting");
    lcd_set_cursor(0, 1);
    lcd_print_fixed16("Please wait...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    bool alarm_latched   = false;
    bool buzzer_silenced = false;

    float temp  = 0.0f;
    float hum   = 0.0f;
    bool  dht_ok = false;
    TickType_t last_dht_tick = xTaskGetTickCount();

    bool silence_prev = false;
    bool reset_prev   = false;

    while (1) {

        TickType_t now = xTaskGetTickCount();

        // --------- Read DHT11 only every SENSOR_PERIOD_MS (2000 ms) ----------
        if ((now - last_dht_tick) >= pdMS_TO_TICKS(SENSOR_PERIOD_MS)) {
            dht_ok = dht11_read(&temp, &hum);
            last_dht_tick = now;
        }

        // --------- Read sensors (PIR, vibration) ----------
        bool pir  = gpio_get_level(GPIO_PIR);
        bool vib  = gpio_get_level(GPIO_VIB);

        // Simple thresholds
        bool env_high = (dht_ok && temp >= 35.0f);

        bool alarm_now = pir || vib || env_high;

        // --------- BUTTONS (active-low) ----------
        bool silence_raw = (gpio_get_level(GPIO_BTN_SILENCE) == 0);
        bool reset_raw   = (gpio_get_level(GPIO_BTN_RESET)   == 0);

        // Edge detection: trigger once on press (high -> low)
        bool silence_edge = (silence_raw && !silence_prev);
        bool reset_edge   = (reset_raw   && !reset_prev);

        silence_prev = silence_raw;
        reset_prev   = reset_raw;

        // Silence: only on *edge* of press
        if (silence_edge) {
            buzzer_silenced = true;
        }

        // Reset: clear alarm + silence state, also edge-triggered
        if (reset_edge) {
            alarm_latched   = false;
            buzzer_silenced = false;
        }

        // Latch the alarm for display/LEDs
        if (alarm_now) {
            alarm_latched = true;
        }

        // Buzzer follows *current* alarm, not latched alarm
        bool buzzer_on = alarm_now && !buzzer_silenced;

        // ---------- LED + Buzzer behaviour ----------
        if (alarm_latched) {
            // Alarm happened at least once
            gpio_set_level(GPIO_LED_GREEN, 0);
            gpio_set_level(GPIO_LED_YELLOW, env_high ? 1 : 0);
            gpio_set_level(GPIO_LED_RED, 1);
            gpio_set_level(GPIO_BUZZER, buzzer_on ? 1 : 0);
        } else {
            // Normal / safe
            gpio_set_level(GPIO_LED_GREEN, 1);
            gpio_set_level(GPIO_LED_YELLOW, 0);
            gpio_set_level(GPIO_LED_RED, 0);
            gpio_set_level(GPIO_BUZZER, 0);
        }

        // ---------- LCD update ----------
        char line[17];

        lcd_clear();

        // Line 1: T=xx ̊ C H=yy % (or error)
        if (dht_ok) {
            snprintf(line, sizeof(line), "T:%2d C  H:%2d%%",
                     (int)temp, (int)hum);
        } else {
            snprintf(line, sizeof(line), "T/H: ERR READ");
        }
        lcd_set_cursor(0, 0);
        lcd_print_fixed16(line);

        // Line 2: PIR/VIB/current Alarm status (live, not latched)
        snprintf(line, sizeof(line),
                 "M:%c V:%c A:%c",
                 pir       ? 'Y' : 'N',
                 vib       ? 'Y' : 'N',
                 alarm_now ? 'Y' : 'N');
        lcd_set_cursor(0, 1);
        lcd_print_fixed16(line);

        // --------- NEW: fast loop for buttons (100 ms) ---------
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
    }
}