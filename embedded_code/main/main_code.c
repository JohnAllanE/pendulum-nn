#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

// dev/tty.usbserial-42310

// Globals
static const char *TAG = "PendulumControl";
static int encoder_offset = -380; // Offset to zero the encoder reading

static int pulse_threshold_start = 200;
static int pulse_threshold_stop = 1000;

static bool build_up_mode = false;
static bool pulsing = false; // Flag to indicate if pulsing is active

// GPIO Pins
#define PIN_A 15
#define PIN_B 16
#define PULSE_PIN 8
#define RESET_PIN 17

// Update the debug GPIO pin to GPIO19
#define DEBUG_PIN 19

// PCNT Handles
static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan_a = NULL;
static pcnt_channel_handle_t pcnt_chan_b = NULL;

// Function prototypes
void setup_pcnt();
void setup_gpio();
void setup_debug_gpio();
void peak_detection_task(void *arg);
void pcnt_watch_point_task(void *arg); // FreeRTOS task
_Bool pcnt_on_reach_callback(struct pcnt_unit_t *unit, const pcnt_watch_event_data_t *edata, void *user_ctx); // Callback
void command_task(void *arg);

// ISR function to check encoder values once per ms
static esp_timer_handle_t encoder_timer;

// Shared variables for ISR and task
static volatile int isr_last_encoder_value = 0;
static volatile int isr_new_encoder_value = 0;
static volatile int64_t isr_time_start = 0; // when encoder value changed previously
static volatile int64_t isr_timestamp = 0;

static volatile bool isr_new_data_flag = false; // flag to indicate new data available

static volatile int direction = 0; // Global current direction

// Encoder read ISR based on timer
typedef struct {
    int encoder_value;      // The new encoder value
    int64_t timestamp;      // The time elapsed since the last encoder change
} encoder_data_t;

// Declare encoder_queue as a global variable
static QueueHandle_t encoder_queue = NULL;

void encoder_timer_callback(void *arg) {

    // Get current time
    int64_t current_time = esp_timer_get_time(); // Current time in microseconds

    // Get current encoder value
    int temp_encoder_value; // Temporary non-volatile variable
    esp_err_t err = pcnt_unit_get_count(pcnt_unit, &temp_encoder_value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "pcnt_unit_get_count failed: %d", err);
        return;
    }

    if (temp_encoder_value == isr_last_encoder_value) {
        return;
    }

    // Calculate the timestamp (time since last change)
    int64_t elapsed_time = current_time - isr_time_start;

    // Create the data struct
    encoder_data_t data = {
        .encoder_value = temp_encoder_value,
        .timestamp = elapsed_time
    };

    // Send the data to the queue (allow a small block to avoid drops)
    if (xQueueSend(encoder_queue, &data, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGW(TAG, "Encoder queue full, data lost");
    }

    // Reset values for the next cycle
    isr_last_encoder_value = temp_encoder_value;
    isr_time_start = current_time;
}

// Function to set up the timer
void setup_encoder_timer() {

    // Timer configuration
    const esp_timer_create_args_t timer_args = {
        .callback = &encoder_timer_callback,
        .name = "encoder_timer"
    };

    // Create the timer
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &encoder_timer));

    // Start the timer to fire every 1 ms
    ESP_ERROR_CHECK(esp_timer_start_periodic(encoder_timer, 1000)); // 1000 µs = 1 ms
}

// ISR function to clear the PCNT counter
void reset_encoder_isr(void *arg) {
    //ESP_LOGI(TAG, "Reset pin triggered, clearing PCNT counter");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit)); // Clear the PCNT counter
}

void setup_gpio() {
    // Configure pulse pin as output and set HIGH
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PULSE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PULSE_PIN, 0); // Initialize to LOW

    // Configure reset pin as input with interrupt
    io_conf.pin_bit_mask = (1ULL << RESET_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Trigger on falling edge
    gpio_config(&io_conf);

    // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // Default interrupt allocation flag
    ESP_ERROR_CHECK(gpio_isr_handler_add(RESET_PIN, reset_encoder_isr, NULL)); // Attach ISR
}

void peak_detection_task(void *arg) {
    ESP_LOGI(TAG, "Peak detection task started");
    int task_last_encoder_value = 0;
    int task_new_direction = 0;
    bool first_sample = true;

    QueueHandle_t encoder_queue = (QueueHandle_t)arg;
    encoder_data_t data;

    while (1) {
        if (xQueueReceive(encoder_queue, &data, portMAX_DELAY) == pdTRUE) {
            int task_new_encoder_value = data.encoder_value;
            int64_t timestamp = data.timestamp;

            if (first_sample) {
                task_last_encoder_value = task_new_encoder_value;
                first_sample = false;
                continue; // wait for the next sample to compute direction
            }

            task_new_direction = (task_new_encoder_value > task_last_encoder_value) ? 1 : -1;

            // See if direction changed
            if (task_new_direction != direction) {

                direction = task_new_direction;
                int amplitude = abs(task_new_encoder_value - encoder_offset);
                //Debug current status
                //ESP_LOGI(TAG, "Encoder: %d, Direction: %d, build_up_mode: %d, amplitude: %d", task_last_encoder_value, direction, build_up_mode, amplitude);
            
                // Decide on pulse mode
                if (!build_up_mode && (amplitude < pulse_threshold_start)) {
                    build_up_mode = true;
                    ESP_LOGI(TAG, "build_up_mode started");
                } else if (build_up_mode && (amplitude > pulse_threshold_stop)) {
                    build_up_mode = false;
                    ESP_LOGI(TAG, "build_up_mode stopped");
                }

                if (!build_up_mode) {
                    ESP_LOGI(TAG, "Peak value: %d, Timestamp: %lld ms", task_last_encoder_value, timestamp / 1000);
                }
            }

            // Update last encoder value for next iteration
            task_last_encoder_value = task_new_encoder_value;
        }
    }
}

void setup_pcnt() {
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = 30000,
        .low_limit = -30000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = PIN_A,
        .level_gpio_num = PIN_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = PIN_B,
        .level_gpio_num = PIN_A,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, encoder_offset));

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach_callback,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

_Bool pcnt_on_reach_callback(struct pcnt_unit_t *unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    if (build_up_mode && (direction == 1)) {
        gpio_set_level(PULSE_PIN, 1);
        pulsing = true;
    }
    return true;          // Return true to indicate the event was handled
}

void pcnt_watch_point_task(void *arg) {
    while (1) {
        if (pulsing) {
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(PULSE_PIN, 0);
            pulsing = false; // Reset the pulse trigger flag
            //ESP_LOGE(TAG, "Pulse sent");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void command_task(void *arg) {
    char command[10];
    while (1) {
        if (scanf("%s", command) > 0) {
            if (strcmp(command, "STREAM") == 0 || strcmp(command, "S") == 0) {
                ESP_LOGI(TAG, "Streaming toggled");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    // Initialize peripherals
    setup_gpio();
    //setup_debug_gpio();
    setup_pcnt();

    // Create the encoder queue
    encoder_queue = xQueueCreate(10, sizeof(encoder_data_t)); // Queue for 10 encoder_data_t structs
    if (encoder_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder queue");
        return;
    }

    // Initialize baseline PCNT count so first timer callback doesn't treat current count as a change
    {
        int initial_count = 0;
        if (pcnt_unit_get_count(pcnt_unit, &initial_count) == ESP_OK) {
            isr_last_encoder_value = initial_count;
        } else {
            ESP_LOGW(TAG, "Failed to read initial pcnt count");
            isr_last_encoder_value = 0;
        }
    }

    // initialize time baseline for the encoder timer callback
    isr_time_start = esp_timer_get_time();

    // Create peak detection task
    if (xTaskCreate(peak_detection_task, "PeakDetectionTask", 4096, encoder_queue, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create PeakDetectionTask");
    }
    // Start the periodic encoder timer
    setup_encoder_timer();

    xTaskCreate(pcnt_watch_point_task, "PCNTWatchPointTask", 2048, NULL, 5, NULL);
    xTaskCreate(command_task, "CommandTask", 2048, NULL, 5, NULL);
}