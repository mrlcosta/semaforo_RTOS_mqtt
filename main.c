#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "lib/ssd1306.h"
#include "lib/font.h"

#include "pio_matrix.pio.h"
#include "config.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "pico/cyw43_arch.h"

#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

// ================ MQTT Configuration ================
#define MQTT_SERVER "192.168.1.157"
#define MQTT_USERNAME "muriel"
#define MQTT_PASSWORD "muriel"
#define MQTT_PORT 1883
#define MQTT_KEEP_ALIVE_S 60
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0
#define MQTT_TOPIC_LEN 100

// MQTT Topics
#define MQTT_TOPIC_MODO "semaforo/modo"
#define MQTT_TOPIC_BUZZER "semaforo/buzzer"
#define MQTT_TOPIC_ESTADO "semaforo/estado"

// ================ Global Variables ================
ssd1306_t ssd;
PIO pio = pio0;
uint sm;
volatile bool modo_diurno = true;
volatile bool alerta_sonoro = true;
volatile int estado_semaforo = VERDE;

// ================ MQTT Structures and Variables ================
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

static MQTT_CLIENT_DATA_T mqtt_state;

// ================ Matrix LED Patterns ================
double COORDENADA_VERDE[PIXELS][3] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
};

double COORDENADA_AMARELO[PIXELS][3] = {
    {0, 0, 0}, {0, 0, 0}, {1, 1, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {1, 1, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {1, 1, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {1, 1, 0}, {0, 0, 0}, {0, 0, 0}
};

double COORDENADA_VERMELHO[PIXELS][3] = {
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0},
    {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}
};

// ================ MQTT Functions ================
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        mqtt_subscribe(state->mqtt_client_inst, MQTT_TOPIC_MODO, MQTT_SUBSCRIBE_QOS, NULL, state);
        mqtt_subscribe(state->mqtt_client_inst, MQTT_TOPIC_BUZZER, MQTT_SUBSCRIBE_QOS, NULL, state);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            printf("Failed to connect to MQTT server\n");
        }
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    if (strcmp(state->topic, MQTT_TOPIC_MODO) == 0) {
        if (strcmp(state->data, "toggle") == 0) {
            modo_diurno = !modo_diurno;
        }
    } else if (strcmp(state->topic, MQTT_TOPIC_BUZZER) == 0) {
        if (strcmp(state->data, "toggle") == 0) {
            alerta_sonoro = !alerta_sonoro;
        }
    }
}

static void mqtt_publish_estado(MQTT_CLIENT_DATA_T *state) {
    char estado_str[32];
    snprintf(estado_str, sizeof(estado_str), "%s,%s", 
             modo_diurno ? "diurno" : "noturno",
             alerta_sonoro ? "sonoro" : "mudo");
    mqtt_publish(state->mqtt_client_inst, MQTT_TOPIC_ESTADO, estado_str, strlen(estado_str), 
                MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, NULL, state);
}

static void start_mqtt_client(MQTT_CLIENT_DATA_T *state) {
    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        printf("Failed to create MQTT client instance\n");
        return;
    }

    state->mqtt_client_info.client_id = "semaforo_pico";
    state->mqtt_client_info.client_user = MQTT_USERNAME;
    state->mqtt_client_info.client_pass = MQTT_PASSWORD;
    state->mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;

    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, MQTT_PORT, 
                          mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        printf("Failed to connect to MQTT broker\n");
        return;
    }

    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
}

// ================ Hardware Functions ================
void buzzer(uint pin, uint delay) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_gpio_level(pin, 2048);
    sleep_ms(delay);
    pwm_set_gpio_level(pin, 0);
}

int32_t rgb_matrix(double r, double g, double b) {
    unsigned char R, G, B;
    R = r * 255;
    G = g * 255;
    B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

void pio_matrix(double coordenada[][3], int32_t valor, PIO pio, uint sm) {
    for (int16_t i = 0; i < PIXELS; i++) {
        double r = coordenada[i][0];
        double g = coordenada[i][1];
        double b = coordenada[i][2];

        valor = rgb_matrix(r, g, b);
        pio_sm_put_blocking(pio, sm, valor);
    }
}

// ================ FreeRTOS Tasks ================
void display_task() {
    char modo[16];
    char message[32];
    char texto[32];

    while (true) {
        ssd1306_fill(&ssd, false);
        ssd1306_rect(&ssd, 3, 3, 122, 60, true, false);

        switch (estado_semaforo) {
        case VERMELHO:
            sprintf(modo, "Modo diurno");
            sprintf(texto, "Pare");
            ssd1306_draw_string(&ssd, texto, 45, 35);
            break;

        case AMARELO:
            sprintf(modo, "Modo diurno");
            sprintf(texto, "Atencao");
            ssd1306_draw_string(&ssd, texto, 40, 35);
            break;

        case VERDE:
            sprintf(modo, "Modo diurno");
            sprintf(texto, "Prossiga");
            ssd1306_draw_string(&ssd, texto, 35, 35);
            break;

        case NOTURNO:
            sprintf(modo, "Modo noturno");
            sprintf(texto, "Preste atencao");
            ssd1306_draw_string(&ssd, texto, 10, 35);
            break;
        }

        ssd1306_draw_string(&ssd, modo, 15, 15);
        ssd1306_send_data(&ssd);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void button_task() {
    while (true) {
        if (!gpio_get(BOTAO_A)) {
            modo_diurno = !modo_diurno;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (!gpio_get(BOTAO_B)) {
            reset_usb_boot(0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void buzzer_task() {
    while (true) {
        if (alerta_sonoro) {
            switch (estado_semaforo) {
            case VERDE:
                buzzer(BUZZER_PIN, 1000);
                vTaskDelay(pdMS_TO_TICKS(3000));
                break;

            case AMARELO:
                buzzer(BUZZER_PIN, 100);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case VERMELHO:
                buzzer(BUZZER_PIN, 500);
                vTaskDelay(pdMS_TO_TICKS(1500));
                break;

            case NOTURNO:
                buzzer(BUZZER_PIN, 300);
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;
            }
        }
    }
}

void led_task() {
    while (true) {
        switch (estado_semaforo) {
        case VERDE:
            gpio_put(LED_VERDE, true);
            gpio_put(LED_VERMELHO, false);
            vTaskDelay(pdMS_TO_TICKS(DELAY));

            if (modo_diurno) {
                estado_semaforo = AMARELO;
            } else {
                estado_semaforo = NOTURNO;
            }
            break;

        case AMARELO:
            gpio_put(LED_VERDE, true);
            gpio_put(LED_VERMELHO, true);
            vTaskDelay(pdMS_TO_TICKS(DELAY));

            if (modo_diurno) {
                estado_semaforo = VERMELHO;
            } else {
                estado_semaforo = NOTURNO;
            }
            break;

        case VERMELHO:
            gpio_put(LED_VERDE, false);
            gpio_put(LED_VERMELHO, true);
            vTaskDelay(pdMS_TO_TICKS(DELAY));

            if (modo_diurno) {
                estado_semaforo = VERDE;
            } else {
                estado_semaforo = NOTURNO;
            }
            break;

        case NOTURNO:
            gpio_put(LED_VERDE, true);
            gpio_put(LED_VERMELHO, true);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_put(LED_VERDE, false);
            gpio_put(LED_VERMELHO, false);
            vTaskDelay(pdMS_TO_TICKS(2000));
            if (modo_diurno) {
                estado_semaforo = VERDE;
            }
            break;
        }
    }
}

void matriz_led_task() {
    while (true) {
        switch (estado_semaforo) {
        case VERDE:
            pio_matrix(COORDENADA_VERDE, 0, pio, sm);
            break;

        case AMARELO:
            pio_matrix(COORDENADA_AMARELO, 0, pio, sm);
            break;

        case VERMELHO:
            pio_matrix(COORDENADA_VERMELHO, 0, pio, sm);
            break;

        case NOTURNO:
            pio_matrix(COORDENADA_AMARELO, 0, pio, sm);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void mqtt_task(void *pvParameters) {
    while (1) {
        if (mqtt_state.mqtt_client_inst && mqtt_client_is_connected(mqtt_state.mqtt_client_inst)) {
            mqtt_publish_estado(&mqtt_state);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void wifi_task() {
    while (1) {
        cyw43_arch_poll();
        sleep_ms(50);
    }
}

// ================ Initialization Functions ================
void init_perifericos() {
    // Inicializa display
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, 128, 64, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);

    // Inicializa botoes
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);

    // inicializa matriz de led
    uint offset = pio_add_program(pio, &pio_matrix_program);
    sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, OUT_PIN);

    // Configurar o pino como saída de PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);

    // Inicializa os Leds
    gpio_init(LED_VERDE);
    gpio_init(LED_VERMELHO);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_set_dir(LED_VERMELHO, GPIO_OUT);

    // Inicializa WiFi
    while (cyw43_arch_init()) {
        printf("Erro ao iniciar rede\n");
        sleep_ms(100);
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    cyw43_arch_enable_sta_mode();

    printf("Conectando no Wi-Fi...\n");
    ssd1306_draw_string(&ssd, "Conectando", 10, 29);
    ssd1306_send_data(&ssd);
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
        printf("Erro ao conectar Wi-Fi\n");
        ssd1306_draw_string(&ssd, "Erro ao conectar", 10, 29);
        ssd1306_send_data(&ssd);
        sleep_ms(100);
        return -1;
    }
    printf("Conectado à rede\n");

    if (netif_default) {
        printf("IP: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Initialize MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &mqtt_state.mqtt_server_address, NULL, NULL);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        start_mqtt_client(&mqtt_state);
    } else {
        printf("Failed to resolve MQTT server address\n");
        return -1;
    }
}

int main() {
    stdio_init_all();
    init_perifericos();

    xTaskCreate(wifi_task, "wifi", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(button_task, "Botao", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(buzzer_task, "Buzzer", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(display_task, "Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(led_task, "Led", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(matriz_led_task, "Matriz led", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(mqtt_task, "MQTT", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
