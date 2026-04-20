#include <LPC17xx.h>
#include <cmsis_os.h>
#include <Board_LED.h>
#include <Board_GLCD.h>
#include <stdio.h>
#include <string.h>

// External declaration for GLCD font
extern GLCD_FONT GLCD_Font_16x24;

// Constants
#define DEBOUNCE_TIME_MS 100
#define MAX_ADC_VALUE 4095
#define MIN_DURATION_MS 100
#define MAX_DURATION_MS 5000

// Pin Definitions
#define JOYSTICK_UP_PIN     (1 << 23)  // P1.23
#define JOYSTICK_DOWN_PIN   (1 << 25)  // P1.25
#define JOYSTICK_CENTER_PIN (1 << 20)  // P1.20
#define JOYSTICK_LEFT_PIN   (1 << 24)  // P1.24
#define JOYSTICK_RIGHT_PIN  (1 << 26)  // P1.26

// Colors
#define COLOR_WHITE  0xFFFFFF
#define COLOR_BLACK  0x000000
#define COLOR_BLUE   0x0000FF
#define COLOR_RED    0xFF0000
#define COLOR_GRAY   0xC0C0C0

// Joystick States
#define JOYSTICK_UP     0x01
#define JOYSTICK_DOWN   0x02
#define JOYSTICK_CENTER 0x04
#define JOYSTICK_LEFT   0x08
#define JOYSTICK_RIGHT  0x10

// Data Structures
typedef enum {
    ACTUATOR_HEATER,
    ACTUATOR_SPRINKLER,
    ACTUATOR_LIGHT,
    ACTUATOR_COUNT
} ActuatorType;

typedef struct {
    const char *name;
    volatile int *threshold;
    volatile int *duration;
    uint32_t pin;
    LPC_GPIO_TypeDef *port;
} ActuatorConfig;

typedef struct {
    volatile int hours;
    volatile int minutes;
    volatile int seconds;
} TimeData;

// ------PHASE 1: SHARED RESOURCES------
// Global Variables
static osMutexId adc_mutex;
static osMutexId glcd_mutex;
static osSemaphoreId actuator_sems[ACTUATOR_COUNT];
static volatile int adc_values[ACTUATOR_COUNT];
static volatile TimeData current_time = {0, 0, 0};
static volatile TimeData timers[ACTUATOR_COUNT] = {
    {0, 0, 30}, // Heater default
    {0, 0, 30}, // Sprinkler default
    {0, 0, 30}  // Light default
};
static volatile int selected_menu = 0;
static volatile int timer_triggered[ACTUATOR_COUNT] = {0, 0, 0};
static volatile int selected_field = 0;
static volatile int selected_actuator = 0;
// UART  Control Thread 
static volatile int uart_triggered[ACTUATOR_COUNT] = {0, 0, 0};     
static volatile int uart_command_state[ACTUATOR_COUNT] = {0, 0, 0}; 

// Actuator Durations
static volatile int heater_duration = 500;
static volatile int sprinkler_duration = 600;
static volatile int light_duration = 700;

// Actuator Thresholds
static volatile int threshold_values[ACTUATOR_COUNT] = {1600, 5000, 4091};

// Actuator Configurations
static ActuatorConfig actuators[ACTUATOR_COUNT] = {
    {"Heater",    &threshold_values[0], &heater_duration,    (1 << 29), LPC_GPIO1},
    {"Sprinkler", &threshold_values[1], &sprinkler_duration, (1 << 31), LPC_GPIO1},
    {"Light",     &threshold_values[2], &light_duration,     (1 << 2),  LPC_GPIO2}
};
// ------PHASE 1: SHARED RESOURCES------

// Function Prototypes
void init_hardware(void);
void init_adc(void);
void init_gpio(void);
void init_uart(void);
void init_timer(void);
void send_uart_string(const char *str);
int read_adc_channel(uint8_t channel);
void toggle_actuator(ActuatorType type);
void sensor_thread(void const *arg);
void uart_thread(void const *arg);
void monitor_thread(void const *arg);
void control_thread(void const *arg);
void uart_receive_thread(void const *arg);
void menu_thread(void const *arg);
void display_menu(int prev_menu);
void show_sensors(void);
void control_actuators(void);
void adjust_threshold(ActuatorType type);
void adjust_durations(void);
void control_timers(void);
void update_time(void);
uint32_t read_joystick(void);

// Hardware Initialization
void init_hardware(void) {
    init_adc();
    init_gpio();
    init_uart();
    init_timer();
}

// ------PHASE 2: ADC HARDWARE INITIALIZATION------
void init_adc(void) {
    LPC_PINCON->PINSEL1 |= (1 << 14) | (1 << 16) | (1 << 18); // AD0.0–0.2 (P0.23–P0.25)
    LPC_SC->PCONP |= (1 << 12); // Enable ADC power
    LPC_ADC->ADCR = (7 << 0) | (4 << 8) | (1 << 21); // 3 channels, clock divider, power ON
}
// ------PHASE 2: ADC HARDWARE INITIALIZATION------

void init_gpio(void) {
    LPC_GPIO1->FIODIR |= (1 << 29) | (1 << 31) | (1 << 28); // Heater, Sprinkler, Status LED
    LPC_GPIO2->FIODIR |= (1 << 2); // Light
    LPC_PINCON->PINSEL3 &= ~((3 << 14) | (3 << 18) | (3 << 8) | (3 << 16) | (3 << 20)); // Joystick GPIO
    LPC_PINCON->PINMODE3 &= ~((3 << 14) | (3 << 18) | (3 << 8) | (3 << 16) | (3 << 20)); // Pull-up
    LPC_GPIO1->FIODIR &= ~(JOYSTICK_UP_PIN | JOYSTICK_DOWN_PIN | JOYSTICK_CENTER_PIN | 
                          JOYSTICK_LEFT_PIN | JOYSTICK_RIGHT_PIN); // Joystick inputs
}

// ------PHASE 5: UART COMMUNICATION------
void init_uart(void) {
    LPC_SC->PCONP |= (1 << 3); // Power up UART0
    LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6); // P0.2 TXD0, P0.3 RXD0
    LPC_UART0->LCR = 0x83; // 8 bits, 1 stop bit, enable DLAB
    LPC_UART0->DLM = 0; LPC_UART0->DLL = 97; // 9600 baud
    LPC_UART0->LCR = 0x03; // 8 bits, 1 stop bit, no parity
}
// ------PHASE 5: UART COMMUNICATION------

void init_timer(void) {
    LPC_SC->PCONP |= (1 << 2); // Power up Timer1
    LPC_TIM1->TCR = 0x02; // Reset timer
    LPC_TIM1->PR = 0; // Prescaler
    LPC_TIM1->MR0 = SystemCoreClock - 1; // 1 Hz
    LPC_TIM1->MCR = 0x03; // Interrupt and reset on MR0
    NVIC_EnableIRQ(TIMER1_IRQn);
    LPC_TIM1->TCR = 0x01; // Start timer
}

// Timer Interrupt Handler
void TIMER1_IRQHandler(void) {
    if (LPC_TIM1->IR & 0x01) {
        LPC_TIM1->IR = 0x01; // Clear interrupt
        update_time();
    }
}

void update_time(void) {
    current_time.seconds++;
    if (current_time.seconds >= 60) {
        current_time.seconds = 0;
        current_time.minutes++;
        if (current_time.minutes >= 60) {
            current_time.minutes = 0;
            current_time.hours++;
            if (current_time.hours >= 24) {
                current_time.hours = 0;
            }
        }
    }
}

// ------PHASE 5: UART COMMUNICATION------
// Utility Functions
void send_uart_string(const char *str) {
    while (*str) {
        while (!(LPC_UART0->LSR & (1 << 5))); // Wait for TX ready
        LPC_UART0->THR = *str++;
    }
}
// ------PHASE 5: UART COMMUNICATION------

// ------PHASE 2: ADC READING LOGIC & sensor_thread------
int read_adc_channel(uint8_t channel) {
    LPC_ADC->ADCR &= ~(0x7 << 0); // Clear channel
    LPC_ADC->ADCR |= (1 << channel); // Select channel
    LPC_ADC->ADCR |= (1 << 24); // Start conversion
    while (!(LPC_ADC->ADGDR & (1U << 31))); // Wait for completion
    return (LPC_ADC->ADGDR >> 4) & 0xFFF; // 12-bit result
}
// ------PHASE 2: ADC READING LOGIC & sensor_thread------


uint32_t read_joystick(void) {
    uint32_t state = 0;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_UP_PIN))     state |= JOYSTICK_UP;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_DOWN_PIN))   state |= JOYSTICK_DOWN;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_CENTER_PIN)) state |= JOYSTICK_CENTER;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_LEFT_PIN))   state |= JOYSTICK_LEFT;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_RIGHT_PIN))  state |= JOYSTICK_RIGHT;
    return state;
}

void toggle_actuator(ActuatorType type) {
    ActuatorConfig *act = &actuators[type];
    if (act->port->FIOPIN & act->pin) {
        act->port->FIOCLR = act->pin; // Turn off
    } else {
        act->port->FIOSET = act->pin; // Turn on
    }
}

// RTOS Threads
// ------PHASE 2: ADC READING LOGIC & sensor_thread------
void sensor_thread(void const *arg) {
    while (1) {
        osMutexWait(adc_mutex, osWaitForever);
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            adc_values[i] = read_adc_channel(i);
        }
        osMutexRelease(adc_mutex);
        osDelay(1000);
    }
}
// ------PHASE 2: ADC READING LOGIC & sensor_thread------


// ------PHASE 5: UART COMMUNICATION------
void uart_thread(void const *arg) {
    char buffer[64];
    while (1) {
        osMutexWait(adc_mutex, osWaitForever);
        snprintf(buffer, sizeof(buffer), "TEMP:%d|MOIST:%d|LIGHT:%d\n",
                 adc_values[0], adc_values[1], adc_values[2]);
        osMutexRelease(adc_mutex);
        send_uart_string(buffer);
        osDelay(3000);
    }
}
// ------PHASE 5: UART COMMUNICATION------

// ------PHASE 3: MONITOR & CONTROL LOGIC------
void monitor_thread(void const *arg) {
    ActuatorType type = *(ActuatorType *)arg;
    while (1) {
        osMutexWait(adc_mutex, osWaitForever);
        if ((type == ACTUATOR_HEATER && adc_values[type] >= *actuators[type].threshold) ||
            (type != ACTUATOR_HEATER && adc_values[type] <= *actuators[type].threshold)) {
            osSemaphoreRelease(actuator_sems[type]);
        }
        osMutexRelease(adc_mutex);
        osDelay(1000);
    }
}
// ------PHASE 3: MONITOR & CONTROL LOGIC------


// ------PHASE 3: MONITOR & CONTROL LOGIC------
void control_thread(void const *arg) {
    ActuatorType type = *(ActuatorType *)arg;
    while (1) {
        osSemaphoreWait(actuator_sems[type], osWaitForever);
        if (uart_triggered[type]) {
            if (uart_command_state[type] == 1) {
                actuators[type].port->FIOSET = actuators[type].pin; 
            } else {
                actuators[type].port->FIOCLR = actuators[type].pin;
            }
            uart_triggered[type] = 0; 
        } 
        else if (timer_triggered[type]) {
            actuators[type].port->FIOSET = actuators[type].pin;
            osDelay(*actuators[type].duration);
            actuators[type].port->FIOCLR = actuators[type].pin;
            timer_triggered[type] = 0;
        } else {
            osMutexWait(adc_mutex, osWaitForever);
            if ((type == ACTUATOR_HEATER && adc_values[type] >= *actuators[type].threshold) ||
                (type != ACTUATOR_HEATER && adc_values[type] <= *actuators[type].threshold)) {
                actuators[type].port->FIOSET = actuators[type].pin;
                osDelay(*actuators[type].duration);
                actuators[type].port->FIOCLR = actuators[type].pin;
            }
            osMutexRelease(adc_mutex);
        }
    }
}
// ------PHASE 3: MONITOR & CONTROL LOGIC------


// ------PHASE 5: UART COMMUNICATION (RX - RECEIVE DATA)------
void uart_receive_thread(void const *arg) {
    char buffer[64];
    int idx = 0;
    while (1) {
        if (LPC_UART0->LSR & 0x01) {
            char c = LPC_UART0->RBR;
            if (c == '\n' || idx >= 63) {
                buffer[idx] = '\0';
                idx = 0;
                if (strncmp(buffer, "CMD:", 4) == 0) {
                    for (int i = 0; i < ACTUATOR_COUNT; i++) {
                        char on_cmd[16], off_cmd[16];
                        snprintf(on_cmd, sizeof(on_cmd), "%s:ON", actuators[i].name);
                        snprintf(off_cmd, sizeof(off_cmd), "%s:OFF", actuators[i].name);
                        if (strstr(buffer, on_cmd)) {
                            uart_command_state[i] = 1;            // Ghi nhận lệnh BẬT
                            uart_triggered[i] = 1;                // Dựng cờ báo hiệu có lệnh
                            osSemaphoreRelease(actuator_sems[i]); // Rung chuông gọi Control Thread
                        }
                        if (strstr(buffer, off_cmd)) {
                            uart_command_state[i] = 0;            // Ghi nhận lệnh TẮT
                            uart_triggered[i] = 1;                // Dựng cờ báo hiệu có lệnh
                            osSemaphoreRelease(actuator_sems[i]); // Rung chuông gọi Control Thread
                        }
                    }
                }
            } else {
                buffer[idx++] = c;
            }
        }
        osDelay(10);
    }
}
// ------PHASE 5: UART COMMUNICATION (RX - RECEIVE DATA)------

// Display Functions
void display_menu(int prev_menu) {
    static const char *menu_items[] = {
        "Show Sensors Data",
        "Manual Control",
        "Adjust Heater Thresh",
        "Adjust Sprinkler Thresh",
        "Adjust Light Thresh",
        "On Duration Adjust",
        "Control Timers"
    };
    static int first_call = 1;

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    if (first_call || prev_menu == -1) {
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "Greenhouse Menu");
        for (int i = 0; i < 7; i++) {
            char display_text[35];
            snprintf(display_text, sizeof(display_text), i == selected_menu ? "> %s" : "%s", menu_items[i]);
            GLCD_SetBackgroundColor(i == selected_menu ? COLOR_GRAY : COLOR_WHITE);
            GLCD_DrawString(0, (i + 2) * 24, display_text);
        }
        first_call = 0;
    } else if (prev_menu != selected_menu) {
        char display_text[35];
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, (prev_menu + 2) * 24, "                    ");
        snprintf(display_text, sizeof(display_text), "%s", menu_items[prev_menu]);
        GLCD_DrawString(0, (prev_menu + 2) * 24, display_text);
        GLCD_SetBackgroundColor(COLOR_GRAY);
        GLCD_DrawString(0, (selected_menu + 2) * 24, "                    ");
        snprintf(display_text, sizeof(display_text), "> %s", menu_items[selected_menu]);
        GLCD_DrawString(0, (selected_menu + 2) * 24, display_text);
    }
    osMutexRelease(glcd_mutex);
}

void show_sensors(void) {
    char buffer[32];
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);

    while (1) {
        osMutexWait(glcd_mutex, osWaitForever);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_SetForegroundColor(COLOR_BLACK);
        snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d", 
                 current_time.hours, current_time.minutes, current_time.seconds);
        GLCD_DrawString(0, 2 * 24, "                    ");
        GLCD_DrawString(0, 2 * 24, buffer);

        osMutexWait(adc_mutex, osWaitForever);
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 24, "Display Sensor Data");
        GLCD_SetForegroundColor(COLOR_BLACK);
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            snprintf(buffer, sizeof(buffer), "%s: %d", actuators[i].name, adc_values[i]);
            GLCD_DrawString(0, (4 + i) * 24, buffer);
        }
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 7 * 24, "Press center to return");
        osMutexRelease(adc_mutex);
        osMutexRelease(glcd_mutex);

        uint32_t joystick = read_joystick();
        uint32_t current_time = osKernelSysTick();
        if (current_time - last_action >= DEBOUNCE_TIME_MS && 
            (joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
            last_action = current_time;
            break;
        }
        prev_joystick = joystick;
        osDelay(1000);
    }

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    display_menu(-1);
}

void control_actuators(void) {
    static int first_call = 1;
    int selected = 0;
    int prev_selected = -1;
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    if (first_call) {
        osMutexWait(glcd_mutex, osWaitForever);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "Actuators Control");
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            char display_text[35];
            GLCD_SetBackgroundColor(i == selected ? COLOR_GRAY : COLOR_WHITE);
            GLCD_DrawString(0, (i + 2) * 24, "                    ");
            snprintf(display_text, sizeof(display_text), i == selected ? "> %s: %s" : "%s: %s",
                     actuators[i].name, 
                     actuators[i].port->FIOPIN & actuators[i].pin ? "ON" : "OFF");
            GLCD_DrawString(0, (i + 2) * 24, display_text);
        }
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 6 * 24, "Up/Dn:Select L/R:ON/OFF");
        GLCD_DrawString(0, 7 * 24, "Center:Exit");
        osMutexRelease(glcd_mutex);
        first_call = 0;
    }

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time = osKernelSysTick();
        int update_display = 0;

        if (current_time - last_action >= DEBOUNCE_TIME_MS) {
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                prev_selected = selected;
                selected = (selected > 0) ? selected - 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                prev_selected = selected;
                selected = (selected < ACTUATOR_COUNT - 1) ? selected + 1 : ACTUATOR_COUNT - 1;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & (JOYSTICK_LEFT | JOYSTICK_RIGHT)) && 
                !(prev_joystick & (JOYSTICK_LEFT | JOYSTICK_RIGHT))) {
                toggle_actuator(selected);
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                first_call = 1;
                last_action = current_time;
                break;
            }

            if (update_display) {
                osMutexWait(glcd_mutex, osWaitForever);
                char display_text[35];
                if (prev_selected != -1) {
                    GLCD_SetBackgroundColor(COLOR_WHITE);
                    GLCD_SetForegroundColor(COLOR_BLUE);
                    GLCD_DrawString(0, (prev_selected + 2) * 24, "                    ");
                    snprintf(display_text, sizeof(display_text), "%s: %s",
                             actuators[prev_selected].name,
                             actuators[prev_selected].port->FIOPIN & actuators[prev_selected].pin ? "ON" : "OFF");
                    GLCD_DrawString(0, (prev_selected + 2) * 24, display_text);
                }
                GLCD_SetBackgroundColor(COLOR_GRAY);
                GLCD_DrawString(0, (selected + 2) * 24, "                    ");
                snprintf(display_text, sizeof(display_text), "> %s: %s",
                         actuators[selected].name,
                         actuators[selected].port->FIOPIN & actuators[selected].pin ? "ON" : "OFF");
                GLCD_SetForegroundColor(COLOR_BLUE);
                GLCD_DrawString(0, (selected + 2) * 24, display_text);
                osMutexRelease(glcd_mutex);
            }
        }
        prev_joystick = joystick;
        osDelay(20);
    }

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    display_menu(-1);
}

void adjust_threshold(ActuatorType type) {
    char buffer[20];
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    GLCD_SetForegroundColor(COLOR_BLUE);
    snprintf(buffer, sizeof(buffer), "Set %s Threshold", actuators[type

].name);
    GLCD_DrawString(0, 0, buffer);
    GLCD_DrawString(0, 2 * 24, "Use joystick Up/Down");
    GLCD_DrawString(0, 3 * 24, "to adjust value");
    GLCD_SetForegroundColor(COLOR_RED);
    GLCD_DrawString(0, 5 * 24, "Center to confirm");
    GLCD_SetForegroundColor(COLOR_BLACK);
    snprintf(buffer, sizeof(buffer), "Threshold: %d", *actuators[type].threshold);
    GLCD_DrawString(0, 4 * 24, buffer);
    osMutexRelease(glcd_mutex);

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time = osKernelSysTick();

        if (current_time - last_action >= DEBOUNCE_TIME_MS) {
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                *actuators[type].threshold += 10;
                if (*actuators[type].threshold > MAX_ADC_VALUE) *actuators[type].threshold = MAX_ADC_VALUE;
                last_action = current_time;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(COLOR_WHITE);
                GLCD_SetForegroundColor(COLOR_BLACK);
                GLCD_DrawString(0, 4 * 24, "                    ");
                snprintf(buffer, sizeof(buffer), "Threshold: %d", *actuators[type].threshold);
                GLCD_DrawString(0, 4 * 24, buffer);
                osMutexRelease(glcd_mutex);
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                *actuators[type].threshold -= 10;
                if (*actuators[type].threshold < 0) *actuators[type].threshold = 0;
                last_action = current_time;
                osMutexWait(glcd_mutex, osWaitForever);
                GLCD_SetBackgroundColor(COLOR_WHITE);
                GLCD_SetForegroundColor(COLOR_BLACK);
                GLCD_DrawString(0, 4 * 24, "                    ");
                snprintf(buffer, sizeof(buffer), "Threshold: %d", *actuators[type].threshold);
                GLCD_DrawString(0, 4 * 24, buffer);
                osMutexRelease(glcd_mutex);
            }
            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                last_action = current_time;
                break;
            }
        }
        prev_joystick = joystick;
        osDelay(20);
    }

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    display_menu(-1);
}

void adjust_durations(void) {
    static int first_call = 1;
    int selected = 0;
    int prev_selected = -1;
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    if (first_call) {
        osMutexWait(glcd_mutex, osWaitForever);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "Adjust ON Durations");
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            char display_text[35];
            GLCD_SetBackgroundColor(i == selected ? COLOR_GRAY : COLOR_WHITE);
            GLCD_DrawString(0, (i + 2) * 24, "                    ");
            snprintf(display_text, sizeof(display_text), i == selected ? "> %s: %d ms" : "%s: %d ms",
                     actuators[i].name, *actuators[i].duration);
            GLCD_DrawString(0, (i + 2) * 24, display_text);
        }
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 6 * 24, "U/D:Select L/R:Adjust");
        GLCD_DrawString(0, 7 * 24, "Center:Confirm");
        osMutexRelease(glcd_mutex);
        first_call = 0;
    }

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time = osKernelSysTick();

        if (current_time - last_action >= DEBOUNCE_TIME_MS) {
            int update_display = 0;
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                prev_selected = selected;
                selected = (selected > 0) ? selected - 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                prev_selected = selected;
                selected = (selected < ACTUATOR_COUNT - 1) ? selected + 1 : ACTUATOR_COUNT - 1;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_LEFT) && !(prev_joystick & JOYSTICK_LEFT)) {
                *actuators[selected].duration -= 100;
                if (*actuators[selected].duration < MIN_DURATION_MS) *actuators[selected].duration = MIN_DURATION_MS;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_RIGHT) && !(prev_joystick & JOYSTICK_RIGHT)) {
                *actuators[selected].duration += 100;
                if (*actuators[selected].duration > MAX_DURATION_MS) *actuators[selected].duration = MAX_DURATION_MS;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                first_call = 1;
                last_action = current_time;
                break;
            }

            if (update_display) {
                osMutexWait(glcd_mutex, osWaitForever);
                char display_text[35];
                if (prev_selected != -1) {
                    GLCD_SetBackgroundColor(COLOR_WHITE);
                    GLCD_DrawString(0, (prev_selected + 2) * 24, "                    ");
                    snprintf(display_text, sizeof(display_text), "%s: %d ms",
                             actuators[prev_selected].name, *actuators[prev_selected].duration);
                    GLCD_SetForegroundColor(COLOR_BLUE);
                    GLCD_DrawString(0, (prev_selected + 2) * 24, display_text);
                }
                GLCD_SetBackgroundColor(COLOR_GRAY);
                GLCD_DrawString(0, (selected + 2) * 24, "                    ");
                snprintf(display_text, sizeof(display_text), "> %s: %d ms",
                         actuators[selected].name, *actuators[selected].duration);
                GLCD_SetForegroundColor(COLOR_BLUE);
                GLCD_DrawString(0, (selected + 2) * 24, display_text);
                osMutexRelease(glcd_mutex);
            }
        }
        prev_joystick = joystick;
        osDelay(20);
    }

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    display_menu(-1);
}

void control_timers(void) {
    static int first_call = 1;
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;
    char buffer[32];
    int prev_selected_field = -1;
    int prev_selected_actuator = -1;
    int update_display = 0;

    // Initialize timer settings
    for (int i = 0; i < ACTUATOR_COUNT; i++) {
        timers[i].hours = 0;
        timers[i].minutes = 0;
        timers[i].seconds = 30;
        timer_triggered[i] = 0;
    }
    selected_actuator = 0;
    selected_field = 0;

    // Initial display setup
    if (first_call) {
        osMutexWait(glcd_mutex, osWaitForever);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "Control Timers");
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 7 * 24, "U/D:Select Act L/R:Field");
        GLCD_DrawString(0, 8 * 24, "Center:Exit");
        osMutexRelease(glcd_mutex);
        first_call = 0;
        update_display = 1;
    }

    while (1) {
        // Check for timer triggers
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            if (current_time.hours == timers[i].hours &&
                current_time.minutes == timers[i].minutes &&
                current_time.seconds == timers[i].seconds) {
                timer_triggered[i] = 1;
                osSemaphoreRelease(actuator_sems[i]);
            }
        }

        // Handle joystick input
        uint32_t joystick = read_joystick();
        uint32_t current_time = osKernelSysTick();

        if (current_time - last_action >= DEBOUNCE_TIME_MS) {
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                prev_selected_actuator = selected_actuator;
                selected_actuator = (selected_actuator > 0) ? selected_actuator - 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                prev_selected_actuator = selected_actuator;
                selected_actuator = (selected_actuator < ACTUATOR_COUNT - 1) ? selected_actuator + 1 : ACTUATOR_COUNT - 1;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_LEFT) && !(prev_joystick & JOYSTICK_LEFT)) {
                prev_selected_field = selected_field;
                selected_field = (selected_field > 0) ? selected_field - 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_RIGHT) && !(prev_joystick & JOYSTICK_RIGHT)) {
                prev_selected_field = selected_field;
                selected_field = (selected_field < 2) ? selected_field + 1 : 2;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP) && selected_field == 0) {
                timers[selected_actuator].hours = (timers[selected_actuator].hours < 23) ? timers[selected_actuator].hours + 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN) && selected_field == 0) {
                timers[selected_actuator].hours = (timers[selected_actuator].hours > 0) ? timers[selected_actuator].hours - 1 : 23;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP) && selected_field == 1) {
                timers[selected_actuator].minutes = (timers[selected_actuator].minutes < 59) ? timers[selected_actuator].minutes + 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN) && selected_field == 1) {
                timers[selected_actuator].minutes = (timers[selected_actuator].minutes > 0) ? timers[selected_actuator].minutes - 1 : 59;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP) && selected_field == 2) {
                timers[selected_actuator].seconds = (timers[selected_actuator].seconds < 59) ? timers[selected_actuator].seconds + 1 : 0;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN) && selected_field == 2) {
                timers[selected_actuator].seconds = (timers[selected_actuator].seconds > 0) ? timers[selected_actuator].seconds - 1 : 59;
                update_display = 1;
                last_action = current_time;
            }
            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                first_call = 1;
                last_action = current_time;
                break;
            }
        }

        // Update display
        if (update_display || prev_selected_actuator != selected_actuator || prev_selected_field != selected_field) {
            osMutexWait(glcd_mutex, osWaitForever);
            GLCD_SetBackgroundColor(COLOR_WHITE);
            for (int i = 0; i < ACTUATOR_COUNT; i++) {
                GLCD_SetForegroundColor(i == selected_actuator ? COLOR_BLUE : COLOR_BLACK);
                snprintf(buffer, sizeof(buffer), "%s%s:%02d:%02d:%02d",
                         i == selected_actuator ? "> " : "  ",
                         actuators[i].name,
                         timers[i].hours,
                         timers[i].minutes,
                         timers[i].seconds);
                GLCD_DrawString(0, (i + 2) * 24, "                    ");
                GLCD_DrawString(0, (i + 2) * 24, buffer);
                if (i == selected_actuator) {
                    // Highlight selected field
                    int x_offset = (selected_field == 0) ? 30 : (selected_field == 1) ? 60 : 90;
                    GLCD_SetForegroundColor(COLOR_RED);
                    GLCD_DrawChar(x_offset, (i + 2) * 24, '^');
                }
            }
            osMutexRelease(glcd_mutex);
            update_display = 0;
            prev_selected_actuator = selected_actuator;
            prev_selected_field = selected_field;
        }

        prev_joystick = joystick;
        osDelay(20);
    }

    osMutexWait(glcd_mutex, osWaitForever);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    osMutexRelease(glcd_mutex);
    display_menu(-1);
}

void menu_thread(void const *arg) {
    init_gpio();
    GLCD_Initialize();
    GLCD_SetFont(&GLCD_Font_16x24);
    display_menu(-1);

    int prev_menu = selected_menu;
    while (1) {
        uint32_t joystick = read_joystick();
        int updated = 0;

        if (!joystick) {
            osDelay(20);
            continue;
        }

        if ((joystick & JOYSTICK_UP) && selected_menu > 0) {
            selected_menu--;
            updated = 1;
        } else if ((joystick & JOYSTICK_DOWN) && selected_menu < 6) {
            selected_menu++;
            updated = 1;
        } else if (joystick & JOYSTICK_CENTER) {
            switch (selected_menu) {
                case 0: show_sensors(); break;
                case 1: control_actuators(); break;
                case 2: adjust_threshold(ACTUATOR_HEATER); break;
                case 3: adjust_threshold(ACTUATOR_SPRINKLER); break;
                case 4: adjust_threshold(ACTUATOR_LIGHT); break;
                case 5: adjust_durations(); break;
                case 6: control_timers(); break;
            }
        }

        if (updated || selected_menu != prev_menu) {
            display_menu(prev_menu);
            prev_menu = selected_menu;
        }
        osDelay(20);
    }
}

// Thread Definitions
osThreadDef(sensor_thread, osPriorityNormal, 1, 0);
osThreadDef(uart_thread, osPriorityNormal, 1, 0);
osThreadDef(monitor_thread, osPriorityNormal, 1, 0);
osThreadDef(control_thread, osPriorityNormal, 1, 0);
osThreadDef(uart_receive_thread, osPriorityNormal, 1, 0);
osThreadDef(menu_thread, osPriorityNormal, 1, 0);

// ------PHASE 1: SYNCHRONIZATION OBJECTS DEFINITIONS------
// Mutex and Semaphore Definitions
osMutexDef(adc_mutex);
osMutexDef(glcd_mutex);
osSemaphoreDef(heater_sem);
osSemaphoreDef(sprinkler_sem);
osSemaphoreDef(light_sem);
// ------PHASE 1: SYNCHRONIZATION OBJECTS DEFINITIONS------

// Main Function
int main(void) {
    SystemCoreClockUpdate();
    init_hardware();

    osKernelInitialize();
    adc_mutex = osMutexCreate(osMutex(adc_mutex));
    glcd_mutex = osMutexCreate(osMutex(glcd_mutex));
    actuator_sems[ACTUATOR_HEATER] = osSemaphoreCreate(osSemaphore(heater_sem), 1);
    actuator_sems[ACTUATOR_SPRINKLER] = osSemaphoreCreate(osSemaphore(sprinkler_sem), 0);
    actuator_sems[ACTUATOR_LIGHT] = osSemaphoreCreate(osSemaphore(light_sem), 0);

    osThreadCreate(osThread(sensor_thread), NULL);
    osThreadCreate(osThread(uart_thread), NULL);
    osThreadCreate(osThread(uart_receive_thread), NULL);
    osThreadCreate(osThread(menu_thread), NULL);
    
    static ActuatorType types[ACTUATOR_COUNT] = {ACTUATOR_HEATER, ACTUATOR_SPRINKLER, ACTUATOR_LIGHT};
    for (int i = 0; i < ACTUATOR_COUNT; i++) {
        osThreadCreate(osThread(monitor_thread), &types[i]);
        osThreadCreate(osThread(control_thread), &types[i]);
    }

    osKernelStart();
    while (1);
}