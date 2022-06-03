#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "HLW8012.h"

#define PIN_ISR (CF_PIN | CF1_PIN )
#define GPIO_INPUT_CF_PIN     5
#define GPIO_INPUT_CF1_PIN    14
// #define PIN_ISR 0
#define ESP_INTR_FLAG_DEFAULT 0 
static xQueueHandle gpio_evt_queue = NULL;

uint8_t _cf_pin;
uint8_t _cf1_pin;
uint8_t _sel_pin;

float _current_resistor = R_CURRENT;
float _voltage_resistor = R_VOLTAGE_HLW;
float V_REF = V_REF_HLW;

float _current_multiplier; // Unit: us/A
float _voltage_multiplier; // Unit: us/V
float _power_multiplier;   // Unit: us/W

uint32_t _pulse_timeout = PULSE_TIMEOUT;    // Unit: us
volatile uint32_t _voltage_pulse_width = 0; // Unit: us
volatile uint32_t _current_pulse_width = 0; // Unit: us
volatile uint32_t _power_pulse_width = 0;   // Unit: us
volatile uint32_t _pulse_count = 0;

float _current = 0;
uint16_t _voltage = 0;
uint16_t _power = 0;

uint8_t _current_mode = 1;
uint8_t _model = 0;
volatile uint8_t _mode;

bool _use_interrupts;

volatile uint32_t _last_cf_interrupt = 0;
volatile uint32_t _last_cf1_interrupt = 0;
volatile uint32_t _first_cf1_interrupt = 0;

float HLW8012_getCurrentMultiplier() { return _current_multiplier; };
float HLW8012_getVoltageMultiplier() { return _voltage_multiplier; };
float HLW8012_getPowerMultiplier() { return _power_multiplier; };

void _calculateDefaultMultipliers();

void HLW8012_checkCFSignal()
{
    if ((esp_timer_get_time() - _last_cf_interrupt) > _pulse_timeout)
        _power_pulse_width = 0;
}
// system_get_time()
void HLW8012_checkCF1Signal()
{
    if ((esp_timer_get_time() - _last_cf1_interrupt) > _pulse_timeout)
    {
        if (_mode == _current_mode)
        {
            _current_pulse_width = 0;
        }
        else
        {
            _voltage_pulse_width = 0;
        }
        HLW8012_toggleMode();
    }
}

void HLW8012_init(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen, uint8_t model, bool use_interrupts)
{
    _cf_pin = cf_pin;
    _cf1_pin = cf1_pin;
    _sel_pin = sel_pin;
    _current_mode = currentWhen;
    _use_interrupts = use_interrupts;
    // gpio_reset_pin(_cf_pin); 
    // gpio_reset_pin(_cf1_pin);
    // gpio_reset_pin(_sel_pin);
    // gpio_set_direction(_cf_pin, GPIO_MODE_INPUT);
    // gpio_set_direction(_cf1_pin, GPIO_MODE_INPUT);
    // gpio_set_direction(_sel_pin, GPIO_MODE_OUTPUT);

    //zero-initialize the config structure.
    gpio_config_t io_output = {};
    //disable interrupt
    io_output.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_output.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_output.pin_bit_mask = (1ULL<<SEL_PIN);
    //disable pull-down mode
    io_output.pull_down_en = 0;
    //disable pull-up mode
    io_output.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_output);

    _calculateDefaultMultipliers();

    _mode = _current_mode;

    gpio_set_level(_sel_pin, _mode);
}

void _calculateDefaultMultipliers()
{
    _current_multiplier = (1000000.0 * 512 * V_REF / _current_resistor / 24.0 / F_OSC);
    _voltage_multiplier = (1000000.0 * 512 * V_REF * _voltage_resistor / 2.0 / F_OSC);
    _power_multiplier = (1000000.0 * 128 * V_REF * V_REF * _voltage_resistor / _current_resistor / 48.0 / F_OSC);
}

void HLW8012_setMode(hlw8012_mode_t mode)
{
    _mode = (mode == MODE_CURRENT) ? _current_mode : 1 - _current_mode;
    gpio_set_level(_sel_pin, _mode);
    if (_use_interrupts)
    {
        _last_cf1_interrupt = _first_cf1_interrupt = esp_timer_get_time();
    }
}

hlw8012_mode_t HLW8012_getMode()
{
    return (_mode == _current_mode) ? MODE_CURRENT : MODE_VOLTAGE;
}

hlw8012_mode_t HLW8012_toggleMode()
{
    hlw8012_mode_t new_mode = HLW8012_getMode() == MODE_CURRENT ? MODE_VOLTAGE : MODE_CURRENT;
    HLW8012_setMode(new_mode);
    return new_mode;
}

uint16_t HLW8012_getActivePower()
{
    HLW8012_checkCFSignal();

    _power = (_power_pulse_width > 0) ? _power_multiplier / _power_pulse_width : 0;
    return _power;
}

uint16_t HLW8012_getCurrent()
{

    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set _current to 0 too

    HLW8012_getActivePower();

    if (_power == 0)
    {
        _current_pulse_width = 0;
    }
    else
    {
        HLW8012_checkCF1Signal();
    }
    _current = (_current_pulse_width > 0) ? _current_multiplier / _current_pulse_width : 0;
    return (uint16_t)(_current * 100);
}

uint16_t HLW8012_getVoltage()
{
    HLW8012_checkCF1Signal();

    _voltage = (_voltage_pulse_width > 0) ? _voltage_multiplier / _voltage_pulse_width : 0;
    return _voltage;
}

uint16_t HLW8012_getApparentPower()
{
    float current = HLW8012_getCurrent();
    uint16_t voltage = HLW8012_getVoltage();
    return voltage * current;
}

float HLW8012_getPowerFactor()
{
    uint16_t active = HLW8012_getActivePower();
    uint16_t apparent = HLW8012_getApparentPower();
    if (active > apparent)
        return 1;
    if (apparent == 0)
        return 0;
    return (float)active / apparent;
}

uint32_t HLW8012_getEnergy()
{

    /*
    Pulse count is directly proportional to energy:
    P = m*f (m=power multiplier, f = Frequency)
    f = N/t (N=pulse count, t = time)
    E = P*t = m*N  (E=energy)
    */
    return _pulse_count * _power_multiplier / 1000000l;
}

void HLW8012_resetEnergy()
{
    _pulse_count = 0;
}

void HLW8012_expectedCurrent(float value)
{
    if (_current == 0)
        HLW8012_getCurrent();
    if (_current > 0)
        _current_multiplier *= (value / _current);
}

void HLW8012_expectedVoltage(uint16_t value)
{
    if (_voltage == 0)
        HLW8012_getVoltage();
    if (_voltage > 0)
        _voltage_multiplier *= ((float)value / _voltage);
}

void HLW8012_expectedActivePower(uint16_t value)
{
    if (_power == 0)
        HLW8012_getActivePower();
    if (_power > 0)
        _power_multiplier *= ((float)value / _power);
}

void HLW8012_resetMultipliers()
{
    _calculateDefaultMultipliers();
}

void HLW8012_setResistors(float current, float voltage_upstream, float voltage_downstream)
{
    if (voltage_downstream > 0)
    {
        _current_resistor = current;
        _voltage_resistor = (voltage_upstream + voltage_downstream) / voltage_downstream;
        _calculateDefaultMultipliers();
    }
}

void HLW8012_cf_interrupt(void)
{
    uint32_t now = esp_timer_get_time();
    _power_pulse_width = now - _last_cf_interrupt;
    _last_cf_interrupt = now;
    _pulse_count++;
    // printf("_pulse_count: %d \n",_pulse_count);
}

void HLW8012_cf1_interrupt(void)
{

    uint32_t now = esp_timer_get_time();
    if ((now - _first_cf1_interrupt) > _pulse_timeout)
    {
        uint32_t pulse_width;
        if (_last_cf1_interrupt == _first_cf1_interrupt)
        {
            pulse_width = 0;
        }
        else
        {
            pulse_width = now - _last_cf1_interrupt;
        }

        if (_mode == _current_mode)
        {
            _current_pulse_width = pulse_width;
        }
        else
        {
            _voltage_pulse_width = pulse_width;
        }

        _mode = 1 - _mode;

        gpio_set_level((_sel_pin), _mode);
        _first_cf1_interrupt = now;
    }

    _last_cf1_interrupt = now;
    // printf("_last_cf1_interrupt: %d \n",_last_cf1_interrupt);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{

    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            // if(flag == true){
            // hard_control = !hard_control;
            // gpio_set_level(LED,hard_control);
            // }
            // printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            // if (gpio_get_level(io_num) == 0)
            // {
            //     timer_start(TIMER_GROUP_0, TIMER_0);
            //     flag = true;
            // }
            // else
            // {
            //     flag = false;
            //     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
            //     timer_pause(TIMER_GROUP_0, TIMER_0);
            // }


        printf("sffcsdfcd \n");
            HLW8012_cf_interrupt();
            HLW8012_cf1_interrupt();
            
        }
    }
}

void external_interrupt_init()
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bit mask of the pins that you want to set,e.g.GPIO0/5
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // disable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    // change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_CF_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_INPUT_CF1_PIN, GPIO_INTR_ANYEDGE);
    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_CF_PIN, gpio_isr_handler, (void *)GPIO_INPUT_CF_PIN);
    gpio_isr_handler_add(GPIO_INPUT_CF1_PIN, gpio_isr_handler, (void *)GPIO_INPUT_CF1_PIN);
    printf("Config interrupt \n");
}
