#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "HLW8012.h"

// #define RELAY_PIN                       4
// #define SEL_PIN                         12
// #define CF1_PIN                         14
// #define CF_PIN                          5

// Check values every 2 seconds
#define UPDATE_TIME                     2000

// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the HLW8012 when closed

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

void calibrate() {

    // Let's first read power, current and voltage
    // with an interval in between to allow the signal to stabilise:

    HLW8012_getActivePower();

    HLW8012_setMode(MODE_CURRENT);
    // unblockingDelay(2000);
    vTaskDelay(2000/ portTICK_PERIOD_MS);
    HLW8012_getCurrent();

    HLW8012_setMode(MODE_VOLTAGE);
    // unblockingDelay(2000);
    vTaskDelay(2000/ portTICK_PERIOD_MS);
    HLW8012_getVoltage();

    // Calibrate using a 60W bulb (pure resistive) on a 230V line
    HLW8012_expectedActivePower(60.0);
    HLW8012_expectedVoltage(230.0);
    HLW8012_expectedCurrent(60.0 / 230.0);

    // Show corrected factors
    printf("[HLW] New current multiplier : %0.2f ",HLW8012_getCurrentMultiplier());
    printf("[HLW] New voltage multiplier : %0.2f",HLW8012_getVoltageMultiplier());
    printf("[HLW] New power multiplier : %0.2f ",HLW8012_getPowerMultiplier());

}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    printf("Pham Thi Yen Linh!\n");
    HLW8012_init(CF_PIN,CF1_PIN,SEL_PIN,1 ,0,false);
    HLW8012_setResistors(CURRENT_RESISTOR,VOLTAGE_RESISTOR_UPSTREAM,VOLTAGE_RESISTOR_DOWNSTREAM);
    printf("[HLW] Default current multiplier : %0.2f\n",HLW8012_getCurrentMultiplier());
    printf("[HLW] Default voltage multiplier : %0.2f\n",HLW8012_getVoltageMultiplier());
    printf("[HLW] Default power multiplier : %0.2f \n",HLW8012_getPowerMultiplier());

    calibrate();
    printf("Pham Thi Yen Linh 1!\n");

    while(1){


    // This UPDATE_TIME should be at least twice the minimum time for the current or voltage
    // signals to stabilize. Experimentally that's about 1 second.

        printf("[HLW] Active Power (W)    : %d ",HLW8012_getActivePower());
        printf("[HLW] Voltage (V)         : %d ",HLW8012_getVoltage());
        printf("[HLW] Current (A)         : %d ",HLW8012_getCurrent()); 
        printf("[HLW] Apparent Power (VA) : %d ",HLW8012_getApparentPower()); 
        printf("[HLW] Power Factor        : %d \n",(int) (100 * HLW8012_getPowerFactor())); 
        
        // When not using interrupts we have to manually switch to current or voltage monitor
        // This means that every time we get into the conditional we only update one of them
        // while the other will return the cached value.
        HLW8012_toggleMode();
        printf("Time after :%lld\n",esp_timer_get_time());
        vTaskDelay(2000/ portTICK_PERIOD_MS);
        printf("Time before :%lld\n",esp_timer_get_time());
    }

}


