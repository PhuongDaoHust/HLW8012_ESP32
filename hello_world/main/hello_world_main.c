#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "HLW8012.h"

#define RELAY_PIN                       12
#define SEL_PIN                         5
#define CF1_PIN                         13
#define CF_PIN                          14

// Check values every 2 seconds
#define UPDATE_TIME                     2000

// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the HLW8012 when closed
#define CURRENT_MODE                    HIGH

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
    
    printf("Hello world!\n");
    HLW8012_init(11,12,13,1,0,false);
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %d\n", esp_get_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}


