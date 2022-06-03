#ifndef _HLW8012_H_
#define _HLW8012_H_

#include <stdio.h>
#include <stdint.h>


#define RELAY_PIN                       4
#define SEL_PIN                         12
#define CF1_PIN                         14
#define CF_PIN                          5

#define GPIO_INPUT_PIN_SEL      ((1ULL << CF_PIN) | (1ULL << CF1_PIN))
#define V_REF_HLW               2.43
#define V_REF_BL0               1.218

// The factor of a 1mOhm resistor
// as per recomended circuit in datasheet
// A 1mOhm resistor allows a ~30A max measurement
#define R_CURRENT           (0.001)

// This is the factor of a voltage divider of 6x 470K upstream and 1k downstream
// Sonoff Pow has 5x 470K
// Smart DGM outlet has 3x 680K
// as per recomended circuit in datasheet
#define R_VOLTAGE_HLW           ((5 * 470) + 1) //2821 //2350

// Frequency of the HLW8012 internal clock
#define F_OSC_HLW           (3579000)

// Maximum pulse with in microseconds
// If longer than this pulse width is reset to 0
// This value is purely experimental.
// Higher values allow for a better precission but reduce sampling rate
// and response speed to change
// Lower values increase sampling rate but reduce precission
// Values below 0.5s are not recommended since current and voltage output
// will have no time to stabilise
#define PULSE_TIMEOUT       (2000000l)
#define F_OSC           (3579000)

// CF1 mode
typedef enum {
    MODE_CURRENT = 0,
    MODE_VOLTAGE
} hlw8012_mode_t;


void  HLW8012_init(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen, uint8_t model,bool use_interrupts);
// currentWhen  - 1 for HLW8012 (old Sonoff Pow), 0 for BL0937
// model - 0 for HLW8012, 1 or other value for BL0937

void HLW8012_setMode(hlw8012_mode_t mode);
hlw8012_mode_t HLW8012_getMode();
hlw8012_mode_t HLW8012_toggleMode();

uint16_t HLW8012_getCurrent(); // A x100 (dyvide by 100 for Amper)
uint16_t HLW8012_getVoltage(); 
uint16_t HLW8012_getActivePower(); // moc czynna
uint16_t HLW8012_getApparentPower(); // moc pozorna
float HLW8012_getPowerFactor();
uint32_t HLW8012_getEnergy(); //in Ws
void HLW8012_resetEnergy();

void HLW8012_setResistors(float current, float voltage_upstream, float voltage_downstream);

void HLW8012_expectedCurrent(float current);
void HLW8012_expectedVoltage(uint16_t current);
void HLW8012_expectedActivePower(uint16_t power);

float HLW8012_getCurrentMultiplier();
float HLW8012_getVoltageMultiplier();
float HLW8012_getPowerMultiplier();

void HLW8012_setCurrentMultiplier(float current_multiplier);
void HLW8012_setVoltageMultiplier(float voltage_multiplier);
void HLW8012_setPowerMultiplier(float power_multiplier);
void HLW8012_resetMultipliers();

void external_interrupt_init();

#endif


