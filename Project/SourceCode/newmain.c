/* 
 * File:   newmain.c
 * Author: abdulaziz mossa
 *
 * Created on December 11, 2024, 6:51 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// Configuration bits

#pragma config FOSC = HS        // Oscillator Selection bits
#pragma config WDTE = OFF       // Watchdog Timer Enable bit
#pragma config PWRTE = OFF      // Power-up Timer Enable bit
#pragma config BOREN = ON       // Brown-out Reset Enable bit
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits
#pragma config CP = OFF         // Flash Program Memory Code Protection bit

#define _XTAL_FREQ 8000000 // Define the crystal oscillator frequency

// Pin Definitions
#define LM35_AN0 RA0 // Analog input for LM35
#define GREEN_LED RB0
#define YELLOW_LED RB1
#define RED_LED RB2
#define BUTTON1 RA1
#define BUTTON2 RA2
#define BUTTON3 RA3
#define MOTOR_IN1 RD0
#define MOTOR_IN2 RD1
#define MOTOR_ENABLE RC2  // PWM output on RC2 (CCP1)

// Function Prototypes
void initADC();
unsigned int readADC(unsigned char channel);

void initPWM();
void setMotorSpeed(unsigned char dutyCycle);
void setMotorDirection(unsigned char direction);


void main(void) {
    // Initialize ports
    TRISA = 0xFF; // Set PORTA as input
    TRISB = 0x00; // Set PORTB as output
    TRISD = 0x00; // Set PORTD as output
    TRISC = 0xFB;   // Set RC2/CCP1 as output for PWM
    
   
    // Enable weak pull-ups for PORTA
    OPTION_REGbits.nRBPU = 0;

    // Initialize PWM
    initPWM(5000);
    initADC();
    
    unsigned int adcValue;
    float temperature;
    
    while (1) {
        // Read LM35 sensor value
        adcValue = readADC(LM35_AN0);
        temperature = adcValue * 0.488; // Convert ADC value to temperature in Celsius
        
        // Control LEDs based on temperature
        if (temperature < 15) {
            GREEN_LED = 1;
            YELLOW_LED = 0;
            RED_LED = 0;
        } else if (temperature >= 15 && temperature <= 25) {
            GREEN_LED = 0;
            YELLOW_LED = 1;
            RED_LED = 0;
        } else {
            GREEN_LED = 0;
            YELLOW_LED = 0;
            RED_LED = 1;
        }
        
        
    
        // Control motor speed and direction based on button input
        if (BUTTON1 == 1) {
            setMotorDirection(1); // Forward
            setMotorSpeed(128);   // Slow speed (50% duty cycle)
        } else if (BUTTON2 == 1) {
            setMotorDirection(1); // Forward
            setMotorSpeed(255);   // Fast speed (100% duty cycle)
        } else if (BUTTON3 == 1) {
            setMotorDirection(0); // Backward
            setMotorSpeed(128);   // Slow speed (50% duty cycle)
        } else {
            setMotorSpeed(0);     // Stop motor
        }
    
    }
    
}

void initADC() {
    ADCON0 = 0x41; // ADC enabled, channel 0 selected
    ADCON1 = 0x80; // Right justify result, Fosc/32
}

unsigned int readADC(unsigned char channel) {
    ADCON0 &= 0xC5; // Clear the channel selection bits
    ADCON0 |= channel << 3; // Select the required channel
    __delay_ms(2); // Acquisition time to charge hold capacitor
    GO_nDONE = 1; // Start conversion
    while (GO_nDONE); // Wait for conversion to complete
    return ((ADRESH << 8) + ADRESL); // Return the result
}

void initPWM(uint32_t freq) {
    uint16_t PR2_value =(_XTAL_FREQ/(freq*4))-1;
    if (PR2_value > 255){
        PR2_value = 255;
    }
    PR2 = PR2_value;       // Set the period 
    TMR2 = 0;         // Clear Timer2 register
    CCPR1L = 0x80;    // Set initial duty cycle to 50%
    CCP1CON = 0x0C;   // Configure CCP1 module in PWM mode
    T2CON = 0x04;     // Enable Timer2 with prescaler 1:1
    TMR2ON = 1;       // Start Timer2
}

void setMotorSpeed(unsigned char dutyCycle) {
    if (dutyCycle > 255) dutyCycle = 255; // Ensure dutyCycle is within range

    CCPR1L = dutyCycle >> 2; // Set the 8 MSBs
    CCP1CON &= 0xCF;         // Clear DC1B1 and DC1B0 bits
    CCP1CON |= (dutyCycle & 0x03) << 4; // Set the 2 LSBs
}

void setMotorDirection(unsigned char direction) {
   if (direction == 1) {
        MOTOR_IN1 = 1;
        MOTOR_IN2 = 0;
    } else {
        MOTOR_IN1 = 0;
        MOTOR_IN2 = 1;
    }
}
