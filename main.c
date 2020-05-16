/**************************************************************************
 * SensorModule
 *
 * @brief   Sensor controller board
 *          Read README.md for further descriptions
 *
 *  Created on: May 9, 2020
 *      Author: Zoltan Gere
 *
 *************************************************************************/

#include <msp430.h>

#include <string.h>
#include <stdio.h>

#include "UART.h"
#include "NTC.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    int ID;
    char sensorType;                // 'T'emp, 'H'umidity, 'P'ressure
    char sensorUnit;                // 'C'elsius, 'R'H, 'P'ascal
    unsigned int sensorAddress;     // 0 - Onboard NTC, 0x03 - 0x77 - I2C address of sensor
    unsigned int slaveAddress;      // I2C slave address toward controller
    float scale;                    // Raw-to-real multiplier
    float offset;                   // Raw-to-real addition
} channel_t;

// Physical unit constants
const char TempUnit = 'C';
const char HumidUnit = 'R';
const char PressUnit = 'P';

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Global variables
// For system
volatile unsigned int ADCResult = 0;        // Used by ADC interrupt handler routine
int i2cRec = 0;                             // I2C receive buffer

// Channels
channel_t Channel0;
int Ch0Value = 0;                           // Adjusted measurement value



/**************************************************************************
 * @brief  Initializes system
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void SystemInit(void)
{
    // Startup clock system in max. DCO setting ~8MHz
    // This value is closer to 10MHz on untrimmed parts
    CSCTL0_H = 0xA5;                          // Unlock register
    CSCTL1 |= DCOFSEL0 + DCOFSEL1;            // Set max. DCO setting
    CSCTL2 = SELA_1 + SELS_3 + SELM_3;        // set ACLK = vlo; MCLK = DCO
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;        // set all dividers
    CSCTL0_H = 0x01;                          // Lock Register

    // Turn off temp.
    REFCTL0 |= REFTCOFF;
    REFCTL0 &= ~REFON;

    // Enable switches
    // P4.0 and P4.1 are configured as switches
    // Port 4 has only two pins
    P4OUT |= BIT0 + BIT1;                      // Configure pullup resistor
    P4DIR &= ~(BIT0 + BIT1);                  // Direction = input
    P4REN |= BIT0 + BIT1;                     // Enable pullup resistor
    P4IES &= ~(BIT0 + BIT1);                    // P4.0 Lo/Hi edge interrupt
    P4IE = BIT0 + BIT1;                         // P4.0 interrupt enabled
    P4IFG = 0;                                // P4 IFG cleared

    // Enable LEDs
    P3OUT &= ~(BIT6 + BIT7 + BIT5 + BIT4);
    P3DIR |= BIT6 + BIT7 + BIT5 + BIT4;
    PJOUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);
    PJDIR |= BIT0 + BIT1 + BIT2 + BIT3;

    // Terminate Unused GPIOs
    // P1.0 - P1.5 is unused
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5);
    P1DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5);
    P1REN |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT5);

    // P1.4 is used as input from NTC voltage divider
    // Set it to output low
    P1OUT &= ~BIT4;
    P1DIR |= BIT4;

    // P2.2 - P2.6 is unused
    P2OUT &= ~(BIT2 + BIT3 + BIT4 + BIT5 + BIT6);
    P2DIR &= ~(BIT2 + BIT3 + BIT4 + BIT5 + BIT6);
    P2REN |= (BIT2 + BIT3 + BIT4 + BIT5 + BIT6);

    // Configure UART pins P2.0 & P2.1
    P2SEL1 |= BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure I2C pins P1.6 & P1.7
    P1SEL1 |= BIT6 + BIT7;
    P1SEL0 &= ~(BIT6 + BIT7);

    // P2.7 is used to power the voltage divider for the NTC thermistor
    P2OUT &= ~BIT7;
    P2DIR |= BIT7;

    // P3.0,P3.1 and P3.2 are accelerometer inputs
    P3OUT &= ~(BIT0 + BIT1 + BIT2);
    P3DIR &= ~(BIT0 + BIT1 + BIT2);
    P3REN |= BIT0 + BIT1 + BIT2;

    // PJ.0,1,2,3 are used as LEDs
    // crystal pins for XT1 are unused
    PJOUT &= ~(BIT4 + BIT5);
    PJDIR &= ~(BIT4 + BIT5);
    PJREN |= BIT4 + BIT5;
}



/**************************************************************************
 * @brief  Long Delay
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void LongDelay()
{
    __delay_cycles(4000000);
    __no_operation();
}



/**************************************************************************
 * @brief  Initialize I2C bus in slave mode
 *
 * @param   id - Which I2C address register to set up
 *          sa - I2C slave address to set
 *
 * @return none
 *************************************************************************/
void I2CSlaveInit(unsigned int id, unsigned int sa)
{
    if ( (id <= 3))
    {
//      1.  Set UCSWRST(BIS.B#UCSWRST,&UCxCTL1).
        UCB0CTL1 |= UCSWRST;
//      2.  Initialize all eUSCI_B registers with UCSWRST= 1 (including UCxCTL1).
        UCB0CTLW0 |= UCMODE_3;
        switch(id)
        {
        case 0:
            UCB0I2COA0 = (1 << 10) | sa;
            break;
        case 1:
            UCB0I2COA1 = (1 << 10) | sa;
            break;
        case 2:
            UCB0I2COA2 = (1 << 10) | sa;
            break;
        case 3:
            UCB0I2COA3 = (1 << 10) | sa;
            break;
        }

//      3.  Configure ports.
        // Pins already configured

//      4.  ClearUCSWRSTthroughsoftware(BIC.B#UCSWRST,&UCxCTL1).
        UCB0CTL1 &= ~UCSWRST;
//      5.  Enable interrupts(optional).
        switch(id)
        {
        case 0:
            UCB0IE |= UCTXIE0 + UCRXIE0;
            break;
        case 1:
            UCB0IE |= UCTXIE1 + UCRXIE1;
            break;
        case 2:
            UCB0IE |= UCTXIE2 + UCRXIE2;
            break;
        case 3:
            UCB0IE |= UCTXIE3 + UCRXIE3;
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * main.c
 */
int main(void)
{
//    char input[16];
    char reading[7];
    unsigned int rawTemp;

    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    // Leds: PJ.0 PJ.1 PJ.2 PJ.3 P3.4 P3.5 P3.6 P3.7
    // NTC: P1.4
    // Switches: P4.0 P4.1

    Channel0.ID = 0;
    Channel0.sensorType = 'T';
    Channel0.sensorUnit = TempUnit;
    Channel0.sensorAddress = 0;
    Channel0.slaveAddress = 0x20;
    Channel0.scale = 1.0;
    Channel0.offset = 0.0;

    SystemInit();                           // Init the Board
    UARTInit();                             // Init UART interface
    I2CSlaveInit(0, Channel0.slaveAddress);                         // Init I2C interface

    Send("SensorModule\r\nFIRMWARE Version 0.1\r\n");
//    Receive(input, 16);
//    Send("\r\nReceived: ");
//    Send(input);
//    Send("\r\n");

    // Clear LEDs
    PJOUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);

    while (1)
    {
        PJOUT |= BIT0;

        LongDelay();

        PJOUT &= ~BIT0;

        LongDelay();

        rawTemp = MeasureTemp();

        snprintf(reading, 7, "%d", rawTemp);
        Send(reading);
        Send("\t");

        Ch0Value = (int)((((float) rawTemp * Channel0.scale) + Channel0.offset) * 256 - 32768 );       // Measure ADC and calculate temperature

        snprintf(reading, 7, "%d", Ch0Value);
        Send(reading);
        Send("\r\n");
    }

    // return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Interrupt Service Routines

/**************************************************************************
 * @brief  ADC10 ISR
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    switch (__even_in_range(ADC10IV, ADC10IV_ADC10IFG))
    {
    case ADC10IV_NONE:
        break;               // No interrupt
    case ADC10IV_ADC10OVIFG:
        break;         // conversion result overflow
    case ADC10IV_ADC10TOVIFG:
        break;        // conversion time overflow
    case ADC10IV_ADC10HIIFG:
        break;         // ADC10HI
    case ADC10IV_ADC10LOIFG:
        break;         // ADC10LO
    case ADC10IV_ADC10INIFG:
        break;         // ADC10IN
    case ADC10IV_ADC10IFG:
        ADCResult = ADC10MEM0;
        __bic_SR_register_on_exit(CPUOFF);
        break;                          // Clear CPUOFF bit from 0(SR)
    default:
        break;
    }
}

volatile int cnt = 0;                   // Counter for bytes transmitted

/**************************************************************************
 * @brief  USCI_B0 ISR
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    switch (__even_in_range(UCB0IV, 0x1e))
    {
    case 0x00:      // Vector 0: No interrupts
        break;
    case 0x0a:       // Vector 22: RXIFG3
        break;
    case 0x0c:       // Vector 22: TXIFG3
        break;
    case 0x0e:       // Vector 22: RXIFG2
        break;
    case 0x10:       // Vector 22: TXIFG2
        break;
    case 0x12:       // Vector 22: RXIFG1
        break;
    case 0x14:       // Vector 22: TXIFG1
        break;
    case 0x16:       // Vector 22: RXIFG0
        // RX ISR is called for every byte received by a master device.
        // inside the eUSCI_B RX interrupt service routine

        // ((UCB0CTLW0 & UCTR) == 1)        // Transmitter

        i2cRec = UCB0RXBUF;
        cnt = 0;
        break;
    case 0x18:      // Vector 24: TXIFG0
        // The TX ISR is executed each time the master requests a byte.
        // inside the eUSCI_B TX interrupt service routine
        switch (i2cRec)
        {
        case 0:     // Raw measurement
            if (cnt == 0){
                UCB0TXBUF = ADCResult;
                cnt++;
            } else
            {
                UCB0TXBUF = (ADCResult >> 8);
            }
            break;
        case 1:     // Adjusted measurement value
            if (cnt == 0){
                UCB0TXBUF = Ch0Value;
                cnt++;
            } else
            {
            UCB0TXBUF = (Ch0Value >> 8);
            }
            break;

        case 2:     // Sensor type
            UCB0TXBUF = Channel0.sensorType;
            break;
        case 3:     // Sensor unit
            UCB0TXBUF = Channel0.sensorUnit;
            break;
        default:
            UCB0TXBUF = 0xFFFF;     // Unknown / error state
            break;
        }
        break;
    default:
        break;
    }
}
