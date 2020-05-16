/*
 * NTC.c
 *
 *  Created on: May 10, 2020
 *      Author: Zoltan Gere
 */

#include <msp430.h>

extern volatile unsigned int ADCResult;



unsigned int Take50ADCMeas(void)
{
    unsigned char CalibCounter = 0;
    unsigned int Value = 0;

    while (CalibCounter < 50)
    {
//        P3OUT ^= BIT4;
        CalibCounter++;
        while (ADC10CTL1 & BUSY)
            ;
        ADC10CTL0 |= ADC10ENC | ADC10SC;       // Start conversion
        __bis_SR_register(CPUOFF + GIE);      // LPM0, ADC10_ISR will force exit
        __no_operation();
        Value += ADCResult;
    }
    Value = Value / 50;

    return Value;
}

void TakeADCMeas(void)
{
    while (ADC10CTL1 & BUSY)
        ;
    ADC10CTL0 |= ADC10ENC | ADC10SC;       // Start conversion
    __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
    __no_operation();                       // For debug only
}

void SetupThermistor(void)
{
    // ~16KHz sampling
    //Turn on Power
    P2DIR |= BIT7;
    P2OUT |= BIT7;

    // Configure ADC
    P1SEL1 |= BIT4;
    P1SEL0 |= BIT4;

    // Allow for settling delay
    __delay_cycles(50000);

    // Configure ADC
    ADC10CTL0 &= ~ADC10ENC;
    ADC10CTL0 = ADC10SHT_7 + ADC10ON;        // ADC10ON, S&H=192 ADC clks
    // ADCCLK = MODOSC = 5MHz
    ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0;
    ADC10CTL2 = ADC10RES;                    // 10-bit conversion results
    ADC10MCTL0 = ADC10INCH_4;                // A4 ADC input select; Vref=AVCC
    ADC10IE = ADC10IE0;                    // Enable ADC conv complete interrupt
}

void ShutDownTherm(void)
{
    // Turn off Vcc
    P2DIR &= ~BIT7;
    P2OUT &= ~BIT7;

    // Turn off ADC Channel
    P1SEL1 &= ~BIT4;
    P1SEL0 &= ~BIT4;

    // Turn off ADC
    ADC10CTL0 &= ~(ADC10ENC + ADC10ON);
    ADC10IE &= ~ADC10IE0;
    ADC10IFG = 0;
}

unsigned int MeasureTemp(void)
{
    // variable initialization
    unsigned int ADCTemp = 0;

    // One time setup and calibration
    SetupThermistor();

    // Take 1 ADC Sample
    TakeADCMeas();

    // Take 50 ADC Sample average;
    ADCTemp = Take50ADCMeas();

    // A/D Reading is in ADCResult
    // 0V - 3.3V corresponds to 0 - 4095
//    ADCTemp = ADCResult;

    // turn off Thermistor bridge for low power
    ShutDownTherm();

    return ADCTemp;
}
