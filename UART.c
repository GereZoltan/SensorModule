/*
 * UART.c
 *
 *  Created on: May 10, 2020
 *      Author: Zoltan Gere
 */

#include <msp430.h>

#include "UART.h"

void UARTInit() {
    // 1. Set UCSWRST
    // 2. Initialize all eUSCI_A registers with UCSWRST = 1
    // 3. Configure ports
    // 4. Clear UCSWRST
    // 5. Enable interrupts through UCRXIE or UCTXIE
    //      See slau272d.pdf, Ch. 18.3.15.4
    // Configure UART 0
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 = UCSSEL_2;                      // Set SMCLK as UCLk
    UCA0BR0 = 52 ;                              // 9600 baud
    // 8000000/(9600*16) - INT(8000000/(9600*16))=0.083
    UCA0BR1 = 0;
    // UCBRFx = 1, UCBRSx = 0x49, UCOS16 = 1 (Refer User Guide)
    UCA0MCTLW = 0x4911 ;
    UCA0CTL1 &= ~UCSWRST;                     // release from reset
}



void Send(const char * text) {
    unsigned char counter = 0;

    while(text[counter] != '\0')
    {
        while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
        UCA0TXBUF = text[counter];
        counter++;
    }
}



void Receive(char * text, int bufSize) {
    unsigned char counter = 0;
    int finished = 0;

    while(!finished)
    {
        while (!(UCA0IFG&UCRXIFG));             // USCI_A0 RX buffer ready?
        text[counter] = UCA0RXBUF;

        if (counter == (bufSize - 2)) {         // Buffer full?
            text[counter+1] = '\0';             // Append null to string
            finished = 1;
        } else if ((text[counter] == '\r') || (text[counter] == '\n')) {    // End of line?
            text[counter] = '\0';               // Overwrite CR/LF with null
            finished = 1;
        } else {                                // Continue
            counter++;
        }
    }
}


