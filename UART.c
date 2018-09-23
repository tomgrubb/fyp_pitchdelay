///*
// * File:   UART.c
// * Author: Tom Grubb
// * 
// * Created on November 20, 2017, 11:05 AM
// */
//
//
//#include "xc.h"
//
//// send a byte over UART
//void UART_write_byte(uint8_t byte)
//{
//    while (U1STAbits.UTXBF); // Wait until UART1TX buffer filled
//    U1TXREG = byte; // place byte in buffer
//}
//
//// Send a string, followed by new line
//void UART_write_string(uint8_t *str)
//{
//    int i;
//    
//    for (i = 0; i < strlen(str); i++) {
//        UART_write_byte(str[i]);
//    }
//    UART_write_byte('\n');
//}
//
//// UART Initialization
//void UART_init(void)
//{   
//    TRISBbits.TRISB7 = 0; // define pin RP7 as OUTPUT
//    RPOR3bits.RP7R = 0b00011; // set UART TX pin as RP7
//    TRISBbits.TRISB6 = 1; // define pin RP6 as INPUT
//    RPINR18bits.U1RXR = 6; // set UART RX pin as RP6
//
//    U1MODE = 0x0000; // set all UART mode fields to zero
//    /*
//     * UARTEN = 0 (UART is disabled)
//     * USIDL = 0 (continue operation in idle mode)
//     * IREN = 0 (IrDA encoder and decoder are disabled)
//     * RTSMD = 0 (!UxRTS is in flow control mode)
//     * UEN = 0 (TX and RX pins are enable, all others are controlled by port latches)
//     * WAKE = 0 (wake-up is disabled)
//     * ABAUD = 0 (auto-baud is disabled)
//     * URXINV = 0 (RX idle state is 1)
//     * BRGH = 0 (low-speed mode)
//     * PDSEL = 0 (8-bit data, no parity)
//     * STTEL = 0 (1 stop bit)
//     */
//    U1BRG = 23; // BAUD RATE pre-scaler
//    U1STA = 0x0000; // reset U1STA register
//    
//    U1MODEbits.UARTEN = 1; // enable UART 1
//    U1STAbits.UTXEN = 1; // enable UART 1 TX
//}
