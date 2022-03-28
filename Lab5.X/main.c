/*
 * File:   main.c
 * Author: ponti
 *
 * Created on March 28, 2022, 10:06 AM
 */

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ            8000000

#include <xc.h>

void setupUART();
void setupLED();
void setupISR();
char UART_Read();
void UART_Write(char data);
//void __interrupt() isr_rx();
void __interrupt() isr_tx();

char data;

void setupUART(){
    // Setup I/O
    TRISC = 0x0;
    TRISCbits.TRISC6 = 1; // TX
    TRISCbits.TRISC7 = 1; // RX
    
    // Initialize UART
    BRGH = 1; 	// High Baud Rate Select Bit
	SPBRG = (_XTAL_FREQ - 9600*16)/(9600*16);
    SYNC = 0; //  EUSART Mode Select bit
    SPEN = 1; // Serial Port Enable bit
    CREN = 1; // Continuous Receive Enable bit
    TXEN = 1; // Transmit Enable bit
    TXSTAbits.TX9 = 0; // Selects 8-bit data transmission
}

void setupLED(){
    // set as digital outputs
    PORTB = 0x0;
    TRISB = 0x0;
    TRISBbits.TRISB0 = 1;    
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;    
    TRISBbits.TRISB3 = 1;

    // Set to no analog
    ANSEL = 0x0;
    ANSELH = 0x0;

}

void setupISRTx(){
    
//    OPTION_REGbits.T0CS = 0;    // Timer increments on instruction clock
//    OPTION_REGbits.INTEDG = 0;  // falling edge trigger the interrupt
//    INTCONbits.INTE = 1;        // enable the external interrupt
    INTCONbits. GIE = 1;        // Global interrupt enable
    INTCONbits.PEIE = 0;        // Peripheral Interrupt enable bit
    PIE1bits.TXIE = 1;          // Interrupt Enable bit TX
    
}

void setupISRRx(){
    
    INTCONbits. GIE = 1;        // Global interrupt enable
    INTCONbits.PEIE = 0;        // Peripheral Interrupt enable bit
    PIE1bits.RCIE = 1;          // Interrupt Enable bit RX
    
}

char UART_Read(){
  while(!RCIF);
  return RCREG;
}

void UART_Write(char data){
  while(!TRMT);
  TXREG = data; // Write register
}

//void __interrupt() isr_rx() // interrupt function 
//{
//    // Requirement 2, Interrupt raised DS1 blink
//    RB1 = 1;
//    __delay_ms(100);
//    RB1 = 0;
//
//    // EUSART Receive Interrupt Flag bit
//    if (RCIF) {
//        // Requirement 3, Receive buffer full blink DS2
//        RB2 = 1;
//        __delay_ms(100);
//        RB2 = 0;
//
//
//        if(RCSTAbits.OERR) // Overrun bit error, clear CREN to clear
//        {
//            // Requirement 4, blink DS3 when overrun error
//            RB3 = 1;
//            __delay_ms(100);
//            RB3 = 0;
//            
//            CREN = 0;
//            NOP();
//            CREN = 1;
//        }
//        
//        // Read data
//        data = RCREG;
//    
//        // Requirement 5, blink all LEDs when received expected data
//        if (data == 'a')
//        {
//            RB0 = 1;
//            RB1 = 1;
//            RB2 = 1;
//            RB3 = 1;
//            __delay_ms(100);
//            RB0 = 0;
//            RB1 = 0;
//            RB2 = 0;
//            RB3 = 0;
//        }
//        
//    }
//    
//}

void __interrupt() isr_tx() // interrupt function 
{
    // Requirement 2, Interrupt raised DS1 blink
    RB1 = 1;
    __delay_ms(100);
    RB1 = 0;

    // EUSART Transmit Interrupt Flag bit
    if (TXIF)
    {
        // Requirement 1, transmission blink DS0
        RB0 = 1;
        __delay_ms(100);
        RB0 = 0;
  
//        UART_Write('a'); 
        
    }
}

void main(void) {
    
    setupUART();
    setupLED();
    setupISRTx();
    
    while(1){
        UART_Write('a'); 

        __delay_ms(100);
    }
    
    return;
}
