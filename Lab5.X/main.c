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
char rx_char();
void tx_char(char a);
 __interrupt() void  isr_rx();


char data;
//
void setupUART(){
    // Setup I/O
    TRISC = 0x0;
    TRISC = 0x80;                  //Rx-Input   Tx-Output
    
    // Initialize UART
    TXSTAbits.BRGH = 1; 	// High Baud Rate Select Bit
	SPBRG = (_XTAL_FREQ - 9600*16)/(9600*16);
    TXSTAbits.SYNC = 0; //  EUSART Mode Select bit
    RCSTAbits.SPEN = 1; // Serial Port Enable bit
    RCSTAbits.CREN = 1; // Continuous Receive Enable bit
    TXSTAbits.TXEN = 1; // Transmit Enable bit
    TXSTAbits.TX9 = 0; // Selects 8-bit data transmission
    
    INTCONbits. GIE = 1;        // Global interrupt enable
    INTCONbits.PEIE = 1;        // Peripheral Interrupt enable bit
    PIE1bits.RCIE = 1;          // Interrupt Enable bit RX
    
}

void setupLED(){
    PORTB = 0x0;
    TRISB = 0x0;
    ANSEL = 0x0;
    ANSELH = 0x0;

}

char rx_char(void)
{
    while(!RCIF);                   //Wait till RCREG is full
    RB2 = 1;
    __delay_ms(100);
    RB2 = 0;
    __delay_ms(100);
    return RCREG;                   //Return value in received data
}

void tx_char(char a)
{
    // Requirement 1
    RB0 = 1;
    __delay_ms(100);
    RB0 = 0;
    TXREG=a;                        //Load TXREG register with data to be sent
    while(!TRMT);                   //Wait till transmission is complete
}   

__interrupt() void ISR(void)
{
    // Requirement 2
    RB1 = 1;
    __delay_ms(100);
    RB1 = 0;
    __delay_ms(100);
    if(RCIF==1)                     //Polling for reception interrupt
    {
        if(RCSTAbits.OERR) // Overrun bit error, clear CREN to clear
        {
            // Requirement 4, blink DS3 when overrun error
            RB3 = 1;
            __delay_ms(100);
            RB3 = 0;
            __delay_ms(100);
            
            CREN = 0;
            NOP();
            CREN = 1;
        }
        
        data = rx_char();                //Receiving data
        
        // Requirement 5, blink all LEDs when received expected data
        if (data == 'a')
        {
            RB0 = 1;
            RB1 = 1;
            RB2 = 1;
            RB3 = 1;
            __delay_ms(5000);
            RB0 = 0;
            RB1 = 0;
            RB2 = 0;
            RB3 = 0;
            __delay_ms(100);
        }
        
    }      
}

void main(void) {
    
    setupUART();
    setupLED();
    
    while(1){
        tx_char('a');
        __delay_ms(100);
    }
    
    return;
    
}

