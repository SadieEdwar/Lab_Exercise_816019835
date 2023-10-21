/*
 * @File:   lab1_q14_ 16019835.c
 * Author: Sadie Edwards
 * ID: 816019835
 *
 * @brief 
 * 
 */

// PIC18F4620 Configuration Bit Settings
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h> //Access to Registers bits
#include <stdio.h> //Use input and output functions
#include <time.h> //Access time structures 

/* Demo table-based cyclic task dispatcher for Linux with
multiple task slots */
#define CLOCKS_PER_SECOND 8000000 //oscillator frequency
#define SLOTX 4 //function table size
#define CYCLEX 5 //function table size
#define SLOT_T 5000 // 5 sec slot time
// gcc cyclicx.c -o cyclicx
int cycle=0, slot=0;
time_t tick; 
clock_t now, then;
struct tm n;

void putch(char c) //location for stdout to for outputting each character using printf()) via USART
{
    while(!TXIF) //Send data to EUSART receive buffer once not empty
        continue;
    TXREG = c; //Parsing the data to be printed to the Read/Write Transmit Buffer register
   
}

char getch_o(void)
{
    while(!PIR1bits.RCIF) //Checks if the EUSART Receive buffer, RCREG is full via the interrupt flags being enabled 
        continue;
    return RCREG; //The character from the user is stored and returned to the EUSART receive buffer
}

void USARTsetup(void)
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 0;
    RCSTAbits.SPEN = 1; //receive status and control register bit 7 (SPEN), serial port enabling RX/DT and TX/CK pins as serial port pins
    RCSTAbits.CREN = 1; // enable continuous receive bit
    TXSTAbits.BRGH = 1; // Selects High baud rate speed
    TXSTAbits.SYNC = 0; // EUSART asynchronous mode select bit
    TXSTAbits.TXEN = 1; // transmit status and control register
    SPBRGH = 0x00; //EUSART baud rate generator register high byte
    SPBRG = 0b00011001; //Setting the baud rate of 19230 for FOSC= 8.00MHz for asynchronous mode
    BAUDCON = 0b00000000;
}

void timer0setup(void)
{
    T0CON = 0b10000010; //A prescaler of 8 is used and the Timer0 is turned On
    INTCONbits.GIE = 1; //Enable all high priority interrupts
    INTCONbits.TMR0IE = 1; //Enables the TMR0 overflow interrupt
}

void __interrupt(high_priority) tcint(void)
{
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) //Checks if the TMR0 register has overflowed once the TMR0 overflow interrupt is enabled
    {
        INTCONbits.TMR0IF = 0; //clear the TMR0 register overflow interrupt flag
        tick++; //A counter to keep track each 250ms passing
    }
    
    return;
}

void sleep(int sec)
{
    now = tick;
    while((tick-now)< (sec*4));// waits for the seconds passed to the function to pass
}

void one()                                       // task code
{
    printf("task 1 running\n"); 
    sleep(1); //wait for 1 second
}
void two() 
{
    printf("task 2 running\n");
    sleep(2); //wait for 2 seconds
}
void three() 
{
    printf("task 3 running\n");
    sleep(3); //wait for 3 seconds
}
void four() {
    printf("task 4 running\n");
    sleep(4); //wait for 4 seconds
}
void five() 
{
    printf("task 5 running\n");
    sleep(5); //wait for 5 seconds
}
void burn() 
{
    clock_t bstart = tick; //stores the counter in the clock structure that measures processor and CPU time.
    while (tick < 20) {
            /* burn time for 5 seconds here */
    }
    tick = 0; //resets the time that has passed since completing each function
    printf("burn time = %2.2ldms\n\n", (20-bstart)*
    250); //prints the time it took between completing functions
    cycle = CYCLEX; //restarts the cycling through of arrays
}
void (*ttable[SLOTX] [CYCLEX]) () = //2 dimensional function pointers to the functions being called in the function arrays
{
    {one, two, burn, burn, burn},
    {one, three, burn, burn, burn}, 
    {one, four, burn, burn, burn}, 
    {burn, burn, burn, burn, burn} 
};
void main(void) 
{
    char result;
    result = getch_o();
    
    printf("%c", result);
    
    timer0setup(); //Function to enable hardware Timer0
    USARTsetup(); //Enable transmitting of data between the chip and keyboard
    int tps = 4; 
    printf("clock ticks/sec = %d\n\n", tps);
    while (1) //cycling through each element of the function pointer table created
    {
    for(slot=0; slot<SLOTX; slot++)
    for(cycle=0; cycle<CYCLEX; cycle++)
    (*ttable[slot] [cycle]) (); // dispatch next task
                    // from table
    }
return;
}	