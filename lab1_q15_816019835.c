/*
 * File:   lab1_q15_816019835.c\
 * Author: Sadie Edwards
 * ID: 816019835
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

#define WAITING 0 //Variable to enter the waiting state initialized at 0
#define LEDON 1  //Variable to enter the LEDON state initialized at 1
#define LEDOFF 2 //Variable to enter the LEDOFF state initialized at 2

unsigned int state; //Switch case condition for entering a certain state of the FSM
char keyPress = '1'; 
int tick = 0, receive = 0, ledON = 0, ledOFF = 1;

void putch(char c) //Empty stub that puts a character to stdout peripheral when using printf() function
{
    while(!TXIF) //Send data to EUSART receive buffer once not empty
        continue;
    TXREG = c; //Parsing the data to be printed to the Read/Write Transmit Buffer register
}

void __interrupt(low_priority) RCint(void) //Interrupt service routine for tasks with low priorities
{
    if (PIR1bits.RCIF && PIE1bits.RCIE) //Checks if the EUSART Receive buffer, RCREG is full via the interrupt flags being enabled  
    {
        RCIF = 0; //sets RCREG to empty
        keyPress = RCREG; //The character from the user is stored in a variable keyPress
        receive = 1; //identifier for when a character is received
        printf("%c", keyPress); //prints the character to the debugger console
    }
    return;
}

void __interrupt(high_priority) tmrint(void) //Interrupt service routine for tasks with high priorities
{
    if(PIE1bits.TMR2IE && PIR1bits.TMR2IF) //Checks if Timer2 has count 10ms by setting its interrupt flag bit once enabled by TMR2IE
    {
        PIR1bits.TMR2IF = 0; //Resets the Timer2 interrupt flag to off
        tick++; //A counter to keep track each 10ms passing
    }
    if(tick > 10000) //Checks if 100 seconds has passed
    {
        tick = 0; //Resets the counter
    }
    return;
}

void setupLED() 
{
    TRISDbits.TRISD2 = 0; //Register for data direction of Port D bit 2
    LATDbits.LATD2 = 0; //Register for read/write to output latch of Port D bit 2
}

void timer2setup(void) 
{
    T2CON = 0b01111111; //A prescaler and postscaler of 16is used and the Timer2 is turned On
    PIE1bits.TMR2IE = 1; //Enables the TMR2 to PR2 match interrupt
    PR2 = 0b01001110; //PR2 register holds the match value, 78 for TMR2
    IPR1bits.TMR2IP = 1; //Sets the TMR2 to PR2 Match Interrupt priority to High
}

void setupINT() 
{
    RCONbits.IPEN = 1;
    INTCONbits.GIE = 1;
}

void USARTsetup(void)
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    RCSTAbits.SPEN = 1; //Configures RX/DT and TX/CK pins as serial port pins
    RCSTA = 0b10110000; //Enables continuous receive
    TXSTAbits.BRGH = 1; // Selects High baud rate speed
    TXSTAbits.SYNC = 0; //Ensures that Asynchronous EUSART mode is selected
    TXSTAbits.TXEN = 1; //Enables Transmit
    SPBRGH = 0x00; //EUSART baud rate generator register high byte
    SPBRG = 0b00011001; //EUSART baud rate generator register low byte with SPBRG value
    BAUDCON = 0b00000000;
    INTCONbits.PEIE = 1; //Enabless all low priority peripheral interrupts
    PIE1bits.RCIE = 1; //Enables the EUSART receive interrupt
    IPR1bits.RCIP = 0; //Sets the EUSART Receive interrupt priority to low
}

void wait()
{
    int now = tick; //takes the current time passed since entering the function
    char hold = keyPress; //stores the character being pressed
    while((tick-now)<50) //wait till current time is greater than 500ms elapsed
    {
        if(hold != keyPress) //checks if the character currently being pressed is the same as the character stored in hold
        { 
            break; //leaves the function if the characters are not the same
        }
    }
   
}
    
void LEDon()
{
    LATDbits.LATD2 = 1; //output high voltage to RD2, turning lED on
    ledON = 1; //storing LED state
    ledOFF = 0;
    receive = 0; //indicates that a character has been debounced
}

void LEDoff()
{
    LATDbits.LATD2 = 0; //output low voltage to RD2, turning LED off
    ledON = 0;
    ledOFF = 1; //storing LED state
    receive = 0; //indicates that a character has been debounced
}

void main(void) {
    timer2setup(); //Function to enable hardware Timer2
    USARTsetup(); //Enable transmitting and receiving of data between the chip and keyboard
    setupINT(); //It enables global interrupts and priority levels on interrupts
    setupLED(); //Function to clear pin output, RD2 to LED
    int current = 0;
	state = 0;
	printf ("\n\n");
    while(1){
        current = tick; 
        if((tick-current) > 0) //only when tick (10ms elapsed) would the switch cases occur
        {
            switch(state) //switches between states of the FSM
            { 
                case WAITING: //if states=WAITING 
                    if(receive == 0) //checks if character was bedounced
                    {
                        state = WAITING; //waits till it receives a character
                    }
                    else if(ledON == 0 && receive == 1) //if it receives a character, it turns on the light if it is off
                    {
                        state = LEDON; 
                    }
                    else if(ledOFF == 0 && receive == 1) //if it receives a character, it turns off the light if it is on
                    {
                        state = LEDOFF;
                    }
                break;
                case LEDON: //turns on LED and waits till a new or same character arrives 
                    LEDon();
                    wait();
                    state = WAITING;
                break; 
                case LEDOFF: //turns off LED and waits till a new or same character arrives 
                    LEDoff();
                    wait();
                    state = WAITING;
                break;
            }
        }
        
    }
    return;
}
