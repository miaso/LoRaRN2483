/*
 * File:   main.c
 * Author: Arturas s134616
 *
 * Created on February 18, 2018, 1:32 AM
 */
#define EX_INT1_InterruptFlagClear()       (IFS1bits.INT1IF = 0)
#define EX_INT1_InterruptDisable()     (IEC1bits.INT1IE = 0)
#define EX_INT1_InterruptEnable()       (IEC1bits.INT1IE = 1)
#define EX_INT1_NegativeEdgeSet()          (INTCON2bits.INT1EP = 1)
#define EX_INT1_PositiveEdgeSet()          (INTCON2bits.INT1EP = 0)
#define EX_INT2_InterruptFlagClear()       (IFS1bits.INT2IF = 0)
#define EX_INT2_InterruptDisable()     (IEC1bits.INT2IE = 0)
#define EX_INT2_InterruptEnable()       (IEC1bits.INT2IE = 1)
#define EX_INT2_PositiveEdgeSet()          (INTCON2bits.INT2EP = 0)
#define EX_INT0_InterruptFlagClear()       (IFS0bits.INT0IF = 0)
#define EX_INT0_InterruptDisable()     (IEC0bits.INT0IE = 0)
#define EX_INT0_NegativeEdgeSet()          (INTCON2bits.INT0EP = 1)
#define EX_INT0_PositiveEdgeSet()          (INTCON2bits.INT0EP = 0)
#define EX_INT0_InterruptEnable()       (IEC0bits.INT0IE = 1)
#define EX_INT2_NegativeEdgeSet()          (INTCON2bits.INT2EP = 1)
/*****************/
/* Uart1 defines */
/*****************/
#define UART1_ENABLE                                    U1MODEbits.UARTEN = 1
#define UART1_DISABLE                                   U1MODEbits.UARTEN = 0
#define UART1_TX_ENABLE                                 U1STAbits.UTXEN = 1
#define UART1_TX_DISABLE                                U1STAbits.UTXEN = 0
#define UART1_TX_BUFFER_EMPTY                           U1STAbits.TRMT
#define UART1_RX_DATA_AVAILABLE                         U1STAbits.URXDA
#define UART1_TX_BUFFER_FULL                            U2STAbits.UTXBF
#define UART1_OERR_FLAG                                 U1STAbits.OERR
#define UART1_OERR_CLEAR                                U1STAbits.OERR = 0
#define UART1_WAKE_UP_ENABLE                            U1MODEbits.WAKE = 1

#define UART1_TX_INTERRUPT_FLAG                         IFS0bits.U1TXIF
#define UART1_TX_INTERRUPT_CLEAR                        IFS0bits.U1TXIF = 0
#define UART1_TX_INTERRUPT_ENABLE                       IEC0bits.U1TXIE = 1
#define UART1_TX_INTERRUPT_DISABLE                      IEC0bits.U1TXIE = 0

#define UART1_RX_INTERRUPT_FLAG                         IFS0bits.U1RXIF
#define UART1_RX_INTERRUPT_CLEAR                        IFS0bits.U1RXIF = 0
#define UART1_RX_INTERRUPT_ENABLE                       IEC0bits.U1RXIE = 1
#define UART1_RX_INTERRUPT_DISABLE                      IEC0bits.U1RXIE = 0
#define UART1_RX_INTERRUPT_PRIORITY                     IPC2bits.U1RXIP



// Configuration bits: selected in the GUI
#define FCY     16000000ULL //TO DO
// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F    // Deep Sleep Watchdog Timer Postscale Select bits->1:68719476736 (25.7 Days)
#pragma config DSWDTOSC = LPRC    // DSWDT Reference Clock Select->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = OFF    // Deep Sleep BOR Enable bit->DSBOR Enabled
#pragma config DSWDTEN = OFF    // Deep Sleep Watchdog Timer Enable->DSWDT Enabled
#pragma config DSSWEN = OFF    // DSEN Bit Enable->Deep Sleep is controlled by the register bit DSEN
#pragma config PLLDIV = PLL4X // DIVIDE2    // USB 96 MHz PLL Prescaler Select bits->Oscillator input divided by 2 (8 MHz input)
#pragma config I2C1SEL = ENABLE    // Alternate I2C1 enable bit->I2C1 uses ASCL1 and ASDA1 pins
#pragma config IOL1WAY = ON    // PPS IOLOCK Set Only Once Enable bit->Once set, the IOLOCK bit cannot be cleared

// CONFIG3
#pragma config WPFP = WPFP127    // Write Protection Flash Page Segment Boundary->Page 127 (0x1FC00)
#pragma config SOSCSEL = OFF    // SOSC Selection bits->Digital (SCLKI) mode
#pragma config WDTWIN = PS25_0    // Window Mode Watchdog Timer Window Width Select->Watch Dog Timer Window Width is 25 percent
#pragma config PLLSS = PLL_FRC    // PLL Secondary Selection Configuration bit->PLL is fed by the on-chip Fast RC (FRC) oscillator
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset Enable
#pragma config WPDIS = WPDIS    // Segment Write Protection Disable->Disabled
#pragma config WPCFG = WPCFGDIS    // Write Protect Configuration Page Select->Disabled
#pragma config WPEND = WPENDMEM    // Segment Write Protection End Page Select->Write Protect from WPFP to the last page of memory

// CONFIG2
#pragma config POSCMD = NONE    // Primary Oscillator Select->Primary Oscillator Disabled
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC
#pragma config OSCIOFCN = ON    // OSCO Pin Configuration->OSCO/CLKO/RA3 functions as port I/O (RA3)
#pragma config FCKSM = CSDCMD    // Clock Switching and Fail-Safe Clock Monitor Configuration bits->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config FNOSC = FRCPLL    // Initial Oscillator Select->Fast RC Oscillator with PLL module (FRCPLL)
#pragma config ALTRB6 = APPEND    // Alternate RB6 pin function enable bit->Append the RP6/ASCL1/PMPD6 functions of RB6 to RA1 pin functions
#pragma config ALTCMPI = CxINC_RB    // Alternate Comparator Input bit->C1INC is on RB13, C2INC is on RB9 and C3INC is on RA0
#pragma config WDTCMX = WDTCLK    // WDT Clock Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config IESO = ON    // Internal External Switchover->Enabled

// CONFIG1
#pragma config WDTPS = PS32768    // Watchdog Timer Postscaler Select->1:32768
#pragma config FWPSA = PR128    // WDT Prescaler Ratio Select->1:128
#pragma config WINDIS = OFF    // Windowed WDT Disable->Standard Watchdog Timer
#pragma config FWDTEN = OFF    // Watchdog Timer Enable->WDT disabled in hardware; SWDTEN bit disabled
#pragma config ICS = PGx3    // Emulator Pin Placement Select bits->Emulator functions are shared with PGEC3/PGED3
#pragma config LPCFG = OFF    // Low power regulator control->Disabled - regardless of RETEN
#pragma config GWRP = OFF    // General Segment Write Protect->Write to program memory allowed
#pragma config GCP = OFF    // General Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF    // JTAG Port Enable->Disabled

#include "xc.h"
#include <stdint.h>
#include <stdlib.h>
//#include <string.h>
#include <stdbool.h>
#include <libpic30.h>




static char hex [] = { '0', '1', '2', '3', '4', '5', '6', '7',
                        '8', '9' ,'A', 'B', 'C', 'D', 'E', 'F' };
 
//The function that performs the conversion. Accepts a buffer with "enough space" TM 
//and populates it with a string of hexadecimal digits.Returns the length in digits
int uintToHexStr(unsigned int num,char* buff)
{
    int len=0,k=0;
    do//for every 4 bits
    {
        //get the equivalent hex digit
        buff[len] = hex[num&0xF];
        len++;
        num>>=4;
    }while(num!=0);
    //since we get the digits in the wrong order reverse the digits in the buffer
    for(;k<len/2;k++)
    {//xor swapping
        buff[k]^=buff[len-k-1];
        buff[len-k-1]^=buff[k];
        buff[k]^=buff[len-k-1];
    }
    //null terminate the buffer and return the length in digits
    buff[len]='\0';
    return len;
}

// this routine found online somewhere, then tweaked
 // returns pointer to ASCII string in a static buffer
 char *itoa(int value) 
 {
     static char buffer[12];        // 12 bytes is big enough for an INT32
     int original = value;        // save original value
 
     int c = sizeof(buffer)-1;
 
     buffer[c] = 0;                // write trailing null in last byte of buffer    
 
     if (value < 0)                 // if it's negative, note that and take the absolute value
         value = -value;
     
     do                             // write least significant digit of value that's left
     {
         buffer[--c] = (value % 10) + '0';    
         value /= 10;
     } while (value);
 
     if (original < 0) 
         buffer[--c] = '-';
 
     return &buffer[c];
 }

volatile uint8_t MMAflag = 0;

void OSCILLATOR_Initialize(void) {
    // CF no clock failure; NOSC FRCPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONL((uint8_t) (0x0100 & 0x00FF));
    // CPDIV 1:1; PLLEN enabled; RCDIV FRC/1; DOZE 1:8; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3020;
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
    OSCTUN = 0x0000;
    // ROEN disabled; ROSEL FOSC; ROSIDL disabled; ROSWEN disabled; ROOUT disabled; ROSLP disabled; 
    REFOCONL = 0x0000;
    // RODIV 0; 
    REFOCONH = 0x0000;
    // ROTRIM 0; 
    REFOTRIML = 0x0000;
}

void PIN_MANAGER_Initialize(void) {
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0040;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x078F;
    TRISB = 0xEFEF;
    TRISC = 0x03BF;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPD1 = 0x0000;
    CNPD2 = 0x0000;
    CNPD3 = 0x0000;
    CNPU1 = 0x0000;
    CNPU2 = 0x0080;
    CNPU3 = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSA = 0x000C;
    ANSB = 0x6241;
    ANSC = 0x0007;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    // RPINR0bits.INT1R = 0x0013;   //RC3->EXT_INT:INT1;
    RPOR11bits.RP22R = 0x0003; //RC6->UART1:U1TX;
    RPINR18bits.U1RXR = 0x0017; //RC7->UART1:U1RX;
    //  RPINR1bits.INT2R = 0x0001;   //RB1->EXT_INT:INT2;

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS

}

void INTERRUPT_Initialize(void) {
    //    RTCI: RTCC - Real-Time Clock and Calendar
    //    Priority: 1
    IPC15bits.RTCIP = 1;
    //    INT0I: INT0 - External Interrupt 0
    //    Priority: 1
    //  IPC0bits.INT0IP = 1;
    //    INT1I: INT1 - External Interrupt 1
    //    Priority: 1
    //  IPC5bits.INT1IP = 1;
    //    INT2I: INT2 - External Interrupt 2
    //    Priority: 1
    //  IPC7bits.INT2IP = 1;


    //    UERI: U1E - UART1 Error
    //    Priority: 1
    // IPC16bits.U1ERIP = 1;
    //    UTXI: U1TX - UART1 Transmitter
    //    Priority: 1
    // IPC3bits.U1TXIP = 1;
    //    URXI: U1RX - UART1 Receiver
    //    Priority: 1
      IPC2bits.U1RXIP = 1;
    //    TI: T1 - Timer1
    //    Priority: 1
    //   IPC0bits.T1IP = 1;

}

void TMR1_Initialize(void) {
    //TMR1 0; 
    TMR1 = 0x0000;
    //Period = 0.001 s; Frequency = 16000000 Hz; PR1 2000; 
    PR1 = 0x07D0;
    //TCKPS 1:8; TON enabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TSYNC disabled; TGATE disabled; 
    T1CON = 0x8010;


    IFS0bits.T1IF = false;
    IEC0bits.T1IE = true;

    // tmr1_obj.timerElapsed = false;

}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {

}

volatile uint8_t serBuf[255];
volatile uint8_t serBufLen=0;
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {

    //
 if (serBufLen>=255){serBufLen=0;}
      serBuf[serBufLen++] = U1RXREG;
     
    //   CurrentUart1State = Uart1ConUpdate(1);

    UART1_RX_INTERRUPT_CLEAR;
    UART1_OERR_CLEAR;
}

void UART1_Initialize(void) {
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U1MODE = (0x8008 & ~(1 << 15)); // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U1STA = 0x0000;
    // BaudRate = 57600; Frequency = 16000000 Hz; U1BRG 68; 
    U1BRG = 0x0044;
    // ADMADDR 0; ADMMASK 0; 
    U1ADMD = 0x0000;
    // T0PD 1 ETU; PTRCL T0; TXRPT Retransmits the error byte once; CONV Direct; SCEN disabled; 
    U1SCCON = 0x0000;
    // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; PARIE disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
    U1SCINT = 0x0000;
    // GTC 0; 
    U1GTC = 0x0000;
    // WTCL 0; 
    U1WTCL = 0x0000;
    // WTCH 0; 
    U1WTCH = 0x0000;

    IEC0bits.U1RXIE = 1;

    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U1MODEbits.UARTEN = 1; // enabling UART ON bit
    U1STAbits.UTXEN = 1;

}
//UART transmit function, parameter Ch is the character to send

void UART1PutChar(char Ch) {
    //transmit ONLY if TX buffer is empty
    while (U1STAbits.UTXBF == 1);
    U1TXREG = Ch;
}

void UART1PutStr(char* s) {
    char c = 0;
    for (; ((c = *s) != 0); s++) {
        UART1PutChar(*s);
    }
}

#define ERROR_HANDLER __attribute__((interrupt,no_auto_psv))
#define ERROR_HANDLER_NORETURN ERROR_HANDLER __attribute__((noreturn))
#define FAILSAFE_STACK_GUARDSIZE 8

typedef enum {
    /* ----- Traps ----- */
    TRAPS_OSC_FAIL = 0, /** Oscillator Fail Trap vector */
    TRAPS_STACK_ERR = 1, /** Stack Error Trap Vector */
    TRAPS_ADDRESS_ERR = 2, /** Address Error Trap Vector */
    TRAPS_MATH_ERR = 3, /** Math Error Trap Vector */
} TRAPS_ERROR_CODE;

/**
 * a private place to store the error code if we run into a severe error
 */
static uint16_t TRAPS_error_code = -1;

void __attribute__((naked, noreturn, weak)) TRAPS_halt_on_error(uint16_t code) {
    TRAPS_error_code = code;
#ifdef __DEBUG    
    __builtin_software_breakpoint();
    /* If we are in debug mode, cause a software breakpoint in the debugger */
#endif
    while (1);

}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    //***User Area Begin->code: INT1 - External Interrupt 1***

    //***User Area End->code: INT1 - External Interrupt 1***
    EX_INT1_InterruptFlagClear();
}

/**
  Interrupt Handler for EX_INT2 - INT2
 */
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    //***User Area Begin->code: INT2 - External Interrupt 2***

    //***User Area End->code: INT2 - External Interrupt 2***
    EX_INT2_InterruptFlagClear();
}
/**
  Interrupt Handler for EX_INT0 - INT0
 */
volatile uint8_t sendflag = 0;

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {
    //***User Area Begin->code: INT0 - External Interrupt 0***
    sendflag = 0x01;

    uint8_t dum = 0;

    dum = 0 + 3 * 6;
    dum = 0 + 3 * 6;

    //***User Area End->code: INT0 - External Interrupt 0***
    EX_INT0_InterruptFlagClear();
    EX_INT0_InterruptDisable();
}
/**
    Section: External Interrupt Initializers
 */

/**
    void EXT_INT_Initialize(void)

    Initializer for the following external interrupts
    INT1
    INT2
    INT0
 */
void EXT_INT_Initialize(void) {
    /*******
     * INT1
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    //   EX_INT1_InterruptFlagClear();   
    //   EX_INT1_PositiveEdgeSet();
    //  EX_INT1_InterruptEnable();
    /*******
     * INT2
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    //  EX_INT2_InterruptFlagClear();   
    //  EX_INT2_PositiveEdgeSet();
    //   EX_INT2_InterruptEnable();
    /*******
     * INT0
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT0_InterruptFlagClear();
    EX_INT0_PositiveEdgeSet();
    EX_INT0_InterruptEnable();
}

/**
 * Sets the stack pointer to a backup area of memory, in case we run into
 * a stack error (in which case we can't really trust the stack pointer)
 */
inline static void use_failsafe_stack(void) {
    static uint8_t failsafe_stack[32];
    asm volatile (
            "   mov    %[pstack], W15\n"
            :
            : [pstack]"r"(failsafe_stack)
            );
    /* Controls where the stack pointer limit is, relative to the end of the
     * failsafe stack
     */
    SPLIM = (uint16_t) (((uint8_t *) failsafe_stack) + sizeof (failsafe_stack)
            - FAILSAFE_STACK_GUARDSIZE);
}

/** Oscillator Fail Trap vector**/
void ERROR_HANDLER_NORETURN _OscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0; //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_OSC_FAIL);
}

/** Stack Error Trap Vector**/
void ERROR_HANDLER_NORETURN _StackError(void) {
    /* We use a failsafe stack: the presence of a stack-pointer error
     * means that we cannot trust the stack to operate correctly unless
     * we set the stack pointer to a safe place.
     */
    use_failsafe_stack();
    INTCON1bits.STKERR = 0; //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_STACK_ERR);
}

/** Address Error Trap Vector**/
void ERROR_HANDLER_NORETURN _AddressError(void) {
    INTCON1bits.ADDRERR = 0; //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_ADDRESS_ERR);
}

/** Math Error Trap Vector**/
void ERROR_HANDLER_NORETURN _MathError(void) {
    INTCON1bits.MATHERR = 0; //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_MATH_ERR);
}

void I2C2_Initialize(void) {
    // initialize the hardware
    // Baud Rate Generator Value: I2CBRG 78;   
    I2C2BRG = 0x004E;
    // ACKEN disabled; STRICT disabled; STREN disabled; GCEN disabled; SMEN disabled; DISSLW disabled; I2CSIDL disabled; ACKDT Sends ACK; SCLREL Holds; RSEN disabled; A10M 7 Bit; PEN disabled; RCEN disabled; SEN disabled; I2CEN enabled; 
    // ACKEN disabled; STRICT disabled; STREN disabled; GCEN disabled; SMEN disabled; DISSLW enabled; I2CSIDL disabled; ACKDT Sends ACK; SCLREL Holds; RSEN disabled; A10M 7 Bit; PEN disabled; RCEN disabled; SEN disabled; I2CEN enabled; 
    I2C2CONL = 0x8200;
    // BCL disabled; D_nA disabled; R_nW disabled; P disabled; S disabled; I2COV disabled; IWCOL disabled; 
    I2C2STAT = 0x0000;

    /* MI2C2 - I2C2 Master Events */
    // clear the master interrupt flag
    IFS3bits.MI2C2IF = 0;
    // enable the master interrupt
    //  IEC3bits.MI2C2IE = 1;

}

void i2c_start(void) {
    int x = 0;
    I2C2CONLbits.ACKDT = 0; //Reset any previous Ack
    __delay_us(10)
    I2C2CONLbits.SEN = 1; //Initiate Start condition
    Nop();
    //the hardware will automatically clear Start Bit
    //wait for automatic clear before proceding
    while (I2C2CONLbits.SEN) {
        __delay_us(1);
        x++;
        if (x > 20)
            break;
    }
    __delay_us(2);
}

void i2c_stop(void) {
    int x = 0;
    I2C2CONLbits.ACKDT = 0; //Reset any previous Ack
    __delay_us(10)
    I2C2CONLbits.PEN = 1; // Stop Condition Enable bit
    Nop();
    //the hardware will automatically clear stop Bit
    //wait for automatic clear before proceding
    while (I2C2CONLbits.PEN) {
        __delay_us(1);
        x++;
        // UART1PutChar(x);
        if (x > 20)
            break;
    }
    I2C2CONLbits.PEN = 0; // Stop Condition Enable bit
    __delay_us(2);
}

void i2c_repeatedStart(void) {
    int x = 0;
    I2C2CONLbits.ACKDT = 0; //Reset any previous Ack
    __delay_us(10)

    I2C2CONLbits.RSEN = 1; //Initiate Start condition
    Nop();

    //the hardware will automatically clear Start Bit
    //wait for automatic clear before proceding
    while (I2C2CONLbits.RSEN) {
        __delay_us(1);
        x++;
        if (x > 20)
            break;
    }
    __delay_us(2);
}

void i2c_restart(void) {
    int x = 0;

    I2C2CONLbits.RSEN = 1; //Initiate restart condition
    Nop();

    //the hardware will automatically clear restart bit
    //wait for automatic clear before proceding
    while (I2C2CONLbits.RSEN) {
        __delay_us(1);
        x++;
        if (x > 20) break;
    }

    __delay_us(2);
}
//Resets the I2C bus to Idle

void reset_i2c_bus(void) {
    int x = 0;

    //initiate stop bit
    I2C2CONLbits.PEN = 1;

    //wait for hardware clear of stop bit
    while (I2C2CONLbits.PEN) {
        __delay_us(2);
        x++;
        if (x > 20) break;
    }
    I2C2CONLbits.RCEN = 0;
    IFS3bits.MI2C2IF = 0; // Clear Interrupt //////////////////
    I2C2STATbits.IWCOL = 0;
    I2C2STATbits.BCL = 0;
    __delay_us(10);
}

//basic I2C byte send

char send_i2c_byte(int data) {
    int i;

    while (I2C2STATbits.TBF) {
    }
    IFS3bits.MI2C2IF = 0; // Clear Interrupt///////////////////////////////////////////////
    I2C2TRN = data; // load the outgoing data byte

    // wait for transmission
    for (i = 0; i < 500; i++) {
        if (!I2C2STATbits.TRSTAT) break;
        __delay_us(1);

    }
    if (i == 500) {
        return (1);
    }

    // Check for NO_ACK from slave, abort if not found
    if (I2C2STATbits.ACKSTAT == 1) {
        // UART1PutStr("no ack-reset");
        reset_i2c_bus();
        return (1);
    }

    __delay_us(2);
    return (0);
}


//function reads data, returns the read data, no ack

char i2c_read(void) {
    int i = 0;
    char data = 0;

    //set I2C module to receive
    I2C2CONLbits.RCEN = 1;

    //if no response, break
    while (!I2C2STATbits.RBF) {
        i++;
        if (i > 2000) break;
    }

    //get data from I2CRCV register
    data = I2C2RCV;

    //return data
    return data;
}

//function reads data, returns the read data, with ack

char i2c_read_ack(void) //does not reset bus!!!
{
    int i = 0;
    char data = 0;

    //set I2C module to receive
    I2C2CONLbits.RCEN = 1;

    //if no response, break
    while (!I2C2STATbits.RBF) {
        i++;
        if (i > 2000) break;
        // UART1PutChar('R');
    }
    i = 0;
    //get data from I2CRCV register
    data = I2C2RCV;

    __delay_us(2);

    //i2c_ack();
    __delay_us(2);

    return data;

}

void i2c_mIdleI2C1(void) {

    while ((I2C2CONL & 0x001F) != 0) { //////////////////////////////////////////////////////
        //  UART1PutChar('d');
    }//
    //Wait for Acken, Rcen, Pen, Rsen and Sen to clear


}

void i2c_ack(void) {
    I2C2CONLbits.ACKDT = 0;
    Nop();
    I2C2CONLbits.ACKEN = 0; //edit
    I2C2CONLbits.RCEN = 0;
    I2C2CONLbits.PEN = 0;
    I2C2CONLbits.RSEN = 0;
    I2C2CONLbits.SEN = 0;
    //__delay_us(1);       
    I2C2CONLbits.ACKEN = 1;
    // while (I2C2CONLbits.ACKEN == 1) {
    //UART1PutChar('A');
    //    };
}

void i2c_nack(void) {
    I2C2CONLbits.ACKDT = 1;
    Nop();
    //I2C1CONbits.ACKEN=0;
    I2C2CONLbits.RCEN = 0;
    I2C2CONLbits.PEN = 0;
    I2C2CONLbits.RSEN = 0;
    I2C2CONLbits.SEN = 0;
    //__delay_us(1);       
    I2C2CONLbits.ACKEN = 1;
    while (I2C2CONLbits.ACKEN == 1) {
        //    UART1PutChar('N');
    };
}


#define mma8451q_READ  0b00111001 
#define mma8451q_WRITE 0b00111000 
#define mpu6050_addr_r 0b11010001
#define mpu6050_addr_w 0b11010000

#define REG_XHI 0x01
#define REG_XLO 0x02
#define REG_YHI 0x03
#define REG_YLO 0x04
#define REG_ZHI	0x05
#define REG_ZLO 0x06

#define REG_WHOAMI 0x0D //Accel read address
#define REG_CTRL1  0x2A
#define REG_CTRL4  0x2D

#define WHOAMI 0x1A

uint8_t mma8451q_read_byte(uint8_t registerAddr) {
    uint8_t value;
    i2c_start();
    send_i2c_byte(mma8451q_WRITE); // address +0 write  
    send_i2c_byte(registerAddr);
    i2c_repeatedStart();
    send_i2c_byte(mma8451q_READ); //adresss +1 read
    value = i2c_read_ack();
    i2c_nack();
    i2c_stop();
    return value;

}

void mma8451q_write_byte(uint8_t registerAddr, uint8_t data) {

    i2c_start();
    MMAflag = 0x03;
    send_i2c_byte(mma8451q_WRITE); // address +0 write  
    MMAflag = 0x04;
    send_i2c_byte(registerAddr);
    MMAflag = 0x05;
    send_i2c_byte(data);
    MMAflag = 0x06;
    i2c_nack();
    i2c_stop();

}

uint8_t mpu6050_read_byte(uint8_t registerAddr) {
    uint8_t value;
    i2c_start();
    send_i2c_byte(mpu6050_addr_w); // address +0 write
    i2c_ack();

    send_i2c_byte(registerAddr);
    i2c_ack();
    i2c_repeatedStart();
    //i2c_start(); 
    send_i2c_byte(mpu6050_addr_r); //adresss +1 read
    i2c_ack();
    //i2c_mIdleI2C1();
    value = i2c_read_ack();
    i2c_ack();
    uint8_t dum = 0;

    dum = 0 + 3 * 6;
    dum = 0 + 3 * 6;
    i2c_nack();
    i2c_stop();
    return value;

}

uint8_t mpu6050_read_byte_tested(uint8_t registerAddr) {
    uint8_t value;

    i2c_start();
    send_i2c_byte(mpu6050_addr_w); // address +0 write
    send_i2c_byte(0x75);
    i2c_repeatedStart();
    send_i2c_byte(mpu6050_addr_r); //adresss +1 read
    value = i2c_read_ack();
    i2c_nack();
    i2c_stop();
    return value;

}

typedef union {
    uint16_t w;
    uint8_t b[2];
} uint16_v;

typedef union {
    int16_t w;
    int8_t b[2];
} int16_v;
volatile int8_t x, y, z;

volatile uint16_v XXX;
volatile int16_v intXXX, intY, intZ;
volatile int16_t acc_X = 0, acc_Y = 0, acc_Z = 0;
volatile uint16_t uacc_X = 0, uacc_Y = 0, uacc_Z = 0;

void mma451_read_xyz(void) {
    // sign extend byte to 16 bits - need to cast to signed since function
    // returns uint8_t which is unsigned
    int8_t XH = (int8_t) mma8451q_read_byte(REG_XHI);
    x = XH;
    int8_t XL = (int8_t) mma8451q_read_byte(REG_XLO);
    acc_X = (XH << 8) | XL;
    uacc_X = (XH << 8) | XL;
    XXX.b[1] = XH;
    XXX.b[0] = XL;
    intXXX.b[1] = XH;
    intXXX.b[0] = XL;

    //	Delay(100);

    // Add two other 2-axis here
    int8_t YH = (int8_t) mma8451q_read_byte(REG_YHI);
    y = YH;
    int8_t YL = (int8_t) mma8451q_read_byte(REG_YLO);
    acc_Y = (int16_t) ((YH << 8) | YL);
    intY.b[1] = YH;
    intY.b[0] = YL;
    //Delay(100);

    int8_t ZH = (int8_t) mma8451q_read_byte(REG_ZHI);
    z = ZH;
    int8_t ZL = (int8_t) mma8451q_read_byte(REG_ZLO);
    acc_Z = (int16_t) ((ZH << 8) | ZL);
    intZ.b[1] = ZH;
    intZ.b[0] = ZL;
    //

}

void mma8451_init() {
    if (mma8451q_read_byte(REG_WHOAMI) != WHOAMI) {
        MMAflag = 0xff;
    }
    mma8451q_write_byte(0x2A, 0x01);

    //mma8451q_write_byte( 0x2B, 0x40);
    //    
    //while(mma8451q_read_byte(0x2B) & 0x40){
    //MMAflag=0x01;
    //} //reset done
    //MMAflag=0x02;


}


volatile uint8_t who, who2;

int main(void) {
    OSCILLATOR_Initialize();
    PIN_MANAGER_Initialize();
    INTERRUPT_Initialize();
    EXT_INT_Initialize();
    // TMR1_Initialize();
    UART1_Initialize();
    I2C2_Initialize();
    //mma8451_init();
    __delay_ms(2000);
    UART1PutStr("mac join otaa\r\n");
    __delay_ms(8000);
    // UART1PutChar(0x13);
    //   UART1PutChar(0x0A);

    while (1) {
        who = mma8451q_read_byte(0x11); //REG_WHOAMI);   
        who2 = mma8451q_read_byte(0x2A); //       

        mma451_read_xyz();

        
        char buff[4];//enough for 64 bits integer
   int length;   
   //convert
   length = uintToHexStr(373,buff);
   //and print the results
        
        
        
        __delay_ms(1000);

        if (sendflag == 0x01) {
            UART1PutStr("mac tx cnf 1 ");
            UART1PutStr("00FF00FF00");
//            uintToHexStr(intXXX.w,buff);
//            UART1PutChar(buff[0]);
//            UART1PutChar(buff[1]);
//            UART1PutChar(buff[2]);
//            UART1PutChar(buff[3]);
//            uintToHexStr(intY.w,buff);
//                   UART1PutChar(buff[0]);
//            UART1PutChar(buff[1]);
//            UART1PutChar(buff[2]);
//            UART1PutChar(buff[3]);
//             uintToHexStr(intZ.w,buff);
//                      UART1PutChar(buff[0]);
//            UART1PutChar(buff[1]);
//            UART1PutChar(buff[2]);
//            UART1PutChar(buff[3]);
         
             UART1PutStr("\r\n");
            //UART SEND MEASUREMENTS 
            __delay_ms(2000);
            sendflag = 0x00;
            EX_INT0_InterruptEnable();

        }


    }
    return 0;
}
