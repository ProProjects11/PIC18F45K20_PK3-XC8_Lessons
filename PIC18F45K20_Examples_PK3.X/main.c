/* 
 * File:   main.c
 * Author: Omar
 *
 * Created on July 7, 2014, 4:53 PM
 */

#include <xc.h>
#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <delays.h>
#include <flash.h>

#define _XTAL_FREQ 1000000 // 1 MHz

//-----------------------------------
#define ACTIVATE_EXAMPLE 12
//-----------------------------------

#if (ACTIVATE_EXAMPLE == 12)

/* #12
 * CCP Module for PWM
 */
void main(void) {
    unsigned char brightness = 125; // = 0x7D

    // Set RD7/P1D pin output so P1D PWM output drives LED7
    TRISDbits.TRISD7 = 0;
    TRISDbits.TRISD6 = 0;
    TRISDbits.TRISD5 = 0;

    // Set up 8-bit Timer2 to generate the PWM period (frequency)
    T2CON = 0b00000111; // Prescale = 1:16, timer on, postscale not used with CCP module
    PR2 = 249; // Timer 2 Period Register = 250 counts
    // Thus, the PWM frequency is:
    // 1MHz clock / 4 = 250kHz instruction rate.
    // (250kHz / 16 prescale) / 250) = 62.5Hz, a period of 16ms.

    // The Duty Cycle is controlled by the ten-bit CCPR1L<7,0>:DC1B1:DC1B0
    // 50% Duty Cycle = 0.5 * (250 * 4) = 500
    CCPR1L = 0x7C; // The 8 most significant bits of the value 500 = 0x1F4 are 0x7D
    // The 2 least significant bits of the value (0b00) are written
    // to the DC1B1 and DC1B0 bits in CCP1CON

    CCP1CON = 0b00001100; // Single pwm output
    PSTRCON = 0b00001110; // P1D/RD7, P1C/RD6, P1B/RD5 are the pwm outputs others are digitals

    //CCP1CON = 0b01001100;
    // P1Mx = 01 Full-Bridge output forward, so we get the PWM
    // signal on P1D to LED7.  Only Single Output (00) is needed,
    // but the P1A pin does not connect to a demo board LED
    // CCP1Mx = 1100, PWM mode with P1D active-high.

    // The LED brightness is affected by by the Duty Cycle, which determines how much
    // of each 16ms period it is on and how much it is off.  As the duty cycle gets
    // less than 50%, it is off more than it is on so the LED becomes dimmer.  As the
    //duty cycle increases above 50%, it is on more than off, so it gets brighter.
    //
    // This increases the brightness over 2 seconds, then decreases it over the next 2 seconds
    // Updating the CCPR1L value more than once per 16ms period has no benefit, so we'll update
    // it a total of 125 times, once per period, which works out to 2 seconds.
    //
    // Although we have nearly ten bits of resolution in the duty cycle (1000 counts)
    // we'll increment the duty cycle by 8 each time as we only have 125 levels over the
    // 2 second period.
    while (1) {
        // increase brightness over 2 seconds.
        do {
            brightness += 2;
            CCPR1L = brightness; // + 8 including 2 bits DC1Bx in CCP1CON
            PIR1bits.TMR2IF = 0; // clear interrupt flag; set on every TMR2 = PR2 match
            while (PIR1bits.TMR2IF == 0); // watch for match, which is end of period.
        } while (brightness < 250);

        Delay1KTCYx(63); // delay about 250ms at peak brightness, just for effect!

        // decrease brightness over 2 seconds.
        do {
            brightness -= 2;
            CCPR1L = brightness; // - 8 including 2 bits DC1Bx in CCP1CON
            PIR1bits.TMR2IF = 0; // clear interrupt flag; set on every TMR2 = PR2 match
            while (PIR1bits.TMR2IF == 0); // watch for match, which is end of period.
        } while (brightness > 1);

        Delay1KTCYx(63); // delay about 250ms at dimmest, it gives a better effect!
    };
}
#endif

#if (ACTIVATE_EXAMPLE == 11)
/* #11
 * Read and Write Flash program memory
 */
uint8_t ProgMemRdAddress(uint16_t address);
void ProgMemErase64(uint16_t address);
void ProgMemWr32(uint16_t address, char *buffer_ptr);

const char hello_str[] @0x100 = "Hello!";
const char mchp_str[] @0x108 = "Microchip";
const char fill_60[] @0x112 = "012345678901234567890123456789012345678901234567890123456789";

void main(void) @0x280 {
    const char *rom_pointer; //Pointer to program memory
    char singlechar = '?';
    char i = 0;
    char Alphabet[32];

    // read using a pointer to data
    rom_pointer = hello_str; // = &hello_str[0]

    do {
        singlechar = *(rom_pointer + i++);
    } while (singlechar != 0); // string is terminated with 0x00 value.

    // read at specific address
    singlechar = ProgMemRdAddress((uint16_t) mchp_str); // returns 'M' from "Microchip".

    // Erase the 64 bytes starting at 0x100
    ProgMemErase64(0x100);
    //ProgMemErase64(0x140);

    // create a RAM buffer with the uppercase alphabet in ASCII plus the characters [\]^_`
    for (i = 0; i < 32; i++) {
        Alphabet[i] = 'A' + i;
    }
    // write the buffer into program memory
    // This doesnt work i dont know why!!!!!!!
    ProgMemWr32(0x100, &Alphabet[0]);

    while (1) {
        ;
    } // all done

}

uint8_t ProgMemRdAddress(uint16_t address) { // reads and returns the flash program memory byte value at the 16-bit address given
    // given in "address".

    const unsigned char *ptr; // Pointer to rom

    ptr = (const unsigned char *) address; // Get the value

    return *ptr;
}

void ProgMemErase64(uint16_t address) { // the program memory of the PIC18F4520 is erased 64-bytes at a time, which must
    // aligned on a 64-byte address boundary
    const unsigned char *ptr;

    // assigning the pointer sets the TBLPTRU:TBLPTRH:TBLPTRL SFR registers
    ptr = (const unsigned char *) (address & 0xFFC0); // ensure erase starts on 64-byte boundary

    // set up the erase.  Program execution will be suspended during the flash erase
    EECON1bits.EEPGD = 1; // point to flash program memory
    EECON1bits.CFGS = 0; // not configuration registers
    EECON1bits.FREE = 1; // we're erasing
    EECON1bits.WREN = 1; // enable write/erase operations

    // execute code sequence, which cannot be interrupted, then execute erase

    //INTCONbits.GIE = 0;   // Disable interrupts
    EECON2 = 0x55; // Begin Write sequence
    EECON2 = 0xAA;
    EECON1bits.WR = 1; // Set WR bit to begin 64-byte erase
    //INTCONbits.GIE = 1;   // re-enable interrupts

    EECON1bits.WREN = 0; // disable write/erase operations
}

void ProgMemWr32(uint16_t address, char *buffer_ptr) { // program memory must be written 32 bytes at a time in the PIC18F4520, starting at a
    // 32-byte address boundary.  It must also be erased first.
    const unsigned char *ptr;
    char i;

    ptr = (const unsigned char *) (address & 0xFFE0); // ensure write starts on 32-byte boundary

    for (i = 0; i < 32; i++) {
        // This doesnt work i dont know why
        //*(ptr + 1) = buffer_ptr[i]; // write the data into the holding registers
    }

    // NOTE! When writing program memory flash, the TBLPTRU:TBLPTRH:TBLPTRL registers
    // must be pointing within the 32-byte block intended to be written.  For example,
    // if you wish to write 32 bytes at 0x100, if the last write increments the pointer
    // past 0x11F to 0x120, the 32 bytes will actually be written at 0x120, not 0x100.

    EECON1bits.EEPGD = 1; // write to flash program memory
    EECON1bits.CFGS = 0; // not configuration registers
    EECON1bits.FREE = 0; // we're not erasing now.
    EECON1bits.WREN = 1; // enable write/erase operations

    // execute code sequence, which cannot be interrupted, then execute write32

    //INTCONbits.GIE = 0;   // Disable interrupts
    EECON2 = 0x55; // Begin Write sequence
    EECON2 = 0xAA;
    EECON1bits.WR = 1; // Set WR bit to begin 32-byte write
    //INTCONbits.GIE = 1;   // re-enable interrupts

    EECON1bits.WREN = 0; // disable write/erase operations
}
#endif

#if (ACTIVATE_EXAMPLE == 112)
/* #11
 * Write and Read Program memory (using xc8 libraries)
 *
 * Notes:
 * - Before write data you must erase the block.
 * - Erase and Write must be in term of blocks (Defined in datasheet)
 * - Read can be done in any adrress (no block term is needed)
 *
 * PIC18F45K22:
 * Erase block = 64 Bytes
 * Write Block = 32 Bytes
 */
unsigned char data[] = {0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81};
unsigned char read_data[8];

void main(void) @0x1000 { // Put main in 1000h
    TRISD = 0x00;
    LATD = 0;

    // Erase block = 64 Bytes = 0x40
    // Write block = 32 Bytes = 0x20
    // Address = 0 - > (block term) (Example: Erase Block 1 = 0x00 -> 0x3F, Erase Block 2 = 0x40 -> 0x7F
    EraseFlash(0x500, 0x53F); // Erase 64 bytes (One erase block)

    WriteBytesFlash(0x510, sizeof (data), &data[0]); // Write to any address previously erased (it doesn't need to be the start of a block address).
    //WriteBlockFlash(0x500,1,&data[0]); // Write one block (32 Bytes), address must be in write block term and all the 32 Bytes must be present in the array. If you dont need all 32 Bytes you can fill them with 0xFF

    ReadFlash(0x510, sizeof (read_data), &read_data[0]); // Read bytes from program memory (Any address and the needed amount of bytes))

    while (1) {
        for (unsigned char x = 0; x < 8; x++) {
            LATD = read_data[x]; // Read address value
            __delay_ms(200);
        }
    }
}
#endif

#if (ACTIVATE_EXAMPLE == 111)
/* #11
 * Read program memory (data stored in code when programmed)
 */
const unsigned char id_rom[] @(0x500) = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void main(void) {
    TRISD = 0x00;
    LATD = 0;

    while (1) {
        const unsigned char *pt;
        pt = (const unsigned char*) 0x500; // Pointing to rom address
        //pt = &id_rom[0]; // Pointing to rom address
        for (unsigned char x = 0; x < 8; x++) {
            LATD = *pt; // Read address value
            pt++; // Point to next byte
            __delay_ms(500);
        }
    }
}
#endif

#if (ACTIVATE_EXAMPLE == 10)
/* #10
 * EEPROM read, write and initialize
 */

void e2prom_write(unsigned char, unsigned char);
unsigned char e2prom_read(unsigned char);

// Diferentes ways to load data to eeprom

// (Pre-loading) This macro only permits to write 8 bytes of data in 0x00-0x07 address
//__EEPROM_DATA(0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80);

// (Pre-loading) Using this macro more than one time will continue the next address data
//__EEPROM_DATA(0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80);
//__EEPROM_DATA(0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01);

void main(void) {
    TRISD = 0x00; // Output
    LATD = 0x00; // Turn off leds

    // (In runtime) Using a user defined function or xc8 function
    for (unsigned char a = 0, d = 1; a < 8; a++, d = d << 1) {
        //e2prom_write(a,d); // User function
        eeprom_write(a, d); // xc8 function
    }

    while (1) {
        for (unsigned char a = 0; a < 16; a++) {
            //LATD = e2prom_read(a); // User function
            LATD = eeprom_read(a); // xc8 function
            __delay_ms(500);
        }
    }
}

void e2prom_write(unsigned char address, unsigned char value) {
    PIR2bits.EEIF = 0; // Erase interrupt flag

    EECON1bits.EEPGD = 0; // Point to DATA memory
    EECON1bits.CFGS = 0; // Access E2PROM

    EEADR = address; // Write address
    EEDATA = value; // Write data
    EECON1bits.WREN = 1; // Enable write

    INTCONbits.GIE = 0; // Disable interrupts

    // Write sequence
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; // Set WR bit to begin write

    INTCONbits.GIE = 1; // Enable interrupts

    while (PIR2bits.EEIF == 0); // Wait for write to finish

    EECON1bits.WREN = 0; // Disable write
}

unsigned char e2prom_read(unsigned char address) {
    EECON1bits.EEPGD = 0; // Point to DATA memory
    EECON1bits.CFGS = 0; // Access EEPROM

    EEADR = address; // Load address

    EECON1bits.RD = 1; // Read
    return EEDATA;
}
#endif

#if (ACTIVATE_EXAMPLE == 91)

/* #91
 * OSC Frequency
 */
#define _XTAL_FREQ 1000000 // 1 MHz

void main(void) {
    TRISD = 0x00; // outputs
    while (1) {
        OSCCONbits.IRCF = 0b011; // 1MHz
        OSCTUNEbits.PLLEN = 0; // = 1MHz
        for (unsigned char i = 1; i > 0; i = i << 1) {
            LATD = i;
            __delay_ms(500);
            __delay_ms(500);
        }
        OSCCONbits.IRCF = 0b111; // 16MHz
        OSCTUNEbits.PLLEN = 1; // PLL on = 16MHz x 4 = 64MHz
        for (unsigned char x = 0; x < 10; x++) {
            for (unsigned char i = 1; i > 0; i = i << 1) {
                LATD = i;
                __delay_ms(500); // not real 500ms because the cpu is runing at 64mhz
                __delay_ms(500);
            }
            for (unsigned char i = 0x80; i > 0; i = i >> 1) {
                LATD = i;
                __delay_ms(500);
                __delay_ms(500);
            }
        }
    }
}
#endif

#if (ACTIVATE_EXAMPLE == 9)

/* #9
 * Change Frequency with INT0 + SW Debounce
 */
typedef enum {
    C_250kHz = 0,
    C_500kHz = 1,
    C_1MHz = 2,
    C_2MHz = 3,
    C_4MHz = 4,
    C_8MHz = 5,
    C_16MHz = 6,
    C_32MHz = 7,
    C_64MHz = 8
} IntOSCFreq;

void InterruptService(void);
void SetupINT0Switch(void);
void EnableInterrupts(void);
void SetIntOSC(IntOSCFreq *ClockSet);

unsigned char LED_Count = 0; // 8-bit variable
unsigned char SwitchDelay = 1; // delay time to "debounce" switch
IntOSCFreq ClockSpeed = C_250kHz;

void main(void) {
    // Init I/O
    TRISD = 0b00000000; // PORTD bits 7:0 are all outputs (0)
    TRISEbits.TRISE0 = 1; // TRISE0 input

    // Set initial clock speed (250kHz)
    SetIntOSC(&ClockSpeed);

    // Init switch and turn on interrupts
    SetupINT0Switch();
    EnableInterrupts();

    while (1) { // delay and count on LEDs here.  Interrupt handles switch and freq changes
        LATD = LED_Count++; // output count to PORTD LEDs

        Delay1KTCYx(64); // delay 64,000 cycles or about 1 sec at 250kHz
    }

}

void SetupINT0Switch(void) { // Set up switch interrupt on INT0
    INTCON2bits.INTEDG0 = 0; // interrupt on falling edge of INT0 (switch pressed)
    INTCONbits.INT0IF = 0; // ensure flag is cleared
    INTCONbits.INT0IE = 1; // enable INT0 interrupt
}

void EnableInterrupts(void) { // Set up global interrupts
    RCONbits.IPEN = 0; // Disable priority levels on interrupts
    INTCONbits.PEIE = 1; // Peripheral interrupts allowed (but none are used)
    INTCONbits.GIE = 1; // Interrupting enabled.
}

void interrupt high_isr(void) {
    int i;

    // Check to see what caused the interrupt
    // (Necessary when more than 1 interrupt at a priority level)

    // Check for INT0 interrupt
    if (INTCONbits.INT0IF) {
        // Delay about 50ms regardless of frequency to debounce switch.
        // NOTE: a delay function from delay.h isn't used as they are
        // inline assembly and inline assembly in an interrupt routine
        // causes a very long context save/restore since the compiler
        // doesn't know what resources are being used and so saves everything.
        i = 125 * SwitchDelay;
        while (i) {
            i--;
        }


        // clear (reset) flag
        INTCONbits.INT0IF = 0;

        if (PORTBbits.RB0 == 0) { // if it's still pressed after 100ms delay.
            // Change oscilator frequency
            SetIntOSC(&ClockSpeed);
        }
    }
    // Check for another interrupt, examples:
    // if (PIR1bits.TMR1IF)     // Timer 1
    // if (PIR1bits.ADIF)       // ADC

} // return from high-priority interrupt

void SetIntOSC(IntOSCFreq *ClockSet) { // This function sets the internal oscillator to the frequency of
    // the ClockSet argument variable, and then increments ClockSet
    // to the next supported frequency.
    switch (*ClockSet) {
        case C_250kHz:
            OSCCON = 0x10; // IRCFx = 001
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_500kHz;
            SwitchDelay = 1;
            break;

        case C_500kHz:
            OSCCON = 0x20; // IRCFx = 010
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_1MHz;
            SwitchDelay = 2;
            break;

        case C_1MHz:
            OSCCON = 0x30; // IRCFx = 011
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_2MHz;
            SwitchDelay = 4;
            break;

        case C_2MHz:
            OSCCON = 0x40; // IRCFx = 100
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_4MHz;
            SwitchDelay = 8;
            break;

        case C_4MHz:
            OSCCON = 0x50; // IRCFx = 101
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_8MHz;
            SwitchDelay = 16;
            break;

        case C_8MHz:
            OSCCON = 0x60; // IRCFx = 110
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_16MHz;
            SwitchDelay = 32;
            break;

        case C_16MHz:
            OSCCON = 0x70; // IRCFx = 111
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_32MHz;
            SwitchDelay = 64;
            break;

        case C_32MHz:
            OSCCON = 0x60; // IRCFx = 110 (8 MHz)
            OSCTUNEbits.PLLEN = 1; // x4 PLL enabled = 32MHz
            *ClockSet = C_64MHz;
            SwitchDelay = 128;
            break;

        case C_64MHz:
            OSCCON = 0x70; // IRCFx = 111 (16 MHz)
            OSCTUNEbits.PLLEN = 1; // x4 PLL enabled = 64MHz
            *ClockSet = C_250kHz;
            SwitchDelay = 255;
            break;

        default:
            // should never get here, but just in case
            OSCCON = 0x10; // IRCFx = 001
            OSCTUNEbits.PLLEN = 0; // x4 PLL disabled
            *ClockSet = C_500kHz;
            break;
    }
}
#endif

#if (ACTIVATE_EXAMPLE == 8)
/* #8
 * SW with INT0 + T0 int + Rotating leds
 */
void Timer0_Init(void);
void ADC_Init(void);
unsigned char ADC_Convert(void);
void Interrupts_Init(void);

#define DetectsInARow   5

typedef enum {
    LEFT2RIGHT,
    RIGHT2LEFT
} LEDDirections;
unsigned char LED_Display = 1;
LEDDirections Direction = LEFT2RIGHT;

void main(void) {
    // Init I/O
    TRISD = 0b00000000; // PORTD bits 7:0 are all outputs (0)
    TRISAbits.TRISA0 = 1; // TRISA0 input

    Timer0_Init(); // Init Timer0
    ADC_Init(); // Init ADC
    Interrupts_Init(); // Init interrupts

    while (1) {
        LATD = LED_Display;
    }
}

void ADC_Init(void) { // initialize the Analog-To-Digital converter.
    ANSEL = 0; //Turn off all other analog inputs
    ANSELbits.ANS0 = 1; // Turn on RA0 analog
    ADCON1 = 0; // REF = Vss and Vcc
    ADCON2 = 0b00111000; // Left Aligment, 20 Tda(40uS), Fosc/2 (2uS)
    ADCON0 = 0b00000001; // CH0, ADC on
}

unsigned char ADC_Convert(void) { // start an ADC conversion and return the 8 most-significant bits of the result
    ADCON0bits.GO_DONE = 1; // Start conversion
    while (ADCON0bits.GO_DONE == 1); // Wait for it to complete
    return ADRESH; // Return high byte of result
}

void Timer0_Init(void) {
    INTCONbits.TMR0IF = 0; // clear roll-over interrupt flag
    INTCON2bits.TMR0IP = 0; // Timer0 is low priority interrupt
    INTCONbits.TMR0IE = 1; // enable the Timer0 interrupt.

    T0CON = 0b00000001; // prescale 1:4 - about 1 second maximum delay.
    TMR0H = 0; // clear timer - always write upper byte first
    TMR0L = 0;

    T0CONbits.TMR0ON = 1; // start timer
}

void Interrupts_Init(void) {
    // Set up switch interrupt on INT0
    INTCON2bits.INTEDG0 = 0; // interrupt on falling edge of INT0 (switch pressed)
    INTCONbits.INT0IF = 0; // ensure flag is cleared
    INTCONbits.INT0IE = 1; // enable INT0 interrupt
    // NOTE: INT0 is ALWAYS a high priority interrupt

    // Set up global interrupts
    RCONbits.IPEN = 1; // Enable priority levels on interrupts
    INTCONbits.GIEL = 1; // Low priority interrupts allowed
    INTCONbits.GIEH = 1; // Interrupting enabled.
}

void interrupt high_isr(void) {
    // Check to see what caused the interrupt
    // (Necessary when more than 1 interrupt at a priority level)

    // Check for INT0 interrupt
    if (INTCONbits.INT0IF) {
        // clear (reset) flag
        INTCONbits.INT0IF = 0;

        // change directions
        if (Direction == LEFT2RIGHT) {
            Direction = RIGHT2LEFT; // change direction
        } else // (Direction == RIGHT2LEFT)
        {
            Direction = LEFT2RIGHT; // change direction
        }
    }

    // Check for another interrupt, examples:
    // if (PIR1bits.TMR1IF)     // Timer 1
    // if (PIR1bits.ADIF)       // ADC
}

void interrupt low_priority low_isr(void) {
    // Check to see what caused the interrupt
    // (Necessary when more than 1 interrupt at a priority level)

    // Check for Timer0 Interrupt
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0; // clear (reset) flag

        // Take an ADC conversion and use it to set Timer0
        TMR0H = ADC_Convert(); // MSB from ADC
        TMR0L = 0; // LSB = 0

        // update display variable
        if (Direction == LEFT2RIGHT) {
            LED_Display <<= 1; // rotate display by 1 from 0 to 7
            if (LED_Display == 0)
                LED_Display = 1; // rotated bit out, so set bit 0
        } else // (Direction == RIGHT2LEFT)
        {
            LED_Display >>= 1; // rotate display by 1 from 7 to 0
            if (LED_Display == 0)
                LED_Display = 0x80; // rotated bit out, so set bit 7
        }
    }
}
#endif

#if (ACTIVATE_EXAMPLE == 71)
/* #7.1
 * ADC value to leds level (No timer and no sw)
 */
void ADC_Init(void);
unsigned char ADC_Convert(void);

void main(void) {
    // Init I/O
    TRISD = 0b00000000; // PORTD bits 7:0 are all outputs (0)
    TRISAbits.TRISA0 = 1; // TRISA0 input

    // Init ADC
    ADC_Init();

    while (1) {
        unsigned char adc_val, result;

        adc_val = ADC_Convert(); // Get adc value
        result = ((unsigned char) (adc_val / 32)) + 1; // Get range / number of leds to turn on
        if (result == 1) { // if 1 check if its below 16 to turn off all leds or turn on just one
            if (adc_val < 16)
                result = 0;
        }
        result = (unsigned char) (pow(2, result)) - 1; // Get the corresponding value to turn on the leds (0,1,3,7,15,31,63,127,255)
        LATD = ~result; // Load value to leds
    }
}

void ADC_Init(void) { // initialize the Analog-To-Digital converter.
    ANSEL = 0; //Turn off all other analog inputs
    ANSELbits.ANS0 = 1; // Turn on RA0 analog
    ADCON1 = 0; // REF = Vss and Vcc
    ADCON2 = 0b00111000; // Left Aligment, 20 Tda(40uS), Fosc/2 (2uS)
    ADCON0 = 0b00000001; // CH0, ADC on
}

unsigned char ADC_Convert(void) { // start an ADC conversion and return the 8 most-significant bits of the result
    ADCON0bits.GO_DONE = 1; // Start conversion
    while (ADCON0bits.GO_DONE == 1); // Wait for it to complete
    return ADRESH; // Return high byte of result
}
#endif

#if (ACTIVATE_EXAMPLE == 7)
/* #7
 * ADC value to leds rotate time
 */
void Timer0_Init(void);
void ADC_Init(void);
unsigned char ADC_Convert(void);

#define Switch_Pin      PORTBbits.RB0
#define DetectsInARow   5

typedef enum {
    LEFT2RIGHT,
    RIGHT2LEFT
} LEDDirections;

unsigned char LED_Display; // 8-bit variable

void main(void) {
    LEDDirections Direction = LEFT2RIGHT;
    BOOL SwitchPressed = FALSE;

    LED_Display = 1; // initialize

    // Init I/O
    TRISD = 0b00000000; // PORTD bits 7:0 are all outputs (0)
    TRISAbits.TRISA0 = 1; // TRISA0 input

    INTCON2bits.RBPU = 0; // enable PORTB internal pullups
    WPUBbits.WPUB0 = 1; // enable pull up on RB0

    // ADCON1 is now set up in the InitADC() function.
    TRISBbits.TRISB0 = 1; // PORTB bit 0 (connected to switch) is input (1)

    // Init Timer0
    Timer0_Init();

    // Init ADC
    ADC_Init();

    while (1) {

        if (Direction == LEFT2RIGHT) {
            LED_Display <<= 1; // rotate display by 1 from 0 to 7
            if (LED_Display == 0)
                LED_Display = 1; // rotated bit out, so set bit 0
        }
        if (Direction == RIGHT2LEFT) {
            LED_Display >>= 1; // rotate display by 1 from 7 to 0
            if (LED_Display == 0)
                LED_Display = 0x80; // rotated bit out, so set bit 7
        }

        LATD = LED_Display; // output LED_Display value to PORTD LEDs

        do { // poll the switch while waiting for the timer to roll over.
            if (Switch_Pin == 1) { // look for switch released.
                SwitchPressed = FALSE;
            } else if (SwitchPressed == FALSE) // && (Switch_Pin == 0) due to if-else
            { // switch was just pressed
                SwitchPressed = TRUE;
                // change  direction
                if (Direction == LEFT2RIGHT)
                    Direction = RIGHT2LEFT;
                else
                    Direction = LEFT2RIGHT;
            }

        } while (INTCONbits.TMR0IF == 0);

        // Timer expired
        INTCONbits.TMR0IF = 0; // Reset Timer flag

        // Take an ADC conversion and use it to set Timer0
        TMR0H = ADC_Convert(); // MSB from ADC
        TMR0L = 0; // LSB = 0

    }

}

void Timer0_Init(void) {
    INTCONbits.TMR0IF = 0; // clear roll-over interrupt flag
    T0CON = 0b00000001; // prescale 1:4 - about 1 second maximum delay.
    TMR0H = 0; // clear timer - always write upper byte first
    TMR0L = 0;
    T0CONbits.TMR0ON = 1; // start timer
}

void ADC_Init(void) { // initialize the Analog-To-Digital converter.
    // First, we need to make sure the AN0 pin is enabled as an analog input
    // as the demo board potentiometer is connected to RA0/AN0
    // Don't forget that RB0/AN12 must be digital!
    ANSEL = 0; //turn off all other analog inputs
    //ANSELH = 0;
    ANSELbits.ANS0 = 1; // turn on RA0 analog

    // Sets bits VCFG1 and VCFG0 in ADCON1 so the ADC voltage reference is VSS to VDD

    ADCON1 = 0;

    // The ADC clock must as short as possible but still greater than the
    // minimum TAD time, datasheet parameter 130.  At the time this lesson was
    // written TAD minimum for the PIC18F45K20 is 1.4us.
    // At 1MHz clock, selecting ADCS = FOSC/2 = 500kHz.  One clock period
    // 1 / 500kHz = 2us, which greater than minimum required 1.4us.
    // So ADCON2 bits ADCS2-0 = 000
    //
    // The ACQT aquisition time should take into accound the internal aquisition
    // time TACQ of the ADC, datasheet paramter 130, and the settling time of
    // of the application circuit connected to the ADC pin.  Since the actual
    // settling time of the RC circuit with the demo board potentiometer is very
    // long but accuracy is not very important to this demo, we'll set ACQT2-0 to
    // 20TAD = 111
    //
    // ADFM = 0 so we can easily read the 8 Most Significant bits from the ADRESH
    // Special Function Register
    ADCON2 = 0b00111000;

    // Select channel 0 (AN0) to read the potentiometer voltage and turn on ADC
    ADCON0 = 0b00000001;
}

unsigned char ADC_Convert(void) { // start an ADC conversion and return the 8 most-significant bits of the result
    ADCON0bits.GO_DONE = 1; // start conversion
    while (ADCON0bits.GO_DONE == 1); // wait for it to complete
    return ADRESH; // return high byte of result
}
#endif

#if (ACTIVATE_EXAMPLE == 6)
/* #6
 * Timer0 + Rotate leds direction
 */
#define LED_Display LATD
#define Switch_Pin PORTBbits.RB0

typedef enum {
    LEFT2RIGHT, RIGHT2LEFT
} LEDDirections;

void main(void) {
    LEDDirections Direction = LEFT2RIGHT;
    BOOL SwitchPressed = FALSE;

    LED_Display = 1; // initialize
    TRISD = 0b00000000; // Display Output
    TRISBbits.TRISB0 = 1; // SW Input
    // Init Timer0
    INTCONbits.TMR0IF = 0; // clear flag
    T0CON = 0b00001000; // 16b, increments every instruction clock, No prescale
    TMR0H = 0;
    TMR0L = 0;
    T0CONbits.TMR0ON = 1;

    while (1) {
        if (Direction == LEFT2RIGHT) {
            LED_Display <<= 1; // rotate display by 1 from 0 to 7
            if (LED_Display == 0)
                LED_Display = 1; // rotated bit out, so set bit 0
        }
        if (Direction == RIGHT2LEFT) {
            LED_Display >>= 1; // rotate display by 1 from 7 to 0
            if (LED_Display == 0)
                LED_Display = 0x80; // rotated bit out, so set bit 7
        }

        LATD = LED_Display; // output LED_Display value to PORTD LEDs

        do { // poll the switch while waiting for the timer to roll over.
            if (Switch_Pin == 1) { // look for switch released.
                SwitchPressed = FALSE;
            } else if (SwitchPressed == FALSE) // && (Switch_Pin == 0) due to if-else
            { // switch was just pressed
                SwitchPressed = TRUE;
                // change  direction
                if (Direction == LEFT2RIGHT)
                    Direction = RIGHT2LEFT;
                else
                    Direction = LEFT2RIGHT;
            }

        } while (INTCONbits.TMR0IF == 0);

        // Timer expired
        INTCONbits.TMR0IF = 0; // Reset Timer flag
    }
}
#endif

#if(ACTIVATE_EXAMPLE == 5)
/* #5
 * Read switch with anti-debounce and release count
 */
#define _XTAL_FREQ 1000000 // 1 MHz
#define LED_Display LATD
#define SW_pin PORTBbits.RB0

void main(void) {
    TRISD = 0x00; // Display Output
    TRISBbits.TRISB0 = 1; // SW Input
    while (1) {
        unsigned char t;
        unsigned char s;
        for (t = 0x01; t > 0; t <<= 1) {
            LED_Display = t;
            do {
                while (SW_pin != 0);
                __delay_ms(50);
                if (SW_pin == 0) {
                    while (SW_pin == 0);
                    s = 1;
                } else {
                    s = 0;
                }
            } while (s == 0);
        }
    }
}
#endif

#if(ACTIVATE_EXAMPLE == 4)
/* #4
 * Rotate led
 */
#define _XTAL_FREQ 1000000 // 1 MHz

void main(void) {
    TRISD = 0x00;
    while (1) {
        unsigned char t;
        for (t = 0x01; t > 0; t <<= 1) {
            LATD = t;
            __delay_ms(500);
        }
    }
}
#endif

#if(ACTIVATE_EXAMPLE == 3)
/* #3
 * Blink LATD7 led
 */
#define _XTAL_FREQ 1000000 // 1 MHz

void main(void) {
    TRISD = 0b01111111;
    while (1) {
        LATDbits.LATD7 = ~LATDbits.LATD7;
        __delay_ms(500);
        __delay_ms(500);
    }
}
#endif

#if(ACTIVATE_EXAMPLE == 2)
/* #2
 *  Blink LATD7 led
 */
#define _XTAL_FREQ 1000000 // 1 MHz

void main(void) {
    TRISD = 0b01111111;
    while (1) {
        LATDbits.LATD7 = ~LATDbits.LATD7;
        __delay_ms(500);
        __delay_ms(500);
    }
}
#endif

#if(ACTIVATE_EXAMPLE == 1)

/* #1
 * Turn on LATD7 led
 */
void main(void) {
    TRISD = 0b01111111;
    LATDbits.LATD7 = 1;
    while (1) {
        ;
    }
}
#endif