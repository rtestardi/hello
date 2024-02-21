// *** main.c *********************************************************
// this is the main program that is launched by startup code; it just
// configures a timer ISR and runs the main program loop.

// *** configuration bits ***

#if defined(__32MK0512GPK064__) || defined(__32MK0512MCM064__)

// PIC32MK0512GPK064 Configuration Bit Settings

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FUSBIDIO2 = OFF          // USB2 USBID Selection (USBID pin is controlled by the port function)
#pragma config FVBUSIO2 = OFF           // USB2 VBUSON Selection bit (VBUSON pin is controlled by the port function)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration bit (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO1 = OFF          // USB1 USBID Selection (USBID pin is controlled by the port function)
#pragma config FVBUSIO1 = OFF           // USB2 VBUSON Selection bit (VBUSON pin is controlled by the port function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_8_16_MHZ // System PLL Input Range (8-16 MHz Input)
#pragma config FPLLICLK = PLL_POSC      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_40        // System PLL Multiplier (PLL Multiply by 40)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (4x Divider)
#pragma config BORSEL = HIGH            // Brown-out trip voltage (BOR trip voltage 2.1v (Non-OPAMP deviced operation))
#pragma config UPLLEN = ON              // USB PLL Enable (USB PLL Enabled)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable Secondary Oscillator)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disabled, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = ECC_DECC_DISABLE_ECCON_WRITABLE// Dynamic Flash ECC Configuration Bits (ECC and Dynamic ECC are disabled (ECCCON<1:0> bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = G3            // Secondary Oscillator Gain Control bits (Gain is G3)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = G3            // Primary Oscillator Coarse Gain Control bits (Gain Level 3 (highest))
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCFGAIN = G3           // Primary Oscillator Fine Gain Control bits (Gain is G3)
#pragma config POSCAGCDLY = AGCRNG_x_25ms// AGC Gain Search Step Settling Time Control (Settling time = 25ms x AGCRNG)
#pragma config POSCAGCRNG = ONE_X       // AGC Lock Range bit (Range 1x)
#pragma config POSCAGC = Automatic      // Primary Oscillator Gain Control bit (Automatic Gain Control for Oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot Enable (Normal EJTAG functionality)

// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ
#pragma config TSEQ = 0x0               // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#else
#error
#endif

#include <xc.h>
#include <sys/kmem.h>
#include <sys/attribs.h>

// *** main init ***

unsigned int oscillator_frequency;
unsigned int cpu_frequency;
unsigned int bus_frequency;

// initialize clocks, caches, microMIPS, vectored interrupts, etc.
void
main_init()
{
    register unsigned int val;
        
    /* unlock system for clock configuration */
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;

    /* Configure UPLL for 48 MHz USB*/
    /* UPOSCEN = UPLL */
    /* PLLODIV = DIV_8 */
    /* PLLMULT = MUL_32 */
    /* PLLIDIV = DIV_1 */
    /* PLLRANGE = RANGE_8_16_MHZ */
    UPLLCON = 0x31f0002;

    // set (most) bus clocks to cpu clock
    PB1DIVbits.PBDIV = 0;
    PB2DIVbits.PBDIV = 0;
    PB3DIVbits.PBDIV = 0;
    PB4DIVbits.PBDIV = 0;
    //PB5DIV = 1;
    //PB6DIV = 3;

    /* Lock system since done with clock configuration */
    SYSKEY = 0x33333333;

    /* Configure CP0.K0 for optimal performance (cached instruction pre-fetch) */
    __builtin_mtc0(16, 0,(__builtin_mfc0(16, 0) | 0x3));

    //CHECONbits.PERCHEEN = 1;
    //CHECONbits.DCHEEN = 1;
    //CHECONbits.ICHEEN = 1;

    /* Configure Wait States and Prefetch */
    CHECONbits.PFMWS = 2;
    CHECONbits.PREFEN = 1;

    // purge cache on flash program
    CHECONbits.DCHECOH = 1;

    // prevent JTAG from stealing our red LED after upgrade!
    CFGCONbits.JTAGEN = 0;

    // configure shadow registers (SRS) for all ISRs
    PRISS = 0x76543210;

    // turn on ISAONEXC so we take microMIPS interrupts and exceptions!
    // N.B. we compile microMIPS code for performance (including ISRs) but link with MIPS startup code (so the debugger works)
    asm("mfc0 %0,$16,3" :  "=r"(val));
    val |= 1<<16;
    asm("mtc0 %0,$16,3" :: "r" (val));

    //INTEnableSystemMultiVectoredInt();
    // set the CP0 cause IV bit high
    asm volatile("mfc0   %0,$13" : "=r"(val));
    val |= 0x00800000;
    asm volatile("mtc0   %0,$13" : "+r"(val));

    INTCONSET = _INTCON_MVEC_MASK;

    // set the CP0 status IE bit high to turn on interrupts
    //INTEnableInterrupts();
    val = 0;
    asm volatile("ei    %0" : "=r"(val));

    oscillator_frequency = 12000000;  // board has 12 MHz crystal
    cpu_frequency = 120000000;  // 120 MHz cpu clock
    bus_frequency = 120000000;  // 120 MHz bus clock
    // and 48 MHz USB clock
}

// *** timer 1 isr ***

int ticks;

void
__ISR(_TIMER_1_VECTOR, IPL4SRS)
timer_isr(void)
{
    // clear the interrupt flag
    IFS0CLR = _IFS0_T1IF_MASK;

    // if half a second has passed...
    if (ticks++ % 500 == 0) {
        // invert LED E3
        LATAINV = 0x400;  // 1Hz on red LED
    }
}

// *** main ***

int
main()  // we're called directly by startup code
{
    // initialize clocks, caches, microMIPS, vectored interrupts, etc.
    main_init();
    
    // configure LEDs E2 (A4) and E3 (A10) for digital output
    ANSELACLR = 0x410;
    TRISACLR = 0x410;
    LATASET = 0x410;

    // set up the timer interrupt with a priority of 4
    IEC0bits.T1IE = 1;
    IPC1bits.T1IP = 4;
    IPC1bits.T1IS = 0;
    
    // configure timer 1 to interrupt every millisecond
    T1CONCLR = _T1CON_ON_MASK;
    T1CON = (1 << _T1CON_TCKPS_POSITION);  // 1:8 prescale
    TMR1 = 0;
    PR1 = bus_frequency/8/1000 - 1;  // 1ms @ 120MHz
    T1CONSET = _T1CON_ON_MASK;

    // main program loop...
    for (;;) {
        // wait a while
        volatile int i;
        for (i = 0; i < 1000000; i++) {
        }
        
        // invert LED E2
        LATAINV = 0x10;  // ~10Hz on green LED
    }
    
    return 0;
}
