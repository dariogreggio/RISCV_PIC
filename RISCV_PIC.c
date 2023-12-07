// Local Header Files
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "RISCV_PIC.h"

#ifdef __PIC32
#include <sys/attribs.h>
#include <sys/kmem.h>
#else
#include <libpic30.h>
#endif

#ifdef __PIC32
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"
#endif



#ifdef __PIC32

// PIC32MZ1024EFE064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
/* Default SYSCLK = 200 MHz (8MHz FRC / FPLLIDIV * FPLLMUL / FPLLODIV) */
//#pragma config FPLLIDIV = DIV_1, FPLLMULT = MUL_50, FPLLODIV = DIV_2
#pragma config FPLLIDIV = DIV_1         // System PLL Input Divider (1x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ// System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_51       // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384)
  // circa 6-7 secondi, 24.7.19
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON             // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3

// DEVADC0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC7

#else
// DSPIC33CH128MP202/502 Configuration Bit Settings

// 'C' source line config statements
#if defined(__dsPIC33CH__)
// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRCPLL    //FRCDIVN          // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = PLLKEN_ON       // PLLKEN (PLLKEN_ON)
#pragma config XTCFG = G3               // XT Config (24-32 MHz crystals)
#pragma config XTBST = ENABLE           // XT Boost (Boost the kick-start)

// FWDT
#pragma config RWDTPS = PS8192       // Run Mode Watchdog Timer Post Scaler select bits (1:8192)
  // 8 secondi, 17/12/19
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS8192       // Sleep Mode Watchdog Timer Post Scaler select bits (1:8192)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled in hardware)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0xFFFF         // Dead Man Timer Interval low word (Lower 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTIVTH
#pragma config DMTIVTH = 0xFFFF         // Dead Man Timer Interval high word (Uper 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTCNTL
#pragma config DMTCNTL = 0xFFFF         // Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMTCNTH
#pragma config DMTCNTH = 0xFFFF         // Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMT
#pragma config DMTDIS = OFF             // Dead Man Timer Disable bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FMBXM
#pragma config MBXM0 = M2S              // Mailbox 0 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM1 = M2S              // Mailbox 1 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM2 = M2S              // Mailbox 2 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM3 = M2S              // Mailbox 3 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM4 = M2S              // Mailbox 4 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM5 = M2S              // Mailbox 5 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM6 = M2S              // Mailbox 6 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM7 = M2S              // Mailbox 7 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM8 = S2M              // Mailbox 8 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM9 = S2M              // Mailbox 9 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM10 = S2M             // Mailbox 10 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM11 = S2M             // Mailbox 11 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM12 = S2M             // Mailbox 12 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM13 = S2M             // Mailbox 13 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM14 = S2M             // Mailbox 14 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM15 = S2M             // Mailbox 15 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))

// FMBXHS1
#pragma config MBXHSA = MBX15           // Mailbox handshake protocol block A register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block A)
#pragma config MBXHSB = MBX15           // Mailbox handshake protocol block B register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block B)
#pragma config MBXHSC = MBX15           // Mailbox handshake protocol block C register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block C)
#pragma config MBXHSD = MBX15           // Mailbox handshake protocol block D register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block D)

// FMBXHS2
#pragma config MBXHSE = MBX15           // Mailbox handshake protocol block E register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block E)
#pragma config MBXHSF = MBX15           // Mailbox handshake protocol block F register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block F)
#pragma config MBXHSG = MBX15           // Mailbox handshake protocol block G register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block G)
#pragma config MBXHSH = MBX15           // Mailbox handshake protocol block H register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block H)

// FMBXHSEN
#pragma config HSAEN = OFF              // Mailbox A data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSBEN = OFF              // Mailbox B data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSCEN = OFF              // Mailbox C data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSDEN = OFF              // Mailbox D data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSEEN = OFF              // Mailbox E data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSFEN = OFF              // Mailbox F data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSGEN = OFF              // Mailbox G data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSHEN = OFF              // Mailbox H data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)

// FCFGPRA0
#pragma config CPRA0 = MSTR             // Pin RA0 Ownership Bits (Master core owns pin.)
#pragma config CPRA1 = MSTR             // Pin RA1 Ownership Bits (Master core owns pin.)
#pragma config CPRA2 = MSTR             // Pin RA2 Ownership Bits (Master core owns pin.)
#pragma config CPRA3 = MSTR             // Pin RA3 Ownership Bits (Master core owns pin.)
#pragma config CPRA4 = MSTR             // Pin RA4 Ownership Bits (Master core owns pin.)

// FCFGPRB0
#pragma config CPRB0 = MSTR             // Pin RB0 Ownership Bits (Master core owns pin.)
#pragma config CPRB1 = MSTR             // Pin RB1 Ownership Bits (Master core owns pin.)
#pragma config CPRB2 = MSTR             // Pin RB2 Ownership Bits (Master core owns pin.)
#pragma config CPRB3 = MSTR             // Pin RB3 Ownership Bits (Master core owns pin.)
#pragma config CPRB4 = MSTR             // Pin RB4 Ownership Bits (Master core owns pin.)
#pragma config CPRB5 = MSTR             // Pin RB5 Ownership Bits (Master core owns pin.)
#pragma config CPRB6 = MSTR             // Pin RB6 Ownership Bits (Master core owns pin.)
#pragma config CPRB7 = MSTR             // Pin RB7 Ownership Bits (Master core owns pin.)
#pragma config CPRB8 = MSTR             // Pin RB8 Ownership Bits (Master core owns pin.)
#pragma config CPRB9 = MSTR             // Pin RB9 Ownership Bits (Master core owns pin.)
#pragma config CPRB10 = MSTR            // Pin RB10 Ownership Bits (Master core owns pin.)
#pragma config CPRB11 = MSTR            // Pin RB11 Ownership Bits (Master core owns pin.)
#pragma config CPRB12 = MSTR            // Pin RB12 Ownership Bits (Master core owns pin.)
#pragma config CPRB13 = MSTR            // Pin RB13 Ownership Bits (Master core owns pin.)
#pragma config CPRB14 = MSTR            // Pin RB14 Ownership Bits (Master core owns pin.)
#pragma config CPRB15 = MSTR            // Pin RB15 Ownership Bits (Master core owns pin.)

// FS1OSCSEL
#pragma config S1FNOSC = FRCPLL  //FRCDIVN        // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config S1IESO = ON              // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FS1OSC
#pragma config S1OSCIOFNC = OFF         // Slave OSC2 Pin Function bit (OSC2 is clock output)
#pragma config S1FCKSM = CSDCMD         // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config S1PLLKEN = S1PLLKEN_ON   // S1PLLKEN (S1PLLKEN_ON)

// FS1WDT
#pragma config S1RWDTPS = PS8192     // Run Mode Watchdog Timer Post Scaler select bits (1:8192)
#pragma config S1RCLKSEL = LPRC         // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config S1WINDIS = ON            // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config S1WDTWIN = WIN25         // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config S1SWDTPS = PS8192     // Sleep Mode Watchdog Timer Post Scaler select bits (1:8192)
#pragma config S1FWDTEN = ON_SW           // Watchdog Timer Enable bit (WDT enabled in hardware)

// FS1ICD
#pragma config S1ICS = PGD1             // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config S1ISOLAT = ON            // Isolate the Slave core subsystem from the master subsystem during Debug (The slave can operate (in debug mode) even if the SLVEN bit in the MSI is zero.)
#pragma config S1NOBTSWP = OFF          // BOOTSWP Instruction Enable/Disable bit (BOOTSWP instruction is disabled)

// FS1DEVOPT
#pragma config S1ALTI2C1 = OFF          // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config S1SPI1PIN = PPS          // S1 SPI1 Pin Select bit (Slave SPI1 uses I/O remap (PPS) pins)
#pragma config S1SSRE = ON              // Slave Slave Reset Enable (Slave generated resets will reset the Slave Enable Bit in the MSI module)
#pragma config S1MSRE = ON              // Master Slave Reset Enable (The master software oriented RESET events (RESET Op-Code, Watchdog timeout, TRAP reset, illegalInstruction) will also cause the slave subsystem to reset.)

// FS1ALTREG
#pragma config S1CTXT1 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config S1CTXT2 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config S1CTXT3 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config S1CTXT4 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)
#endif
#endif


const char CopyrightString[]= {'A','R','M','/','A','c','o','r','n',' ','E','m','u','l','a','t','o','r',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0',' ','-',' ', '1','4','/','0','4','/','2','2', 0 };

const char Copyr1[]="(C) Dario's Automation 2020-2022 - G.Dar\xd\xa\x0";



// Global Variables:
extern BOOL fExit,debug;
extern BYTE DoIRQ,DoNMI,DoHalt,DoReset,ColdReset;
extern BYTE ram_seg[];
extern BYTE rom_seg[];			
extern BYTE Keyboard[8];
extern volatile BYTE CIA1IRQ,CIA2IRQ,VICIRQ;
extern const unsigned char fontLCD_eu[],fontLCD_jp[];


#ifdef __PIC32

WORD displayColor[3]={BLACK,WHITE};
#endif

int UpdateScreen(BYTE c) {
  int x,y,x1,y1;
  SWORD color;
	INT8 i,j;
	BYTE *fontPtr;

   
#ifdef __PIC32
  
#define LCD_MAX_X 20
#define LCD_MAX_Y 4
#define DIGIT_X_SIZE 6
#define DIGIT_Y_SIZE 8
  
  y=(_TFTHEIGHT-(LCD_MAX_Y*DIGIT_Y_SIZE))/2;
  
//	fillRect(x,y,DIGIT_X_SIZE+3,DIGIT_Y_SIZE+1,BLACK);
  
  if(c)
    color=WHITE;
  else
    color=LIGHTGRAY;

  for(y1=0; y1<LCD_MAX_Y; y1++) {
      x=(_TFTWIDTH-(LCD_MAX_X*DIGIT_X_SIZE))/2;
      
      for(x1=0; x1<LCD_MAX_X; x1++) {


	fontPtr=fontLCD_eu+((UINT16)c)*10;
    for(i=0; i<8; i++) {
      UINT8 line;
      
	line = pgm_read_byte(fontPtr+i);
      
      for(j=0; j<6; j++, line >>= 1) {
        if(line & 0x1) {
			drawPixel(x+i, y+j, color);
        	} 
    	  }
  	  }
          
      }
      y+=DIGIT_Y_SIZE;
    }
  
#endif
	}


int main(void) {

  // disable JTAG port
//  DDPCONbits.JTAGEN = 0;
  
#ifdef __PIC32
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPB15Rbits.RPB15R = 4;        // Assign RPB15 as U6TX, pin 30
  U6RXRbits.U6RXR = 2;      // Assign RPB14 as U6RX, pin 29 
#ifdef USA_SPI_HW
  RPG8Rbits.RPG8R = 6;        // Assign RPG8 as SDO2, pin 6
//  SDI2Rbits.SDI2R = 1;        // Assign RPG7 as SDI2, pin 5
#endif
  RPD5Rbits.RPD5R = 12;        // Assign RPD5 as OC1, pin 53; anche vaga uscita audio :)
  CFGCONbits.IOLOCK = 1;      // PPS Lock

//  PPSOutput(4,RPC4,OC1);   //buzzer 4KHz , qua rimappabile 
  
#else
  
#if defined(__dsPIC33CH__)
   __builtin_write_RPCON(0x0000); //Unlock PPS
 
  RPINR18bits.U1RXR=36;                        /* RP36 RB4 to UART1 RX */
  RPOR2bits.RP37R=1;                           /* RP37 RB5 to UART1 TX */
//  RPOR7bits.RP47R=6;                           /* RP47 to SCK1; se PWM, allora non dovrebbe servire nulla... */
  
  __builtin_write_RPCON(0x0800); //Re lock control registers (based on possibly erroneous unlocking earlier)
#endif

#endif
  
  
#ifdef DEBUG_TESTREFCLK
// test REFCLK
  PPSOutput(4,RPC4,REFCLKO2);   // RefClk su pin 1 (RG15, buzzer)
	REFOCONbits.ROSSLP=1;
	REFOCONbits.ROSEL=1;
	REFOCONbits.RODIV=0;
	REFOCONbits.ROON=1;
	TRISFbits.TRISF3=1;
#endif

//	PPSLock;

   // Disable all Interrupts
  __builtin_disable_interrupts();
  
  
#ifdef __PIC32

//  SPLLCONbits.PLLMULT=10;
  
  OSCTUN=0;
  OSCCONbits.FRCDIV=0;
  
  // Switch to FRCDIV, SYSCLK=8MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x00; // FRC
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
    // At this point, SYSCLK is ~8MHz derived directly from FRC
 //http://www.microchip.com/forums/m840347.aspx
  // Switch back to FRCPLL, SYSCLK=200MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x01; // SPLL
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
  // At this point, SYSCLK is ~200MHz derived from FRC+PLL
//***
  mySYSTEMConfigPerformance();
  //myINTEnableSystemMultiVectoredInt(();
  
#else 
  
#if defined(__dsPIC33CH__)
  // Configure PLL prescaler, both PLL postscalers, and PLL feedback divider
  CLKDIVbits.PLLPRE = 1; // N1=1
  PLLFBDbits.PLLFBDIV = 180; // M = 125
  PLLDIVbits.VCODIV = 0b11;
  PLLDIVbits.POST1DIV = 4; // N2=5
  PLLDIVbits.POST2DIV = 1; // N3=1
  // Initiate Clock Switch to FRC with PLL (NOSC=0b001)
  __builtin_write_OSCCONH(0x01);
  __builtin_write_OSCCONL(OSCCON | 0x01);
  // Wait for Clock switch to occur
  while(OSCCONbits.OSWEN != 0);
  // Wait for PLL to lock
  while(OSCCONbits.LOCK != 1);
#endif

//Note: F PLLO = F PLLI * M/(N1 * N2 * N3); F PLLI = 8; M = 125; N1 = 1; N2 = 5; N3 = 1;
//so F PLLO = 8 * 125/(1 * 5 * 1) = 200 MHz or 50 MIPS.
  
#endif
  
    
	TRISB=0b0000000000110000;			// AN4,5 (rb4..5)
#ifdef __PIC32
	TRISC=0b0000000000000000;
	TRISD=0b0000000000001100;			// 2 pulsanti
	TRISE=0b0000000000000000;			// 3 led
	TRISF=0b0000000000000000;			// 
	TRISG=0b0000000000000000;			// SPI2 (rg6..8)
#endif

#if defined(__dsPIC33CH__)
  ANSELB=0;
#endif
#ifdef __PIC32
  ANSELE=0;
  ANSELG=0;
#endif

#ifdef __PIC32
  CNPUDbits.CNPUD2=1;   // switch/pulsanti
  CNPUDbits.CNPUD3=1;
  CNPUGbits.CNPUG6=1;   // I2C tanto per
  CNPUGbits.CNPUG8=1;  
#endif
      
  
  Timer_Init();
  PWM_Init();
  UART_Init(/*230400L*/ 115200L);

#ifdef __PIC32
  myINTEnableSystemMultiVectoredInt();
#ifndef __MPLAB_DEBUGGER_SIMULATOR
  ShortDelay(50000); 
#endif
#endif

  
//    	ColdReset=0;    Emulate(0);

#ifndef __MPLAB_DEBUGGER_SIMULATOR
#ifdef __PIC32
  Adafruit_ST7735_1(0,0,0,0,-1);
  Adafruit_ST7735_initR(INITR_BLACKTAB);
#endif
  
//  displayInit(NULL);
  
#ifdef m_LCDBLBit
  m_LCDBLBit=1;
#endif
  
#ifdef __PIC32
//	begin();
	clearScreen();

// init done
	setTextWrap(1);
//	setTextColor2(WHITE, BLACK);

	drawBG();
#endif
  
  __delay_ms(200);

#endif

  
	ColdReset=0;

  // http://shell-storm.org/online/Online-Assembler-and-Disassembler/
  *(DWORD *)(rom_seg+0x0000)=0xea000000|((0x110-0x0-8)>>2);
  *(DWORD *)(rom_seg+0x0004)=0xea000000;
  *(DWORD *)(rom_seg+0x0008)=0xea000000|((0x200-0x8-8)>>2);  // swi handler
  *(DWORD *)(rom_seg+0x000C)=0xe2000000;
  *(DWORD *)(rom_seg+0x0010)=0xea000000;
  *(DWORD *)(rom_seg+0x0014)=0xea000000;  // 
  *(DWORD *)(rom_seg+0x0018)=0xea000000|((0x204-0x18-8)>>2);  // irq
  *(DWORD *)(rom_seg+0x001c)=0xea000000|((0x204-0x1c-8)>>2);  // fiq
  *(DWORD *)(rom_seg+0x0110)=0xe10fc000;  // mrs ip,cpsr    esco da supervisor e passo a user preservando alcuni flag http://www.cs.otago.ac.nz/cosc440/readings/arm-syscall.pdf  
  *(DWORD *)(rom_seg+0x0114)=0xe3ccc01f;  // bic ip,ip,0b00011111
  *(DWORD *)(rom_seg+0x0118)=0xe38cc0d0;  // orr ip,ip,0b11010000   
  *(DWORD *)(rom_seg+0x011C)=0xe129f00c;  // msr cpsr,ip (IP=R12)
  *(DWORD *)(rom_seg+0x0120)=0xe3a00001;  // mov r0,#1
  *(DWORD *)(rom_seg+0x0124)=0xe28f100c;  // adr r1,msg1
  *(DWORD *)(rom_seg+0x0128)=0xe3a0200c;  // mov r2,#12
  *(DWORD *)(rom_seg+0x012C)=0xef900004;  // swi #00900004
  *(DWORD *)(rom_seg+0x0130)=0xef900001;  // swi #00900001
  *(DWORD *)(rom_seg+0x0134)=0xea000000|(0x00ffffff & ((0x120-0x134-8)>>2));  // B rifo = 0x110
  *(DWORD *)(rom_seg+0x0138)=0x6c6c6548;  // msg1: .ascii "Hello World\n"
  *(DWORD *)(rom_seg+0x013C)=0x6f57206f;
  *(DWORD *)(rom_seg+0x0140)=0x0a646c72;
  *(DWORD *)(rom_seg+0x0200)=0xe1a00000;  // swi handler; nop
  *(DWORD *)(rom_seg+0x0204)=0xe1b0f00e;  // movs pc,r14
  *(DWORD *)(rom_seg+0x0208)=0xE14FB000;  // MRS     R11,SPSR           ; ARMv3 and later
  *(DWORD *)(rom_seg+0x020C)=0xE129F00C;  // MSR     CPSR_c_f,R12       ; ARMv3 and later
          
  Emulate(0);

  }


#ifdef __PIC32


void mySYSTEMConfigPerformance(void) {
  unsigned PLLIDIV;
  unsigned PLLMUL;
  unsigned PLLODIV;
  float CLK2USEC;
  unsigned SYSCLK;
  static char PLLODIVVAL[]={
    2,2,4,8,16,32,32,32
    };

  PLLIDIV=SPLLCONbits.PLLIDIV+1;
  PLLMUL=SPLLCONbits.PLLMULT+1;
  PLLODIV=PLLODIVVAL[SPLLCONbits.PLLODIV];

  SYSCLK=(FCY*PLLMUL)/(PLLIDIV*PLLODIV);
  CLK2USEC=SYSCLK/1000000.0f;

  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  if(SYSCLK<=60000000)
    PRECONbits.PFMWS=0;
  else if(SYSCLK<=120000000)
    PRECONbits.PFMWS=1;
  else if(SYSCLK<=200000000)
    PRECONbits.PFMWS=2;
  else if(SYSCLK<=252000000)
    PRECONbits.PFMWS=4;
  else
    PRECONbits.PFMWS=7;

  PRECONbits.PFMSECEN=0;    // non c'è nella versione "2019" ...
  PRECONbits.PREFEN=0x1;

  SYSKEY = 0x0;
  }

void myINTEnableSystemMultiVectoredInt(void) {

  PRISS = 0x76543210;
  INTCONSET = _INTCON_MVEC_MASK /*0x1000*/;    //MVEC
  asm volatile ("ei");
  //__builtin_enable_interrupts();
  }

/* CP0.Count counts at half the CPU rate */
#define TICK_HZ (CPU_HZ / 2)

/* wait at least usec microseconds */
#if 0
void delay_usec(unsigned long usec) {
unsigned long start, stop;

  /* get start ticks */
  start = readCP0Count();

  /* calculate number of ticks for the given number of microseconds */
  stop = (usec * 1000000) / TICK_HZ;

  /* add start value */
  stop += start;

  /* wait till Count reaches the stop value */
  while (readCP0Count() < stop)
    ;
  }
#endif

void xdelay_us(uint32_t us) {
  
  if(us == 0) {
    return;
    }
  unsigned long start_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
  unsigned long now_count;
  long cycles = ((GetSystemClock() + 1000000U) / 2000000U) * us;
  do {
    now_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
    } while ((unsigned long)(now_count-start_count) < cycles);
  }

void __attribute__((used)) DelayUs(unsigned int usec) {
  unsigned int tWait, tStart;

  tWait=(GetSystemClock()/2000000)*usec;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
  }

void __attribute__((used)) DelayMs(unsigned int ms) {
  
  for(;ms;ms--)
    DelayUs(1000);
  }

// ===========================================================================
// ShortDelay - Delays (blocking) for a very short period (in CoreTimer Ticks)
// ---------------------------------------------------------------------------
// The DelayCount is specified in Core-Timer Ticks.
// This function uses the CoreTimer to determine the length of the delay.
// The CoreTimer runs at half the system clock. 100MHz
// If CPU_CLOCK_HZ is defined as 80000000UL, 80MHz/2 = 40MHz or 1LSB = 25nS).
// Use US_TO_CT_TICKS to convert from uS to CoreTimer Ticks.
// ---------------------------------------------------------------------------

void ShortDelay(                       // Short Delay
  DWORD DelayCount)                   // Delay Time (CoreTimer Ticks)
{
  DWORD StartTime;                    // Start Time
  StartTime = ReadCoreTimer();         // Get CoreTimer value for StartTime
  while ( (DWORD )(ReadCoreTimer() - StartTime) < DelayCount ) 
    ClrWdt();
  }
#endif
 

void Timer_Init(void) {

#ifdef __PIC32

  T2CON=0;
  T2CONbits.TCS = 0;                  // clock from peripheral clock
  T2CONbits.TCKPS = 7;                // 1:256 prescaler (pwm clock=390625Hz)
  T2CONbits.T32 = 0;                  // 16bit
//  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
  PR2 = 65535;                         // per ora faccio solo onda quadra, v. SID
  T2CONbits.TON = 1;                  // start timer per PWM
  
  // TIMER 3 INITIALIZATION (TIMER IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).
  T3CON=0;
  T3CONbits.TCS = 0;                  // clock from peripheral clock
  T3CONbits.TCKPS = 4;                // 1:16 prescaler
  PR3 = 3906;                         // rollover every n clocks; 
  T3CONbits.TON = 1;                  // start timer 

  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC3bits.T3IS=0;
  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole
  
#else
  T1CON=0;
  T1CONbits.TCS = 0;                  // clock from peripheral clock
  T1CONbits.TCKPS = 3;                // 1:256 prescaler (pwm clock=)
//  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
  PR1 = 65535;                         // per ora faccio solo onda quadra, v. SID
  T1CONbits.TON = 1;                  // start timer per PWM
  
//  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
//  IPC3bits.T3IS=0;
//  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole
  
#endif
  
	}

void PWM_Init(void) {
  
#ifdef __PIC32

  CFGCONbits.OCACLK=0;      // sceglie timer per PWM
  
  OC1CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
//  OC1R    = 500;		 // su PIC32 è read-only!
//  OC1RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC1R    = 32768;		 // su PIC32 è read-only!
  OC1RS   = 0;        // per ora faccio solo onda quadra, v. SID reg. 0-1
  OC1CONbits.ON = 1;   // on
#else
  
#endif
  }

void UART_Init(DWORD baudRate) {
  
#ifdef __PIC32
  
  U6MODE=0b0000000000001000;    // BRGH=1
  U6STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U6BRG=baudRateDivider;
  U6MODEbits.ON=1;
  
#if 0
  ANSELDCLR = 0xFFFF;
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPD11Rbits.RPD11R = 3;        // Assign RPD11 as U1TX
  U1RXRbits.U1RXR = 3;      // Assign RPD10 as U1RX
  CFGCONbits.IOLOCK = 1;      // PPS Lock

  // Baud related stuffs.
  U1MODEbits.BRGH = 1;      // Setup High baud rates.
  unsigned long int baudRateDivider = ((GetSystemClock()/(4*baudRate))-1);
  U1BRG = baudRateDivider;  // set BRG

  // UART Configuration
  U1MODEbits.ON = 1;    // UART1 module is Enabled
  U1STAbits.UTXEN = 1;  // TX is enabled
  U1STAbits.URXEN = 1;  // RX is enabled

  // UART Rx interrupt configuration.
  IFS1bits.U1RXIF = 0;  // Clear the interrupt flag
  IFS1bits.U1TXIF = 0;  // Clear the interrupt flag

  INTCONbits.MVEC = 1;  // Multi vector interrupts.

  IEC1bits.U1RXIE = 1;  // Rx interrupt enable
  IEC1bits.U1EIE = 1;
  IPC7bits.U1IP = 7;    // Rx Interrupt priority level
  IPC7bits.U1IS = 3;    // Rx Interrupt sub priority level
#endif
  
#else
  U1MODE=0b0000000010110000;    // TX,RX; 8bit async; BRGH=1
  U1STA= 0b0000000000000000;    // 
  DWORD baudRateDivider = ((GetPeripheralClock()/(2*baudRate))-1);    //v. anche BCLKSEL in U1MODEH
  U1BRG=baudRateDivider;
  TRISBbits.TRISB4=1;
  TRISBbits.TRISB5=0;
  U1MODEbits.UARTEN=1;
  
//  IEC0bits.U1RXIE=1;
#endif
  
  }

char BusyUART1(void) {
  
#ifdef __PIC32
  return(!U6STAbits.TRMT);
#else
  return(!U1STAbits.TRMT);
#endif
  }

char DataRdyUART1(void) {
  
#ifdef __PIC32
  return(U6STAbits.URXDA);
#else
#if defined(__dsPIC33CH__)
  return(!U1STAHbits.URXBE);
#endif
#endif
  }

void putsUART1(unsigned int *buffer) {
  char *temp_ptr = (char *)buffer;

    // transmit till NULL character is encountered 
#ifdef __PIC32

  if(U6MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U6STAbits.UTXBF); /* wait if the buffer is full */
            U6TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U6STAbits.UTXBF);  /* wait if the buffer is full */
            U6TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
#else

    // transmit till NULL character is encountered 
#if defined(__dsPIC33CH__)
  if(U1MODEbits.MOD == 3)        /* VERIFICARE! check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U1STAHbits.UTXBF); /* wait if the buffer is full */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U1STAHbits.UTXBF);  /* wait if the buffer is full */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
#endif
#endif
  }

unsigned int ReadUART1(void) {
  
#ifdef __PIC32
  if(U6MODEbits.PDSEL == 3)
    return (U6RXREG);
  else
    return (U6RXREG & 0xFF);
#else
#if defined(__dsPIC33CH__)
    if(U1MODEbits.MOD == 3)     // VERIFICARE
    return (U1RXREG);
  else
    return (U1RXREG & 0xFF);
#endif
#endif
  }

void WriteUART1(unsigned int data) {
  
#ifdef __PIC32
  if(U6MODEbits.PDSEL == 3)
    U6TXREG = data;
  else
    U6TXREG = data & 0xFF;
#else
#if defined(__dsPIC33CH__)
  if(U1MODEbits.MOD == 3)
    U1TXREG = data;
  else
    U1TXREG = data & 0xFF;
#endif
#endif
  }


#ifdef __PIC32
void __attribute__((no_fpu)) __ISR(_UART1_RX_VECTOR) UART1_ISR(void) {
  
  LATDbits.LATD4 ^= 1;    // LED to indicate the ISR.
  char curChar = U1RXREG;
  IFS3bits.U1RXIF = 0;  // Clear the interrupt flag!
  }

#endif


int emulateKBD(char ch) {
  int i;

	switch(ch) {
    case 0:
      for(i=0; i<8; i++)
        Keyboard[i]=0xff;
      break;
		case ' ':
			Keyboard[7] &= 0xef;
			break;
		case 'A':
			Keyboard[1] &= 0xfb;
			break;
		case 'B':
			Keyboard[3] &= 0xef;
			break;
		case 'C':
			Keyboard[2] &= 0xef;
			break;
		case 'D':
			Keyboard[2] &= 0xfb;
			break;
		case 'E':
			Keyboard[1] &= 0xbf;
			break;
		case 'F':
			Keyboard[2] &= 0xdf;
			break;
		case 'G':
			Keyboard[3] &= 0xfb;
			break;
		case 'H':
			Keyboard[3] &= 0xdf;
			break;
		case 'I':
			Keyboard[4] &= 0xfd;
			break;
		case 'J':
			Keyboard[4] &= 0xfb;
			break;
		case 'K':
			Keyboard[4] &= 0xdf;
			break;
		case 'L':
			Keyboard[5] &= 0xfb;
			break;
		case 'M':
			Keyboard[4] &= 0xef;
			break;
		case 'N':
			Keyboard[4] &= 0x7f;
			break;
		case 'O':
			Keyboard[4] &= 0xbf;
			break;
		case 'P':
			Keyboard[5] &= 0xfd;
			break;
		case 'Q':
			Keyboard[7] &= 0xbf;
			break;
		case 'R':
			Keyboard[2] &= 0xfd;
			break;
		case 'S':
			Keyboard[1] &= 0xdf;
			break;
		case 'T':
			Keyboard[2] &= 0xbf;
			break;
		case 'U':
			Keyboard[3] &= 0xbf;
			break;
		case 'V':
			Keyboard[3] &= 0x7f;
			break;
		case 'W':
			Keyboard[1] &= 0xfd;
			break;
		case 'X':
			Keyboard[2] &= 0x7f;
			break;
		case 'Y':
			Keyboard[3] &= 0xfd;
			break;
		case 'Z':
			Keyboard[1] &= 0xef;
			break;
		case '0':
			Keyboard[4] &= 0xf7;
			break;
		case '1':
			Keyboard[7] &= 0xfe;
			break;
		case '2':
			Keyboard[7] &= 0xf7;
			break;
		case '3':
			Keyboard[1] &= 0xfe;
			break;
		case '4':
			Keyboard[1] &= 0xf7;
			break;
		case '5':
			Keyboard[2] &= 0xfe;
			break;
		case '6':
			Keyboard[2] &= 0xf7;
			break;
		case '7':
			Keyboard[3] &= 0xfe;
			break;
		case '8':
			Keyboard[3] &= 0xf7;
			break;
		case '9':
			Keyboard[4] &= 0xfe;
			break;
		case '.':
			Keyboard[5] &= 0xef;
			break;
		case ',':		// ,
			Keyboard[5] &= 0x7f;		 // ,
			break;
		case '£':
			Keyboard[6] &= 0xfe;		 // 
			break;
		case '?':
			Keyboard[6] &= 0xfe;		 // non va, ci vuole anche shift...
			Keyboard[6] &= 0xef;
			break;
		case '\r':
			Keyboard[0] &= 0xfd;
			break;
		}
  }

const char keysFeed[]="10 PRINT TI:?\r20 POKE 53281,2\rPOKE 54273,100\rLIST\rRUN\r";
volatile BYTE keysFeedPtr=sizeof(keysFeed)-1;


#ifdef __PIC32

void __attribute__((no_fpu)) __ISR(_TIMER_3_VECTOR,ipl4AUTO) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static BYTE divider,dividerVICpatch;
  static WORD dividerEmulKbd;
  static BYTE keysFeedPhase=0;

//  LED2 ^= 1;      // check timing: 1600Hz, 9/11/19 (fuck berlin day))
  
  divider++;
  if(divider>=32) {   // 50 Hz per TOD
    divider=0;
//    CIA1IRQ=1;
    }
//  CIA2IRQ=1; fare...
  dividerVICpatch++;
  if(dividerVICpatch>1) {    // troppo lento il display...
    dividerVICpatch=0;
//    VICIRQ=1;       // refresh screen in 256/8=32 passate, 50 volte al secondo
    }
// v.  CIA1RegR[0xe];

  if(keysFeed[keysFeedPtr]) {
    dividerEmulKbd++;
    if(dividerEmulKbd>=300) {   // ~.2Hz per emulazione tastiera! (più veloce di tot non va...))
      dividerEmulKbd=0;
      if(!keysFeedPhase) {
        keysFeedPhase=1;
        emulateKBD(keysFeed[keysFeedPtr]);
        }
      else {
        keysFeedPhase=0;
        emulateKBD(NULL);
        keysFeedPtr++;
        }
      }
    }
    
  IFS0CLR = _IFS0_T3IF_MASK;
  }
#endif


#ifdef __PIC32

// ---------------------------------------------------------------------------------------
// declared static in case exception condition would prevent
// auto variable being created
static enum {
	EXCEP_IRQ = 0,			// interrupt
	EXCEP_AdEL = 4,			// address error exception (load or ifetch)
	EXCEP_AdES,				// address error exception (store)
	EXCEP_IBE,				// bus error (ifetch)
	EXCEP_DBE,				// bus error (load/store)
	EXCEP_Sys,				// syscall
	EXCEP_Bp,				// breakpoint
	EXCEP_RI,				// reserved instruction
	EXCEP_CpU,				// coprocessor unusable
	EXCEP_Overflow,			// arithmetic overflow
	EXCEP_Trap,				// trap (possible divide by zero)
	EXCEP_IS1 = 16,			// implementation specfic 1
	EXCEP_CEU,				// CorExtend Unuseable
	EXCEP_C2E				// coprocessor 2
  } _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

void __attribute__((weak)) _general_exception_handler(uint32_t __attribute__((unused)) code, uint32_t __attribute__((unused)) address) {
  }

void __attribute__((nomips16,used)) _general_exception_handler_entry(void) {
  
	asm volatile("mfc0 %0,$13" : "=r" (_epc_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_epc_code & 0x0000007C) >> 2;

  _general_exception_handler(_excep_code, _excep_addr);

	while (1)	{
		// Examine _excep_code to identify the type of exception
		// Examine _excep_addr to find the address that caused the exception
    }
  }

#else

void _ISR __attribute__((__no_auto_psv__)) _AddressError(void) {

	Nop();
	Nop();
	}

void _ISR __attribute__((__no_auto_psv__)) _StackError(void) {

	Nop();
	Nop();
	}


#if defined(__dsPIC33CH__)
// prove...
void _ISR __attribute__ ((no_auto_psv)) _MSIAInterrupt(void){
  uint16_t temp1;
  // bah IFS8bits.MSIAIF=0; 
  //Read the data from slave
  temp1=MSI1MBX6D;
  // set the flag for the next round of data transfer
  IFS8bits.MSIAIF=0;
  //Flag=1;
  }

void _ISR __attribute__ ((no_auto_psv)) _MSIS1Interrupt(void){

  // set the flag for the next round of data transfer
  IFS8bits.MSIS1IF=0;
  }
#endif

#endif

