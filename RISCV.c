// 6 dicembre 2023
// https://msyksphinz-self.github.io/riscv-isadoc/html/rvi.html#mret
//
//

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
//#include <graph.h>
//#include <dos.h>
//#include <malloc.h>
//#include <memory.h>
//#include <fcntl.h>
//#include <io.h>
#include <xc.h>

#ifdef __PIC32
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"
#endif

#include "RISCV_PIC.h"


#pragma check_stack(off)
// #pragma check_pointer( off )
#pragma intrinsic( _enable, _disable )



BYTE fExit=0;
BYTE debug=0;

#ifdef __PIC32
#define RAM_SIZE 0x40000
#define ROM_SIZE 16384				// 
#else
#define RAM_SIZE 0x2000
#define ROM_SIZE 4096				// 
#endif
#pragma pack(1)          // Put it here OR
struct __attribute__((__packed__)) RISCV_OPCODE {
  union __attribute__((__packed__)) {
    struct __attribute__((__packed__)) {
      unsigned int opcode: 7;
      unsigned int rd: 5;
      unsigned int funct3: 3;
      unsigned int rs1: 5;
      unsigned int rs2: 5;
      unsigned int funct7: 7;
      } typeR;
    struct __attribute__((__packed__)) {
      unsigned int opcode: 7;
      unsigned int rd: 5;
      unsigned int funct3: 3;
      unsigned int rs1: 5;
      unsigned int imm: 12;
      } typeI;
    struct __attribute__((__packed__)) {
      unsigned int opcode: 7;
      unsigned int imm1: 5;
      unsigned int funct3: 3;
      unsigned int rs1: 5;
      unsigned int rs2: 5;
      unsigned int imm2: 7;
      } typeS;
    struct __attribute__((__packed__)) {
      unsigned int opcode: 7;
      unsigned int imm3: 1;
      unsigned int imm1: 4;
      unsigned int funct3: 3;
      unsigned int rs1: 5;
      unsigned int rs2: 5;
      unsigned int imm2: 6;
      unsigned int imm4: 1;
      } typeB;
    struct __attribute__((__packed__)) {
      unsigned int opcode: 7;
      unsigned int rd: 5;
      unsigned int imm: 20;
      } typeA,typeU;    // bah...
    struct __attribute__((__packed__)) {
      unsigned int opcode: 7;
      unsigned int rd: 5;
      unsigned int imm3: 8;
      unsigned int imm2: 1;
      unsigned int imm1: 10;
      unsigned int imm4: 1;
      } typeJ;
    };
  };
struct __attribute__((__packed__)) RISCV_IMMEDIATE {
  union __attribute__((__packed__)) {
    struct __attribute__((__packed__)) {
      unsigned int inst20: 1;
      unsigned int inst21_24: 4;
      unsigned int inst25_30: 6;
      unsigned int inst31: 21;
      } I_IMMEDIATE;
    struct __attribute__((__packed__)) {
      unsigned int inst7: 1;
      unsigned int inst8_11: 4;
      unsigned int inst25_30: 6;
      unsigned int inst31: 21;
      } S_IMMEDIATE;
    struct __attribute__((__packed__)) {
      unsigned int dummy: 1;
      unsigned int inst8_11: 4;
      unsigned int inst25_30: 6;
      unsigned int inst7: 1;
      unsigned int inst31: 20;
      } B_IMMEDIATE;
    struct __attribute__((__packed__)) {
      unsigned int dummy: 12;
      unsigned int inst12_19: 8;
      unsigned int inst20_30: 11;
      unsigned int inst31: 1;
      } U_IMMEDIATE;
    struct __attribute__((__packed__)) {
      unsigned int dummy: 1;
      unsigned int inst21_24: 4;
      unsigned int inst25_30: 6;
      unsigned int inst20: 1;
      unsigned int inst12_19: 8;
      unsigned int inst31: 12;
      } J_IMMEDIATE;
    };
  };
  
#ifdef RISCV_64
union __attribute__((__packed__)) REGISTER {
  uint64_t d;
  struct __attribute__((__packed__)) { 
    DWORD l;
    DWORD h;
    } d32;
  struct __attribute__((__packed__)) { 
    WORD l;
    WORD h;
    } x;
  struct __attribute__((__packed__)) { 
    BYTE l;
    BYTE h;
    BYTE m;
    BYTE u;
    } b;
  };
#else
union __attribute__((__packed__)) REGISTER {
  DWORD d;
  struct __attribute__((__packed__)) { 
    WORD l;
    WORD h;
    } x;
  struct __attribute__((__packed__)) { 
    BYTE l;
    BYTE h;
    BYTE m;
    BYTE u;
    } b;
  };
#endif
#define ID_MODE 0x0000001F
#define ID_THUMB 0x00000020
#define ID_FIRQ 0x00000040
#define ID_IRQ 0x00000080
#define ID_ASYNCABORT 0x00000100
#define ID_ENDIANNESS 0x00000200
#define ID_THUMBH 0x0000FC00
#define ID_GE 0x000F0000
#define ID_JAZELLE 0x01000000
#define ID_THUMBL 0x06000000
#define ID_CUMULATIVE 0x08000000
union __attribute__((__packed__)) REGISTRO_CSR {   // 
  DWORD d;
  struct __attribute__((__packed__)) {
    unsigned int UIE: 1;     // 
    unsigned int WPRI: 3;
    unsigned int UPIE: 1;
    };
  };
/*0x000 ustatus User status register.
0x004 uie User interrupt-enable register.
0x005 utvec User trap handler base address.
0x040 uscratch Scratch register for user trap handlers.
0x041 uepc User exception program counter.
0x042 ucause User trap cause.
0x043 utval User bad address or instruction.
0x044 uip User interrupt pending.*/
//union REGISTRO_CSR ustatus,uie,utvec,uscratch,uepc,ucause,utval,uip;
  // dovrebbero essercene 3 gruppi, user machine e super...
  // in tutto 4096 :D bestie!
union REGISTRO_CSR mstatus,mie,mtvec,mscratch,mepc,mcause,mtval,mip,
      misa,mcounteren,menvcfg,mstatush,mscratch,mtinst,mtval2,mcycle,
      mvendorid,marchid,mimpid,mhartid,mconfigptr;

//https://five-embeddev.com/riscv-isa-manual/latest/machine.html
//https://five-embeddev.com/quickref/interrupts.html

BYTE ram_seg[RAM_SIZE];

#define _pc regs[31].d
#define _sp regs[13].d
#define _lr regs[1].d   // "di solito", dice - opp 5
#define _x0 regs[0].d   // il "registro x0" è dummy!!

uint64_t mtime,mtimecmp;    // sempre 64bit!
  
BYTE rom_seg[ROM_SIZE];			
BYTE Keyboard[8]={255,255,255,255,255,255,255,255};
BYTE DoReset=1,DoIRQ=0;
BYTE ColdReset=1;
union __attribute__((__packed__)) {
  struct __attribute__((__packed__)) RISCV_OPCODE opcode;
	struct __attribute__((__packed__)) {
		BYTE l;
		BYTE h;
		BYTE m;
		BYTE u;
		} b;
	DWORD d;
	} Pipe;
enum CPU_MODES {
#ifdef ARM_V4
  MODE_USER=16,
  MODE_FIQ=17,
  MODE_IRQ=18,
  MODE_SVC=19,
  MODE_ABORT=23,
  MODE_UNDEFINED=27,
  MODE_SYSTEM=31
#else
  MODE_USER=0,
  MODE_FIQ=1,
  MODE_IRQ=2,
  MODE_SVC=3
#endif  
  };
#ifdef RISCV_64
union __attribute__((__packed__)) RESULT {
  struct __attribute__((__packed__)) {
    BYTE l;
    BYTE h;
    BYTE m;
    BYTE u;
    } b;
  struct __attribute__((__packed__)) { 
    WORD l;
    WORD h;
    } x;
  struct __attribute__((__packed__)) { 
    WORD l;
    WORD h;
    } d32;
  uint64_t d;
  };
#else
union __attribute__((__packed__)) RESULT {
  struct __attribute__((__packed__)) {
    BYTE l;
    BYTE h;
    BYTE m;
    BYTE u;
    } b;
  struct __attribute__((__packed__)) { 
    WORD l;
    WORD h;
    } x;
  DWORD d;
  };
#endif

union REGISTRO_CSR *getCSR(DWORD );

BYTE GetValue(DWORD t) {
	register BYTE i;

//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {
    i=ram_seg[t];
    }

	return i;
	}

WORD GetValue16(DWORD t) {
	register WORD i;

  t &= 0xfffffffe;
//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {
    i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
    }

	return i;
	}

DWORD GetValue32(DWORD t) {
	register union __attribute__((__packed__)) RESULT i;

//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {
    i.b.l=ram_seg[t++];
    t &= 0xfffffffc;      // la rotazione dei non-aligned!
    i.b.h=ram_seg[t++];
    t &= 0xfffffffc;
    i.b.m=ram_seg[t++];
    t &= 0xfffffffc;
    i.b.u=ram_seg[t];
    }

	return i.d;
	}

#ifdef RISCV_64 
uint64_t GetPipe(uint64_t t) {

	if(t < ROM_SIZE) {			// 
	  Pipe.d=*(uint64_t *)&rom_seg[t & 0xfffffffc];
		}
	return Pipe.d;
	}
#else
DWORD GetPipe(DWORD t) {

	if(t < ROM_SIZE) {			// 
	  Pipe.d=*(DWORD *)&rom_seg[t & 0xfffffffc];
		}
	return Pipe.d;
	}
#endif

void PutValue(DWORD t,BYTE t1) {

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {		// 
    ram_seg[t]=t1;
    }
  }

void PutValue16(DWORD t,WORD t1) {

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

  t &= 0xfffffffe;
//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {		// 
    ram_seg[t++]=LOBYTE(t1);
    ram_seg[t]=HIBYTE(t1);
    }
  }

void PutValue32(DWORD t,DWORD t1) {

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {		// 
    ram_seg[t++]=LOBYTE(LOWORD(t1));
    t &= 0xfffffffc;      // la rotazione dei non-aligned!
    ram_seg[t++]=HIBYTE(LOWORD(t1));
    t &= 0xfffffffc;
    ram_seg[t++]=LOBYTE(HIWORD(t1));
    t &= 0xfffffffc;
    ram_seg[t]=HIBYTE(HIWORD(t1));
    }
  }


int Emulate(int mode) {
#ifdef RISCV_16
  union __attribute__((__packed__)) REGISTER regs[16];    // ma r15 è usato come PC
#else
  union __attribute__((__packed__)) REGISTER regs[32];    // ma r31 è usato come PC
#endif
  union __attribute__((__packed__)) REGISTER regs_FIQ[8],regs_IRQ[8],regs_Svc[8],regs_Abort[8],regs_Undef[8]; // alcuni sono di meno ma ok
    
	register DWORD i,j;
  DWORD n;
  register union __attribute__((__packed__)) RESULT res1,res2,res3;
  int c=0;

	_pc=0;
	do {

		c++;
		if(!(c & 4095)) {
      ClrWdt();
// yield()
			UpdateScreen(1);
    	}
    
    mcycle.d++;
    mtime++;      // bah e questo dunque??

		if(ColdReset)
			continue;

		/*
		if((_pc >= 0xa000) && (_pc <= 0xbfff)) {
			printf("%04x    %02x\n",_pc,GetRAMValue(_pc));
			}
			*/
		if(debug) {
			printf("%08lx    %08lx\n",_pc,GetValue32(_pc));
			}
		/*if(kbhit()) {
			getch();
			printf("%04x    %02x\n",_pc,GetRAMValue(_pc));
			printf("281-284: %02x %02x %02x %02x\n",*(p1+0x281),*(p1+0x282),*(p1+0x283),*(p1+0x284));
			printf("2b-2c: %02x %02x\n",*(p1+0x2b),*(p1+0x2c));
			printf("33-34: %02x %02x\n",*(p1+0x33),*(p1+0x34));
			printf("37-38: %02x %02x\n",*(p1+0x37),*(p1+0x38));
			}*/
		if(DoReset) {
      _sp=0;
 			_pc=0x00000000;   // implementation-defined, dice...
      //https://five-embeddev.com/riscv-isa-manual/latest/machine.html#sec:reset
      DoIRQ=0;
			DoReset=0;
      mstatus.d=0;
      mie.d=0;
      mtvec.d=0;
      mscratch.d=0;
      mepc.d=0;
      mcause.d=0;
      mtval.d=0;
      mip.d=0;
      }
		if(DoIRQ) {
//      https://stackoverflow.com/questions/61913210/risc-v-interrupt-handling-flow
      if(mstatus.UIE) {
        mstatus.UPIE=mstatus.UIE;

        mip.d=_pc;      // boh

// esempio https://stackoverflow.com/questions/57870131/risc-v-jump-to-interrupt-handler        
        
        mstatus.UIE=0;
        }
			}


//printf("Pipe: %08x, Pipe2w: %04x, Pipe2b1: %02x,%02x\n",Pipe,Pipe2.word,Pipe2.bytes.byte1,Pipe2.bytes.byte2);
    // ~250-400nS @200MHz secondo Simulatore PIC32, 
	    LED2 ^= 1;      // test ~700nS, 
    // ~1.2-1.5uS @200MHz secondo Simulatore dsPIC33, (ottimizzazioni=0 causa bug BFEXT)

#ifdef RISCV_64 
		GetPipe(_pc);   // dice che il PC "vale" sempre 4 o 8 oltre l'istruzione attuale... per la pipeline
    _pc+=8;
#else
		GetPipe(_pc);   // dice che il PC "vale" sempre 4 o 8 oltre l'istruzione attuale... per la pipeline
    _pc+=4;
#endif

      switch(Pipe.opcode.typeA.opcode) {
// The standard NOP is ADDI x0, x0, 0. 
        case 0b0110111:         // LUI load immediate
          res3.d=Pipe.opcode.typeA.imm << 12;
          regs[Pipe.opcode.typeI.rd].d=res3.d;
          break;
          
        case 0b0010111:         // AUIPC
          res3.d=_pc+(Pipe.opcode.typeA.imm << 12);
          regs[Pipe.opcode.typeI.rd].d=res3.d;
          break;
          
        case 0b1101111:         // JAL Branch and link
          res3.d=((Pipe.opcode.typeJ.imm1 << 1) | 
                  (Pipe.opcode.typeJ.imm2 << 11) | 
                  (Pipe.opcode.typeJ.imm3 << 12) | 
                  (Pipe.opcode.typeJ.imm4 << 20));
          regs[Pipe.opcode.typeI.rd].d=_pc;
          _pc += (signed int)res3.d;
          break;
          
        case 0b1100111:         // JALR
          res3.d=regs[Pipe.opcode.typeI.rs1].d+(signed int)Pipe.opcode.typeI.imm;
          res3.d &= 0xfffffffe;
          regs[Pipe.opcode.typeI.rd].d=_pc;
          _pc = res3.d;
          break;

        case 0b1100011:         // BEx
          switch(Pipe.opcode.typeB.funct3) {
            case 0b000:      // BEQ (equal)
              if(Pipe.opcode.typeB.rs1==Pipe.opcode.typeB.rs2) {
                res3.d=((Pipe.opcode.typeB.imm1 << 1) | 
                  (Pipe.opcode.typeB.imm2 << 5) | 
                  (Pipe.opcode.typeB.imm3 << 11) | 
                  (Pipe.opcode.typeB.imm4 << 12));
                _pc+=(signed int)res3.d;
                }
              break;
            case 0b001:      // BNE (not equal)
              if(Pipe.opcode.typeB.rs1!=Pipe.opcode.typeB.rs2) {
                res3.d=((Pipe.opcode.typeB.imm1 << 1) | 
                  (Pipe.opcode.typeB.imm2 << 5) | 
                  (Pipe.opcode.typeB.imm3 << 11) | 
                  (Pipe.opcode.typeB.imm4 << 12));
                _pc+=(signed int)res3.d;
                }
              break;
            case 0b100:      // BLT (unsigned higher o same)
              if((signed int)Pipe.opcode.typeB.rs1<(signed int)Pipe.opcode.typeB.rs2) {
                res3.d=((Pipe.opcode.typeB.imm1 << 1) | 
                  (Pipe.opcode.typeB.imm2 << 5) | 
                  (Pipe.opcode.typeB.imm3 << 11) | 
                  (Pipe.opcode.typeB.imm4 << 12));
                _pc+=(signed int)res3.d;
                }
              break;
            case 0b101:      // BGE (unsigned lower)
              if((signed int)Pipe.opcode.typeB.rs1>=(signed int)Pipe.opcode.typeB.rs2) {
                res3.d=((Pipe.opcode.typeB.imm1 << 1) | 
                  (Pipe.opcode.typeB.imm2 << 5) | 
                  (Pipe.opcode.typeB.imm3 << 11) | 
                  (Pipe.opcode.typeB.imm4 << 12));
                _pc+=(signed int)res3.d;
                }
              break;
            case 0b110:      // BLTU (negative)
              if(Pipe.opcode.typeB.rs1<Pipe.opcode.typeB.rs2) {
                res3.d=((Pipe.opcode.typeB.imm1 << 1) | 
                  (Pipe.opcode.typeB.imm2 << 5) | 
                  (Pipe.opcode.typeB.imm3 << 11) | 
                  (Pipe.opcode.typeB.imm4 << 12));
                _pc+=(signed int)res3.d;
                }
              break;
            case 0b111:      // BGEU (positive o zero)
              if(Pipe.opcode.typeB.rs1>=Pipe.opcode.typeB.rs2) {
                res3.d=((Pipe.opcode.typeB.imm1 << 1) | 
                  (Pipe.opcode.typeB.imm2 << 5) | 
                  (Pipe.opcode.typeB.imm3 << 11) | 
                  (Pipe.opcode.typeB.imm4 << 12));
                _pc+=(signed int)res3.d;
                }
              break;
            }
          break;

        case 0b0000011:         // LB LH LW LBU LHU
          switch(Pipe.opcode.typeI.funct3) {
            case 0b000:     // LB
              res3.b.l=GetValue(regs[Pipe.opcode.typeI.rs1].d + (signed int)Pipe.opcode.typeI.imm);
              regs[Pipe.opcode.typeI.rd].d=(signed int)res3.b.l;
              break;
            case 0b001:     // LH
              res3.x.l=GetValue16(regs[Pipe.opcode.typeI.rs1].d + (signed int)Pipe.opcode.typeI.imm);
              regs[Pipe.opcode.typeI.rd].d=(signed int)res3.x.l;
              break;
            case 0b010:     // LW
              res3.d=GetValue32(regs[Pipe.opcode.typeI.rs1].d + (signed int)Pipe.opcode.typeI.imm);
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b100:     // LBU
              res3.b.l=GetValue(regs[Pipe.opcode.typeI.rs1].d + (signed int)Pipe.opcode.typeI.imm);
              regs[Pipe.opcode.typeI.rd].d=res3.b.l;
              break;
            case 0b101:     // LHU
              res3.x.l=GetValue16(regs[Pipe.opcode.typeI.rs1].d + (signed int)Pipe.opcode.typeI.imm);
              regs[Pipe.opcode.typeI.rd].d=res3.x.l;
              break;
#ifdef RISCV_64
            case 0b110:     // LWU
              break;
            case 0b011:     // LD
              break;
#endif
            }
          break;
          
        case 0b0100011:         // SB SH SW
          switch(Pipe.opcode.typeS.funct3) {
            case 0b000:     // SB
              res3.b.l=regs[Pipe.opcode.typeS.rs2].b.l;
              PutValue(regs[Pipe.opcode.typeS.rs1].d + 
                      (signed int)(Pipe.opcode.typeS.imm1 |
                      (Pipe.opcode.typeS.imm1 << 5)),res3.b.l);
              break;
            case 0b001:     // SH
              res3.x.l=regs[Pipe.opcode.typeS.rs2].x.l;
              PutValue16(regs[Pipe.opcode.typeS.rs1].d + 
                      (signed int)(Pipe.opcode.typeS.imm1 |
                      (Pipe.opcode.typeS.imm1 << 5)),res3.x.l);
              break;
            case 0b010:     // SW
              res3.d=regs[Pipe.opcode.typeS.rs2].d;
              PutValue(regs[Pipe.opcode.typeS.rs1].d + 
                      (signed int)(Pipe.opcode.typeS.imm1 |
                      (Pipe.opcode.typeS.imm1 << 5)),res3.d);
              break;
            }
          break;
          
        case 0b0010011:         // ADDI SLTI SLTIU XORI ORI ANDI, SLLI SRLI SRAI
          switch(Pipe.opcode.typeI.funct3) {
            case 0b000:     // ADDI
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=(signed int)Pipe.opcode.typeI.imm;
              res3.d=res1.d + res2.d;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b010:     // SLTI
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=(signed int)Pipe.opcode.typeI.imm;
              res3.d=res1.d - res2.d ? 1 : 0;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b011:     // SLTIU
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=Pipe.opcode.typeI.imm;
              res3.d=res1.d - res2.d ? 1 : 0;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b100:     // XORI
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=(signed int)Pipe.opcode.typeI.imm;
              res3.d=res1.d ^ res2.d;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b110:     // ORI
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=(signed int)Pipe.opcode.typeI.imm;
              res3.d=res1.d | res2.d;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b111:     // ANDI
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=(signed int)Pipe.opcode.typeI.imm;
              res3.d=res1.d & res2.d;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
              
            case 0b001:     // SLLI
              res1.d=regs[Pipe.opcode.typeI.rs1].d;
              res2.d=Pipe.opcode.typeI.imm & 0b11111;   // ovvero Pipe.opcode.typeR.rs2...
              res3.d=res1.d << res2.d;
              regs[Pipe.opcode.typeI.rd].d=res3.d;
              break;
            case 0b101:     // SRLI SRAI
              if(!Pipe.opcode.typeR.funct7 /* & 0b0100000 */) { // SRLI
                res1.d=regs[Pipe.opcode.typeI.rs1].d;
                res2.d=Pipe.opcode.typeI.imm & 0b11111;
                res3.d=res1.d >> res2.d;
                regs[Pipe.opcode.typeI.rd].d=res3.d;
                }
              else {        // SRAI
                res1.d=regs[Pipe.opcode.typeI.rs1].d;
                res2.d=Pipe.opcode.typeI.imm & 0b11111;
                res3.d=(signed int)res1.d >> res2.d;
                regs[Pipe.opcode.typeI.rd].d=res3.d;
                }
              break;
#ifdef RISCV_64   // bah ci son già nel 32...
#endif
            }
          break;
          
        case 0b0110011:         // ADD SUB SLL SLT SLTU XOR SRL SRA OR AND 
          switch(Pipe.opcode.typeR.funct3) {
            case 0b000:     // ADD SUB
              if(!Pipe.opcode.typeR.funct7) {  // ADD
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=res1.d + res2.d;
                regs[Pipe.opcode.typeR.rd].d=res3.d;
                }
              else {        // SUB
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=res1.d - res2.d;
                regs[Pipe.opcode.typeR.rd].d=res3.d;
                }
#ifdef RISCV_32M
#endif
              break;
            case 0b001:     // SLL
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=res1.d << (res2.d & 0b11111);
                regs[Pipe.opcode.typeR.rd].d=res3.d;
#ifdef RISCV_32M
#endif
              break;
            case 0b010:     // SLT
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=(signed int)res1.d < (signed int)res2.d ? 1 : 0;
                regs[Pipe.opcode.typeR.rd].d=res3.d;
#ifdef RISCV_32M
#endif
              break;
            case 0b011:     // SLTU
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=res1.d < res2.d ? 1 : 0;
                regs[Pipe.opcode.typeR.rd].d=res3.d;
#ifdef RISCV_32M
#endif
              break;
            case 0b100:     // XOR
              res1.d=regs[Pipe.opcode.typeR.rs1].d;
              res2.d=regs[Pipe.opcode.typeR.rs2].d;
              res3.d=res1.d ^ res2.d;
              regs[Pipe.opcode.typeR.rd].d=res3.d;
#ifdef RISCV_32M
#endif
              break;
            case 0b101:     // SRL SRA
              if(!Pipe.opcode.typeR.funct7) {  // SRL
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=res1.d >> (res2.d & 0b11111);
                regs[Pipe.opcode.typeR.rd].d=res3.d;
                }
              else {        // SRA
                res1.d=regs[Pipe.opcode.typeR.rs1].d;
                res2.d=regs[Pipe.opcode.typeR.rs2].d;
                res3.d=(signed int)res1.d >> (res2.d & 0b11111);
                regs[Pipe.opcode.typeR.rd].d=res3.d;
                }
#ifdef RISCV_32M
#endif
              break;
            case 0b110:     // OR
              res1.d=regs[Pipe.opcode.typeR.rs1].d;
              res2.d=regs[Pipe.opcode.typeR.rs2].d;
              res3.d=res1.d | res2.d;
              regs[Pipe.opcode.typeR.rd].d=res3.d;
#ifdef RISCV_32M
#endif
              break;
            case 0b111:     // AND
              res1.d=regs[Pipe.opcode.typeR.rs1].d;
              res2.d=regs[Pipe.opcode.typeR.rs2].d;
              res3.d=res1.d & res2.d;
              regs[Pipe.opcode.typeR.rd].d=res3.d;
#ifdef RISCV_32M
#endif
              break;
            }
          break;
          
#ifdef RISCV_64 
        case 0b0011011:         // ADDIW SLLIW SRLIW SRAIW 
          switch(Pipe.opcode.typeI.funct3) {
            case 0b000:     // ADDIW
              res1.d32.l=regs[Pipe.opcode.typeI.rs1].d32.l;
              res2.d32.l=Pipe.opcode.typeI.imm;
              res3.d32.l=res1.d32.l + res2.d32.l;
              regs[Pipe.opcode.typeI.rd].d32.l=(signed int)res3.d32.l;
              break;
            case 0b001:     // SLLIW
              res1.d32.l=regs[Pipe.opcode.typeI.rs1].d32.l;
              res2.d32.l=Pipe.opcode.typeI.imm;
              res3.d32.l=res1.d32.l << (res2.d32.l & 0b11111);
              regs[Pipe.opcode.typeA.rd].d32.l=res3.d32.l;
              break;
            case 0b101:     // SRLIW SRAIW
              if(!Pipe.opcode.typeR.funct7) { // SRLIW
                res1.d32.l=regs[Pipe.opcode.typeI.rs1].d32.l;
                res2.d32.l=Pipe.opcode.typeI.imm;
                res3.d32.l=res1.d32.l >> (res2.d32.l & 0b11111);
                regs[Pipe.opcode.typeI.rd].d32.l=res3.d32.l;
                }
              else {        // SRAIW
                res1.d32.l=regs[Pipe.opcode.typeI.rs1].d32.l;
                res2.d32.l=Pipe.opcode.typeI.imm;
                res3.d32.l=(signed int)res1.d32.l >> (res2.d32.l & 0b11111);
                regs[Pipe.opcode.typeI.rd].d32.l=res3.d32.l;
                }
              break;
            }
          break;
          
        case 0b0111011:         // ADDW SUBW SLLW SRLW SRAW
          switch(Pipe.opcode.typeI.funct3) {
            case 0b000:     // ADDW SUBW
              if(!Pipe.opcode.typeR.funct7) { // ADDW
                res1.d32.l=regs[Pipe.opcode.typeR.rs1].d32.l;
                res2.d32.l=regs[Pipe.opcode.typeR.rs2].d32.l;
                res3.d32.l=res1.d32.l + res2.d32.l;
                regs[Pipe.opcode.typeR.rd].d32.l=res3.d32.l;
                }
              else {        // SUBW
                res1.d32.l=regs[Pipe.opcode.typeR.rs1].d32.l;
                res2.d32.l=regs[Pipe.opcode.typeR.rs2].d32.l;
                res3.d32.l=res1.d32.l - res2.d32.l;
                regs[Pipe.opcode.typeR.rd].d32.l=res3.d32.l;
                }
#ifdef RISCV_64M
#endif
              break;
            case 0b001:     // SLLW
              res1.d32.l=regs[Pipe.opcode.typeR.rs1].d32.l;
              res2.d32.l=regs[Pipe.opcode.typeR.rs2].d32.l;
              res3.d32.l=res1.d32.l << (res2.d32.l & 0b11111);
              regs[Pipe.opcode.typeR.rd].d32.l=res3.d32.l;
              break;
            case 0b101:     // SRLW SRAW
              if(!Pipe.opcode.typeR.funct7) { // SRLW
                res1.d32.l=regs[Pipe.opcode.typeR.rs1].d32.l;
                res2.d32.l=regs[Pipe.opcode.typeR.rs2].d32.l;
                res3.d32.l=res1.d32.l << (res2.d32.l & 0b11111);
                regs[Pipe.opcode.typeR.rd].d32.l=res3.d32.l;
                }
              else {        // SRAW
                res1.d32.l=regs[Pipe.opcode.typeI.rs1].d32.l;
                res2.d32.l=Pipe.opcode.typeA.imm;
                res3.d32.l=(signed int)res1.d32.l >> (res2.d32.l & 0b11111);
                regs[Pipe.opcode.typeA.rd].d32.l=res3.d32.l;
                }
              break;
#ifdef RISCV_64M
            case 0b100:     // DIVW
              break;
            case 0b101:     // DIVUW
              break;
            case 0b101:     // DIVUW
              break;
            case 0b110:     // REMW
              break;
            case 0b111:     // REMUW
              break;
#endif
            }
          break;
#endif
          
        case 0b0001111:     // FENCE
          break;
          
        case 0b1110011:     // ECALL, EBREAK, CSRx
          switch(Pipe.opcode.typeI.funct3) {
            DWORD t;
            union REGISTRO_CSR *pcsr;
            case 0b000:     // EBREAK
              switch(Pipe.opcode.typeI.imm) {    // ECALL
                case 0b000000000000:
                  //https://stackoverflow.com/questions/64863737/risc-v-software-interrupts

                  mcause;
                  mtval;
                  mip.d=_pc;      // boh

                  break;
                case 0b000000000001:        // EBREAK
                
                  mcause;
                  mtval;
                  mip;

                  break;
                case 0b011000000010:        // MRET
                  mstatus.UIE=mstatus.UPIE;
                  mstatus.UPIE=1;
                  _pc=mip.d;
                  break;
                case 0b000100000101:        // WFI
                  break;
                }
              break;
            case 0b001:     // CSRRW
              pcsr=getCSR(Pipe.opcode.typeI.imm);
              t=pcsr->d;
              pcsr->d = regs[Pipe.opcode.typeI.rs1].d;
              regs[Pipe.opcode.typeI.rs1].d=t;
              break;
            case 0b010:     // CSRRS
              pcsr=getCSR(Pipe.opcode.typeI.imm);
              t=pcsr->d;
              pcsr->d |= regs[Pipe.opcode.typeI.rs1].d;
              regs[Pipe.opcode.typeI.rs1].d=t;
              break;
            case 0b011:     // CSRRC
              pcsr=getCSR(Pipe.opcode.typeI.imm);
              t=pcsr->d;
              pcsr->d &= ~regs[Pipe.opcode.typeI.rs1].d;
              regs[Pipe.opcode.typeI.rs1].d=t;
              break;
            case 0b101:     // CSRRWI
              pcsr=getCSR(Pipe.opcode.typeI.imm);
              regs[Pipe.opcode.typeI.rs1].d=pcsr->d;
              pcsr->d = Pipe.opcode.typeI.rs1;
              break;
            case 0b110:     // CSRRSI
              pcsr=getCSR(Pipe.opcode.typeI.imm);
              t=pcsr->d;
              pcsr->d = t | Pipe.opcode.typeI.rs1;
              regs[Pipe.opcode.typeI.rs1].d=t;
              break;
            case 0b111:     // CSRRCI
              pcsr=getCSR(Pipe.opcode.typeI.imm);
              t=pcsr->d;
              pcsr->d = t & ~Pipe.opcode.typeI.rs1;
              regs[Pipe.opcode.typeI.rs1].d=t;
              break;
            }
          break;
          
        case 6:         // Coprocessor data transfer
          break;
          
        case 7:         // Coprocessor data operation, register transfer o software interrupt
          break;
        }
noAggFlag:
      continue;
      
AggFlag:
        ;

    
		} while(!fExit);
    
	}

union REGISTRO_CSR *getCSR(DWORD n) {
  
  switch(n) {   // per non allocarli "tutti" faccio così
    case 0x300:
      return &mstatus;
      break;
    case 0x301:
      return &misa;
      break;
    case 0x304:
      return &mie;
      break;
    case 0x305:
      return &mtvec;
      break;
    case 0x306:
      return &mcounteren;
      break;
    case 0x30a:
      return &menvcfg;
      break;
    case 0x310:
      return &mstatush;
      break;
    case 0x340:
      return &mscratch;
      break;
    case 0x341:
      return &mepc;
      break;
    case 0x342:
      return &mcause;
      break;
    case 0x343:
      return &mtval;
      break;
    case 0x344:
      return &mip;
      break;
    case 0x34a:
      return &mtinst;
      break;
    case 0x34b:
      return &mtval2;
      break;

    case 0xb00:
      return &mcycle;
      break;

    case 0xf11:
      return &mvendorid;
      break;
    case 0xf12:
      return &marchid;
      break;
    case 0xf13:
      return &mimpid;
      break;
    case 0xf14:
      return &mhartid;
      break;
    case 0xf15:
      return &mconfigptr;
      break;
    
    }
  return NULL;
  }

