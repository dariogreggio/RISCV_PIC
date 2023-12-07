// 6 dicembre 2023

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


//#define RISCV_64 1
//#define RISCV_32M 1
//#define RISCV_64M 1
//#define RISCV_16 1


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
#define ID_OVERFLOW 0x10000000
#define ID_CARRY 0x20000000
#define ID_ZERO 0x40000000
#define ID_NEGATIVE 0x80000000
#ifdef ARM_V4
union __attribute__((__packed__)) REGISTRO_CSR {   // 
  DWORD d;
  struct __attribute__((__packed__)) {
    unsigned int Mode: 5;     // enum CPU_MODES 
    unsigned int Thumb: 1;
    unsigned int FIRQ: 1;
    unsigned int IRQ: 1;
    unsigned int AsyncAbort: 1;
    unsigned int Endianness: 1;
    unsigned int Thumb_IfThenH: 6;
    unsigned int GE: 4;
    unsigned int unused: 4;
    unsigned int Jazelle: 1;
    unsigned int Thumb_IfThenL: 2;
    unsigned int Cumulative: 1;
    unsigned int Overflow: 1;
    unsigned int Carry: 1;
    unsigned int Zero: 1;
    unsigned int Negative: 1;
    };
  };
union REGISTRO_CSR _csr,_csr2;
#define _csr_mode _csr.Mode
#else
union __attribute__((__packed__)) REGISTRO_CSR {   // 
  BYTE b;
  struct __attribute__((__packed__)) {
    unsigned int dummy: 2;     // enum CPU_MODES; il vero Mode � in r15<0..1>
    unsigned int FIRQ: 1;
    unsigned int IRQ: 1;
    unsigned int Overflow: 1;
    unsigned int Carry: 1;
    unsigned int Zero: 1;
    unsigned int Negative: 1;
    };
  };
union __attribute__((__packed__)) REGISTER31 {
  DWORD d;
  struct __attribute__((__packed__)) { 
    WORD l;
    WORD h;
    } x;
  struct __attribute__((__packed__)) { 
    union __attribute__((__packed__)) {
      BYTE l;
      unsigned int Mode :2; 
      };
    BYTE h;
    BYTE m;
    union __attribute__((__packed__)) {
      BYTE u;
      union __attribute__((__packed__)) REGISTRO_CSR s; 
      };
    } b;
  };
#define _csr ((union REGISTER31 *)&regs[31])->b.s
union REGISTRO_CSR _csr2;
// In the original mode, the Status Register and Program Counter were combined in a single register, as follows: https://heyrick.eu/armwiki/The_Status_register
#define _csr_mode ((union REGISTER31 *)&regs[31])->b.Mode
#endif
#ifdef ARM_V4
union REGISTRO_CSR _spsr[5];   // gestire buchi in Modes!
#else
union REGISTRO_CSR _spsr[3];
#endif

BYTE ram_seg[RAM_SIZE];

#define _pc regs[31].d
#define _sp regs[13].d
#define _lr regs[14].d
#define _ip regs[12].d
  
BYTE rom_seg[ROM_SIZE];			
BYTE Keyboard[8]={255,255,255,255,255,255,255,255};
BYTE DoReset=1,DoIRQ=0,DoSWI=0,DoFIQ=0;
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

BYTE GetValue(DWORD t) {
	register BYTE i;

//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {
    i=ram_seg[t];
    }

	return i;
	}

#ifdef ARM_V4
WORD GetValue16(DWORD t) {
	register WORD i;

  t &= 0xfffffffe;
//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {
    i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
    }

	return i;
	}
#endif

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

DWORD GetPipe(DWORD t) {

	if(t < ROM_SIZE) {			// 
	  Pipe.d=*(DWORD *)&rom_seg[t & 0xfffffffc];
		}
	return Pipe.d;
	}

void PutValue(DWORD t,BYTE t1) {

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {		// 
    ram_seg[t]=t1;
    }
  }

#ifdef ARM_V4
void PutValue16(DWORD t,WORD t1) {

// printf("rom_seg: %04x, p: %04x\n",rom_seg,p);

  t &= 0xfffffffe;
//	if(t < ROM_SIZE) {			// mappare...
  if(t < RAM_SIZE) {		// 
    ram_seg[t++]=LOBYTE(t1);
    ram_seg[t]=HIBYTE(t1);
    }
  }
#endif

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
  union __attribute__((__packed__)) REGISTER regs[16];    // ma r15 � usato come REGISTER15!
#else
  union __attribute__((__packed__)) REGISTER regs[32];    // ma r31 � usato come REGISTER31!
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
 			_pc=0x00000000;
      _csr_mode=MODE_SVC;    //  credo parta cos�... e i registri??
      DoIRQ=DoFIQ=DoSWI=0;
			DoReset=0;
			}
    if(DoSWI) {
			DoSWI=0;
      for(i=13; i<15; i++)
        regs_Svc[i-8]=regs[i];
      _spsr[2]=_csr;     // forse
      _csr_mode=MODE_SVC;
      _csr.FIRQ=_csr.IRQ=1;
      _lr=_pc;
			_pc=0x00000008;   // a 4 c'� Undef'd, a C prefetch, a 10 DataAbort 
//	    LED1 ^= 1;      // test NMI, /2020
  		}
		if(DoFIQ) {     // https://www.embedded.com/wp-content/uploads/media-1042868-0806esdlynx01.gif
      if(!_csr.FIRQ) {   // finire! 
        for(i=8; i<15; i++)
          regs_FIQ[i-8]=regs[i];
        _spsr[0]=_csr;     // forse
        _csr_mode=MODE_FIQ;
        _csr.FIRQ=1;   // forse..
        _lr=_pc;
  			_pc=0x0000001C;
        }
			}
		if(DoIRQ) {     // 
      if(!_csr.IRQ) {
        for(i=13; i<15; i++)
          regs_IRQ[i-8]=regs[i];
        _spsr[1]=_csr;     // forse
        _csr_mode=MODE_IRQ;
        _csr.IRQ=1;   // forse..
        _lr=_pc;
  			_pc=0x00000018;
        }
			}


//printf("Pipe: %08x, Pipe2w: %04x, Pipe2b1: %02x,%02x\n",Pipe,Pipe2.word,Pipe2.bytes.byte1,Pipe2.bytes.byte2);
    // ~250-400nS @200MHz secondo Simulatore PIC32, 17/1/2020
	    LED2 ^= 1;      // test ~700nS, 18/1/2020
    // ~1.2-1.5uS @200MHz secondo Simulatore dsPIC33, 20/1/2020 (ottimizzazioni=0 causa bug BFEXT)

		GetPipe(_pc);   // dice che il PC "vale" sempre 4 o 8 oltre l'istruzione attuale... per la pipeline
    _pc+=4;    // +4 o +8 o boh??
//    if(!_csr.Thumb) {   // fare...
//      }

      switch(Pipe.opcode.typeA.opcode) {
// The standard NOP is ADDI x0, x0, 0. 
        case 0b0110111:         // LUI load immediate
          if(Pipe.opcode.type0.optype & 1) {
            i=Pipe.opcode.type3.shiftAmount;
            switch(Pipe.opcode.type3.shift) {
              case 0b00:
                while(i--) {
                  _csr2.Carry=res2.d & 0x80000000 ? 1 : 0;
                  res2.d <<= 1;
                  }
                break;
              case 0b01:
                while(i--) {
                  _csr2.Carry=res2.d & 1;
                  res2.d >>= 1;
                  }
                break;
              case 0b10:
                while(i--) {
                  _csr2.Carry=res2.d & 1;
                  res2.d = ((signed long)res2.d) >> 1;
                  }
                break;
              case 0b11:
                if(!i) {    // VERIFICARE caso 0 e altro
                  _csr2.Carry=_csr.Carry;   // il carry vero o solo se S ??
                  j=res2.d & 1;
                  res2.d >>= 1;
                  res2.d |= _csr2.Carry ? 0x80000000 : 0;
                  _csr.Carry=j;
                  }
                else {
                  _csr2.Carry=_csr.Carry;   // il carry vero o solo se S ??
                  while(i--) {
                    j=res2.d & 1;
                    res2.d >>= 1;
                    res2.d |= _csr2.Carry ? 0x80000000 : 0;
                    _csr2.Carry=j;
                    }
                  }
                break;
              }
            }
          else {
            res2.d=Pipe.opcode.type2.imm;
            }
          
          if(Pipe.opcode.type2.P) {
            if(Pipe.opcode.type2.UpDown)
              n= regs[Pipe.opcode.type2.Rn].d + res2.d;
            else
              n= regs[Pipe.opcode.type2.Rn].d - res2.d;
            }
          if(!Pipe.opcode.type2.P) {
            if(Pipe.opcode.type2.UpDown)
              n= regs[Pipe.opcode.type2.Rn].d + res2.d;
            else
              n= regs[Pipe.opcode.type2.Rn].d - res2.d;
            }
          if(Pipe.opcode.type2.L) {
            if(Pipe.opcode.type2.B)
              res3.d=GetValue(n);
            else
              res3.d=GetValue32(n);
            regs[Pipe.opcode.type0.Rd].d=res3.d;
            }
          else {
            if(Pipe.opcode.type2.B)
              PutValue(n,regs[Pipe.opcode.type0.Rd].b.l);
            else
              PutValue32(n,regs[Pipe.opcode.type0.Rd].d);
            res3.d=regs[Pipe.opcode.type0.Rd].d;
            }
          if(Pipe.opcode.type2.W)
            regs[Pipe.opcode.type2.Rn].d = n;
          break;
          
        case 0b0110111:         // AUIPC
          break;
          
        case 0b1101111:         // JAL Branch
        case 0b1100111:         // JALR
          if(Pipe.opcode.type5.L) {
            _lr=_pc;    // -4 o -8 o boh??
            }
          _pc += 4/* PER ORA COSI' ! */;    
          _pc += ((signed long)Pipe.opcode.type5.address) << 2;
//          _pc &= 0x03ffffff;    // 26 bit IL SIGN EXTEND non fa come dovrebbe... evidentemente :) Quindi imposto address come signed-int (anche se mi pare non sia standard in bitfield...)
          break;

        case 0b1100011:         // BEx
    //    i=Pipe.opcode.type0.cond;
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:      // BEQ (Z set)  (equal)
              if(_csr.Zero)
                goto cond_ok;
              break;
            case 0b001:      // BNE (Z clear)  (not equal)
              if(!_csr.Zero)
                goto cond_ok;
              break;
            case 0b100:      // BLT HS/CS (C set)  (unsigned higher o same)
              if(_csr.Carry)
                goto cond_ok;
              break;
            case 0b101:      // BGE LO/CC (C clear) (unsigned lower)
              if(!_csr.Carry)
                goto cond_ok;
              break;
            case 0b110:      // BLTU  MI (N set) (negative)
              if(_csr.Negative)
                goto cond_ok;
              break;
            case 0b111:      // BGEU  PL (N clear) (positive o zero)
              if(!_csr.Negative)
                goto cond_ok;
              break;
            }
          break;

        case 0b0000011:         // LB LH LW LBU LHU
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // LB
              break;
            case 0b001:     // LH
              break;
            case 0b010:     // LW
              break;
            case 0b100:     // LBU
              break;
            case 0b101:     // LHU
              break;
#ifdef RISCV_64
            case 0b110:     // LWU
              break;
            case 0b011:     // LD
              break;
#endif
            }
          if(Pipe.opcode.type0.optype & 1) {
            i=Pipe.opcode.type3.shiftAmount;
            switch(Pipe.opcode.type3.shift) {
              case 0b00:
                while(i--) {
                  _csr2.Carry=res2.d & 0x80000000 ? 1 : 0;
                  res2.d <<= 1;
                  }
                break;
              case 0b01:
                while(i--) {
                  _csr2.Carry=res2.d & 1;
                  res2.d >>= 1;
                  }
                break;
              case 0b10:
                while(i--) {
                  _csr2.Carry=res2.d & 1;
                  res2.d = ((signed long)res2.d) >> 1;
                  }
                break;
              case 0b11:
                if(!i) {    // VERIFICARE caso 0 e altro
                  _csr2.Carry=_csr.Carry;   // il carry vero o solo se S ??
                  j=res2.d & 1;
                  res2.d >>= 1;
                  res2.d |= _csr2.Carry ? 0x80000000 : 0;
                  _csr.Carry=j;
                  }
                else {
                  _csr2.Carry=_csr.Carry;   // il carry vero o solo se S ??
                  while(i--) {
                    j=res2.d & 1;
                    res2.d >>= 1;
                    res2.d |= _csr2.Carry ? 0x80000000 : 0;
                    _csr2.Carry=j;
                    }
                  }
                break;
              }
            }
          else {
            res2.d=Pipe.opcode.type2.imm;
            }
          
          if(Pipe.opcode.type2.P) {
            if(Pipe.opcode.type2.UpDown)
              n= regs[Pipe.opcode.type2.Rn].d + res2.d;
            else
              n= regs[Pipe.opcode.type2.Rn].d - res2.d;
            }
          if(!Pipe.opcode.type2.P) {
            if(Pipe.opcode.type2.UpDown)
              n= regs[Pipe.opcode.type2.Rn].d + res2.d;
            else
              n= regs[Pipe.opcode.type2.Rn].d - res2.d;
            }
          if(Pipe.opcode.type2.L) {
            if(Pipe.opcode.type2.B)
              res3.d=GetValue(n);
            else
              res3.d=GetValue32(n);
            regs[Pipe.opcode.type0.Rd].d=res3.d;
            }
          else {
            if(Pipe.opcode.type2.B)
              PutValue(n,regs[Pipe.opcode.type0.Rd].b.l);
            else
              PutValue32(n,regs[Pipe.opcode.type0.Rd].d);
            res3.d=regs[Pipe.opcode.type0.Rd].d;
            }
          if(Pipe.opcode.type2.W)
            regs[Pipe.opcode.type2.Rn].d = n;
          break;
          
        case 0b0100011:         // SB SH SW
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // SB
              break;
            case 0b001:     // SH
              break;
            case 0b010:     // SW
              break;
            }
          break;
          
        case 0b0010011:         // ADDI SLTI SLTIU XORI ORI ANDI, SLLI SRLI SRAI
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // ADDI
              break;
            case 0b010:     // SLTI
              break;
            case 0b011:     // SLTIU
              break;
            case 0b100:     // XORI
              break;
            case 0b110:     // ORI
              break;
            case 0b111:     // ANDI
              break;
              
            case 0b001:     // SLLI
              break;
            case 0b101:     // SRLI SRAI
              if(!Pipe.opcode.typeA.funct7) { // SRLI
                }
              else {        // SRAI
                }
              break;
#ifdef RISCV_64   // bah ci son gi� nel 32...
#endif
            }
          break;
          
        case 0b0110011:         // ADD SUB SLL SLT SLTU XOR SRL SRA OR AND 
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // ADD SUB
              if(!Pipe.opcode.typeA.funct7) {  // ADD
                }
              else {        // SUB
                }
#ifdef RISCV_32M
#endif
              break;
            case 0b001:     // SLL
#ifdef RISCV_32M
#endif
              break;
            case 0b010:     // SLT
#ifdef RISCV_32M
#endif
              break;
            case 0b011:     // SLTU
#ifdef RISCV_32M
#endif
              break;
            case 0b100:     // XOR
#ifdef RISCV_32M
#endif
              break;
            case 0b101:     // SRL SRLA
              if(!Pipe.opcode.typeA.funct7) {  // SRL
                }
              else {        // SRA
                }
#ifdef RISCV_32M
#endif
              break;
            case 0b110:     // OR
#ifdef RISCV_32M
#endif
              break;
            case 0b111:     // AND
#ifdef RISCV_32M
#endif
              break;
            }
          break;
          
#ifdef RISCV_64 
        case 0b0011011:         // ADDIW SLLIW SRLIW SRAIW 
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // ADDIW
              break;
            case 0b001:     // SLLIW
              break;
            case 0b101:     // SRLIW SRAIW
              if(!Pipe.opcode.typeA.funct7) { // SRLIW
                }
              else {        // SRAIW
                }
              break;
            }
          break;
          
        case 0b0111011:         // ADDW SUBW SLLW SRLW SRAW
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // ADDW SUBW
              if(!Pipe.opcode.typeA.funct7) { // ADDW
                }
              else {        // SUBW
                }
#ifdef RISCV_64M
#endif
              break;
            case 0b001:     // SLLW
              break;
            case 0b101:     // SRLW SRAW
              if(!Pipe.opcode.typeA.funct7) { // SRLW
                }
              else {        // SRAW
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
          
        case 0:         // data processing, PSR transfer (anche halfword, v4); branch exchange (v4)
          if(Pipe.opcode.type0b.u2 == 0b1001) {   // caso speciale MULtiply
            if(Pipe.opcode.type0b.u3 == 0b000) {   // 32
              if(Pipe.opcode.type0b.A) {
                res1.d=regs[Pipe.opcode.type0b.Rm].d;
                res2.d=regs[Pipe.opcode.type0b.Rs].d;
                res3.d = res1.d*res2.d;
                res2.d=regs[Pipe.opcode.type0b.Rn].d;
                res3.d = res3.d+res2.d;
                }
              else {
                res1.d=regs[Pipe.opcode.type0b.Rm].d;
                res2.d=regs[Pipe.opcode.type0b.Rs].d;
                res3.d = res1.d*res2.d;
                }
              if(Pipe.opcode.type0b.S)
                goto AggFlagZ;
              else
                goto noAggFlag;
              }
#ifdef ARM_V4   // pure V3, dice
            else if(Pipe.opcode.type0b.u3 == 0b010) {   // 64 unsigned
              unsigned long long ll;
              if(Pipe.opcode.type0b.A) {
                unsigned long long ll2;
                ll2=regs[Pipe.opcode.type0b.Rd].d;
                ll2 <<= 32;
                ll2 |= regs[Pipe.opcode.type0b.Rn].d;
                ll = (unsigned long long)regs[Pipe.opcode.type0b.Rm].d * (unsigned long long)regs[Pipe.opcode.type0b.Rs].d;
                ll += ll2;
                regs[Pipe.opcode.type0b.Rd].d = ll >> 32;
                regs[Pipe.opcode.type0b.Rn].d = ll & 0xffffffff;
                }
              else {
                ll = (unsigned long long)regs[Pipe.opcode.type0b.Rm].d * (unsigned long long)regs[Pipe.opcode.type0b.Rs].d;
                regs[Pipe.opcode.type0b.Rd].d = ll >> 32;
                regs[Pipe.opcode.type0b.Rn].d = ll & 0xffffffff;
                }
              }
            else if(Pipe.opcode.type0b.u3 == 0b011) {   // 64 signed
              unsigned long long ll;
              if(Pipe.opcode.type0b.A) {
                unsigned long long ll2;   // signed o no??
                ll2=regs[Pipe.opcode.type0b.Rd].d;
                ll2 <<= 32;
                ll2 |= regs[Pipe.opcode.type0b.Rn].d;
                ll = (signed long long)regs[Pipe.opcode.type0b.Rm].d * (signed long long)regs[Pipe.opcode.type0b.Rs].d;
                ll += ll2;
                regs[Pipe.opcode.type0b.Rd].d = ll >> 32;
                regs[Pipe.opcode.type0b.Rn].d = ll & 0xffffffff;
                }
              else {
                ll = (signed long long)regs[Pipe.opcode.type0b.Rm].d * (signed long long)regs[Pipe.opcode.type0b.Rs].d;
                regs[Pipe.opcode.type0b.Rd].d = ll >> 32;
                regs[Pipe.opcode.type0b.Rn].d = ll & 0xffffffff;
                }
              }
#endif
            if(Pipe.opcode.type0b.S)
              goto AggFlagZ;
            else
              goto noAggFlag;
            }   // multiply
          
#ifdef ARM_V4
          else if(Pipe.opcode.type0c.u2 == 1 && Pipe.opcode.type0c.u3 == 1 && Pipe.opcode.type0c.u4 == 0b0000) {   // 
            if(Pipe.opcode.type0c.u5 == 1) {   // 
              res2.d=Pipe.opcode.type0d.ofsL | (Pipe.opcode.type0d.ofsH << 4);
              }
            else {
              res2.d=regs[Pipe.opcode.type0c.Rm].d;
              }
            if(Pipe.opcode.type0c.P) {
              if(Pipe.opcode.type0c.UpDown)
                n= regs[Pipe.opcode.type0c.Rn].d + res2.d;
              else
                n= regs[Pipe.opcode.type0c.Rn].d - res2.d;
              }
            if(!Pipe.opcode.type0c.P) {
              if(Pipe.opcode.type0c.UpDown)
                n= regs[Pipe.opcode.type0c.Rn].d + res2.d;
              else
                n= regs[Pipe.opcode.type0c.Rn].d - res2.d;
              }
            if(Pipe.opcode.type0c.L) {
              switch(Pipe.opcode.type0c.SH) {
                case 0:   // quasi-swp ma non ammesso
                  break;
                case 2:
                  res3.d=(signed long)GetValue(n);
                  break;
                case 1:
                  res3.d=GetValue16(n);
                  break;
                case 3:
                  res3.d=(signed long)GetValue16(n);
                  break;
                regs[Pipe.opcode.type0.Rd].d=res3.d;
                }
              }
            else {
              switch(Pipe.opcode.type0c.SH) {
                case 0:   // swp
//          else if(Pipe.opcode.type0e.u2 == 0b1001 && Pipe.opcode.type0e.u3 == 0b0000 && Pipe.opcode.type0e.u4 == 0b00 && Pipe.opcode.type0e.u5 == 0b10) {   // swp
                  if(Pipe.opcode.type0e.u5 == 0b10 && Pipe.opcode.type0e.u4 == 0b00) {   // dovrebbero essere SOLO cos� per SWP
              // c'� qualche dubbio su come si usano i 3 registri...
                    if(Pipe.opcode.type0e.B) {
                      res3.b.l=GetValue(regs[Pipe.opcode.type0e.Rn].d);
                      PutValue(regs[Pipe.opcode.type0e.Rn].d,regs[Pipe.opcode.type0e.Rm].b.l);
                      regs[Pipe.opcode.type0e.Rd].b.l=res3.b.l;
                      }
                    else {
                      res3.d=GetValue32(n);
                      PutValue32(regs[Pipe.opcode.type0e.Rn].d,regs[Pipe.opcode.type0e.Rm].d);
                      regs[Pipe.opcode.type0e.Rd].d=res3.d;
                      }
                    }
                  break;
                case 2:   // dice che "non dovrebbe accadere" signed store! 
                  PutValue(n,regs[Pipe.opcode.type0.Rd].b.l);
                  break;
                case 1:
                  PutValue16(n,regs[Pipe.opcode.type0.Rd].d);
                  break;
                case 3:
                  PutValue16(n,regs[Pipe.opcode.type0.Rd].d);
                  break;
                regs[Pipe.opcode.type0.Rd].d=res3.d;
                }
              res3.d=regs[Pipe.opcode.type0.Rd].d;
              }
            if(Pipe.opcode.type0c.W)
              regs[Pipe.opcode.type0c.Rn].d = n;

            goto noAggFlag;
            }
          else if((Pipe.d & 0x0fffffff0) == 0b00000001001011111111111100010000) {   // bah
            _pc = regs[Pipe.opcode.type0.Rn].d;
            _csr.Thumb=Pipe.opcode.type0.Rn & 1;
            }
#else
          else if(Pipe.opcode.type0e.u2 == 0b1001 && Pipe.opcode.type0e.u3 == 0b0000 && Pipe.opcode.type0e.u5 == 0b10 && Pipe.opcode.type0e.u4 == 0b00) {   // SWP
            // c'� qualche dubbio su come si usano i 3 registri...
            if(Pipe.opcode.type0e.B) {
              res3.b.l=GetValue(regs[Pipe.opcode.type0e.Rn].d);
              PutValue(regs[Pipe.opcode.type0e.Rn].d,regs[Pipe.opcode.type0e.Rm].b.l);
              regs[Pipe.opcode.type0e.Rd].b.l=res3.b.l;
              }
            else {
              res3.d=GetValue32(n);
              PutValue32(regs[Pipe.opcode.type0e.Rn].d,regs[Pipe.opcode.type0e.Rm].d);
              regs[Pipe.opcode.type0e.Rd].d=res3.d;
              }

            goto noAggFlag;
            }
#endif
          
        case 1:         // data processing, PSR transfer 
          res1.d=regs[Pipe.opcode.type0.Rn].d;
          if(Pipe.opcode.type0.optype & 1) {
            res2.d=Pipe.opcode.type1.imm;
            while(Pipe.opcode.type1.rotate--) {
              i=res2.d & 3;     // non serve, direi
              res2.d >>= 2;
              res2.d |= i << 30;
              }
            }
          else {
            res2.d=regs[Pipe.opcode.type0.Rm].d;
            if(Pipe.opcode.type0.shiftType)
              i=regs[Pipe.opcode.type0a.Rs].b.l;
            else
              i=Pipe.opcode.type0.shiftAmount;
            switch(Pipe.opcode.type0.shift) {
              case 0b00:
                while(i--) {
                  _csr2.Carry=res2.d & 0x80000000 ? 1 : 0;
                  res2.d <<= 1;
                  }
                break;
              case 0b01:
                while(i--) {
                  _csr2.Carry=res2.d & 1;
                  res2.d >>= 1;
                  }
                break;
              case 0b10:
                while(i--) {
                  _csr2.Carry=res2.d & 1;
                  res2.d = ((signed long)res2.d) >> 1;
                  }
                break;
              case 0b11:
                if(!i) {    // VERIFICARE caso 0 e altro
                  _csr2.Carry=_csr.Carry;   // il carry vero o solo se S ??
                  j=res2.d & 1;
                  res2.d >>= 1;
                  res2.d |= _csr2.Carry ? 0x80000000 : 0;
                  _csr.Carry=j;
                  }
                else {
                  _csr2.Carry=_csr.Carry;   // il carry vero o solo se S ??
                  while(i--) {
                    j=res2.d & 1;
                    res2.d >>= 1;
                    res2.d |= _csr2.Carry ? 0x80000000 : 0;
                    _csr2.Carry=j;
                    }
                  }
                break;
              }
            }
          switch(Pipe.opcode.type0.opcode) {
            unsigned long long ll;
            case 0b0000:    // AND
              res3.d=res1.d & res2.d;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              break;
            case 0b0001:    // EOR
              res3.d=res1.d ^ res2.d;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              break;
            case 0b0010:    // SUB
              ll=(signed long long)res1.d - res2.d;
              res3.d=ll;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              _csr2.Carry=ll & 0xffffffff00000000 ? 1 : 0;
              break;
            case 0b0011:    // RSB
              ll=(signed long long)res2.d - res1.d;
              res3.d=ll;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              _csr2.Carry=ll & 0xffffffff00000000 ? 1 : 0;
              break;
            case 0b0100:    // ADD
              ll=(signed long long)res1.d + res2.d;
              res3.d=ll;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              _csr2.Carry=ll & 0xffffffff00000000 ? 1 : 0;
              break;
            case 0b0101:    // ADDC
              ll=(signed long long)res1.d + res2.d + _csr.Carry;
              res3.d=ll;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              _csr2.Carry=ll & 0xffffffff00000000 ? 1 : 0;
              break;
            case 0b0110:    // SBC
              ll=(signed long long)res1.d - res2.d + _csr.Carry - 1;
              res3.d=ll;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              _csr2.Carry=ll & 0xffffffff00000000 ? 1 : 0;
              break;
            case 0b0111:    // RSC
              ll=(signed long long)res2.d - res1.d + _csr.Carry - 1;
              res3.d=ll;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              _csr2.Carry=ll & 0xffffffff00000000 ? 1 : 0;
              break;
            case 0b1000:    // TST
#ifdef ARM_V4
              if(Pipe.opcode.type0b.S)
                res3.d=res1.d & res2.d;
              else {
                if(Pipe.opcode.type2.imm==0b000000000000 && Pipe.opcode.type0b.Rd==0b1111) {    // MSR
                  regs[Pipe.opcode.type2.Rd].d=_csr.d;
                  }
                }
#else
              // https://github.com/NationalSecurityAgency/ghidra/issues/654
              res3.d=res1.d & res2.d;
              if(Pipe.opcode.type0.Rd == 15)
                goto upd_csr_26;
#endif
              break;
            case 0b1001:    // TEQ
#ifdef ARM_V4
              if(Pipe.opcode.type0b.S)
                res3.d=res1.d ^ res2.d;
              else {
                if(Pipe.opcode.type0e.Rd==0b1111 && Pipe.opcode.type0e.u2==0b0000 && Pipe.opcode.type0e.u3==0b0000) {    // MSR
                  if(Pipe.opcode.type0e.Rn==0b1001) 
                    _csr.d=regs[Pipe.opcode.type0.Rm].d;
                  else if(Pipe.opcode.type0e.Rn==0b1000) {
                    if(Pipe.opcode.type0.optype & 1)
                      _csr.d=regs[Pipe.opcode.type0.Rm].d;
                    else
                      _csr.d=res2.d;
                    }
                  }
                }
#else
              res3.d=res1.d ^ res2.d;
              if(Pipe.opcode.type0.Rd == 15)
                goto upd_csr_26;
#endif
              break;
            case 0b1010:    // CMP
#ifdef ARM_V4
              if(Pipe.opcode.type0b.S)
                res3.d=res1.d - res2.d;
              else {
                if(Pipe.opcode.type2.imm==0b000000000000 && Pipe.opcode.type0b.Rd==0b1111) {    // MSR
                  regs[Pipe.opcode.type2.Rd].d=_spsr[_csr_mode].d;
                  }
                }
#else
              res3.d=res1.d - res2.d;
              if(Pipe.opcode.type0.Rd == 15)
                goto upd_csr_26;
#endif
              break;
            case 0b1011:    // CMN
#ifdef ARM_V4
              if(Pipe.opcode.type0b.S)
                res3.d=res1.d + res2.d;
              else {
                if(Pipe.opcode.type0e.Rd==0b1111 && Pipe.opcode.type0e.u2==0b0000 && Pipe.opcode.type0e.u3==0b0000) {    // MSR
                  if(Pipe.opcode.type0e.Rn==0b1001) 
                    _spsr[_csr_mode].d=regs[Pipe.opcode.type0.Rm].d;
                  else if(Pipe.opcode.type0e.Rn==0b1000) {
                    if(Pipe.opcode.type0.optype & 1)
                      _spsr[_csr_mode].d=regs[Pipe.opcode.type0.Rm].d;
                    else
                      _spsr[_csr_mode].d=res2.d;
                    }
                  }
                }
#else
              res3.d=res1.d + res2.d;
              if(Pipe.opcode.type0.Rd == 15) {
upd_csr_26:
                if(_csr_mode==MODE_USER)
                  _csr.b = (_csr.b & 0x0f) | (res3.b.u & 0b11110000);
                else {
                  _csr.b = res3.b.u & 0b11111100;
                  _csr_mode = res3.b.l & 0b00000011;
                  }
                }
#endif
              break;
            case 0b1100:    // ORR
              res3.d=res1.d | res2.d;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              break;
            case 0b1101:    // MOV
              res3.d=res2.d;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              break;
            case 0b1110:    // BIC
              res3.d=res1.d & ~res2.d;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              break;
            case 0b1111:    // MVN
              res3.d=~res2.d;
              regs[Pipe.opcode.type0.Rd].d=res3.d;
              break;
            }
          if(Pipe.opcode.type0b.S)
            goto AggFlag;
          break;
          
        case 4:         // load/store multiple reg
          n=regs[Pipe.opcode.type4.Rn].d;
          if(Pipe.opcode.type4.UpDown) {
            // Lowest register number is always transferred to/from lowest memory location accessed.
            for(i=0,j=1; i<16; i++,j<<=1) {
              if(Pipe.opcode.type4.Rlist & j) {
                if(Pipe.opcode.type4.P)
                  n+=4;
                if(Pipe.opcode.type4.L)
                  regs[i].d=GetValue32(n);
                else
                  PutValue32(n,regs[i].d);
                if(!Pipe.opcode.type4.P)
                  n+=4;
                }
              }
            }
          else {
            for(i=15,j=0x8000; i>=0; i--,j>>=1) {
              if(Pipe.opcode.type4.Rlist & j) {
                if(Pipe.opcode.type4.P)
                  n-=4;
                if(Pipe.opcode.type4.L)
                  regs[i].d=GetValue32(n);
                else
                  PutValue32(n,regs[i].d);
                if(!Pipe.opcode.type4.P)
                  n-=4;
                }
              }
            }
          if(Pipe.opcode.type4.W)
            regs[Pipe.opcode.type4.Rn].d = n;
          if(Pipe.opcode.type4.S) {   // qua indica Force PSR ecc!
            if(Pipe.opcode.type4.Rlist & 0x8000) {
              if(Pipe.opcode.type4.L)
                _csr=_spsr[_csr_mode & 0xf /*diciamo*/];
              else ;
              // ALTRIMENTI significa che doveva prendere i registri da User bank e non da quello privileged (se siamo != user)
              }
            else ; // idem...
            }
          break;
          
        case 0b0001111:     // FENCE
          break;
          
        case 0b1110011:     // ECALL, EBREAK, CSRx
          switch(Pipe.opcode.typeA.funct3) {
            case 0b000:     // EBREAK
              if(!Pipe.opcode.typeI.imm) {    // ECALL
                }
              else {        // EBREAK
                }
              break;
            case 0b001:     // CSRRW
              break;
            case 0b010:     // CSRRS
              break;
            case 0b011:     // CSRRC
              break;
            case 0b101:     // CSRRWI
              break;
            case 0b110:     // CSRRSI
              break;
            case 0b111:     // CSRRCI
              break;
            }
          break;
          
        case 6:         // Coprocessor data transfer
          break;
          
        case 7:         // Coprocessor data operation, register transfer o software interrupt
          if(Pipe.opcode.type7.u3) {    // SWI
            DoSWI=1;
            // FINIRE!
            }
          else {
            }
          break;
        }
noAggFlag:
      continue;
      
AggFlag:
          /*When Rd is R15 and the S flag is set the result of the operation is placed in R15 and
the SPSR corresponding to the current mode is moved to the CSR. This allows state
changes which atomically restore both PC and CSR. This form of instruction should
not be used in User mode.*/
      if(Pipe.opcode.type0.Rd == 15) {    // verificare che Rd sia valido; 
        switch(_csr_mode) {
          case MODE_IRQ:
            for(i=13; i<15; i++)    /// VERIFICARE!
              regs[i]=regs_IRQ[i-8];
            break;
          case MODE_FIQ:
            for(i=8; i<15; i++)
              regs[i]=regs_FIQ[i-8];
            break;
          case MODE_SVC:
            for(i=13; i<15; i++)
              regs[i]=regs_Svc[i-8];
            break;
          }
#ifdef ARM_V4
        _csr=_spsr[_csr_mode & 0xf /*diciamo*/];
#else
        _csr=_spsr[_csr_mode];
#endif
        }
      else {
        _csr.Negative=res3.d & 0x80000000 ? 1 : 0;
        _csr.Overflow= ((res1.d & 0x40000000) + (res2.d & 0x40000000)) != ((res1.d & 0x80000000) + (res2.d & 0x80000000));
#warning verificare calcolo V
AggFlagZ:
        _csr.Carry=_csr2.Carry;
        _csr.Zero=!res3.d;
        }

    
		} while(!fExit);
	}


#if 0
main(int argc, char *argv[]) {
	int i,j,k;
	unsigned char _based(rom_seg) *s;

	if(argc >=2) {
		debug=1;
		}
	if((rom_seg=_bheapseg(0x6000)) == _NULLSEG)
		goto fine;
	if((p=_bmalloc(rom_seg,0x6000)) == _NULLOFF)
		goto fine;
	if((ram_seg=_bheapseg(0xffe8)) == _NULLSEG)
		goto fine;
	if((p1=_bmalloc(ram_seg,0xffe8)) == _NULLOFF)
		goto fine;
	_pswmemset(p,0,0x6000);
	_pswmemset(p1,0,0xffe8);
	stack_seg=ram_seg+10;
/*
		for(i=0; i< 8192; i++) {
			printf("Indirizzo %04x, valore %02x\n",s,*s);
			s++;
			}
		*/
		close(i);
		OldTimer=_dos_getvect(0x8);
		OldCtrlC=_dos_getvect(0x23);
		OldKeyb = _dos_getvect( 9 );
		_dos_setvect(0x8,NewTimer);
		_dos_setvect(0x23,NewCtrlC);
		_dos_setvect( 9, NewKeyb );
		_setvideomode(_MRES16COLOR);
		_clearscreen(_GCLEARSCREEN);
		_displaycursor( _GCURSOROFF );
		Emulate();
		_dos_setvect(0x8,OldTimer);
		_dos_setvect(0x23,OldCtrlC);
		_dos_setvect( 9, OldKeyb );
		_displaycursor( _GCURSORON );
		_setvideomode(_DEFAULTMODE);
		}
fine:
	if(p1 != _NULLOFF)
		_bfree(ram_seg,p1);
	else
		puts("no off");
	if(ram_seg != _NULLSEG)
		_bfreeseg(ram_seg);
	else
		puts("no seg");
	if(p != _NULLOFF)
		_bfree(rom_seg,p);
	else
		puts("no off");
	if(rom_seg != _NULLSEG)
		_bfreeseg(rom_seg);
	else
		puts("no seg");
	}
#endif

