#include <cassert>
#include <cstring>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
  REGISTER_B = 0,
  REGISTER_C = 1,
  REGISTER_D = 2,
  REGISTER_E = 3,
  REGISTER_H = 4,
  REGISTER_L = 5,
  REGISTER_iHL = 6,
  REGISTER_F = 6,
  REGISTER_A = 7,
} Register;

typedef enum {
  REGISTER_BC = 0,
  REGISTER_DE = 1,
  REGISTER_HL = 2,
  REGISTER_AF = 3,
} RegisterPair;

#define CYCLES_PER_FRAME 70224

typedef struct {
  int32_t frameTimer;

  uint8_t rom[1024 * 1024 * 8]; // ROM - 8 MB
  uint8_t wram[8192];           // Work RAM - 8 KB
  uint8_t vram[8192];           // Video RAM - 8 KB
  uint8_t hram[127];            // High RAM - 127 bytes

  uint8_t io[256];

  uint8_t oam[160];

  // Order:
  // B, C, D, E, H, L, F, A
  // In reality, the Game Boy CPU encodes registers in order of:
  // B, C, D, E, H, L, [HL], A
  // but [HL] isn't an actual register so we store F in its place
  uint8_t regs[8];
  uint16_t pc;
  uint16_t sp;
  bool ime;

} GB_core_t;

void GB_init(GB_core_t *gb) {
  memset(gb, 0, sizeof(GB_core_t));

  gb->pc = 0x100;
  gb->regs[REGISTER_A] = 0x01;
  gb->regs[REGISTER_F] = 0xB0;
  gb->regs[REGISTER_B] = 0x00;
  gb->regs[REGISTER_C] = 0x13;
  gb->regs[REGISTER_D] = 0x00;
  gb->regs[REGISTER_E] = 0xD8;
  gb->regs[REGISTER_H] = 0x01;
  gb->regs[REGISTER_L] = 0x4D;
  gb->sp = 0xFFFE;
}

uint8_t GB_read_io(GB_core_t *gb, uint16_t addr) {
  printf("IO read addr:%04X\n", addr);

  return 0;
}

void GB_write_io(GB_core_t *gb, uint16_t addr, uint8_t val) {
  printf("IO write addr:%04X val:%02X\n", addr, val);
}

uint8_t GB_read(GB_core_t *gb, uint16_t addr) {
  switch (addr >> 12) {
  case 0x0:
  case 0x1:
  case 0x2:
  case 0x3:
  case 0x4:
  case 0x5:
  case 0x6:
  case 0x7:
    return gb->rom[addr];
  case 0x8:
  case 0x9:
    return gb->vram[addr & 0x1FFF];
  case 0xA: // Cartridge RAM
  case 0xB: // Cartridge RAM
    return 0xFF;
  case 0xC: // WRAM
  case 0xD: // WRAM
  case 0xE: // Echo RAM
    return gb->wram[addr & 0x1FFF];
  case 0xF: // Echo RAM / OAM / IO / HRAM
    if (addr < 0xFE00) {
      return gb->wram[addr & 0x1FFF];
    } else if (addr >= 0xFE00 && addr < 0xFEA0) {
      return gb->oam[addr & 0xFF];
    } else if (addr >= 0xFF00) {
      if (addr >= 0xFF80 && addr < 0xFFFF) {
        return gb->hram[addr - 0xFF80];
      } else {
        return GB_read_io(gb, addr);
      }
    }
  }

  return 0;
}

void GB_write(GB_core_t *gb, uint16_t addr, uint8_t val) {
  switch (addr >> 12) {
  case 0x8:
  case 0x9:
    gb->vram[addr & 0x1FFF] = val;
    break;
  case 0xA: // Cartridge RAM
  case 0xB: // Cartridge RAM
    break;
  case 0xC: // WRAM
  case 0xD: // WRAM
  case 0xE: // Echo RAM
    gb->wram[addr & 0x1FFF] = val;
    break;
  case 0xF: // Echo RAM / OAM / IO / HRAM
    if (addr < 0xFE00) {
      gb->wram[addr & 0x1FFF] = val;
    } else if (addr >= 0xFE00 && addr < 0xFEA0) {
      gb->oam[addr & 0xFF] = val;
    } else if (addr >= 0xFF00) {
      if (addr >= 0xFF80 && addr < 0xFFFF) {
        gb->hram[addr - 0xFF80] = val;
      } else {
        GB_write_io(gb, addr, val);
      }
    }
  }
}

void GBCPU_print_status(GB_core_t *gb, uint16_t pc) {
  printf("AF:%02X%02X BC:%02X%02X DE:%02X%02X HL:%02X%02X PC:%04X SP:%04X\n",
         gb->regs[REGISTER_A], gb->regs[REGISTER_F], gb->regs[REGISTER_B],
         gb->regs[REGISTER_C], gb->regs[REGISTER_D], gb->regs[REGISTER_E],
         gb->regs[REGISTER_H], gb->regs[REGISTER_L], pc, gb->sp);
}

uint16_t GBCPU_get_reg_pair(GB_core_t *gb, RegisterPair pair) {
  switch (pair) {
  case REGISTER_BC:
    return (gb->regs[REGISTER_B] << 8) | gb->regs[REGISTER_C];
  case REGISTER_DE:
    return (gb->regs[REGISTER_D] << 8) | gb->regs[REGISTER_E];
  case REGISTER_HL:
    return (gb->regs[REGISTER_H] << 8) | gb->regs[REGISTER_L];
  case REGISTER_AF:
    return (gb->regs[REGISTER_A] << 8) | gb->regs[REGISTER_F];
  }
}

void GBCPU_set_reg_pair(GB_core_t *gb, RegisterPair pair, uint16_t val) {
  switch (pair) {
  case REGISTER_BC:
    gb->regs[REGISTER_B] = val >> 8;
    gb->regs[REGISTER_C] = val;
    break;
  case REGISTER_DE:
    gb->regs[REGISTER_D] = val >> 8;
    gb->regs[REGISTER_E] = val;
    break;
  case REGISTER_HL:
    gb->regs[REGISTER_H] = val >> 8;
    gb->regs[REGISTER_L] = val;
    break;
  case REGISTER_AF:
    gb->regs[REGISTER_A] = val >> 8;
    gb->regs[REGISTER_F] = val;
    break;
  }
}

uint8_t GBCPU_get_reg(GB_core_t *gb, uint8_t reg) {
  if (reg == REGISTER_iHL) {
    return GB_read(gb, GBCPU_get_reg_pair(gb, REGISTER_HL));
  } else {
    return gb->regs[reg];
  }
}

void GBCPU_set_reg(GB_core_t *gb, uint8_t reg, uint8_t val) {
  if (reg == REGISTER_iHL) {
    GB_write(gb, GBCPU_get_reg_pair(gb, REGISTER_HL), val);
  } else {
    gb->regs[reg] = val;
  }
}

uint8_t GBCPU_next8(GB_core_t *gb) { return GB_read(gb, gb->pc++); }

uint16_t GBCPU_next16(GB_core_t *gb) {
  uint8_t b0 = GB_read(gb, gb->pc++);
  uint8_t b1 = GB_read(gb, gb->pc++);

  return (b1 << 8) | b0;
}

bool GBCPU_getZ(GB_core_t *gb) { return gb->regs[REGISTER_F] & (1 << 7); }

bool GBCPU_getN(GB_core_t *gb) { return gb->regs[REGISTER_F] & (1 << 6); }

bool GBCPU_getH(GB_core_t *gb) { return gb->regs[REGISTER_F] & (1 << 5); }

bool GBCPU_getC(GB_core_t *gb) { return gb->regs[REGISTER_F] & (1 << 4); }

void GBCPU_setZ(GB_core_t *gb, bool val) {
  gb->regs[REGISTER_F] &= ~(1 << 7);
  gb->regs[REGISTER_F] |= val << 7;
}

void GBCPU_setN(GB_core_t *gb, bool val) {
  gb->regs[REGISTER_F] &= ~(1 << 6);
  gb->regs[REGISTER_F] |= val << 6;
}

void GBCPU_setH(GB_core_t *gb, bool val) {
  gb->regs[REGISTER_F] &= ~(1 << 5);
  gb->regs[REGISTER_F] |= val << 5;
}

void GBCPU_setC(GB_core_t *gb, bool val) {
  gb->regs[REGISTER_F] &= ~(1 << 4);
  gb->regs[REGISTER_F] |= val << 4;
}

bool GBCPU_get_cond(GB_core_t *gb, uint8_t cond) {
  bool flag = cond & 0b10 ? GBCPU_getC(gb) : GBCPU_getZ(gb);
  bool flagShouldBeSet = cond & 0b01;

  return flag == flagShouldBeSet;
}

uint8_t GBCPU_execute(GB_core_t *gb) {
  uint8_t opcode = GB_read(gb, gb->pc++);

  switch (opcode) {

  // NOP
  case 0x00:
    break;

  // DI
  case 0xF3:
    gb->ime = false;
    break;

  // JP u16
  case 0xC3:
    gb->pc = GBCPU_next16(gb);
    break;

  // LD (FF00+u8), A
  case 0xE0:
    GB_write(gb, 0xFF00 | GBCPU_next8(gb), gb->regs[REGISTER_A]);
    break;
  // LD A, (FF00+u8)
  case 0xF0:
    gb->regs[REGISTER_A] = GB_read(gb, 0xFF00 | GBCPU_next8(gb));
    break;

  // LD r16, u16
  case 0x01:
  case 0x11:
  case 0x21:
    GBCPU_set_reg_pair(gb, (RegisterPair)((opcode >> 4) & 0b11),
                       GBCPU_next16(gb));
    break;
  case 0x31:
    gb->sp = GBCPU_next16(gb);
    break;

  // LD [HL+], A
  case 0x22: {
    uint16_t pre = GBCPU_get_reg_pair(gb, REGISTER_HL);
    GB_write(gb, pre, gb->regs[REGISTER_A]);
    GBCPU_set_reg_pair(gb, REGISTER_HL, pre + 1);
    break;
  }

  // LD [HL-], A
  case 0x32: {
    uint16_t pre = GBCPU_get_reg_pair(gb, REGISTER_HL);
    GB_write(gb, pre, gb->regs[REGISTER_A]);
    GBCPU_set_reg_pair(gb, REGISTER_HL, pre - 1);
    break;
  }

  // JR CC, i8
  case 0x20:
  case 0x30:
  case 0x28:
  case 0x38: {
    uint8_t offset = GBCPU_next8(gb);
    if (GBCPU_get_cond(gb, (opcode >> 3) & 0b11)) {
      gb->pc += (int8_t)offset;
    }
    break;
  }

  // DEC r8
  case 0x05:
  case 0x15:
  case 0x25:
  case 0x35:
  case 0x0D:
  case 0x1D:
  case 0x2D:
  case 0x3D: {
    uint8_t old = GBCPU_get_reg(gb, (opcode >> 3) & 0b111);
    uint8_t res = old - 1;

    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, res);

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, true);
    GBCPU_setH(gb, 1 > (old & 0xF));
    break;
  }

  // LD r8, u8
  case 0x06:
  case 0x16:
  case 0x26:
  case 0x36:
  case 0x0E:
  case 0x1E:
  case 0x2E:
  case 0x3E:
    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, GBCPU_next8(gb));
    break;

  // CP A, u8
  case 0xFE: {
    uint8_t op = GBCPU_next8(gb);
    uint8_t res = gb->regs[REGISTER_A] - op;

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, true);
    GBCPU_setH(gb, (gb->regs[REGISTER_A] & 0xF) < (op & 0xF));
    GBCPU_setC(gb, op > gb->regs[REGISTER_A]);
    break;
  }

  // XOR r8
  case 0xA8:
  case 0xA9:
  case 0xAA:
  case 0xAB:
  case 0xAC:
  case 0xAD:
  case 0xAE:
  case 0xAF: {
    uint8_t op = GBCPU_get_reg(gb, opcode & 0b111);
    uint8_t res = gb->regs[REGISTER_A] ^ op;

    gb->regs[REGISTER_A] = res;

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, false);
    GBCPU_setC(gb, false);
    break;
  }

  default:
    printf("Unknown opcode: %02X @ PC:%04X\n", opcode, gb->pc - 1);
    GBCPU_print_status(gb, gb->pc - 1);
    assert(0);
    break;
  }

  return 4;
}

void GB_run_to_next_frame(GB_core_t *gb) {
  // Since we don't know how long the next instruction will take,
  // we use a persistent timer in case an instruction overshoots
  // the alloted time of 70224 cycles per frame
  gb->frameTimer += CYCLES_PER_FRAME;

  while (gb->frameTimer > 0) {
    gb->frameTimer -= GBCPU_execute(gb);
  }
}