#include <cassert>
#include <cstring>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
  GBCPU_REG_B = 0,
  GBCPU_REG_C = 1,
  GBCPU_REG_D = 2,
  GBCPU_REG_E = 3,
  GBCPU_REG_H = 4,
  GBCPU_REG_L = 5,
  GBCPU_REG_iHL = 6,
  GBCPU_REG_F = 6,
  GBCPU_REG_A = 7,
} Register;

typedef enum {
  GBCPU_REG_BC = 0,
  GBCPU_REG_DE = 1,
  GBCPU_REG_HL = 2,
  GBCPU_REG_AF = 3,
  GBCPU_REG_SP = 3,
} RegisterPair;

typedef enum { GB_IO_LCDC = 0x40, GB_IO_STAT = 0x41, GB_IO_LY = 0x44 } GbIo;

typedef enum {
  GBPPU_MODE_HBLANK = 0,
  GBPPU_MODE_VBLANK = 1,
  GBPPU_MODE_OAM = 2,
  GBPPU_MODE_DRAWING = 3
} GbPpuMode;

typedef enum { GB_IO_LCDC_ENABLE = 1 << 7 } GbIoLcdcBits;

#define CYCLES_PER_FRAME 70224

typedef struct {
  int32_t frameTimer;

  uint8_t rom[1024 * 1024 * 8]; // ROM - 8 MB
  uint8_t wram[8192];           // Work RAM - 8 KB
  uint8_t vram[8192];           // Video RAM - 8 KB
  uint8_t hram[127];            // High RAM - 127 bytes

  uint8_t io[256];

  uint8_t oam[160];

  // CPU

  uint8_t currentInstrCycles;

  // Order:
  // B, C, D, E, H, L, F, A
  // In reality, the Game Boy CPU encodes registers in order of:
  // B, C, D, E, H, L, [HL], A
  // but [HL] isn't an actual register so we store F in its place
  uint8_t regs[8];
  uint16_t pc;
  uint16_t sp;
  bool ime;

  // PPU
  int32_t modeTimer;

} GB_core_t;

uint8_t GBPPU_get_mode(GB_core_t *gb) { return gb->io[GB_IO_STAT] & 0b11; }
void GBPPU_set_mode(GB_core_t *gb, GbPpuMode mode) {
  gb->io[GB_IO_STAT] &= ~0b11;
  gb->io[GB_IO_STAT] |= mode;
}

void GBPPU_write_lcdc(GB_core_t *gb, uint8_t val) {
  // bit 7 off to on
  if (val & GB_IO_LCDC_ENABLE && !(gb->io[GB_IO_LCDC] & GB_IO_LCDC_ENABLE)) {
    printf("PPU enable\n");
    GBPPU_set_mode(gb, GBPPU_MODE_OAM);
  }
  // bit 7 on to off
  if (val & GB_IO_LCDC_ENABLE && !(gb->io[GB_IO_LCDC] & GB_IO_LCDC_ENABLE)) {
    printf("PPU disable\n");
    GBPPU_set_mode(gb, GBPPU_MODE_OAM);
  }

  gb->io[GB_IO_LCDC] = val;
}

void GBPPU_step(GB_core_t *gb, uint8_t cycles) {
  gb->modeTimer += cycles;

  if (gb->io[GB_IO_LCDC] & GB_IO_LCDC_ENABLE) {
    switch (GBPPU_get_mode(gb)) {
    case GBPPU_MODE_OAM:
      if (gb->modeTimer >= 80) {
        gb->modeTimer -= 80;

        GBPPU_set_mode(gb, GBPPU_MODE_DRAWING);
      }
      break;
    case GBPPU_MODE_DRAWING:
      if (gb->modeTimer >= 172) {
        gb->modeTimer -= 172;

        GBPPU_set_mode(gb, GBPPU_MODE_HBLANK);
      }
    case GBPPU_MODE_HBLANK:
      if (gb->modeTimer >= 204) {
        gb->modeTimer -= 204;

        gb->io[GB_IO_LY]++;

        if (gb->io[GB_IO_LY] == 144) {
          GBPPU_set_mode(gb, GBPPU_MODE_VBLANK);
        } else {
          GBPPU_set_mode(gb, GBPPU_MODE_OAM);
        }
      }
      break;
    case GBPPU_MODE_VBLANK:
      if (gb->modeTimer >= 456) {
        gb->modeTimer -= 456;

        gb->io[GB_IO_LY]++;

        if (gb->io[GB_IO_LY] == 154) {
          gb->io[GB_IO_LY] = 0;
          GBPPU_set_mode(gb, GBPPU_MODE_OAM);
        }
      }
    }
  }
}

uint8_t GB_read_io(GB_core_t *gb, uint16_t addr) {
  switch (addr & 0xFF) {
  case GB_IO_LY:
    return gb->io[addr & 0xFF];
  }

  printf("unhandled IO read addr:%04X\n", addr);

  return 0xFF;
}

void GB_write_io(GB_core_t *gb, uint16_t addr, uint8_t val) {
  switch (addr & 0xFF) {
  case GB_IO_LCDC:
    GBPPU_write_lcdc(gb, val);
    return;
  }

  printf("unhandled IO write addr:%04X val:%02X\n", addr, val);
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
         gb->regs[GBCPU_REG_A], gb->regs[GBCPU_REG_F], gb->regs[GBCPU_REG_B],
         gb->regs[GBCPU_REG_C], gb->regs[GBCPU_REG_D], gb->regs[GBCPU_REG_E],
         gb->regs[GBCPU_REG_H], gb->regs[GBCPU_REG_L], pc, gb->sp);
}

uint16_t GBCPU_get_reg_pair(GB_core_t *gb, uint8_t pair) {
  switch (pair) {
  case GBCPU_REG_BC:
    return (gb->regs[GBCPU_REG_B] << 8) | gb->regs[GBCPU_REG_C];
  case GBCPU_REG_DE:
    return (gb->regs[GBCPU_REG_D] << 8) | gb->regs[GBCPU_REG_E];
  case GBCPU_REG_HL:
    return (gb->regs[GBCPU_REG_H] << 8) | gb->regs[GBCPU_REG_L];
  case GBCPU_REG_AF:
    return (gb->regs[GBCPU_REG_A] << 8) | gb->regs[GBCPU_REG_F];
  }

  return 0;
}

void GBCPU_set_reg_pair(GB_core_t *gb, uint8_t pair, uint16_t val) {
  switch (pair) {
  case GBCPU_REG_BC:
    gb->regs[GBCPU_REG_B] = val >> 8;
    gb->regs[GBCPU_REG_C] = val;
    break;
  case GBCPU_REG_DE:
    gb->regs[GBCPU_REG_D] = val >> 8;
    gb->regs[GBCPU_REG_E] = val;
    break;
  case GBCPU_REG_HL:
    gb->regs[GBCPU_REG_H] = val >> 8;
    gb->regs[GBCPU_REG_L] = val;
    break;
  case GBCPU_REG_AF:
    gb->regs[GBCPU_REG_A] = val >> 8;
    gb->regs[GBCPU_REG_F] = val;
    break;
  }
}

uint16_t GBCPU_get_reg_pair2(GB_core_t *gb, uint8_t pair) {
  if (pair == GBCPU_REG_SP) {
    return gb->sp;
  } else {
    return GBCPU_get_reg_pair(gb, pair);
  }
}

void GBCPU_set_reg_pair2(GB_core_t *gb, uint8_t pair, uint16_t val) {
  if (pair == GBCPU_REG_SP) {
    gb->sp = val;
  } else {
    GBCPU_set_reg_pair(gb, pair, val);
  }
}

uint8_t GBCPU_get_reg(GB_core_t *gb, uint8_t reg) {
  if (reg == GBCPU_REG_iHL) {
    return GB_read(gb, GBCPU_get_reg_pair(gb, GBCPU_REG_HL));
  } else {
    return gb->regs[reg];
  }
}

void GBCPU_set_reg(GB_core_t *gb, uint8_t reg, uint8_t val) {
  if (reg == GBCPU_REG_iHL) {
    GB_write(gb, GBCPU_get_reg_pair(gb, GBCPU_REG_HL), val);
  } else {
    gb->regs[reg] = val;
  }
}

uint8_t GBCPU_next8(GB_core_t *gb) {
  gb->currentInstrCycles += 4;

  return GB_read(gb, gb->pc++);
}

uint16_t GBCPU_next16(GB_core_t *gb) {
  gb->currentInstrCycles += 8;

  uint8_t b0 = GB_read(gb, gb->pc++);
  uint8_t b1 = GB_read(gb, gb->pc++);

  return (b1 << 8) | b0;
}

bool GBCPU_getZ(GB_core_t *gb) { return gb->regs[GBCPU_REG_F] & (1 << 7); }

bool GBCPU_getN(GB_core_t *gb) { return gb->regs[GBCPU_REG_F] & (1 << 6); }

bool GBCPU_getH(GB_core_t *gb) { return gb->regs[GBCPU_REG_F] & (1 << 5); }

bool GBCPU_getC(GB_core_t *gb) { return gb->regs[GBCPU_REG_F] & (1 << 4); }

void GBCPU_setZ(GB_core_t *gb, bool val) {
  gb->regs[GBCPU_REG_F] &= ~(1 << 7);
  gb->regs[GBCPU_REG_F] |= val << 7;
}

void GBCPU_setN(GB_core_t *gb, bool val) {
  gb->regs[GBCPU_REG_F] &= ~(1 << 6);
  gb->regs[GBCPU_REG_F] |= val << 6;
}

void GBCPU_setH(GB_core_t *gb, bool val) {
  gb->regs[GBCPU_REG_F] &= ~(1 << 5);
  gb->regs[GBCPU_REG_F] |= val << 5;
}

void GBCPU_setC(GB_core_t *gb, bool val) {
  gb->regs[GBCPU_REG_F] &= ~(1 << 4);
  gb->regs[GBCPU_REG_F] |= val << 4;
}

bool GBCPU_get_cond(GB_core_t *gb, uint8_t cond) {
  bool flag = cond & 0b10 ? GBCPU_getC(gb) : GBCPU_getZ(gb);
  bool flagShouldBeSet = cond & 0b01;

  return flag == flagShouldBeSet;
}

uint8_t GBCPU_read(GB_core_t *gb, uint16_t addr) {
  gb->currentInstrCycles += 4;
  return GB_read(gb, addr);
}

void GBCPU_write(GB_core_t *gb, uint16_t addr, uint8_t val) {
  gb->currentInstrCycles += 4;
  GB_write(gb, addr, val);
}

void GBCPU_push(GB_core_t *gb, uint16_t val) {
  GBCPU_write(gb, --gb->sp, val >> 8);
  GBCPU_write(gb, --gb->sp, val);
}

uint16_t GBCPU_pop(GB_core_t *gb) {
  uint8_t lower = GBCPU_read(gb, gb->sp++);
  uint8_t upper = GBCPU_read(gb, gb->sp++);

  return (upper << 8) | lower;
}

uint8_t GBCPU_execute(GB_core_t *gb) {
  gb->currentInstrCycles = 0;

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
    GB_write(gb, 0xFF00 | GBCPU_next8(gb), gb->regs[GBCPU_REG_A]);
    break;
  // LD A, (FF00+u8)
  case 0xF0:
    gb->regs[GBCPU_REG_A] = GB_read(gb, 0xFF00 | GBCPU_next8(gb));
    break;

  // LD (FF00+C), A
  case 0xE2:
    GB_write(gb, 0xFF00 | gb->regs[GBCPU_REG_C], gb->regs[GBCPU_REG_A]);
    break;
  // LD A, (FF00+C)
  case 0xF2:
    gb->regs[GBCPU_REG_A] = GB_read(gb, 0xFF00 | gb->regs[GBCPU_REG_C]);
    break;

  // LD (u16), A
  case 0xEA:
    GB_write(gb, GBCPU_next16(gb), gb->regs[GBCPU_REG_A]);
    break;
  // LD A, (u16)
  case 0xFA:
    gb->regs[GBCPU_REG_A] = GB_read(gb, GBCPU_next16(gb));
    break;

  // LD r16, u16
  case 0x01:
  case 0x11:
  case 0x21:
  case 0x31:
    GBCPU_set_reg_pair2(gb, (RegisterPair)((opcode >> 4) & 0b11),
                       GBCPU_next16(gb));
    break;

  // LD [HL+], A
  case 0x22: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    GB_write(gb, pre, gb->regs[GBCPU_REG_A]);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre + 1);
    break;
  }
  // LD [HL-], A
  case 0x32: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    GB_write(gb, pre, gb->regs[GBCPU_REG_A]);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre - 1);
    break;
  }

  // LD A, [HL+]
  case 0x2A: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    gb->regs[GBCPU_REG_A] = GB_read(gb, pre);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre + 1);
    break;
  }
  // LD A, [HL-]
  case 0x3A: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    gb->regs[GBCPU_REG_A] = GB_read(gb, pre);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre - 1);
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

  // CALL
  case 0xCD: {
    uint16_t target = GBCPU_next16(gb);
    GBCPU_push(gb, gb->pc);
    gb->pc = target;
    break;
  }

  // INC r8
  case 0x04:
  case 0x14:
  case 0x24:
  case 0x34:
  case 0x0C:
  case 0x1C:
  case 0x2C:
  case 0x3C: {
    uint8_t old = GBCPU_get_reg(gb, (opcode >> 3) & 0b111);
    uint8_t res = old + 1;

    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, res);

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, (old & 0xF) >= 0xF);
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

  // DEC r16
  case 0x0B:
  case 0x1B:
  case 0x2B:
  case 0x3B:
    GBCPU_set_reg_pair2(gb, (opcode >> 4) & 0b11, GBCPU_get_reg_pair2(gb, (opcode >> 4) & 0b11) - 1);
    break;

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
    uint8_t res = gb->regs[GBCPU_REG_A] - op;

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, true);
    GBCPU_setH(gb, (gb->regs[GBCPU_REG_A] & 0xF) < (op & 0xF));
    GBCPU_setC(gb, op > gb->regs[GBCPU_REG_A]);
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
    uint8_t res = gb->regs[GBCPU_REG_A] ^ op;

    gb->regs[GBCPU_REG_A] = res;

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

  return gb->currentInstrCycles;
}

void GB_run_to_next_frame(GB_core_t *gb) {
  // Since we don't know how long the next instruction will take,
  // we use a persistent timer in case an instruction overshoots
  // the alloted time of 70224 cycles per frame
  gb->frameTimer += CYCLES_PER_FRAME;

  while (gb->frameTimer > 0) {
    uint8_t cycles = GBCPU_execute(gb);

    gb->frameTimer -= cycles;

    GBPPU_step(gb, cycles);
  }
}

void GB_init(GB_core_t *gb) {
  memset(gb, 0, sizeof(GB_core_t));

  gb->pc = 0x100;
  gb->regs[GBCPU_REG_A] = 0x01;
  gb->regs[GBCPU_REG_F] = 0xB0;
  gb->regs[GBCPU_REG_B] = 0x00;
  gb->regs[GBCPU_REG_C] = 0x13;
  gb->regs[GBCPU_REG_D] = 0x00;
  gb->regs[GBCPU_REG_E] = 0xD8;
  gb->regs[GBCPU_REG_H] = 0x01;
  gb->regs[GBCPU_REG_L] = 0x4D;
  gb->sp = 0xFFFE;

  GBPPU_write_lcdc(gb, 0x91);
}
