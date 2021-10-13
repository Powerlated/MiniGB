#include <cassert>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 144

#define CYCLES_PER_FRAME 70224

#define BIT(n) (0x1U << (n))

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

typedef enum {
  GB_IO_JOYP = 0x00,
  GB_IO_DIV = 0x04,
  GB_IO_IF = 0x0F,
  GB_IO_LCDC = 0x40,
  GB_IO_STAT = 0x41,
  GB_IO_SCY = 0x42,
  GB_IO_SCX = 0x43,
  GB_IO_LY = 0x44,
  GB_IO_DMA = 0x46,
  GB_IO_BGP = 0x47,
  GB_IO_OBP0 = 0x48,
  GB_IO_OBP1 = 0x49,
  GB_IO_IE = 0xFF,
} GbIo;

typedef enum {
  GBPPU_MODE_HBLANK = 0,
  GBPPU_MODE_VBLANK = 1,
  GBPPU_MODE_OAM = 2,
  GBPPU_MODE_DRAWING = 3
} GbPpuMode;

typedef enum {
  GB_INTR_VBLANK = 0,
  GB_INTR_STAT = 1,
  GB_INTR_TIMER = 2,
  GB_INTR_SERIAL = 3,
  GB_INTR_JOYPAD = 4,
} GbInterrupt;

typedef struct {
  int32_t frame_timer;

  uint8_t rom[32768]; // ROM - 32 KB (Tetris is enough)
  uint8_t wram[8192]; // Work RAM - 8 KB
  uint8_t vram[8192]; // Video RAM - 8 KB
  uint8_t hram[127];  // High RAM - 127 bytes

  uint8_t io[256];

  uint8_t oam[160];

  // CPU
  uint8_t current_instr_cycles;

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
  uint8_t current_screen_buffer;
  uint32_t screen_buffer_0[SCREEN_WIDTH * SCREEN_HEIGHT];
  uint32_t screen_buffer_1[SCREEN_WIDTH * SCREEN_HEIGHT];
  int32_t mode_timer;
  int32_t mode_timer_target;

  // Joypad
  bool button_start;
  bool button_select;
  bool button_b;
  bool button_a;
  bool button_down;
  bool button_up;
  bool button_left;
  bool button_right;

  // Timer
  uint16_t internal_timer;
} GB_core_t;

void GB_flag_interrupt(GB_core_t *gb, GbInterrupt interrupt) {
  gb->io[GB_IO_IF] |= 1 << interrupt;
}

uint32_t *GBPPU_get_display_screen_buffer(GB_core_t *gb) {
  return gb->current_screen_buffer ? gb->screen_buffer_0 : gb->screen_buffer_1;
}

uint32_t *GBPPU_get_internal_screen_buffer(GB_core_t *gb) {
  return gb->current_screen_buffer ? gb->screen_buffer_1 : gb->screen_buffer_0;
}

void GBPPU_swap_buffers(GB_core_t *gb) { gb->current_screen_buffer ^= 1; }

uint8_t GBPPU_get_mode(GB_core_t *gb) { return gb->io[GB_IO_STAT] & 0b11; }
void GBPPU_set_mode(GB_core_t *gb, GbPpuMode mode) {
  gb->io[GB_IO_STAT] &= ~0b11;
  gb->io[GB_IO_STAT] |= mode;
}

void GB_write_io_lcdc(GB_core_t *gb, uint8_t val) {
  // bit 7 off to on
  if (val & BIT(7) && !(gb->io[GB_IO_LCDC] & BIT(7))) {
    // printf("PPU enable\n");
    gb->mode_timer_target = 80;
    GBPPU_set_mode(gb, GBPPU_MODE_OAM);
  }
  // bit 7 on to off
  if (!(val & BIT(7)) && (gb->io[GB_IO_LCDC] & BIT(7))) {
    // printf("PPU disable\n");
    gb->mode_timer = 0;
    GBPPU_set_mode(gb, GBPPU_MODE_HBLANK);

    gb->io[GB_IO_LY] = 0;
  }

  gb->io[GB_IO_LCDC] = val;
}

void GBPPU_render_scanline(GB_core_t *gb) {
  uint32_t *screen = GBPPU_get_internal_screen_buffer(gb);
  if (gb->io[GB_IO_LCDC] & BIT(0)) {
    uint32_t screen_base = gb->io[GB_IO_LY] * 160;

    uint8_t tile_y = gb->io[GB_IO_LY] >> 3;
    uint8_t fine_y = gb->io[GB_IO_LY] & 0b111;

    uint16_t map_base = gb->io[GB_IO_LCDC] & BIT(3) ? 0x1C00 : 0x1800;

    map_base += tile_y * 32;

    for (uint32_t i = 0; i < 20; i++) {
      uint8_t tile_id = gb->vram[map_base];
      uint16_t tile_addr;

      if (gb->io[GB_IO_LCDC] & BIT(4)) {
        tile_addr = 0x0000 + tile_id * 16;
      } else {
        tile_addr = 0x1000 + (int8_t)tile_id * 16;
      }

      uint8_t b0 = gb->vram[tile_addr + fine_y * 2];
      uint8_t b1 = gb->vram[tile_addr + fine_y * 2 + 1];

      for (uint32_t j = 0; j < 8; j++) {
        uint8_t color_raw = ((b1 >> 6) & 0b10) | ((b0 >> 7) & 1);
        uint8_t color = (gb->io[GB_IO_BGP] >> color_raw * 2) & 0b11;

        screen[screen_base++] = (0x00555555 * (3 - color)) | 0xFF000000;

        b0 <<= 1;
        b1 <<= 1;
      }

      map_base++;
    }
  }

  if (gb->io[GB_IO_LCDC] & BIT(1)) {

    for (uint32_t i = 0; i < 160; i += 4) {
      uint8_t y = gb->oam[i + 0];
      uint8_t x = gb->oam[i + 1];
      uint8_t tile = gb->oam[i + 2];
      uint8_t attr = gb->oam[i + 3];

      int16_t screen_y = (int16_t)y - 16;
      int16_t screen_x = (int16_t)x - 8;

      if (gb->io[GB_IO_LY] >= screen_y && gb->io[GB_IO_LY] < screen_y + 8) {
        uint32_t screen_base = gb->io[GB_IO_LY] * 160 + screen_x;

        uint8_t palette = gb->io[attr & BIT(4) ? GB_IO_OBP1 : GB_IO_OBP0];
        uint16_t tile_addr = tile * 16;
        uint8_t fine_y = (gb->io[GB_IO_LY] - screen_y) & 7;

        uint8_t b0 = gb->vram[tile_addr + fine_y * 2];
        uint8_t b1 = gb->vram[tile_addr + fine_y * 2 + 1];

        for (uint32_t j = 0; j < 8; j++) {
          uint8_t color_raw = ((b1 >> 6) & 0b10) | ((b0 >> 7) & 1);
          uint8_t color = (palette >> color_raw * 2) & 0b11;

          if (screen_x >= 0 && screen_x < 160 && color_raw != 0) {
            screen[screen_base] = (0x00555555 * (3 - color_raw)) | 0xFF000000;
          }

          b0 <<= 1;
          b1 <<= 1;

          screen_x++;
          screen_base++;
        }
      }
    }
  }
}

void GB_write_io_if(GB_core_t *gb, uint8_t val) {
  gb->io[GB_IO_IF] = 0b11100000 | val;
}

uint8_t GB_read_io_joyp(GB_core_t *gb) {
  uint8_t val = gb->io[GB_IO_JOYP] | 0b11001111;

  if (!(gb->io[GB_IO_JOYP] & BIT(5))) {
    if (gb->button_start)
      val &= ~BIT(3);
    if (gb->button_select)
      val &= ~BIT(2);
    if (gb->button_b)
      val &= ~BIT(1);
    if (gb->button_a)
      val &= ~BIT(0);
  }
  if (!(gb->io[GB_IO_JOYP] & BIT(4))) {
    if (gb->button_down)
      val &= ~BIT(3);
    if (gb->button_up)
      val &= ~BIT(2);
    if (gb->button_left)
      val &= ~BIT(1);
    if (gb->button_right)
      val &= ~BIT(0);
  }

  return val;
}

void GB_timer_tick(GB_core_t *gb, uint8_t cycles) {
  gb->internal_timer += cycles;
}

void GB_ppu_tick(GB_core_t *gb, uint8_t cycles) {
  if (gb->io[GB_IO_LCDC] & BIT(7)) {
    gb->mode_timer += cycles;

    if (gb->mode_timer >= gb->mode_timer_target) {
      gb->mode_timer -= gb->mode_timer_target;

      switch (GBPPU_get_mode(gb)) {
      case GBPPU_MODE_OAM:
        gb->mode_timer_target = 172;
        GBPPU_set_mode(gb, GBPPU_MODE_DRAWING);
        break;
      case GBPPU_MODE_DRAWING:
        gb->mode_timer_target = 204;
        GBPPU_render_scanline(gb);
        GBPPU_set_mode(gb, GBPPU_MODE_HBLANK);
        break;
      case GBPPU_MODE_HBLANK:
        gb->io[GB_IO_LY]++;

        if (gb->io[GB_IO_LY] == 144) {
          GBPPU_swap_buffers(gb);
          GB_flag_interrupt(gb, GB_INTR_VBLANK);

          gb->mode_timer_target = 456;
          GBPPU_set_mode(gb, GBPPU_MODE_VBLANK);
        } else {
          gb->mode_timer_target = 80;
          GBPPU_set_mode(gb, GBPPU_MODE_OAM);
        }
        break;
      case GBPPU_MODE_VBLANK:
        gb->io[GB_IO_LY]++;

        if (gb->io[GB_IO_LY] == 154) {
          gb->io[GB_IO_LY] = 0;
          gb->mode_timer_target = 80;
          GBPPU_set_mode(gb, GBPPU_MODE_OAM);
        }
      }
    }
  }
}

uint8_t GB_read_io(GB_core_t *gb, uint16_t addr) {
  switch (addr & 0xFF) {
  case GB_IO_JOYP:
    return GB_read_io_joyp(gb);
  case GB_IO_DIV:
    return gb->internal_timer >> 8;
  case GB_IO_LY:
  case GB_IO_IF:
  case GB_IO_STAT:
  case GB_IO_SCY:
  case GB_IO_SCX:
  case GB_IO_BGP:
  case GB_IO_OBP0:
  case GB_IO_OBP1:
  case GB_IO_IE:
    return gb->io[addr & 0xFF];
  }

  // printf("unhandled IO read addr:%04X\n", addr);

  return 0xFF;
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

void GB_write_io(GB_core_t *gb, uint16_t addr, uint8_t val) {
  switch (addr & 0xFF) {
  case GB_IO_IF:
    GB_write_io_if(gb, val);
    return;
  case GB_IO_LCDC:
    GB_write_io_lcdc(gb, val);
    return;
  case GB_IO_DMA:
    for (uint32_t i = 0; i < 160; i++) {
      gb->oam[i] = GB_read(gb, (val << 8) + i);
    }
    return;
  case GB_IO_JOYP:
  case GB_IO_SCY:
  case GB_IO_SCX:
  case GB_IO_BGP:
  case GB_IO_OBP0:
  case GB_IO_OBP1:
  case GB_IO_IE:
    gb->io[addr & 0xFF] = val;
    return;
  }

  // printf("unhandled IO write addr:%04X val:%02X\n", addr, val);
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
  // printf("AF:%02X%02X BC:%02X%02X DE:%02X%02X HL:%02X%02X PC:%04X SP:%04X\n",
  //        gb->regs[GBCPU_REG_A], gb->regs[GBCPU_REG_F], gb->regs[GBCPU_REG_B],
  //        gb->regs[GBCPU_REG_C], gb->regs[GBCPU_REG_D], gb->regs[GBCPU_REG_E],
  //        gb->regs[GBCPU_REG_H], gb->regs[GBCPU_REG_L], pc, gb->sp);

  printf("A: %02X F: %02X B: %02X C: %02X D: %02X E: %02X H: %02X L: %02X "
         "SP:%04X PC: 00:%04X (%02X %02X %02X)\n",
         gb->regs[GBCPU_REG_A], gb->regs[GBCPU_REG_F], gb->regs[GBCPU_REG_B],
         gb->regs[GBCPU_REG_C], gb->regs[GBCPU_REG_D], gb->regs[GBCPU_REG_E],
         gb->regs[GBCPU_REG_H], gb->regs[GBCPU_REG_L], gb->sp, pc,
         GB_read(gb, pc), GB_read(gb, pc + 1), GB_read(gb, pc + 2));
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
    gb->regs[GBCPU_REG_F] = val & 0xF0;
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
  gb->current_instr_cycles += 4;

  return GB_read(gb, gb->pc++);
}

uint16_t GBCPU_next16(GB_core_t *gb) {
  gb->current_instr_cycles += 8;

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

void GBCPU_stall_cycles(GB_core_t *gb, int32_t cycles) {
  gb->current_instr_cycles += cycles;
}

uint8_t GBCPU_read(GB_core_t *gb, uint16_t addr) {
  gb->current_instr_cycles += 4;
  return GB_read(gb, addr);
}

void GBCPU_write(GB_core_t *gb, uint16_t addr, uint8_t val) {
  gb->current_instr_cycles += 4;
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

// ALU ops
void GBCPU_instr_add(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] + op;

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, false);
  GBCPU_setH(gb, (gb->regs[GBCPU_REG_A] & 0xF) + (op & 0xF) > 0xF);
  GBCPU_setC(gb, (uint32_t)gb->regs[GBCPU_REG_A] + op > 0xFF);

  gb->regs[GBCPU_REG_A] = res;
}

void GBCPU_instr_adc(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] + op + GBCPU_getC(gb);

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, false);
  GBCPU_setH(gb,
             (gb->regs[GBCPU_REG_A] & 0xF) + (op & 0xF) + GBCPU_getC(gb) > 0xF);
  GBCPU_setC(gb, (uint32_t)gb->regs[GBCPU_REG_A] + op + GBCPU_getC(gb) > 0xFF);

  gb->regs[GBCPU_REG_A] = res;
}

void GBCPU_instr_sub(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] - op;

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, true);
  GBCPU_setH(gb, (gb->regs[GBCPU_REG_A] & 0xF) < (op & 0xF));
  GBCPU_setC(gb, op > gb->regs[GBCPU_REG_A]);

  gb->regs[GBCPU_REG_A] = res;
}

void GBCPU_instr_sbc(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] - op - GBCPU_getC(gb);

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, true);
  GBCPU_setH(gb, (gb->regs[GBCPU_REG_A] & 0xF) < (op & 0xF) + GBCPU_getC(gb));
  GBCPU_setC(gb, op > gb->regs[GBCPU_REG_A] - GBCPU_getC(gb));

  gb->regs[GBCPU_REG_A] = res;
}

void GBCPU_instr_and(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] & op;

  gb->regs[GBCPU_REG_A] = res;

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, false);
  GBCPU_setH(gb, true);
  GBCPU_setC(gb, false);
}

void GBCPU_instr_xor(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] ^ op;

  gb->regs[GBCPU_REG_A] = res;

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, false);
  GBCPU_setH(gb, false);
  GBCPU_setC(gb, false);
}

void GBCPU_instr_or(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] | op;

  gb->regs[GBCPU_REG_A] = res;

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, false);
  GBCPU_setH(gb, false);
  GBCPU_setC(gb, false);
}

void GBCPU_instr_cp(GB_core_t *gb, uint8_t op) {
  uint8_t res = gb->regs[GBCPU_REG_A] - op;

  GBCPU_setZ(gb, res == 0);
  GBCPU_setN(gb, true);
  GBCPU_setH(gb, (gb->regs[GBCPU_REG_A] & 0xF) < (op & 0xF));
  GBCPU_setC(gb, op > gb->regs[GBCPU_REG_A]);
}

uint8_t GBCPU_execute(GB_core_t *gb) {
  gb->current_instr_cycles = 0;

  // GBCPU_print_status(gb, gb->pc);

  uint8_t flagged_enabled = gb->io[GB_IO_IE] & gb->io[GB_IO_IF] & 0x1F;
  if (gb->ime && flagged_enabled != 0) {
    GBCPU_push(gb, gb->pc);
    uint8_t intr = __builtin_ctz(flagged_enabled);
    // printf("Interrupt dispatch: %d", intr);
    gb->io[GB_IO_IF] &= ~(1 << intr);
    gb->io[GB_IO_IF] |= 0b11100000;
    gb->pc = 0x40 + intr * 8;
    gb->ime = false;
  }

  uint8_t opcode = GBCPU_next8(gb);

  // We're going to turn formatting off here because
  // it's extremely annoying when trying to keep cases on the same line
  // clang-format off
  switch (opcode) {

  // NOP
  case 0x00:
    break;

  // RLCA
  case 0x07: {
    uint8_t val = gb->regs[GBCPU_REG_A];
    
    gb->regs[GBCPU_REG_A] = (val << 1) | (val >> 7);

    GBCPU_setZ(gb, false);
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, false);
    GBCPU_setC(gb, val & BIT(7));
    break;
  }

  // RLA
  case 0x17: {
    uint8_t val = gb->regs[GBCPU_REG_A];
    
    gb->regs[GBCPU_REG_A] = (val << 1) | (GBCPU_getC(gb) >> 7);

    GBCPU_setZ(gb, false);
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, false);
    GBCPU_setC(gb, val & BIT(7));
    break;
  }

  // RRA
  case 0x1F: {
    uint8_t val = gb->regs[GBCPU_REG_A];
    
    gb->regs[GBCPU_REG_A] = (val >> 1) | (GBCPU_getC(gb) << 7);

    GBCPU_setZ(gb, false);
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, false);
    GBCPU_setC(gb, val & BIT(0));
    break;
  }

  // DAA
  case 0x27: 
    if (!GBCPU_getN(gb)) {
        if (GBCPU_getC(gb) || gb->regs[GBCPU_REG_A] > 0x99) {
            gb->regs[GBCPU_REG_A] += 0x60;
            GBCPU_setC(gb, true);
        }
        if (GBCPU_getH(gb) || (gb->regs[GBCPU_REG_A] & 0x0f) > 0x09) {
            gb->regs[GBCPU_REG_A] += 0x6;
        }
    }
    else {
        if (GBCPU_getC(gb)) {
            gb->regs[GBCPU_REG_A] -= 0x60;
            GBCPU_setC(gb, true);
        }
        if (GBCPU_getH(gb)) {
            gb->regs[GBCPU_REG_A] -= 0x6;
        }
    }
    GBCPU_setZ(gb, gb->regs[GBCPU_REG_A] == 0);
    GBCPU_setH(gb, false);
    break;

  // DI
  case 0xF3:
    gb->ime = false;
    break;

  // EI
  case 0xFB:
    gb->ime = true;
    break;

  // JP u16
  case 0xC3:
    gb->pc = GBCPU_next16(gb);
    GBCPU_stall_cycles(gb, 4);
    break;

  // JP HL
  case 0xE9:
    gb->pc = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    break;


  // LD (FF00+u8), A
  case 0xE0:
    GBCPU_write(gb, 0xFF00 | GBCPU_next8(gb), gb->regs[GBCPU_REG_A]);
    break;
  // LD A, (FF00+u8)
  case 0xF0:
    gb->regs[GBCPU_REG_A] = GBCPU_read(gb, 0xFF00 | GBCPU_next8(gb));
    break;

  // LD (FF00+C), A
  case 0xE2:
    GBCPU_write(gb, 0xFF00 | gb->regs[GBCPU_REG_C], gb->regs[GBCPU_REG_A]);
    break;
  // LD A, (FF00+C)
  case 0xF2:
    gb->regs[GBCPU_REG_A] = GBCPU_read(gb, 0xFF00 | gb->regs[GBCPU_REG_C]);
    break;

  // LD (u16), A
  case 0xEA:
    GBCPU_write(gb, GBCPU_next16(gb), gb->regs[GBCPU_REG_A]);
    break;
  // LD A, (u16)
  case 0xFA:
    gb->regs[GBCPU_REG_A] = GBCPU_read(gb, GBCPU_next16(gb));
    break;

  // LD r16, u16
  case 0x01: case 0x11: case 0x21: case 0x31:
    GBCPU_set_reg_pair2(gb, (opcode >> 4) & 0b11, GBCPU_next16(gb));
    break;

  // LD [r16], A
  case 0x02:
  case 0x12:
    GBCPU_write(gb, GBCPU_get_reg_pair(gb, (opcode >> 4) & 0b11), gb->regs[GBCPU_REG_A]);
    break;

  // LD A, [r16]
  case 0x0A:
  case 0x1A:
    gb->regs[GBCPU_REG_A] = GBCPU_read(gb, GBCPU_get_reg_pair(gb, (opcode >> 4) & 0b11));
    break;

  // LD [HL+], A
  case 0x22: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    GBCPU_write(gb, pre, gb->regs[GBCPU_REG_A]);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre + 1);
    break;
  }
  // LD [HL-], A
  case 0x32: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    GBCPU_write(gb, pre, gb->regs[GBCPU_REG_A]);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre - 1);
    break;
  }

  // LD A, [HL+]
  case 0x2A: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    gb->regs[GBCPU_REG_A] = GBCPU_read(gb, pre);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre + 1);
    break;
  }
  // LD A, [HL-]
  case 0x3A: {
    uint16_t pre = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);
    gb->regs[GBCPU_REG_A] = GBCPU_read(gb, pre);
    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, pre - 1);
    break;
  }

  // JR i8
  case 0x18: 
    gb->pc += (int8_t)GBCPU_next8(gb);
    GBCPU_stall_cycles(gb, 4);
    break;

  // JR CC, i8
  case 0x20: case 0x30: case 0x28: case 0x38: {
    uint8_t offset = GBCPU_next8(gb);
    if (GBCPU_get_cond(gb, (opcode >> 3) & 0b11)) {
      gb->pc += (int8_t)offset;
      GBCPU_stall_cycles(gb, 4);
    }
    break;
  }

  // JP CC, u16
  case 0xC2: case 0xD2: case 0xCA: case 0xDA: {
    uint16_t target = GBCPU_next16(gb);
    if (GBCPU_get_cond(gb, (opcode >> 3) & 0b11)) {
      gb->pc = target;
      GBCPU_stall_cycles(gb, 4);
    }
    break;
  }

  // CALL
  case 0xCD: {
    uint16_t target = GBCPU_next16(gb);
    GBCPU_stall_cycles(gb, 4);
    GBCPU_push(gb, gb->pc);
    gb->pc = target;
    break;
  }

  // CALL CC
  case 0xC4: case 0xCC: case 0xD4: case 0xDC: {
    uint16_t target = GBCPU_next16(gb);

    if (GBCPU_get_cond(gb, (opcode >> 3) & 0b11)) {
      GBCPU_stall_cycles(gb, 4);
      
      GBCPU_push(gb, gb->pc);
      gb->pc = target;
    }
    break;
  }

  // RETI
  case 0xD9: {
    gb->pc = GBCPU_pop(gb);
    gb->ime = true;
    GBCPU_stall_cycles(gb, 4);
    break;
  }

  // RET
  case 0xC9: {
    gb->pc = GBCPU_pop(gb);
    GBCPU_stall_cycles(gb, 4);
    break;
  }

  // RET CC
  case 0xC0: case 0xD0: case 0xC8: case 0xD8:  {
      GBCPU_stall_cycles(gb, 4);
    if (GBCPU_get_cond(gb, (opcode >> 3) & 0b11)) {
      gb->pc = GBCPU_pop(gb);
    }
    break;
  }

  // PUSH r16
  case 0xC5: case 0xD5: case 0xE5: case 0xF5:
    GBCPU_stall_cycles(gb, 4);
    GBCPU_push(gb, GBCPU_get_reg_pair(gb, (opcode >> 4) & 0b11)); 
    break;

  // POP r16
  case 0xC1: case 0xD1: case 0xE1: case 0xF1:
    GBCPU_set_reg_pair(gb, (opcode >> 4) & 0b11, GBCPU_pop(gb)); 
    break;

  // ADD HL, r16
  case 0x09: case 0x19: case 0x29: case 0x39: {
    uint16_t hl_val = GBCPU_get_reg_pair(gb, GBCPU_REG_HL);  
    uint16_t r16_val = GBCPU_get_reg_pair2(gb, (opcode >> 4) & 0b11);  

    GBCPU_set_reg_pair(gb, GBCPU_REG_HL, hl_val + r16_val);

    GBCPU_setN(gb, false);
    GBCPU_setH(gb, (hl_val & 0xFFF) + (r16_val & 0xFFF) > 0xFFF);
    GBCPU_setC(gb, (uint32_t)hl_val + r16_val > 0xFFFF);

    GBCPU_stall_cycles(gb, 4);
    break;
  }

  // RST
  case 0xC7: case 0xCF: case 0xD7: case 0xDF: case 0xE7: case 0xEF: case 0xF7: case 0xFF: {
    GBCPU_stall_cycles(gb, 4);

    uint16_t target = opcode & 0b111000;
    GBCPU_push(gb, gb->pc);
    gb->pc = target;
    break;
  }

  // INC r8
  case 0x04: case 0x14: case 0x24: case 0x34: case 0x0C: case 0x1C: case 0x2C: case 0x3C: {
    uint8_t old = GBCPU_get_reg(gb, (opcode >> 3) & 0b111);
    uint8_t res = old + 1;

    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, res);

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, (old & 0xF) >= 0xF);
    break;
  }

  // DEC r8
  case 0x05: case 0x15: case 0x25: case 0x35: case 0x0D: case 0x1D: case 0x2D: case 0x3D: {
    uint8_t old = GBCPU_get_reg(gb, (opcode >> 3) & 0b111);
    uint8_t res = old - 1;

    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, res);

    GBCPU_setZ(gb, res == 0);
    GBCPU_setN(gb, true);
    GBCPU_setH(gb, 1 > (old & 0xF));
    break;
  }

  // INC r16
  case 0x03: case 0x13: case 0x23: case 0x33:
    GBCPU_set_reg_pair2(gb, (opcode >> 4) & 0b11, GBCPU_get_reg_pair2(gb, (opcode >> 4) & 0b11) + 1);
    GBCPU_stall_cycles(gb, 4);
    break;

  // DEC r16
  case 0x0B: case 0x1B: case 0x2B: case 0x3B:
    GBCPU_set_reg_pair2(gb, (opcode >> 4) & 0b11, GBCPU_get_reg_pair2(gb, (opcode >> 4) & 0b11) - 1);
    GBCPU_stall_cycles(gb, 4);
    break;

  // LD r8, u8
  case 0x06: case 0x16: case 0x26: case 0x36: case 0x0E: case 0x1E: case 0x2E: case 0x3E:
    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, GBCPU_next8(gb));
    break;

  // CPL
  case 0x2F:
    gb->regs[GBCPU_REG_A] ^= 0xFF;

    GBCPU_setN(gb, true);
    GBCPU_setH(gb, true);
    break;

  // CCF
  case 0x3F:
    GBCPU_setN(gb, false);
    GBCPU_setH(gb, false);
    GBCPU_setC(gb, !GBCPU_getC(gb));
    break;

  // ADD u8
  case 0xC6:
    GBCPU_instr_add(gb, GBCPU_next8(gb));
    break;
  // ADC u8
  case 0xCE:
    GBCPU_instr_adc(gb, GBCPU_next8(gb));
    break;
  // SUB u8
  case 0xD6:
    GBCPU_instr_sub(gb, GBCPU_next8(gb));
    break;
  // SBC u8
  case 0xDE:
    GBCPU_instr_sbc(gb, GBCPU_next8(gb));
    break;
  // AND u8
  case 0xE6:
    GBCPU_instr_and(gb, GBCPU_next8(gb));
    break;
  // XOR u8
  case 0xEE:
    GBCPU_instr_xor(gb, GBCPU_next8(gb));
    break;
  // OR u8
  case 0xF6: 
    GBCPU_instr_or(gb, GBCPU_next8(gb));
    break;
  // CP u8
  case 0xFE:
    GBCPU_instr_cp(gb, GBCPU_next8(gb));
    break;

  // ADD r8
  case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x87:
    GBCPU_instr_add(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // ADC r8
  case 0x88: case 0x89: case 0x8A: case 0x8B: case 0x8C: case 0x8D: case 0x8E: case 0x8F:
    GBCPU_instr_adc(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // SUB r8
  case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: case 0x95: case 0x96: case 0x97:
    GBCPU_instr_sub(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // SBC r8
  case 0x98: case 0x99: case 0x9A: case 0x9B: case 0x9C: case 0x9D: case 0x9E: case 0x9F:
    GBCPU_instr_sbc(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // AND r8
  case 0xA0: case 0xA1: case 0xA2: case 0xA3: case 0xA4: case 0xA5: case 0xA6: case 0xA7:
    GBCPU_instr_and(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // XOR r8
  case 0xA8: case 0xA9: case 0xAA: case 0xAB: case 0xAC: case 0xAD: case 0xAE: case 0xAF:
    GBCPU_instr_xor(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // OR r8
  case 0xB0: case 0xB1: case 0xB2: case 0xB3: case 0xB4: case 0xB5: case 0xB6: case 0xB7:
    GBCPU_instr_or(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // CP r8
  case 0xB8: case 0xB9: case 0xBA: case 0xBB: case 0xBC: case 0xBD: case 0xBE: case 0xBF:
    GBCPU_instr_cp(gb, GBCPU_get_reg(gb, opcode & 0b111));
    break;

  // LD r8, r8
  case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47:
  case 0x48: case 0x49: case 0x4A: case 0x4B: case 0x4C: case 0x4D: case 0x4E: case 0x4F:
  case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57:
  case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5E: case 0x5F:
  case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67:
  case 0x68: case 0x69: case 0x6A: case 0x6B: case 0x6C: case 0x6D: case 0x6E: case 0x6F:
  case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: /* HALT */ case 0x77:
  case 0x78: case 0x79: case 0x7A: case 0x7B: case 0x7C: case 0x7D: case 0x7E: case 0x7F:
    GBCPU_set_reg(gb, (opcode >> 3) & 0b111, GBCPU_get_reg(gb, opcode & 0b111));
    break;
  // clang-format on 

  case 0xCB: {
    uint8_t opcode2 = GBCPU_next8(gb);

    uint8_t reg_id = opcode2 & 0b111;
    uint8_t reg_id2 = (opcode2 >> 3) & 0b111;
    uint8_t reg = GBCPU_get_reg(gb, reg_id);

    switch (opcode2 >> 6) {
      case 0:
        {
          switch ((opcode2 >> 3) & 0b111) {
            case 2: // RL
            {
              uint8_t res = (reg << 1) | GBCPU_getC(gb);

              GBCPU_set_reg(gb, reg_id, res);
              
              GBCPU_setZ(gb, res == 0);
              GBCPU_setN(gb, false);
              GBCPU_setH(gb, false);
              GBCPU_setC(gb, reg & BIT(7));
              break;
            }
            case 3: // RR
            {
              uint8_t res = (reg >> 1) | (GBCPU_getC(gb) << 7);

              GBCPU_set_reg(gb, reg_id, res);
              
              GBCPU_setZ(gb, res == 0);
              GBCPU_setN(gb, false);
              GBCPU_setH(gb, false);
              GBCPU_setC(gb, reg & BIT(0));
              break;
            }
            case 4: // SLA
            {
              uint8_t res = reg << 1;
              
              GBCPU_set_reg(gb, reg_id, res);

              GBCPU_setZ(gb, res == 0);
              GBCPU_setN(gb, false);
              GBCPU_setH(gb, false);
              GBCPU_setC(gb, reg & BIT(7));
              break;
            }
            case 5: // SRA
            {
              uint8_t res = (int8_t)reg >> 1;
              
              GBCPU_set_reg(gb, reg_id, res);

              GBCPU_setZ(gb, res == 0);
              GBCPU_setN(gb, false);
              GBCPU_setH(gb, false);
              GBCPU_setC(gb, reg & BIT(0));
              break;
            }
            case 6: // SWAP
            {
              uint8_t lower = (reg >> 0) & 0xF;
              uint8_t upper = (reg >> 4) & 0xF;

              uint8_t res = (lower << 4) | upper;

              GBCPU_set_reg(gb, reg_id, res);

              GBCPU_setZ(gb, res == 0);
              GBCPU_setN(gb, false);
              GBCPU_setH(gb, false);
              GBCPU_setC(gb, false);
              break;
            }
            case 7: // SRL
            {
              uint8_t res = (reg >> 1) & 0xFF;

              GBCPU_set_reg(gb, reg_id, res);

              GBCPU_setZ(gb, res == 0);
              GBCPU_setN(gb, false);
              GBCPU_setH(gb, false);
              GBCPU_setC(gb, reg & BIT(0));
              break;
            }
            default:
              printf("Unknown CB opcode: %02X @ PC:%04X\n", opcode2, gb->pc - 2);
              GBCPU_print_status(gb, gb->pc - 1);
              assert(0);
              break;
          }
          break;
        }
      case 1: // BIT
        GBCPU_setZ(gb, !(reg & (1 << reg_id2)));
        GBCPU_setN(gb, false);
        GBCPU_setH(gb, true);
        break;
      case 2: // RES
        GBCPU_set_reg(gb, reg_id, reg & ~(1 << reg_id2));
        break;
      case 3: // SET
        GBCPU_set_reg(gb, reg_id, reg | (1 << reg_id2));
        break;

      default:
        printf("Unknown CB opcode: %02X @ PC:%04X\n", opcode2, gb->pc - 2);
        GBCPU_print_status(gb, gb->pc - 1);
        assert(0);
        break;
    }
    
    break;
  }

  default:
    printf("Unknown opcode: %02X @ PC:%04X\n", opcode, gb->pc - 1);
    GBCPU_print_status(gb, gb->pc - 1);
    assert(0);
    break;
  }

  return gb->current_instr_cycles;
}

uint32_t GB_run_to_next_frame(GB_core_t *gb) {
  uint32_t total_cycles = 0;

  // Since we don't know how long the next instruction will take,
  // we use a persistent timer in case an instruction overshoots
  // the alloted time of 70224 cycles per frame
  gb->frame_timer += CYCLES_PER_FRAME;

  while (gb->frame_timer > 0) {
    uint8_t cycles = GBCPU_execute(gb);
    total_cycles += cycles;

    gb->frame_timer -= cycles;

    GB_ppu_tick(gb, cycles);
    GB_timer_tick(gb, cycles);
  }

  return total_cycles;
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

  GB_write_io_lcdc(gb, 0x91);
  GB_write_io_if(gb, 0xE1);
}
