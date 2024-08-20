struct GameBoyRegister {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    f: u8,
    h: u8,
    l: u8,
}

struct GameBoyState {
    memory: [u8; 0xFFFF + 1],
    registers: GameBoyRegister,
    sp: u16,
    pc: u16,
}

impl GameBoyState {
    fn new() -> GameBoyState {
        GameBoyState {
            memory: [0; 0xFFFF + 1],
            registers: GameBoyRegister {
                a: 0x01,
                b: 0x00,
                c: 0x13,
                d: 0x00,
                e: 0xD8,
                f: 0xB0,
                h: 0x01,
                l: 0x4D,
            },
            sp: 0xFFFE,
            pc: 0x100,
        }
    }

    fn load_rom(&mut self, rom: Vec<u8>) {
        for i in 0..rom.len() {
            self.memory[i] = rom[i];
        }
    }

    fn fetch_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn read_byte(&mut self) -> u8 {
        let byte = self.fetch_byte(self.pc);
        self.pc += 1;
        return byte;
    }

    fn read_nn_lsb_msb(&mut self) -> (u16, u8, u8) {
        let lsb = self.fetch_byte(self.pc);
        let msb = self.fetch_byte(self.pc + 1);
        self.pc += 2;
        let nn: u16 = ((msb as u16) << 8) | (lsb as u16);
        return (nn, lsb, msb);
    }

    fn read_nn_from_stack(&mut self) -> u16 {
        let lsb = self.memory[self.sp as usize];
        self.sp += 1;
        let msb = self.memory[self.sp as usize];
        self.sp += 1;
        return ((msb as u16) << 8) | (lsb as u16);
    }

    fn dump_registers(&self) {
        print!("A: 0x{:02X}  ", self.registers.a);
        print!("B: 0x{:02X}  ", self.registers.b);
        print!("C: 0x{:02X}  ", self.registers.c);
        print!("D: 0x{:02X}  ", self.registers.d);
        print!("E: 0x{:02X}  ", self.registers.e);
        print!("F: 0x{:02X}  ", self.registers.f);
        print!("H: 0x{:02X}  ", self.registers.h);
        print!("L: 0x{:02X}  ", self.registers.l);
        println!();
    }

    fn dump_memory(&self) {
        for i in 0..self.memory.len() {
            print!("{:02X} ", self.memory[i]);
            if (i+1) % 32 == 0 {
                println!();
            }
        }
        println!();
    }
}

fn debug_print_op(op: u8, name: &str, state: &GameBoyState) {
    println!("> [0x{:02X}]\t{:<16}\tpc: 0x{:02X}  sp: 0x{:02X}", op, name, state.pc, state.sp);
    // state.dump_registers();
}

fn main() -> ! {
    let rom_file: &str = "./rom/snake.gb";
    let rom: Vec<u8> = std::fs::read(rom_file).unwrap();
    let mut gb = GameBoyState::new();
    gb.load_rom(rom);

    gb.dump_memory();
    gb.dump_registers();

    println!("Starting emulation loop");

    // set lcd status at 0xFF44
    // NOTE: this is a hack to avoid the panic on the first read of the lcd status
    gb.memory[0xFF44] = 0x90;

    loop {
        let op: u8 = gb.read_byte();

        match op {
            0x00 => {
                debug_print_op(op, "NOP", &gb);
            }
            0x01 => {
                debug_print_op(op, "LD BC, nn", &gb);
                let (_nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.b = msb;
                gb.registers.c = lsb;
            }
            0x02 => {
                debug_print_op(op, "LD (BC), A", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                gb.memory[bc as usize] = gb.registers.a;
            }
            0x03 => {
                debug_print_op(op, "INC BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                let result = bc.wrapping_add(1);
                gb.registers.b = (result >> 8) as u8;
                gb.registers.c = (result & 0xFF) as u8;
            }
            0x04 => {
                debug_print_op(op, "INC B", &gb);
                let result = gb.registers.b.wrapping_add(1);
                gb.registers.b = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.b & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x05 => {
                debug_print_op(op, "DEC B", &gb);
                let result = gb.registers.b.wrapping_sub(1);
                gb.registers.b = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.b & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x06 => {
                debug_print_op(op, "LD B, n", &gb);
                gb.registers.b = gb.read_byte();
            }
            0x07 => {
                debug_print_op(op, "RLCA", &gb);
                let carry = gb.registers.a & 0x80;
                gb.registers.a = (gb.registers.a << 1) | (carry >> 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
            }
            0x08 => {
                debug_print_op(op, "LD (nn), SP", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.memory[nn as usize] = (gb.sp & 0xFF) as u8;
                gb.memory[(nn + 1) as usize] = (gb.sp >> 8) as u8;
            }
            0x09 => {
                debug_print_op(op, "ADD HL, BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(bc);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
            }
            0x0A => {
                debug_print_op(op, "LD A, (BC)", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                gb.registers.a = gb.fetch_byte(bc);
            }
            0x0B => {
                debug_print_op(op, "DEC BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                let result = bc.wrapping_sub(1);
                gb.registers.b = (result >> 8) as u8;
                gb.registers.c = (result & 0xFF) as u8;
            }
            0x0C => {
                debug_print_op(op, "INC C", &gb);
                let result = gb.registers.c.wrapping_add(1);
                gb.registers.c = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.c & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x0D => {
                debug_print_op(op, "DEC C", &gb);
                let result = gb.registers.c.wrapping_sub(1);
                gb.registers.c = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.c & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x0E => {
                debug_print_op(op, "LD C, n", &gb);
                gb.registers.c = gb.read_byte();
            }
            0x0F => {
                debug_print_op(op, "RRCA", &gb);
                let carry = gb.registers.a & 0x01;
                gb.registers.a = (gb.registers.a >> 1) | (carry << 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
            }
            0x11 => {
                debug_print_op(op, "LD DE, nn", &gb);
                let (_nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.d = msb;
                gb.registers.e = lsb;
            }
            0x12 => {
                debug_print_op(op, "LD (DE), A", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.memory[de as usize] = gb.registers.a;
            }
            0x13 => {
                debug_print_op(op, "INC DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                let result = de.wrapping_add(1);
                gb.registers.d = (result >> 8) as u8;
                gb.registers.e = (result & 0xFF) as u8;
            }
            0x14 => {
                debug_print_op(op, "INC D", &gb);
                let result = gb.registers.d.wrapping_add(1);
                gb.registers.d = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.d & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x15 => {
                debug_print_op(op, "DEC D", &gb);
                let result = gb.registers.d.wrapping_sub(1);
                gb.registers.d = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.d & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x16 => {
                debug_print_op(op, "LD D, n", &gb);
                gb.registers.d = gb.read_byte();
            }
            0x17 => {
                debug_print_op(op, "RLA", &gb);
                let carry = gb.registers.a & 0x80;
                gb.registers.a = (gb.registers.a << 1) | (carry >> 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
            }
            0x18 => {
                debug_print_op(op, "JR e", &gb);
                let e = gb.read_byte() as i8;
                gb.pc = gb.pc.wrapping_add(e as u16);
            }
            0x19 => {
                debug_print_op(op, "ADD HL, DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(de);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
            }
            0x1A => {
                debug_print_op(op, "LD A, (DE)", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.registers.a = gb.fetch_byte(de);
            }
            0x1B => {
                debug_print_op(op, "DEC DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                let result = de.wrapping_sub(1);
                gb.registers.d = (result >> 8) as u8;
                gb.registers.e = (result & 0xFF) as u8;
            }
            0x1C => {
                debug_print_op(op, "INC E", &gb);
                let result = gb.registers.e.wrapping_add(1);
                gb.registers.e = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.e & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x1D => {
                debug_print_op(op, "DEC E", &gb);
                let result = gb.registers.e.wrapping_sub(1);
                gb.registers.e = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.e & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x1E => {
                debug_print_op(op, "LD E, n", &gb);
                gb.registers.e = gb.read_byte();
            }
            0x1F => {
                debug_print_op(op, "RRA", &gb);
                let carry = gb.registers.a & 0x01;
                gb.registers.a = (gb.registers.a >> 1) | (carry << 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
            }
            0x20 => {
                debug_print_op(op, "JR NZ, e", &gb);
                let e = gb.read_byte() as i8;
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = gb.pc.wrapping_add(e as u16);
                }
            }
            0x21 => {
                debug_print_op(op, "LD HL, nn", &gb);
                let (_nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.h = msb;
                gb.registers.l = lsb;
            }
            0x22 => {
                debug_print_op(op, "LD (HL+), A", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.a;
                gb.registers.l = gb.registers.l.wrapping_add(1);
                if gb.registers.l == 0 {
                    gb.registers.h = gb.registers.h.wrapping_add(1);
                }
            }
            0x23 => {
                debug_print_op(op, "INC HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(1);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
            }
            0x24 => {
                debug_print_op(op, "INC H", &gb);
                let result = gb.registers.h.wrapping_add(1);
                gb.registers.h = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.h & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x25 => {
                debug_print_op(op, "DEC H", &gb);
                let result = gb.registers.h.wrapping_sub(1);
                gb.registers.h = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.h & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x26 => {
                debug_print_op(op, "LD H, n", &gb);
                gb.registers.h = gb.read_byte();
            }
            0x27 => {
                debug_print_op(op, "DAA", &gb);
                // NOTE: validate implementation of decimal adjust
                let mut a = gb.registers.a;
                let mut adjust = 0;
                if gb.registers.f & 0b00100000 != 0 || (a & 0xF) > 9 {
                    adjust |= 0x06;
                }
                if gb.registers.f & 0b00010000 != 0 || a > 0x99 {
                    adjust |= 0x60;
                    gb.registers.f |= 0b00010000;
                }
                a = a.wrapping_add(adjust);
                gb.registers.a = a;
                gb.registers.f |= if a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= if adjust & 0x60 != 0 { 0b00100000 } else { 0 };
            }
            0x28 => {
                debug_print_op(op, "JR Z, n", &gb);
                let n = gb.read_byte();
                if gb.registers.f & 0b10000000 != 0 {
                    gb.pc = gb.pc.wrapping_add(n as u16);
                }
            }
            0x29 => {
                debug_print_op(op, "ADD HL, HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(hl);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
            }
            0x2A => {
                debug_print_op(op, "LD A, (HL+)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.a = gb.fetch_byte(hl);
                gb.registers.l = gb.registers.l.wrapping_add(1);
                if gb.registers.l == 0 {
                    gb.registers.h = gb.registers.h.wrapping_add(1);
                }
            }
            0x2B => {
                debug_print_op(op, "DEC HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_sub(1);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
            }
            0x2C => {
                debug_print_op(op, "INC L", &gb);
                let result = gb.registers.l.wrapping_add(1);
                gb.registers.l = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.l & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x2D => {
                debug_print_op(op, "DEC L", &gb);
                let result = gb.registers.l.wrapping_sub(1);
                gb.registers.l = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.l & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x2E => {
                debug_print_op(op, "LD L, n", &gb);
                gb.registers.l = gb.read_byte();
            }
            0x2F => {
                debug_print_op(op, "CPL", &gb);
                gb.registers.a = !gb.registers.a;
                gb.registers.f |= 0b01100000;
            }
            0x30 => {
                debug_print_op(op, "JR NC, e", &gb);
                let e = gb.read_byte() as i8;
                if gb.registers.f & 0b00010000 == 0 {
                    gb.pc = gb.pc.wrapping_add(e as u16);
                }
            }
            0x31 => {
                debug_print_op(op, "LD SP, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.sp = nn;
            }
            0x32 => {
                debug_print_op(op, "LD (HL-), A", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.a;
                gb.registers.l = gb.registers.l.wrapping_sub(1);
                if gb.registers.l == 0xFF {
                    gb.registers.h = gb.registers.h.wrapping_sub(1);
                }
            }
            0x33 => {
                debug_print_op(op, "INC SP", &gb);
                gb.sp = gb.sp.wrapping_add(1);
            }
            0x34 => {
                debug_print_op(op, "INC (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = gb.fetch_byte(hl).wrapping_add(1);
                gb.memory[hl as usize] = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (result & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x35 => {
                debug_print_op(op, "DEC (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = gb.fetch_byte(hl).wrapping_sub(1);
                gb.memory[hl as usize] = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (result & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x36 => {
                debug_print_op(op, "LD (HL), n", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.read_byte();
            }
            0x37 => {
                debug_print_op(op, "SCF", &gb);
                gb.registers.f &= 0b10010000;
                gb.registers.f |= 0b00010000;
            }
            0x38 => {
                debug_print_op(op, "JR C, e", &gb);
                let e = gb.read_byte() as i8;
                if gb.registers.f & 0b00010000 != 0 {
                    gb.pc = gb.pc.wrapping_add(e as u16);
                }
            }
            0x39 => {
                debug_print_op(op, "ADD HL, SP", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(gb.sp);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
            }
            0x3A => {
                debug_print_op(op, "LD A, (HL-)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.a = gb.fetch_byte(hl);
                gb.registers.l = gb.registers.l.wrapping_sub(1);
                if gb.registers.l == 0xFF {
                    gb.registers.h = gb.registers.h.wrapping_sub(1);
                }
            }
            0x3B => {
                debug_print_op(op, "DEC SP", &gb);
                gb.sp = gb.sp.wrapping_sub(1);
            }
            0x3C => {
                debug_print_op(op, "INC A", &gb);
                let result = gb.registers.a.wrapping_add(1);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x3D => {
                debug_print_op(op, "DEC A", &gb);
                let result = gb.registers.a.wrapping_sub(1);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00010000;
            }
            0x3E => {
                debug_print_op(op, "LD A, n", &gb);
                gb.registers.a = gb.read_byte();
            }
            0x3F => {
                debug_print_op(op, "CCF", &gb);
                gb.registers.f &= 0b10010000;
                gb.registers.f |= if gb.registers.f & 0b00010000 == 0 { 0b00010000 } else { 0 };
            }
            0x40 => {
                debug_print_op(op, "LD B, B", &gb);
                gb.registers.b = gb.registers.b;
            }
            0x41 => {
                debug_print_op(op, "LD B, C", &gb);
                gb.registers.b = gb.registers.c;
            }
            0x42 => {
                debug_print_op(op, "LD B, D", &gb);
                gb.registers.b = gb.registers.d;
            }
            0x43 => {
                debug_print_op(op, "LD B, E", &gb);
                gb.registers.b = gb.registers.e;
            }
            0x44 => {
                debug_print_op(op, "LD B, H", &gb);
                gb.registers.b = gb.registers.h;
            }
            0x45 => {
                debug_print_op(op, "LD B, L", &gb);
                gb.registers.b = gb.registers.l;
            }
            0x46 => {
                debug_print_op(op, "LD B, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.b = gb.fetch_byte(hl);
            }
            0x47 => {
                debug_print_op(op, "LD B, A", &gb);
                gb.registers.b = gb.registers.a;
            }
            0x48 => {
                debug_print_op(op, "LD C, B", &gb);
                gb.registers.c = gb.registers.b;
            }
            0x49 => {
                debug_print_op(op, "LD C, C", &gb);
                gb.registers.c = gb.registers.c;
            }
            0x4A => {
                debug_print_op(op, "LD C, D", &gb);
                gb.registers.c = gb.registers.d;
            }
            0x4B => {
                debug_print_op(op, "LD C, E", &gb);
                gb.registers.c = gb.registers.e;
            }
            0x4C => {
                debug_print_op(op, "LD C, H", &gb);
                gb.registers.c = gb.registers.h;
            }
            0x4D => {
                debug_print_op(op, "LD C, L", &gb);
                gb.registers.c = gb.registers.l;
            }
            0x4E => {
                debug_print_op(op, "LD C, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.c = gb.fetch_byte(hl);
            }
            0x4F => {
                debug_print_op(op, "LD C, A", &gb);
                gb.registers.c = gb.registers.a;
            }
            0x50 => {
                debug_print_op(op, "LD D, B", &gb);
                gb.registers.d = gb.registers.b;
            }
            0x51 => {
                debug_print_op(op, "LD D, C", &gb);
                gb.registers.d = gb.registers.c;
            }
            0x52 => {
                debug_print_op(op, "LD D, D", &gb);
                gb.registers.d = gb.registers.d;
            }
            0x53 => {
                debug_print_op(op, "LD D, E", &gb);
                gb.registers.d = gb.registers.e;
            }
            0x54 => {
                debug_print_op(op, "LD D, H", &gb);
                gb.registers.d = gb.registers.h;
            }
            0x55 => {
                debug_print_op(op, "LD D, L", &gb);
                gb.registers.d = gb.registers.l;
            }
            0x56 => {
                debug_print_op(op, "LD D, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.d = gb.fetch_byte(hl);
            }
            0x57 => {
                debug_print_op(op, "LD D, A", &gb);
                gb.registers.d = gb.registers.a;
            }
            0x58 => {
                debug_print_op(op, "LD E, B", &gb);
                gb.registers.e = gb.registers.b;
            }
            0x59 => {
                debug_print_op(op, "LD E, C", &gb);
                gb.registers.e = gb.registers.c;
            }
            0x5A => {
                debug_print_op(op, "LD E, D", &gb);
                gb.registers.e = gb.registers.d;
            }
            0x5B => {
                debug_print_op(op, "LD E, E", &gb);
                gb.registers.e = gb.registers.e;
            }
            0x5C => {
                debug_print_op(op, "LD E, H", &gb);
                gb.registers.e = gb.registers.h;
            }
            0x5D => {
                debug_print_op(op, "LD E, L", &gb);
                gb.registers.e = gb.registers.l;
            }
            0x5E => {
                debug_print_op(op, "LD E, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.e = gb.fetch_byte(hl);
            }
            0x5F => {
                debug_print_op(op, "LD E, A", &gb);
                gb.registers.e = gb.registers.a;
            }
            0x60 => {
                debug_print_op(op, "LD H, B", &gb);
                gb.registers.h = gb.registers.b;
            }
            0x61 => {
                debug_print_op(op, "LD H, C", &gb);
                gb.registers.h = gb.registers.c;
            }
            0x62 => {
                debug_print_op(op, "LD H, D", &gb);
                gb.registers.h = gb.registers.d;
            }
            0x63 => {
                debug_print_op(op, "LD H, E", &gb);
                gb.registers.h = gb.registers.e;
            }
            0x64 => {
                debug_print_op(op, "LD H, H", &gb);
                gb.registers.h = gb.registers.h;
            }
            0x65 => {
                debug_print_op(op, "LD H, L", &gb);
                gb.registers.h = gb.registers.l;
            }
            0x66 => {
                debug_print_op(op, "LD H, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.h = gb.fetch_byte(hl);
            }
            0x67 => {
                debug_print_op(op, "LD H, A", &gb);
                gb.registers.h = gb.registers.a;
            }
            0x68 => {
                debug_print_op(op, "LD L, B", &gb);
                gb.registers.l = gb.registers.b;
            }
            0x69 => {
                debug_print_op(op, "LD L, C", &gb);
                gb.registers.l = gb.registers.c;
            }
            0x6A => {
                debug_print_op(op, "LD L, D", &gb);
                gb.registers.l = gb.registers.d;
            }
            0x6B => {
                debug_print_op(op, "LD L, E", &gb);
                gb.registers.l = gb.registers.e;
            }
            0x6C => {
                debug_print_op(op, "LD L, H", &gb);
                gb.registers.l = gb.registers.h;
            }
            0x6D => {
                debug_print_op(op, "LD L, L", &gb);
                gb.registers.l = gb.registers.l;
            }
            0x6E => {
                debug_print_op(op, "LD L, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.l = gb.fetch_byte(hl);
            }
            0x6F => {
                debug_print_op(op, "LD L, A", &gb);
                gb.registers.l = gb.registers.a;
            }
            0x70 => {
                debug_print_op(op, "LD (HL), B", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.b;
            }
            0x71 => {
                debug_print_op(op, "LD (HL), C", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.c;
            }
            0x72 => {
                debug_print_op(op, "LD (HL), D", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.d;
            }
            0x73 => {
                debug_print_op(op, "LD (HL), E", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.e;
            }
            0x74 => {
                debug_print_op(op, "LD (HL), H", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.h;
            }
            0x75 => {
                debug_print_op(op, "LD (HL), L", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.l;
            }
            0x76 => {
                debug_print_op(op, "HALT", &gb);
                panic!();
            }
            0x77 => {
                debug_print_op(op, "LD (HL), A", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.a;
            }
            0x78 => {
                debug_print_op(op, "LD A, B", &gb);
                gb.registers.a = gb.registers.b;
            }
            0x79 => {
                debug_print_op(op, "LD A, C", &gb);
                gb.registers.a = gb.registers.c;
            }
            0x7A => {
                debug_print_op(op, "LD A, D", &gb);
                gb.registers.a = gb.registers.d;
            }
            0x7B => {
                debug_print_op(op, "LD A, E", &gb);
                gb.registers.a = gb.registers.e;
            }
            0x7C => {
                debug_print_op(op, "LD A, H", &gb);
                gb.registers.a = gb.registers.h;
            }
            0x7D => {
                debug_print_op(op, "LD A, L", &gb);
                gb.registers.a = gb.registers.l;
            }
            0x7E => {
                debug_print_op(op, "LD A, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.a = gb.fetch_byte(hl);
            }
            0x7F => {
                debug_print_op(op, "LD A, A", &gb);
                gb.registers.a = gb.registers.a;
            }
            0x80 => {
                debug_print_op(op, "ADD A, B", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.b);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.b { 0b00010000 } else { 0 };
            }
            0x83 => {
                debug_print_op(op, "ADD A, E", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.e);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.e { 0b00010000 } else { 0 };
            }
            0x90 => {
                debug_print_op(op, "SUB B", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.b);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.b { 0b00010000 } else { 0 };
            }
            0x96 => {
                debug_print_op(op, "SUB (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                let result = gb.registers.a.wrapping_sub(data);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
            }
            0x9F => {
                debug_print_op(op, "SBC A, A", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 { 1 } else { 0 };
                let result = gb.registers.a.wrapping_sub(gb.registers.a).wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.a { 0b00010000 } else { 0 };
            }
            0xA0 => {
                debug_print_op(op, "AND B", &gb);
                gb.registers.a &= gb.registers.b;
            }
            0xAF => {
                debug_print_op(op, "XOR A", &gb);
                gb.registers.a ^= gb.registers.a;
            }
            0xB0 => {
                debug_print_op(op, "OR B", &gb);
                gb.registers.a |= gb.registers.b;
            }
            0xB1 => {
                debug_print_op(op, "OR C", &gb);
                gb.registers.a |= gb.registers.c;
            }
            0xB2 => {
                debug_print_op(op, "OR D", &gb);
                gb.registers.a |= gb.registers.d;
            }
            0xB3 => {
                debug_print_op(op, "OR E", &gb);
                gb.registers.a |= gb.registers.e;
            }
            0xB4 => {
                debug_print_op(op, "OR H", &gb);
                gb.registers.a |= gb.registers.h;
            }
            0xB5 => {
                debug_print_op(op, "OR L", &gb);
                gb.registers.a |= gb.registers.l;
            }
            0xB6 => {
                debug_print_op(op, "OR (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                gb.registers.a |= data;
            }
            0xB7 => {
                debug_print_op(op, "OR A", &gb);
                gb.registers.a |= gb.registers.a;
            }
            0xB8 => {
                debug_print_op(op, "CP B", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.b);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.b { 0b00010000 } else { 0 };
            }
            0xB9 => {
                debug_print_op(op, "CP C", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.c);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.c & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.c { 0b00010000 } else { 0 };
            }
            0xBA => {
                debug_print_op(op, "CP D", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.d);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.d & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.d { 0b00010000 } else { 0 };
            }
            0xBB => {
                debug_print_op(op, "CP E", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.e);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.e { 0b00010000 } else { 0 };
            }
            0xBC => {
                debug_print_op(op, "CP H", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.h);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.h & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.h { 0b00010000 } else { 0 };
            }
            0xBD => {
                debug_print_op(op, "CP L", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.l);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.l & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.l { 0b00010000 } else { 0 };
            }
            0xBE => {
                debug_print_op(op, "CP (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                let result = gb.registers.a.wrapping_sub(data);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
            }
            0xBF => {
                debug_print_op(op, "CP A", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.a);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < gb.registers.a { 0b00010000 } else { 0 };
            }
            0xC0 => {
                debug_print_op(op, "RET NZ", &gb);
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = gb.read_nn_from_stack();
                }
            }
            0xC1 => {
                debug_print_op(op, "POP BC", &gb);
                let bc = gb.read_nn_from_stack();
                gb.registers.b = (bc >> 8) as u8;
                gb.registers.c = (bc & 0xFF) as u8;
            }
            0xC2 => {
                debug_print_op(op, "JP NZ, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = nn;
                }
            }
            0xC3 => {
                debug_print_op(op, "JP nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.pc = nn;
            }
            0xC4 => {
                debug_print_op(op, "CALL NZ, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b10000000 == 0 {
                    let pc_msb = gb.pc >> 8;
                    let pc_lsb = gb.pc & 0xFF;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_msb as u8;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_lsb as u8;
                    gb.pc = nn;
                }
            }
            0xC8 => {
                debug_print_op(op, "RET Z", &gb);
                if gb.registers.f & 0b10000000 != 0 {
                    gb.pc = gb.read_nn_from_stack();
                }
            }
            0xC9 => {
                debug_print_op(op, "RET", &gb);
                gb.pc = gb.read_nn_from_stack();
            }
            0xCB => {
                let cb_op = gb.read_byte();
                
                match cb_op {
                    0x10 => {
                        debug_print_op(op, "RL B", &gb);
                        let result = gb.registers.b << 1;
                        gb.registers.b = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.b & 0x80) != 0 { 0b00100000 } else { 0 };
                        gb.registers.f |= 0b00010000;
                    }
                    0x21 => {
                        debug_print_op(op, "SLA C", &gb);
                        let result = gb.registers.c << 1;
                        gb.registers.c = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.c & 0x80) != 0 { 0b00100000 } else { 0 };
                        gb.registers.f |= 0b00010000;
                    }
                    _ => {
                        panic!("unknown CB opcode: 0x{:02X}", cb_op);
                    }
                }
            }
            0xCD => {
                debug_print_op(op, "CALL nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = nn;
            }
            0xD0 => {
                debug_print_op(op, "RET NC", &gb);
                if gb.registers.f & 0b00010000 == 0 {
                    gb.pc = gb.read_nn_from_stack();
                }
            }
            0xD1 => {
                debug_print_op(op, "POP DE", &gb);
                let de = gb.read_nn_from_stack();
                gb.registers.d = (de >> 8) as u8;
                gb.registers.e = (de & 0xFF) as u8;
            }
            0xD2 => {
                debug_print_op(op, "JP NC, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b00010000 == 0 {
                    gb.pc = nn;
                }
            }
            0xD4 => {
                debug_print_op(op, "CALL NC, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b00010000 == 0 {
                    let pc_msb = gb.pc >> 8;
                    let pc_lsb = gb.pc & 0xFF;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_msb as u8;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_lsb as u8;
                    gb.pc = nn;
                }
            }
            0xD5 => {
                debug_print_op(op, "PUSH DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (de >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (de & 0xFF) as u8;
            }
            0xD6 => {
                debug_print_op(op, "SUB n", &gb);
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_sub(n);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
            }
            0xE0 => {
                debug_print_op(op, "LDH (n), A", &gb);
                let n = gb.read_byte();
                gb.memory[0xFF00 + n as usize] = gb.registers.a;
                println!(">>>> HARDWARE WRITE: 0x{:02X} 0x{:02X}", (0xFF00 as u16 + n as u16), gb.registers.a);
            }
            0xE1 => {
                debug_print_op(op, "POP HL", &gb);
                let hl = gb.read_nn_from_stack();
                gb.registers.h = (hl >> 8) as u8;
                gb.registers.l = (hl & 0xFF) as u8;
            }
            0xE2 => {
                debug_print_op(op, "LD (C), A", &gb);
                gb.memory[0xFF00 + gb.registers.c as usize] = gb.registers.a;
            }
            0xE5 => {
                debug_print_op(op, "PUSH HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (hl >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (hl & 0xFF) as u8;
            }
            0xEA => {
                debug_print_op(op, "LD (nn), A", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.memory[nn as usize] = gb.registers.a;
            }
            0xF0 => {
                debug_print_op(op, "LDH A, (n)", &gb);
                let n = gb.read_byte();
                gb.registers.a = gb.memory[0xFF00 + n as usize];
                println!(">>>> HARDWARE READ: 0x{:02X} 0x{:02X}", (0xFF00 as u16 + n as u16), gb.registers.a);
            }
            0xF3 => {
                debug_print_op(op, "DI", &gb);
            }
            0xFA => {
                debug_print_op(op, "LD A, (nn)", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.registers.a = gb.memory[nn as usize];
            }
            0xFB => {
                debug_print_op(op, "EI", &gb);
            }
            0xFE => {
                debug_print_op(op, "CP n", &gb);
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_sub(n);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) { 0b00100000 } else { 0 };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
            }
            0xFF => {
                debug_print_op(op, "RST 38H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0038;
            }
            _ => {
                panic!("unknown opcode: 0x{:02X}", op);
            }
        }

        // std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
