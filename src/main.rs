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
    memory: [u8; 0xFFFF],
    registers: GameBoyRegister,
    sp: u16,
    pc: u16,
}

impl GameBoyState {
    fn new() -> GameBoyState {
        GameBoyState {
            memory: [0; 0xFFFF],
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
        for i in 0..rom.len()-1 {
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
                let (nn, lsb, msb) = gb.read_nn_lsb_msb();
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
                let (nn, lsb, msb) = gb.read_nn_lsb_msb();
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
            0x16 => {
                debug_print_op(op, "LD D, n", &gb);
                gb.registers.d = gb.read_byte();
            }
            0x18 => {
                debug_print_op(op, "JR e", &gb);
                let e = gb.read_byte() as i8;
                gb.pc = gb.pc.wrapping_add(e as u16);
            }
            0x1A => {
                debug_print_op(op, "LD A, (DE)", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.registers.a = gb.fetch_byte(de);
            }
            0x1E => {
                debug_print_op(op, "LD E, n", &gb);
                gb.registers.e = gb.read_byte();
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
                let (nn, lsb, msb) = gb.read_nn_lsb_msb();
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
            0x28 => {
                debug_print_op(op, "JR Z, n", &gb);
                let n = gb.read_byte();
                if gb.registers.f & 0b10000000 != 0 {
                    gb.pc = gb.pc.wrapping_add(n as u16);
                }
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
            0x2E => {
                debug_print_op(op, "LD L, n", &gb);
                gb.registers.l = gb.read_byte();
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
            0x78 => {
                debug_print_op(op, "LD A, B", &gb);
                gb.registers.a = gb.registers.b;
            }
            0x79 => {
                debug_print_op(op, "LD A, C", &gb);
                gb.registers.a = gb.registers.c;
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
            0xB1 => {
                debug_print_op(op, "OR C", &gb);
                gb.registers.a |= gb.registers.c;
            }
            0xC0 => {
                debug_print_op(op, "RET NZ", &gb);
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = gb.read_nn_from_stack();
                }
            }
            0xC3 => {
                debug_print_op(op, "JP nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.pc = nn;
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
            0xE0 => {
                debug_print_op(op, "LDH (n), A", &gb);
                let n = gb.read_byte();
                gb.memory[0xFF00 + n as usize] = gb.registers.a;
                println!(">>>> HARDWARE WRITE: 0x{:02X} 0x{:02X}", (0xFF00 as u16 + n as u16), gb.registers.a);
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
            _ => {
                panic!("unknown opcode: 0x{:02X}", op);
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
