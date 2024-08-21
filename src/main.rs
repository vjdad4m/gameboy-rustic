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
    ime: bool,
    cycles: u128,
    div_cycles: u128,
    tima_cycles: u128,
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
            ime: false,
            cycles: 0,
            div_cycles: 0,
            tima_cycles: 0,
        }
    }

    fn load_rom(&mut self, rom: Vec<u8>) {
        for i in 0..rom.len() {
            self.memory[i] = rom[i];
        }
    }

    fn reset(&mut self) {
        // intial memory state from http://www.codeslinger.co.uk/pages/projects/gameboy/hardware.html.
        self.registers.a = 0x01;
        self.registers.b = 0x00;
        self.registers.c = 0x13;
        self.registers.d = 0x00;
        self.registers.e = 0xD8;
        self.registers.f = 0xB0;
        self.registers.h = 0x01;
        self.registers.l = 0x4D;
        self.sp = 0xFFFE;
        self.pc = 0x100;
        self.ime = false;
        self.cycles = 0;
        self.div_cycles = 0;
        self.tima_cycles = 0;
        self.memory[0xFF10] = 0x80;
        self.memory[0xFF11] = 0xBF;
        self.memory[0xFF12] = 0xF3;
        self.memory[0xFF14] = 0xBF;
        self.memory[0xFF16] = 0x3F;
        self.memory[0xFF19] = 0xBF;
        self.memory[0xFF1A] = 0x7F;
        self.memory[0xFF1B] = 0xFF;
        self.memory[0xFF1C] = 0x9F;
        self.memory[0xFF1E] = 0xBF;
        self.memory[0xFF20] = 0xFF;
        self.memory[0xFF23] = 0xBF;
        self.memory[0xFF24] = 0x77;
        self.memory[0xFF25] = 0xF3;
        self.memory[0xFF26] = 0xF1;
        self.memory[0xFF40] = 0x91;
        self.memory[0xFF47] = 0xFC;
        self.memory[0xFF48] = 0xFF;
        self.memory[0xFF49] = 0xFF;
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
    }

    fn dump_memory(&self) {
        for i in 0..self.memory.len() {
            print!("{:02X} ", self.memory[i]);
            if (i + 1) % 32 == 0 {
                println!();
            }
        }
        println!();
    }
}

fn debug_print_op(op: u8, name: &str, state: &GameBoyState) {
    print!(
        "> [0x{:02X}]\t{:<16}\tpc: 0x{:02X}  sp: 0x{:02X}\tcounter: {:<8}\t",
        op, name, state.pc, state.sp, state.cycles
    );
    state.dump_registers();
    println!("\t{}", state.tima_cycles);
}

fn main() -> ! {
    let rom_file: &str = "./rom/snake.gb";
    let rom: Vec<u8> = std::fs::read(rom_file).unwrap();
    let mut gb = GameBoyState::new();
    gb.load_rom(rom);
    gb.reset();

    gb.dump_memory();
    gb.dump_registers();

    println!("Starting emulation loop");

    // set lcd status at 0xFF44
    // NOTE: this is a hack to avoid the panic on the first read of the lcd status
    gb.memory[0xFF44] = 0x90;

    loop {
        let cycles_start: u128 = gb.cycles;
        let div_initial: u8 = gb.memory[0xFF04];

        // check for interrupts
        if gb.ime {
            let interrupt_flags: u8 = gb.memory[0xFF0F];
            let interrupt_enable: u8 = gb.memory[0xFFFF];
            let interrupt_request: u8 = interrupt_flags & interrupt_enable;
            // handle vblank interrupt
            if interrupt_request & 0b00000001 != 0 {
                println!(">>>> VBLANK INTERRUPT");
                gb.ime = false;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc & 0xFF) as u8;
                gb.pc = 0x40;
                gb.memory[0xFF0F] &= !0b00000001;
            }
            // handle lcd interrupt
            else if interrupt_request & 0b00000010 != 0 {
                println!(">>>> LCD INTERRUPT");
                gb.ime = false;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc & 0xFF) as u8;
                gb.pc = 0x48;
                gb.memory[0xFF0F] &= !0b00000010;
            }
            // handle timer interrupt
            else if interrupt_request & 0b00000100 != 0 {
                println!(">>>> TIMER INTERRUPT");
                gb.ime = false;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc & 0xFF) as u8;
                gb.pc = 0x50;
                gb.memory[0xFF0F] &= !0b00000100;
            }
            // handle serial interrupt
            if interrupt_request & 0b00001000 != 0 {
                println!(">>>> SERIAL INTERRUPT");
                gb.ime = false;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc & 0xFF) as u8;
                gb.pc = 0x58;
                gb.memory[0xFF0F] &= !0b00001000;
            }
            // handle joypad interrupt
            else if interrupt_request & 0b00010000 != 0 {
                println!(">>>> JOYPAD INTERRUPT");
                gb.ime = false;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (gb.pc & 0xFF) as u8;
                gb.pc = 0x60;
                gb.memory[0xFF0F] &= !0b00010000;
            }
        }

        let op: u8 = gb.read_byte();

        match op {
            0x00 => {
                debug_print_op(op, "NOP", &gb);
                gb.cycles += 1;
            }
            0x01 => {
                debug_print_op(op, "LD BC, nn", &gb);
                let (_nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.b = msb;
                gb.registers.c = lsb;
                gb.cycles += 3;
            }
            0x02 => {
                debug_print_op(op, "LD (BC), A", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                gb.memory[bc as usize] = gb.registers.a;
                gb.cycles += 2;
            }
            0x03 => {
                debug_print_op(op, "INC BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                let result = bc.wrapping_add(1);
                gb.registers.b = (result >> 8) as u8;
                gb.registers.c = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x04 => {
                debug_print_op(op, "INC B", &gb);
                let result = gb.registers.b.wrapping_add(1);
                gb.registers.b = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.b & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x05 => {
                debug_print_op(op, "DEC B", &gb);
                let result = gb.registers.b.wrapping_sub(1);
                gb.registers.b = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.b & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x06 => {
                debug_print_op(op, "LD B, n", &gb);
                gb.registers.b = gb.read_byte();
                gb.cycles += 2;
            }
            0x07 => {
                debug_print_op(op, "RLCA", &gb);
                let carry = gb.registers.a & 0x80;
                gb.registers.a = (gb.registers.a << 1) | (carry >> 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
                gb.cycles += 1;
            }
            0x08 => {
                debug_print_op(op, "LD (nn), SP", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.memory[nn as usize] = (gb.sp & 0xFF) as u8;
                gb.memory[(nn + 1) as usize] = (gb.sp >> 8) as u8;
                gb.cycles += 5;
            }
            0x09 => {
                debug_print_op(op, "ADD HL, BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(bc);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x0A => {
                debug_print_op(op, "LD A, (BC)", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                gb.registers.a = gb.fetch_byte(bc);
                gb.cycles += 2;
            }
            0x0B => {
                debug_print_op(op, "DEC BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                let result = bc.wrapping_sub(1);
                gb.registers.b = (result >> 8) as u8;
                gb.registers.c = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x0C => {
                debug_print_op(op, "INC C", &gb);
                let result = gb.registers.c.wrapping_add(1);
                gb.registers.c = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.c & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x0D => {
                debug_print_op(op, "DEC C", &gb);
                let result = gb.registers.c.wrapping_sub(1);
                gb.registers.c = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.c & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x0E => {
                debug_print_op(op, "LD C, n", &gb);
                gb.registers.c = gb.read_byte();
                gb.cycles += 2;
            }
            0x0F => {
                debug_print_op(op, "RRCA", &gb);
                let carry = gb.registers.a & 0x01;
                gb.registers.a = (gb.registers.a >> 1) | (carry << 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
                gb.cycles += 1;
            }
            0x11 => {
                debug_print_op(op, "LD DE, nn", &gb);
                let (_nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.d = msb;
                gb.registers.e = lsb;
                gb.cycles += 3;
            }
            0x12 => {
                debug_print_op(op, "LD (DE), A", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.memory[de as usize] = gb.registers.a;
                gb.cycles += 2;
            }
            0x13 => {
                debug_print_op(op, "INC DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                let result = de.wrapping_add(1);
                gb.registers.d = (result >> 8) as u8;
                gb.registers.e = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x14 => {
                debug_print_op(op, "INC D", &gb);
                let result = gb.registers.d.wrapping_add(1);
                gb.registers.d = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.d & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x15 => {
                debug_print_op(op, "DEC D", &gb);
                let result = gb.registers.d.wrapping_sub(1);
                gb.registers.d = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.d & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x16 => {
                debug_print_op(op, "LD D, n", &gb);
                gb.registers.d = gb.read_byte();
                gb.cycles += 2;
            }
            0x17 => {
                debug_print_op(op, "RLA", &gb);
                let carry = gb.registers.a & 0x80;
                gb.registers.a = (gb.registers.a << 1) | (carry >> 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
                gb.cycles += 1;
            }
            0x18 => {
                debug_print_op(op, "JR e", &gb);
                let e = gb.read_byte() as i8;
                gb.pc = gb.pc.wrapping_add(e as u16);
                gb.cycles += 3;
            }
            0x19 => {
                debug_print_op(op, "ADD HL, DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(de);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x1A => {
                debug_print_op(op, "LD A, (DE)", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.registers.a = gb.fetch_byte(de);
                gb.cycles += 2;
            }
            0x1B => {
                debug_print_op(op, "DEC DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                let result = de.wrapping_sub(1);
                gb.registers.d = (result >> 8) as u8;
                gb.registers.e = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x1C => {
                debug_print_op(op, "INC E", &gb);
                let result = gb.registers.e.wrapping_add(1);
                gb.registers.e = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.e & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x1D => {
                debug_print_op(op, "DEC E", &gb);
                let result = gb.registers.e.wrapping_sub(1);
                gb.registers.e = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.e & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x1E => {
                debug_print_op(op, "LD E, n", &gb);
                gb.registers.e = gb.read_byte();
                gb.cycles += 2;
            }
            0x1F => {
                debug_print_op(op, "RRA", &gb);
                let carry = gb.registers.a & 0x01;
                gb.registers.a = (gb.registers.a >> 1) | (carry << 7);
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if carry == 1 { 0b00010000 } else { 0 };
                gb.cycles += 1;
            }
            0x20 => {
                debug_print_op(op, "JR NZ, e", &gb);
                let e = gb.read_byte() as i8;
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = gb.pc.wrapping_add(e as u16);
                    gb.cycles += 1;
                }
                gb.cycles += 2;
            }
            0x21 => {
                debug_print_op(op, "LD HL, nn", &gb);
                let (_nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.h = msb;
                gb.registers.l = lsb;
                gb.cycles += 3;
            }
            0x22 => {
                debug_print_op(op, "LD (HL+), A", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.a;
                gb.registers.l = gb.registers.l.wrapping_add(1);
                if gb.registers.l == 0 {
                    gb.registers.h = gb.registers.h.wrapping_add(1);
                }
                gb.cycles += 2;
            }
            0x23 => {
                debug_print_op(op, "INC HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(1);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x24 => {
                debug_print_op(op, "INC H", &gb);
                let result = gb.registers.h.wrapping_add(1);
                gb.registers.h = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.h & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x25 => {
                debug_print_op(op, "DEC H", &gb);
                let result = gb.registers.h.wrapping_sub(1);
                gb.registers.h = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.h & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x26 => {
                debug_print_op(op, "LD H, n", &gb);
                gb.registers.h = gb.read_byte();
                gb.cycles += 2;
            }
            0x27 => {
                // NOTE: DAA taken from https://forums.nesdev.org/viewtopic.php?t=15944
                debug_print_op(op, "DAA", &gb);
                let mut a = gb.registers.a;
                let n_flag = gb.registers.f & 0b01000000 != 0;
                let c_flag = gb.registers.f & 0b00010000 != 0;
                let h_flag = gb.registers.f & 0b00100000 != 0;
                if !n_flag {
                    if c_flag || a > 0x99 {
                        a = a.wrapping_add(0x60);
                        gb.registers.f |= 0b00010000;
                    }
                    if h_flag || (a & 0x0F) > 0x09 {
                        a = a.wrapping_add(0x06);
                    }
                } else {
                    if c_flag {
                        a = a.wrapping_sub(0x60);
                    }
                    if h_flag {
                        a = a.wrapping_sub(0x06);
                    }
                }
                gb.registers.a = a;
                gb.registers.f = if a == 0 { 0b10000000 } else { 0 };
                gb.registers.f &= !0b00100000;
                gb.cycles += 1;
            }
            0x28 => {
                debug_print_op(op, "JR Z, n", &gb);
                let n = gb.read_byte();
                if gb.registers.f & 0b10000000 != 0 {
                    gb.pc = gb.pc.wrapping_add(n as u16);
                }
                gb.cycles += 2;
            }
            0x29 => {
                debug_print_op(op, "ADD HL, HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(hl);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x2A => {
                debug_print_op(op, "LD A, (HL+)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.a = gb.fetch_byte(hl);
                gb.registers.l = gb.registers.l.wrapping_add(1);
                if gb.registers.l == 0 {
                    gb.registers.h = gb.registers.h.wrapping_add(1);
                }
                gb.cycles += 2;
            }
            0x2B => {
                debug_print_op(op, "DEC HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_sub(1);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x2C => {
                debug_print_op(op, "INC L", &gb);
                let result = gb.registers.l.wrapping_add(1);
                gb.registers.l = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.l & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x2D => {
                debug_print_op(op, "DEC L", &gb);
                let result = gb.registers.l.wrapping_sub(1);
                gb.registers.l = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.l & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x2E => {
                debug_print_op(op, "LD L, n", &gb);
                gb.registers.l = gb.read_byte();
                gb.cycles += 2;
            }
            0x2F => {
                debug_print_op(op, "CPL", &gb);
                gb.registers.a = !gb.registers.a;
                gb.registers.f |= 0b01100000;
                gb.cycles += 1;
            }
            0x30 => {
                debug_print_op(op, "JR NC, e", &gb);
                let e = gb.read_byte() as i8;
                if gb.registers.f & 0b00010000 == 0 {
                    gb.pc = gb.pc.wrapping_add(e as u16);
                    gb.cycles += 1;
                }
                gb.cycles += 2;
            }
            0x31 => {
                debug_print_op(op, "LD SP, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.sp = nn;
                gb.cycles += 3;
            }
            0x32 => {
                debug_print_op(op, "LD (HL-), A", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.a;
                gb.registers.l = gb.registers.l.wrapping_sub(1);
                if gb.registers.l == 0xFF {
                    gb.registers.h = gb.registers.h.wrapping_sub(1);
                }
                gb.cycles += 2;
            }
            0x33 => {
                debug_print_op(op, "INC SP", &gb);
                gb.sp = gb.sp.wrapping_add(1);
                gb.cycles += 2;
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
                gb.cycles += 3;
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
                gb.cycles += 3;
            }
            0x36 => {
                debug_print_op(op, "LD (HL), n", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.read_byte();
                gb.cycles += 3;
            }
            0x37 => {
                debug_print_op(op, "SCF", &gb);
                gb.registers.f &= 0b10010000;
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x38 => {
                debug_print_op(op, "JR C, e", &gb);
                let e = gb.read_byte() as i8;
                if gb.registers.f & 0b00010000 != 0 {
                    gb.pc = gb.pc.wrapping_add(e as u16);
                    gb.cycles += 1;
                }
                gb.cycles += 2;
            }
            0x39 => {
                debug_print_op(op, "ADD HL, SP", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let result = hl.wrapping_add(gb.sp);
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.cycles += 2;
            }
            0x3A => {
                debug_print_op(op, "LD A, (HL-)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.a = gb.fetch_byte(hl);
                gb.registers.l = gb.registers.l.wrapping_sub(1);
                if gb.registers.l == 0xFF {
                    gb.registers.h = gb.registers.h.wrapping_sub(1);
                }
                gb.cycles += 2;
            }
            0x3B => {
                debug_print_op(op, "DEC SP", &gb);
                gb.sp = gb.sp.wrapping_sub(1);
                gb.cycles += 2;
            }
            0x3C => {
                debug_print_op(op, "INC A", &gb);
                let result = gb.registers.a.wrapping_add(1);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x3D => {
                debug_print_op(op, "DEC A", &gb);
                let result = gb.registers.a.wrapping_sub(1);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) == 0 {
                    0b10000000
                } else {
                    0
                };
                gb.registers.f |= 0b00010000;
                gb.cycles += 1;
            }
            0x3E => {
                debug_print_op(op, "LD A, n", &gb);
                gb.registers.a = gb.read_byte();
                gb.cycles += 2;
            }
            0x3F => {
                debug_print_op(op, "CCF", &gb);
                gb.registers.f &= 0b10010000;
                gb.registers.f |= if gb.registers.f & 0b00010000 == 0 {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x40 => {
                debug_print_op(op, "LD B, B", &gb);
                gb.registers.b = gb.registers.b;
                gb.cycles += 1;
            }
            0x41 => {
                debug_print_op(op, "LD B, C", &gb);
                gb.registers.b = gb.registers.c;
                gb.cycles += 1;
            }
            0x42 => {
                debug_print_op(op, "LD B, D", &gb);
                gb.registers.b = gb.registers.d;
                gb.cycles += 1;
            }
            0x43 => {
                debug_print_op(op, "LD B, E", &gb);
                gb.registers.b = gb.registers.e;
                gb.cycles += 1;
            }
            0x44 => {
                debug_print_op(op, "LD B, H", &gb);
                gb.registers.b = gb.registers.h;
                gb.cycles += 1;
            }
            0x45 => {
                debug_print_op(op, "LD B, L", &gb);
                gb.registers.b = gb.registers.l;
                gb.cycles += 1;
            }
            0x46 => {
                debug_print_op(op, "LD B, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.b = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x47 => {
                debug_print_op(op, "LD B, A", &gb);
                gb.registers.b = gb.registers.a;
                gb.cycles += 1;
            }
            0x48 => {
                debug_print_op(op, "LD C, B", &gb);
                gb.registers.c = gb.registers.b;
                gb.cycles += 1;
            }
            0x49 => {
                debug_print_op(op, "LD C, C", &gb);
                gb.registers.c = gb.registers.c;
                gb.cycles += 1;
            }
            0x4A => {
                debug_print_op(op, "LD C, D", &gb);
                gb.registers.c = gb.registers.d;
                gb.cycles += 1;
            }
            0x4B => {
                debug_print_op(op, "LD C, E", &gb);
                gb.registers.c = gb.registers.e;
                gb.cycles += 1;
            }
            0x4C => {
                debug_print_op(op, "LD C, H", &gb);
                gb.registers.c = gb.registers.h;
                gb.cycles += 1;
            }
            0x4D => {
                debug_print_op(op, "LD C, L", &gb);
                gb.registers.c = gb.registers.l;
                gb.cycles += 1;
            }
            0x4E => {
                debug_print_op(op, "LD C, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.c = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x4F => {
                debug_print_op(op, "LD C, A", &gb);
                gb.registers.c = gb.registers.a;
                gb.cycles += 1;
            }
            0x50 => {
                debug_print_op(op, "LD D, B", &gb);
                gb.registers.d = gb.registers.b;
                gb.cycles += 1;
            }
            0x51 => {
                debug_print_op(op, "LD D, C", &gb);
                gb.registers.d = gb.registers.c;
                gb.cycles += 1;
            }
            0x52 => {
                debug_print_op(op, "LD D, D", &gb);
                gb.registers.d = gb.registers.d;
                gb.cycles += 1;
            }
            0x53 => {
                debug_print_op(op, "LD D, E", &gb);
                gb.registers.d = gb.registers.e;
                gb.cycles += 1;
            }
            0x54 => {
                debug_print_op(op, "LD D, H", &gb);
                gb.registers.d = gb.registers.h;
                gb.cycles += 1;
            }
            0x55 => {
                debug_print_op(op, "LD D, L", &gb);
                gb.registers.d = gb.registers.l;
                gb.cycles += 1;
            }
            0x56 => {
                debug_print_op(op, "LD D, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.d = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x57 => {
                debug_print_op(op, "LD D, A", &gb);
                gb.registers.d = gb.registers.a;
                gb.cycles += 1;
            }
            0x58 => {
                debug_print_op(op, "LD E, B", &gb);
                gb.registers.e = gb.registers.b;
                gb.cycles += 1;
            }
            0x59 => {
                debug_print_op(op, "LD E, C", &gb);
                gb.registers.e = gb.registers.c;
                gb.cycles += 1;
            }
            0x5A => {
                debug_print_op(op, "LD E, D", &gb);
                gb.registers.e = gb.registers.d;
                gb.cycles += 1;
            }
            0x5B => {
                debug_print_op(op, "LD E, E", &gb);
                gb.registers.e = gb.registers.e;
                gb.cycles += 1;
            }
            0x5C => {
                debug_print_op(op, "LD E, H", &gb);
                gb.registers.e = gb.registers.h;
                gb.cycles += 1;
            }
            0x5D => {
                debug_print_op(op, "LD E, L", &gb);
                gb.registers.e = gb.registers.l;
                gb.cycles += 1;
            }
            0x5E => {
                debug_print_op(op, "LD E, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.e = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x5F => {
                debug_print_op(op, "LD E, A", &gb);
                gb.registers.e = gb.registers.a;
                gb.cycles += 1;
            }
            0x60 => {
                debug_print_op(op, "LD H, B", &gb);
                gb.registers.h = gb.registers.b;
                gb.cycles += 1;
            }
            0x61 => {
                debug_print_op(op, "LD H, C", &gb);
                gb.registers.h = gb.registers.c;
                gb.cycles += 1;
            }
            0x62 => {
                debug_print_op(op, "LD H, D", &gb);
                gb.registers.h = gb.registers.d;
                gb.cycles += 1;
            }
            0x63 => {
                debug_print_op(op, "LD H, E", &gb);
                gb.registers.h = gb.registers.e;
                gb.cycles += 1;
            }
            0x64 => {
                debug_print_op(op, "LD H, H", &gb);
                gb.registers.h = gb.registers.h;
                gb.cycles += 1;
            }
            0x65 => {
                debug_print_op(op, "LD H, L", &gb);
                gb.registers.h = gb.registers.l;
                gb.cycles += 1;
            }
            0x66 => {
                debug_print_op(op, "LD H, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.h = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x67 => {
                debug_print_op(op, "LD H, A", &gb);
                gb.registers.h = gb.registers.a;
                gb.cycles += 1;
            }
            0x68 => {
                debug_print_op(op, "LD L, B", &gb);
                gb.registers.l = gb.registers.b;
                gb.cycles += 1;
            }
            0x69 => {
                debug_print_op(op, "LD L, C", &gb);
                gb.registers.l = gb.registers.c;
                gb.cycles += 1;
            }
            0x6A => {
                debug_print_op(op, "LD L, D", &gb);
                gb.registers.l = gb.registers.d;
                gb.cycles += 1;
            }
            0x6B => {
                debug_print_op(op, "LD L, E", &gb);
                gb.registers.l = gb.registers.e;
                gb.cycles += 1;
            }
            0x6C => {
                debug_print_op(op, "LD L, H", &gb);
                gb.registers.l = gb.registers.h;
                gb.cycles += 1;
            }
            0x6D => {
                debug_print_op(op, "LD L, L", &gb);
                gb.registers.l = gb.registers.l;
                gb.cycles += 1;
            }
            0x6E => {
                debug_print_op(op, "LD L, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.l = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x6F => {
                debug_print_op(op, "LD L, A", &gb);
                gb.registers.l = gb.registers.a;
                gb.cycles += 1;
            }
            0x70 => {
                debug_print_op(op, "LD (HL), B", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.b;
                gb.cycles += 2;
            }
            0x71 => {
                debug_print_op(op, "LD (HL), C", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.c;
                gb.cycles += 2;
            }
            0x72 => {
                debug_print_op(op, "LD (HL), D", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.d;
                gb.cycles += 2;
            }
            0x73 => {
                debug_print_op(op, "LD (HL), E", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.e;
                gb.cycles += 2;
            }
            0x74 => {
                debug_print_op(op, "LD (HL), H", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.h;
                gb.cycles += 2;
            }
            0x75 => {
                debug_print_op(op, "LD (HL), L", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.l;
                gb.cycles += 2;
            }
            0x76 => {
                debug_print_op(op, "HALT", &gb);
                panic!();
            }
            0x77 => {
                debug_print_op(op, "LD (HL), A", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.memory[hl as usize] = gb.registers.a;
                gb.cycles += 2;
            }
            0x78 => {
                debug_print_op(op, "LD A, B", &gb);
                gb.registers.a = gb.registers.b;
                gb.cycles += 1;
            }
            0x79 => {
                debug_print_op(op, "LD A, C", &gb);
                gb.registers.a = gb.registers.c;
                gb.cycles += 1;
            }
            0x7A => {
                debug_print_op(op, "LD A, D", &gb);
                gb.registers.a = gb.registers.d;
                gb.cycles += 1;
            }
            0x7B => {
                debug_print_op(op, "LD A, E", &gb);
                gb.registers.a = gb.registers.e;
                gb.cycles += 1;
            }
            0x7C => {
                debug_print_op(op, "LD A, H", &gb);
                gb.registers.a = gb.registers.h;
                gb.cycles += 1;
            }
            0x7D => {
                debug_print_op(op, "LD A, L", &gb);
                gb.registers.a = gb.registers.l;
                gb.cycles += 1;
            }
            0x7E => {
                debug_print_op(op, "LD A, (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.registers.a = gb.fetch_byte(hl);
                gb.cycles += 2;
            }
            0x7F => {
                debug_print_op(op, "LD A, A", &gb);
                gb.registers.a = gb.registers.a;
                gb.cycles += 1;
            }
            0x80 => {
                debug_print_op(op, "ADD B", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.b);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.b {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x81 => {
                debug_print_op(op, "ADD C", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.c);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.c & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.c {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x82 => {
                debug_print_op(op, "ADD D", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.d);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.d & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.d {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x83 => {
                debug_print_op(op, "ADD E", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.e);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.e {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x84 => {
                debug_print_op(op, "ADD H", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.h);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.h & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.h {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x85 => {
                debug_print_op(op, "ADD L", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.l);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.l & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.l {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x86 => {
                debug_print_op(op, "ADD (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                let result = gb.registers.a.wrapping_add(data);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0x87 => {
                debug_print_op(op, "ADD A", &gb);
                let result = gb.registers.a.wrapping_add(gb.registers.a);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.a {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x88 => {
                debug_print_op(op, "ADC B", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.b)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.b {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x89 => {
                debug_print_op(op, "ADC C", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.c)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.c & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.c {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x8A => {
                debug_print_op(op, "ADC D", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.d)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.d & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.d {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x8B => {
                debug_print_op(op, "ADC E", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.e)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.e {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x8C => {
                debug_print_op(op, "ADC H", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.h)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.h & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.h {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x8D => {
                debug_print_op(op, "ADC L", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.l)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.l & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.l {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x8E => {
                debug_print_op(op, "ADC (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb.registers.a.wrapping_add(data).wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0x8F => {
                debug_print_op(op, "ADC A", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_add(gb.registers.a)
                    .wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.a {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x90 => {
                debug_print_op(op, "SUB B", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.b);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.b {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x91 => {
                debug_print_op(op, "SUB C", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.c);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.c & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.c {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x92 => {
                debug_print_op(op, "SUB D", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.d);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.d & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.d {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x93 => {
                debug_print_op(op, "SUB E", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.e);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.e {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x94 => {
                debug_print_op(op, "SUB H", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.h);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.h & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.h {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x95 => {
                debug_print_op(op, "SUB L", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.l);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.l & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.l {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
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
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0x97 => {
                debug_print_op(op, "SUB A", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.a);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.a {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x98 => {
                debug_print_op(op, "SBC B", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.b)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.b {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x99 => {
                debug_print_op(op, "SBC C", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.c)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.c & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.c {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x9A => {
                debug_print_op(op, "SBC D", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.d)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.d & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.d {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x9B => {
                debug_print_op(op, "SBC E", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.e)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.e {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x9C => {
                debug_print_op(op, "SBC H", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.h)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.h & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.h {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x9D => {
                debug_print_op(op, "SBC L", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.l)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.l & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.l {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0x9E => {
                debug_print_op(op, "SBC (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb.registers.a.wrapping_sub(data).wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0x9F => {
                debug_print_op(op, "SBC A", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let result = gb
                    .registers
                    .a
                    .wrapping_sub(gb.registers.a)
                    .wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.a {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xA0 => {
                debug_print_op(op, "AND B", &gb);
                gb.registers.a &= gb.registers.b;
                gb.cycles += 1;
            }
            0xA1 => {
                debug_print_op(op, "AND C", &gb);
                gb.registers.a &= gb.registers.c;
                gb.cycles += 1;
            }
            0xA2 => {
                debug_print_op(op, "AND D", &gb);
                gb.registers.a &= gb.registers.d;
                gb.cycles += 1;
            }
            0xA3 => {
                debug_print_op(op, "AND E", &gb);
                gb.registers.a &= gb.registers.e;
                gb.cycles += 1;
            }
            0xA4 => {
                debug_print_op(op, "AND H", &gb);
                gb.registers.a &= gb.registers.h;
                gb.cycles += 1;
            }
            0xA5 => {
                debug_print_op(op, "AND L", &gb);
                gb.registers.a &= gb.registers.l;
                gb.cycles += 1;
            }
            0xA6 => {
                debug_print_op(op, "AND (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                gb.registers.a &= data;
                gb.cycles += 2;
            }
            0xA7 => {
                debug_print_op(op, "AND A", &gb);
                gb.registers.a &= gb.registers.a;
                gb.cycles += 1;
            }
            0xA8 => {
                debug_print_op(op, "XOR B", &gb);
                gb.registers.a ^= gb.registers.b;
                gb.cycles += 1;
            }
            0xA9 => {
                debug_print_op(op, "XOR C", &gb);
                gb.registers.a ^= gb.registers.c;
                gb.cycles += 1;
            }
            0xAA => {
                debug_print_op(op, "XOR D", &gb);
                gb.registers.a ^= gb.registers.d;
                gb.cycles += 1;
            }
            0xAB => {
                debug_print_op(op, "XOR E", &gb);
                gb.registers.a ^= gb.registers.e;
                gb.cycles += 1;
            }
            0xAC => {
                debug_print_op(op, "XOR H", &gb);
                gb.registers.a ^= gb.registers.h;
                gb.cycles += 1;
            }
            0xAD => {
                debug_print_op(op, "XOR L", &gb);
                gb.registers.a ^= gb.registers.l;
                gb.cycles += 1;
            }
            0xAE => {
                debug_print_op(op, "XOR (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                gb.registers.a ^= data;
                gb.cycles += 2;
            }
            0xAF => {
                debug_print_op(op, "XOR A", &gb);
                gb.registers.a ^= gb.registers.a;
                gb.cycles += 1;
            }
            0xB0 => {
                debug_print_op(op, "OR B", &gb);
                gb.registers.a |= gb.registers.b;
                gb.cycles += 1;
            }
            0xB1 => {
                debug_print_op(op, "OR C", &gb);
                gb.registers.a |= gb.registers.c;
                gb.cycles += 1;
            }
            0xB2 => {
                debug_print_op(op, "OR D", &gb);
                gb.registers.a |= gb.registers.d;
                gb.cycles += 1;
            }
            0xB3 => {
                debug_print_op(op, "OR E", &gb);
                gb.registers.a |= gb.registers.e;
                gb.cycles += 1;
            }
            0xB4 => {
                debug_print_op(op, "OR H", &gb);
                gb.registers.a |= gb.registers.h;
                gb.cycles += 1;
            }
            0xB5 => {
                debug_print_op(op, "OR L", &gb);
                gb.registers.a |= gb.registers.l;
                gb.cycles += 1;
            }
            0xB6 => {
                debug_print_op(op, "OR (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                gb.registers.a |= data;
                gb.cycles += 2;
            }
            0xB7 => {
                debug_print_op(op, "OR A", &gb);
                gb.registers.a |= gb.registers.a;
                gb.cycles += 1;
            }
            0xB8 => {
                debug_print_op(op, "CP B", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.b);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.b & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.b {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xB9 => {
                debug_print_op(op, "CP C", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.c);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.c & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.c {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xBA => {
                debug_print_op(op, "CP D", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.d);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.d & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.d {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xBB => {
                debug_print_op(op, "CP E", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.e);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.e & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.e {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xBC => {
                debug_print_op(op, "CP H", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.h);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.h & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.h {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xBD => {
                debug_print_op(op, "CP L", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.l);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.l & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.l {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xBE => {
                debug_print_op(op, "CP (HL)", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                let data = gb.fetch_byte(hl);
                let result = gb.registers.a.wrapping_sub(data);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (data & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < data { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0xBF => {
                debug_print_op(op, "CP A", &gb);
                let result = gb.registers.a.wrapping_sub(gb.registers.a);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (gb.registers.a & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < gb.registers.a {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 1;
            }
            0xC0 => {
                debug_print_op(op, "RET NZ", &gb);
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = gb.read_nn_from_stack();
                    gb.cycles += 3;
                }
                gb.cycles += 2;
            }
            0xC1 => {
                debug_print_op(op, "POP BC", &gb);
                let bc = gb.read_nn_from_stack();
                gb.registers.b = (bc >> 8) as u8;
                gb.registers.c = (bc & 0xFF) as u8;
                gb.cycles += 3;
            }
            0xC2 => {
                debug_print_op(op, "JP NZ, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b10000000 == 0 {
                    gb.pc = nn;
                    gb.cycles += 1;
                }
                gb.cycles += 3;
            }
            0xC3 => {
                debug_print_op(op, "JP nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.pc = nn;
                gb.cycles += 4;
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
                    gb.cycles += 3;
                }
                gb.cycles += 3;
            }
            0xC5 => {
                debug_print_op(op, "PUSH BC", &gb);
                let bc = ((gb.registers.b as u16) << 8) | (gb.registers.c as u16);
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (bc >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (bc & 0xFF) as u8;
                gb.cycles += 4;
            }
            0xC6 => {
                debug_print_op(op, "ADD n", &gb);
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_add(n);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0xC7 => {
                debug_print_op(op, "RST 00H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0000;
                gb.cycles += 4;
            }
            0xC8 => {
                debug_print_op(op, "RET Z", &gb);
                if gb.registers.f & 0b10000000 != 0 {
                    gb.pc = gb.read_nn_from_stack();
                    gb.cycles += 3;
                }
                gb.cycles += 2;
            }
            0xC9 => {
                debug_print_op(op, "RET", &gb);
                gb.pc = gb.read_nn_from_stack();
                gb.cycles += 4;
            }
            0xCA => {
                debug_print_op(op, "JP Z, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b10000000 != 0 {
                    gb.pc = nn;
                    gb.cycles += 1;
                }
                gb.cycles += 3;
            }
            0xCB => {
                gb.cycles += 1;
                let cb_op = gb.read_byte();

                match cb_op {
                    0x00 => {
                        debug_print_op(op, "RLC B", &gb);
                        let carry = gb.registers.b >> 7;
                        let result = (gb.registers.b << 1) | carry;
                        gb.registers.b = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if carry != 0 { 0b00010000 } else { 0 };
                        gb.cycles += 1;
                    }
                    0x10 => {
                        debug_print_op(op, "RL B", &gb);
                        let result = gb.registers.b << 1;
                        gb.registers.b = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.b & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x20 => {
                        debug_print_op(op, "SLA B", &gb);
                        let result = gb.registers.b << 1;
                        gb.registers.b = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.b & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x21 => {
                        debug_print_op(op, "SLA C", &gb);
                        let result = gb.registers.c << 1;
                        gb.registers.c = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.c & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x22 => {
                        debug_print_op(op, "SLA D", &gb);
                        let result = gb.registers.d << 1;
                        gb.registers.d = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.d & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x23 => {
                        debug_print_op(op, "SLA E", &gb);
                        let result = gb.registers.e << 1;
                        gb.registers.e = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.e & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x24 => {
                        debug_print_op(op, "SLA H", &gb);
                        let result = gb.registers.h << 1;
                        gb.registers.h = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.h & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x25 => {
                        debug_print_op(op, "SLA L", &gb);
                        let result = gb.registers.l << 1;
                        gb.registers.l = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.l & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x26 => {
                        debug_print_op(op, "SLA (HL)", &gb);
                        let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                        let data = gb.fetch_byte(hl);
                        let result = data << 1;
                        gb.memory[hl as usize] = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (data & 0x80) != 0 { 0b00100000 } else { 0 };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 3;
                    }
                    0x27 => {
                        debug_print_op(op, "SLA A", &gb);
                        let result = gb.registers.a << 1;
                        gb.registers.a = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.a & 0x80) != 0 {
                            0b00100000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00010000;
                        gb.cycles += 1;
                    }
                    0x28 => {
                        debug_print_op(op, "SRA B", &gb);
                        let result = (gb.registers.b >> 1) | (gb.registers.b & 0x80);
                        gb.registers.b = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.b & 0x01) != 0 {
                            0b00010000
                        } else {
                            0
                        };
                        gb.cycles += 1;
                    }
                    0x29 => {
                        debug_print_op(op, "SRA C", &gb);
                        let result = (gb.registers.c >> 1) | (gb.registers.c & 0x80);
                        gb.registers.c = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if (gb.registers.c & 0x01) != 0 {
                            0b00010000
                        } else {
                            0
                        };
                        gb.cycles += 1;
                    }
                    0x30 => {
                        debug_print_op(op, "SWAP B", &gb);
                        let result =
                            ((gb.registers.b & 0x0F) << 4) | ((gb.registers.b & 0xF0) >> 4);
                        gb.registers.b = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.cycles += 1;
                    }
                    0x3D => {
                        debug_print_op(op, "SRL L", &gb);
                        let carry = gb.registers.l & 0x01;
                        let result = gb.registers.l >> 1;
                        gb.registers.l = result;
                        gb.registers.f = 0;
                        gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                        gb.registers.f |= 0b01000000;
                        gb.registers.f |= if carry != 0 { 0b00010000 } else { 0 };
                        gb.cycles += 1;
                    }
                    0x40 => {
                        debug_print_op(op, "BIT 0, B", &gb);
                        gb.registers.f &= 0b10010000;
                        gb.registers.f |= if (gb.registers.b & 0x01) == 0 {
                            0b10000000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00100000;
                        gb.cycles += 1;
                    }
                    0x45 => {
                        debug_print_op(op, "BIT 0, L", &gb);
                        gb.registers.f &= 0b10010000;
                        gb.registers.f |= if (gb.registers.l & 0x01) == 0 {
                            0b10000000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00100000;
                        gb.cycles += 1;
                    }
                    0x50 => {
                        debug_print_op(op, "BIT 2, B", &gb);
                        gb.registers.f &= 0b10010000;
                        gb.registers.f |= if (gb.registers.b & 0x04) == 0 {
                            0b10000000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00100000;
                        gb.cycles += 1;
                    }
                    0x51 => {
                        debug_print_op(op, "BIT 2, C", &gb);
                        gb.registers.f &= 0b10010000;
                        gb.registers.f |= if (gb.registers.c & 0x04) == 0 {
                            0b10000000
                        } else {
                            0
                        };
                        gb.registers.f |= 0b00100000;
                        gb.cycles += 1;
                    }
                    _ => {
                        panic!("unknown CB opcode: 0x{:02X}", cb_op);
                    }
                }
            }
            0xCC => {
                debug_print_op(op, "CALL Z, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b10000000 != 0 {
                    let pc_msb = gb.pc >> 8;
                    let pc_lsb = gb.pc & 0xFF;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_msb as u8;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_lsb as u8;
                    gb.pc = nn;
                    gb.cycles += 3;
                }
                gb.cycles += 3;
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
                gb.cycles += 6;
            }
            0xCE => {
                debug_print_op(op, "ADC n", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_add(n).wrapping_add(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0xCF => {
                debug_print_op(op, "RST 08H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0008;
                gb.cycles += 4;
            }
            0xD0 => {
                debug_print_op(op, "RET NC", &gb);
                if gb.registers.f & 0b00010000 == 0 {
                    gb.pc = gb.read_nn_from_stack();
                    gb.cycles += 3;
                }
                gb.cycles += 2;
            }
            0xD1 => {
                debug_print_op(op, "POP DE", &gb);
                let de = gb.read_nn_from_stack();
                gb.registers.d = (de >> 8) as u8;
                gb.registers.e = (de & 0xFF) as u8;
                gb.cycles += 3;
            }
            0xD2 => {
                debug_print_op(op, "JP NC, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b00010000 == 0 {
                    gb.pc = nn;
                    gb.cycles += 1;
                }
                gb.cycles += 3;
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
                    gb.cycles += 3;
                }
                gb.cycles += 3;
            }
            0xD5 => {
                debug_print_op(op, "PUSH DE", &gb);
                let de = ((gb.registers.d as u16) << 8) | (gb.registers.e as u16);
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (de >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (de & 0xFF) as u8;
                gb.cycles += 4;
            }
            0xD6 => {
                debug_print_op(op, "SUB n", &gb);
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_sub(n);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0xD7 => {
                debug_print_op(op, "RST 10H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0010;
                gb.cycles += 4;
            }
            0xD8 => {
                debug_print_op(op, "RET C", &gb);
                if gb.registers.f & 0b00010000 != 0 {
                    gb.pc = gb.read_nn_from_stack();
                    gb.cycles += 3;
                }
                gb.cycles += 2;
            }
            0xD9 => {
                debug_print_op(op, "RETI", &gb);
                gb.pc = gb.read_nn_from_stack();
                gb.ime = true;
                gb.cycles += 4;
            }
            0xDA => {
                debug_print_op(op, "JP C, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b00010000 != 0 {
                    gb.pc = nn;
                    gb.cycles += 1;
                }
                gb.cycles += 3;
            }
            0xDC => {
                debug_print_op(op, "CALL C, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                if gb.registers.f & 0b00010000 != 0 {
                    let pc_msb = gb.pc >> 8;
                    let pc_lsb = gb.pc & 0xFF;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_msb as u8;
                    gb.sp -= 1;
                    gb.memory[gb.sp as usize] = pc_lsb as u8;
                    gb.pc = nn;
                    gb.cycles += 3;
                }
                gb.cycles += 3;
            }
            0xDE => {
                debug_print_op(op, "SBC n", &gb);
                let carry = if gb.registers.f & 0b00010000 != 0 {
                    1
                } else {
                    0
                };
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_sub(n).wrapping_sub(carry);
                gb.registers.a = result;
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
                gb.cycles += 2;
            }
            0xDF => {
                debug_print_op(op, "RST 18H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0018;
                gb.cycles += 4;
            }
            0xE0 => {
                debug_print_op(op, "LDH (n), A", &gb);
                let n = gb.read_byte();
                gb.memory[0xFF00 + n as usize] = gb.registers.a;
                gb.cycles += 3;
                println!(
                    ">>>> HARDWARE WRITE: 0x{:02X} 0x{:02X}",
                    (0xFF00 as u16 + n as u16),
                    gb.registers.a
                );
            }
            0xE1 => {
                debug_print_op(op, "POP HL", &gb);
                let hl = gb.read_nn_from_stack();
                gb.registers.h = (hl >> 8) as u8;
                gb.registers.l = (hl & 0xFF) as u8;
                gb.cycles += 3;
            }
            0xE2 => {
                debug_print_op(op, "LDH (C), A", &gb);
                gb.memory[0xFF00 + gb.registers.c as usize] = gb.registers.a;
                gb.cycles += 2;
                println!(
                    ">>>> HARDWARE WRITE: 0x{:02X} 0x{:02X}",
                    (0xFF00 as u16 + gb.registers.c as u16),
                    gb.registers.a
                );
            }
            0xE5 => {
                debug_print_op(op, "PUSH HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (hl >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (hl & 0xFF) as u8;
                gb.cycles += 4;
            }
            0xE6 => {
                debug_print_op(op, "AND n", &gb);
                let n = gb.read_byte();
                gb.registers.a &= n;
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b00100000;
                gb.registers.f |= 0b00100000;
                gb.cycles += 2;
            }
            0xE7 => {
                debug_print_op(op, "RST 20H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0020;
                gb.cycles += 4;
            }
            0xE8 => {
                debug_print_op(op, "ADD SP, e", &gb);
                let e = gb.read_byte() as i8 as i16;
                let sp = gb.sp as i16;
                let result = sp.wrapping_add(e);
                gb.sp = result as u16;
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.registers.f = 0;
                gb.registers.f |= if (sp & 0xF) + (e & 0xF) > 0xF {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if (sp & 0xFF) + (e & 0xFF) > 0xFF {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 4;
            }
            0xE9 => {
                debug_print_op(op, "JP HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.pc = hl;
                gb.cycles += 1;
            }
            0xEA => {
                debug_print_op(op, "LD (nn), A", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.memory[nn as usize] = gb.registers.a;
                gb.cycles += 4;
            }
            0xEE => {
                debug_print_op(op, "XOR n", &gb);
                let n = gb.read_byte();
                gb.registers.a ^= n;
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.cycles += 2;
            }
            0xEF => {
                debug_print_op(op, "RST 28H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0028;
                gb.cycles += 4;
            }
            0xF0 => {
                debug_print_op(op, "LDH A, (n)", &gb);
                let n = gb.read_byte();
                gb.registers.a = gb.memory[0xFF00 + n as usize];
                gb.cycles += 3;
                println!(
                    ">>>> HARDWARE READ: 0x{:02X} 0x{:02X}",
                    (0xFF00 as u16 + n as u16),
                    gb.registers.a
                );
            }
            0xF1 => {
                debug_print_op(op, "POP AF", &gb);
                let af = gb.read_nn_from_stack();
                gb.registers.a = (af >> 8) as u8;
                gb.registers.f = (af & 0xFF) as u8;
                gb.cycles += 3;
            }
            0xF2 => {
                debug_print_op(op, "LD A, (C)", &gb);
                gb.registers.a = gb.memory[0xFF00 + gb.registers.c as usize];
                gb.cycles += 2;
                println!(
                    ">>>> HARDWARE READ: 0x{:02X} 0x{:02X}",
                    (0xFF00 as u16 + gb.registers.c as u16),
                    gb.registers.a
                );
            }
            0xF3 => {
                debug_print_op(op, "DI", &gb);
                gb.ime = false;
                gb.cycles += 1;
            }
            0xF5 => {
                debug_print_op(op, "PUSH AF", &gb);
                let af = ((gb.registers.a as u16) << 8) | (gb.registers.f as u16);
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (af >> 8) as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = (af & 0xFF) as u8;
                gb.cycles += 4;
            }
            0xF6 => {
                debug_print_op(op, "OR n", &gb);
                let n = gb.read_byte();
                gb.registers.a |= n;
                gb.registers.f = 0;
                gb.registers.f |= if gb.registers.a == 0 { 0b10000000 } else { 0 };
                gb.cycles += 2;
            }
            0xF7 => {
                debug_print_op(op, "RST 30H", &gb);
                let pc_msb = gb.pc >> 8;
                let pc_lsb = gb.pc & 0xFF;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_msb as u8;
                gb.sp -= 1;
                gb.memory[gb.sp as usize] = pc_lsb as u8;
                gb.pc = 0x0030;
                gb.cycles += 4;
            }
            0xF8 => {
                debug_print_op(op, "LD HL, SP+e", &gb);
                let e = gb.read_byte() as i8 as i16;
                let sp = gb.sp as i16;
                let result = sp.wrapping_add(e);
                gb.sp = result as u16;
                gb.registers.h = (result >> 8) as u8;
                gb.registers.l = (result & 0xFF) as u8;
                gb.registers.f = 0;
                gb.registers.f |= if (sp & 0xF) + (e & 0xF) > 0xF {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if (sp & 0xFF) + (e & 0xFF) > 0xFF {
                    0b00010000
                } else {
                    0
                };
                gb.cycles += 3;
            }
            0xF9 => {
                debug_print_op(op, "LD SP, HL", &gb);
                let hl = ((gb.registers.h as u16) << 8) | (gb.registers.l as u16);
                gb.sp = hl;
                gb.cycles += 2;
            }
            0xFA => {
                debug_print_op(op, "LD A, (nn)", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.registers.a = gb.memory[nn as usize];
                gb.cycles += 4;
            }
            0xFB => {
                debug_print_op(op, "EI", &gb);
                gb.ime = true;
                gb.cycles += 1;
            }
            0xFE => {
                debug_print_op(op, "CP n", &gb);
                let n = gb.read_byte();
                let result = gb.registers.a.wrapping_sub(n);
                gb.registers.f = 0;
                gb.registers.f |= if result == 0 { 0b10000000 } else { 0 };
                gb.registers.f |= 0b01000000;
                gb.registers.f |= if (gb.registers.a & 0xF) < (n & 0xF) {
                    0b00100000
                } else {
                    0
                };
                gb.registers.f |= if gb.registers.a < n { 0b00010000 } else { 0 };
                gb.cycles += 2;
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
                gb.cycles += 4;
            }
            _ => {
                panic!("unknown opcode: 0x{:02X}", op);
            }
        }

        let cycles_elapsed: u128 = gb.cycles - cycles_start;
        gb.div_cycles += cycles_elapsed;

        // handle DIV register every 256 cycles
        let div = gb.memory[0xFF04];
        // check if div was written to
        if div != div_initial {
            gb.memory[0xFF04] = 0x00;
        }
        if gb.div_cycles >= 256 {
            gb.memory[0xFF04] = gb.memory[0xFF04].wrapping_add(1);
            gb.div_cycles -= 256;
        }

        // handle TIMA register
        let tac = gb.memory[0xFF07];
        if tac & 0b00000100 != 0 {
            let cycles_per_tick = match tac & 0b00000011 {
                0b00 => 1024,
                0b01 => 16,
                0b10 => 64,
                0b11 => 256,
                _ => panic!("invalid TAC value: 0x{:02X}", tac),
            };
            gb.tima_cycles += cycles_elapsed;
            if gb.tima_cycles >= cycles_per_tick {
                gb.tima_cycles -= cycles_per_tick;
                gb.memory[0xFF05] = gb.memory[0xFF05].wrapping_add(1);
                if gb.memory[0xFF05] == 0x00 {
                    gb.memory[0xFF05] = gb.memory[0xFF06]; // reload TIMA with TMA
                    gb.memory[0xFF0F] |= 0b00000100; // set TIMA overflow flag
                }
            }
        }

        // std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
