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
                a: 0,
                b: 0,
                c: 0,
                d: 0,
                e: 0,
                f: 0,
                h: 0,
                l: 0,
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
    state.dump_registers();
}

fn main() -> ! {
    let rom_file: &str = "./rom/snake.gb";
    let rom: Vec<u8> = std::fs::read(rom_file).unwrap();
    let mut gb = GameBoyState::new();
    gb.load_rom(rom);

    gb.dump_memory();
    gb.dump_registers();

    println!("Starting emulation loop");

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
            0x11 => {
                debug_print_op(op, "LD DE, nn", &gb);
                let (nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.d = msb;
                gb.registers.e = lsb;
            }
            0x21 => {
                debug_print_op(op, "LD HL, nn", &gb);
                let (nn, lsb, msb) = gb.read_nn_lsb_msb();
                gb.registers.h = msb;
                gb.registers.l = lsb;
            }
            0x31 => {
                debug_print_op(op, "LD SP, nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.sp = nn;
            }
            0x3E => {
                debug_print_op(op, "LD A, n", &gb);
                gb.registers.a = gb.read_byte();
            }
            0xC3 => {
                debug_print_op(op, "JP nn", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.pc = nn;
            }
            0xC9 => {
                debug_print_op(op, "RET", &gb);
                let pc_lsb = gb.memory[gb.sp as usize];
                gb.sp += 1;
                let pc_msb = gb.memory[gb.sp as usize];
                gb.sp += 1;
                gb.pc = ((pc_msb as u16) << 8) | (pc_lsb as u16);
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
            }
            0xEA => {
                debug_print_op(op, "LD (nn), A", &gb);
                let (nn, _lsb, _msb) = gb.read_nn_lsb_msb();
                gb.memory[nn as usize] = gb.registers.a;
            }
            _ => {
                panic!("unknown opcode: 0x{:02X}", op);
            }
        }
    }
}
