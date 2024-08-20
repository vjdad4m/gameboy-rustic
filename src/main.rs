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

    fn dump_registers(&self) {
        println!("A: {:02X}", self.registers.a);
        println!("B: {:02X}", self.registers.b);
        println!("C: {:02X}", self.registers.c);
        println!("D: {:02X}", self.registers.d);
        println!("E: {:02X}", self.registers.e);
        println!("F: {:02X}", self.registers.f);
        println!("H: {:02X}", self.registers.h);
        println!("L: {:02X}", self.registers.l);
    }

    fn dump_memory(&self) {
        for i in 0..0xFFFF {
            if (i+1) % 32 == 0 {
                println!();
            }
            print!("{:02X} ", self.memory[i]);
        }
        println!();
    }
}

fn main() {
    let rom_file: &str = "./rom/gb-test-roms/cpu_instrs/cpu_instrs.gb";
    let rom: Vec<u8> = std::fs::read(rom_file).unwrap();
    let mut gb = GameBoyState::new();
    gb.load_rom(rom);

    gb.dump_memory();
    gb.dump_registers();

    println!("Starting emulation loop");

    loop {
        let op: u8 = gb.fetch_byte(gb.pc);
        gb.pc += 1;

        match op {
            0x00 => {
                println!("NOP");
            }
            _ => {
                panic!("Unknown opcode: {:02X}", op);
            }
        }
    }
}
