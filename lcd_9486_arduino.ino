#define RD_PORT PORTC
#define RD_PIN  0
#define WR_PORT PORTC
#define WR_PIN  1
#define CD_PORT PORTC
#define CD_PIN  2
#define CS_PORT PORTC
#define CS_PIN  3
#define RESET_PORT PORTC
#define RESET_PIN  4

#define BMASK         0x03              //more intuitive style for mixed Ports
#define DMASK         0xfc              //does exactly the same as previous
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) & BMASK); PORTD = (PORTD & ~DMASK) | ((x) & DMASK); }
#define read_8()      ( (PINB & BMASK) | (PIND & DMASK) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x) >> 8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))

#define RD_ACTIVE    PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE      PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT    PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE    PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE      PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT    PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND   PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA      PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT    PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE    PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE      PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT    PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE   PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT PIN_OUTPUT(RESET_PORT, RESET_PIN)

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }       //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE      //PWLR=TRDL=150ns, tDDR=100ns

#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }

#define TFTLCD_DELAY8 0x7f

static void write_table(const void *table, int16_t size) {
  uint8_t *p = (uint8_t *) table;
  while (size > 0) {
    uint8_t cmd = pgm_read_byte(p++);
    uint8_t len = pgm_read_byte(p++);
    if (cmd == TFTLCD_DELAY8) {
      delay(len);
      len = 0;
    } else {
      CS_ACTIVE;
      CD_COMMAND;
      write8(cmd);
      for (uint8_t d = 0; d++ < len; ) {
        uint8_t x = pgm_read_byte(p++);
        CD_DATA;
        write8(x);
      }
         CS_IDLE;
      }
        size -= len + 2;
    }
}

void reset(void) {
  setWriteDir();
  CTL_INIT();
  CS_IDLE;
  RD_IDLE;
  WR_IDLE;
  RESET_IDLE;
  delay(50);
  RESET_ACTIVE;
  delay(100);
  RESET_IDLE;
  delay(100);
}

void begin() {
  static const uint8_t regs[] PROGMEM = {
    0xC0,   2,  0x0d, 0x0d,                // power control 1
    0xC1,   2,  0x43, 0x00,                // power control 2
    0xC2,   1,  0x00,                      // power control 3
    0xC5,   4,  0x00, 0x48, 0x00, 0x48,    // vcom control 1
    0xB4,   1,  0x00,                      // display inversion control
    0xB6,   3,  0x02, 0x02, 0x3B,          // display function control
    0xE0,  15,  0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, // positive gamma control
    0xE1,  15,  0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00, // negative gamma control
  };
  
  static const uint8_t t0[] PROGMEM = {
    0x3a,  1,  0x55,        // interface pixel format
    0xB6,  2,  0x00, 0x22,  // display function control
    0x36,  1,  0xa8,        // memory access control, rotation, 0x08, 0x68, 0xc8, 0xa8
    0x11,  0                // sleep out
  };
  
  static const uint8_t td[] PROGMEM = {
    0x2c, 50,  0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, // memory write
               0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8,
               0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8,
               0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8,
               0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8
  };
  
  static const uint8_t t1[] PROGMEM = {
    0x29,  0                // display on
  };
  
  write_table(&regs, sizeof(regs));
  write_table(&t0, sizeof(t0));
  write_table(&td, sizeof(td));
  write_table(&t1, sizeof(t1));
}
  
void setup() {
  reset();
  begin(); 
}

void loop() {
  delay(100);
}
