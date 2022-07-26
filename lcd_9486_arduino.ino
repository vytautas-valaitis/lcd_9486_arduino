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
#define DMASK         0xFC              //does exactly the same as previous
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) & BMASK); PORTD = (PORTD & ~DMASK) | ((x) & DMASK); }
#define read_8()      ( (PINB & BMASK) | (PIND & DMASK) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
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
#define WriteCmd(x)  { CD_COMMAND; write16(x); CD_DATA; }
#define WriteData(x) { write16(x); }

#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

#define TFTLCD_DELAY8 0x7F

int16_t WIDTH = 480;
int16_t HEIGHT = 320;

//int w = 480, h = 320;
//uint32_t bytes = w * h;
//uint8_t *buffer = (uint8_t *)malloc(bytes);
//memset(buffer, 0, bytes);


static void WriteCmdData(uint16_t cmd, uint16_t dat) {
  CS_ACTIVE;
  WriteCmd(cmd);
  WriteData(dat);
  CS_IDLE;
}

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block) {
  CS_ACTIVE;
  WriteCmd(cmd);
  while (N-- > 0) {
    uint8_t u8 = *block++;
      write8(u8);
    }
  CS_IDLE;
}

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


void writePixel(int16_t x, int16_t y, uint16_t color) {
//  setAddrWindow(x, y, x, y);
//  WriteCmdData(_MW, color);
}

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1) {
//      WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
//        WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);
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
    0xC0,   2,  0x0d, 0x0d,                // power control 1 [0e 0e]
    0xC1,   2,  0x43, 0x00,                // power control 2 [43 00]
    0xC2,   1,  0x00,                      // power control 3 [33]
    0xC5,   4,  0x00, 0x48, 0x00, 0x48,    // vcom control 1 [00 40 00 40]
    0xB4,   1,  0x00,                      // inversion Control [00]
    0xB6,   3,  0x02, 0x02, 0x3B,          // display function control [02 02 3b]
    0xE0,  15,  0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, // gamma
    0xE1,  15,  0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00, // gamma
  };
  
  /*Waveshare_ILI9486.cpp
  static const uint8_t regs[] PROGMEM ={ 
    0xC0,  2,  0x19, 0x1a,
    0xC1,  1,  0x45, 0x00,
    0xC2,  4,  0x33,               //  Power/Reset on default
    0xC5,  1,  0x00, 0x28,         //  VCOM control
    0xB1,  3,  0xA0, 0x11,         //  Frame rate control
    0xB4,  1,  0x02,               //  Display Z Inversion
    0xB6,  3,  0x00, 0x42, 0x3B,   //  Display Control Function      
    0xE0, 15,  0x1F, 0x25, 0x22, 0x0B, 0x06, 0x0A, 0x4E, 0xC6, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  positive gamma control
    0XE1, 15,  0x1F, 0x3F, 0x3F, 0x0F, 0x1F, 0x0F, 0x46, 0x49, 0x31, 0x05, 0x09, 0x03, 0x1C, 0x1A, 0x00  //  negative gamma control
  };*/
  /*
  static const uint8_t reset_off[] PROGMEM = {
    0x01, 0,            // Soft Reset
    TFTLCD_DELAY8, 150, // .kbv will power up with ONLY reset, sleep out, display on
    0x28, 0,            // Display Off
    0x3A, 1, 0x55,      // Pixel read=565, write=565.
  };
  
  static const uint8_t wake_on[] PROGMEM = {
    0x11, 0,              // Sleep Out
    TFTLCD_DELAY8, 150,
    0x29, 0,              // Display On
  };*/
  
  static const uint8_t t0[] PROGMEM = {
    0x3a, 1, 0x55,
    0xB6, 2, 0x00, 0x22,
    0x36, 1, 0xa8,        // rotation, 0x08, 0x68, 0xc8, 0xa8
    0x11, 0               // sleep out
  };
  
  static const uint8_t td[] PROGMEM = {
  //    0x2a, 0, // transfer block
    0x2c, 100,  0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8,
  //    0x2b, 0  // transfer block
  };
  
  static const uint8_t t1[] PROGMEM = {
    0x29, 0               // Turn on display
  };
  
  //write_table(&reset_off, sizeof(reset_off));
  write_table(&regs, sizeof(regs));
  //write_table(&wake_on, sizeof(wake_on));
  write_table(&t0, sizeof(t0));
  write_table(&td, sizeof(td));
  write_table(&t1, sizeof(t1));
}

/*
 *  struct ActiveBounds
  {
    uint8_t data[8];
  };

  inline void lcdWriteActiveRect(uint16_t xUL, uint16_t yUL, uint16_t xSize, uint16_t ySize)
  {
    uint16_t xStart = xUL, xEnd = xUL + xSize - 1;
    uint16_t yStart = yUL, yEnd = yUL + ySize - 1;

    //  Writing datablocks is quite a bit faster than transfering out one byte at a
    //  time.
    ActiveBounds b = {0, (uint8_t)(xStart >> 8), 0, (uint8_t)(xStart & 0xFF), 0, (uint8_t)(xEnd >> 8), 0, (uint8_t)(xEnd & 0xFF)};
    lcdWriteReg(0x2a);
    digitalWrite(LCD_DC, HIGH);
    SPI.transfer((byte *)&b, sizeof(b));

    b = {0, (uint8_t)(yStart >> 8), 0, (uint8_t)(yStart & 0xFF), 0, (uint8_t)(yEnd >> 8), 0, (uint8_t)(yEnd & 0xFF)};
    lcdWriteReg(0x2b);
    digitalWrite(LCD_DC, HIGH);
    SPI.transfer((byte *)&b, sizeof(b));
  }
}

 */
// invertDisplay(boolean i)     lcdWriteReg(i ? 0x21 : 0x20);
// idleMode(bool idle)          lcdWriteReg(idle ? 0x39 : 0x38);    
void setup() {
  // id = 0x9486
  // portrait is 320 x 480 (width x height)
  reset();
  begin(); 

  /*uint16_t GS, SS_v, ORG, NL;
  _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
  GS = (val & 0x80) ? (1 << 15) : 0;
  WriteCmdData(0x60, GS | 0x2700); 
  SS_v = (val & 0x40) ? (1 << 8) : 0;
  WriteCmdData(0x01, SS_v);     // set Driver Output Control
  ORG = (val & 0x20) ? (1 << 3) : 0;
  uint16_t _lcd_madctl = ORG | 0x0030;
  WriteCmdData(0x03, _lcd_madctl);
  WriteCmdData(_MW, TFT_GREEN);
  _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ 0;
  WriteCmdData(0x61, _lcd_rev); */
}

void loop() {
//  writePixel(10, 10, TFT_GREEN);
//  writePixel(100, 10, TFT_GREEN);
//  writePixel(10, 100, TFT_GREEN);
//  writePixel(100, 100, TFT_GREEN);
  delay(1000);
}
