//*********************************************************************************************************
//********************* ADX PICO - SMD Version ARDUINO based DIGITAL MODES 4 BAND HF TRANSCEIVER ***************************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_PICO_V2.0 - Version release date: 10/07/2022
// Baris DINC - TA7W/OH2UDS - 2022
//
// Release Notes 2.0
// - TWI libraruy replaced with bitbanging
// - si5351 library replaced with minimal functions
// - CAT interface added
// - Si5351 hw replacements by chineese clone (MM5351) is now supported
//
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
//*************************************[ LICENCE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino

// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//*****************[ SI5351 VFO CALIBRATION PROCEDURE ]****************************************
// For SI5351 VFO Calibration Procedure follow these steps:
// 1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope that can measure 1 Mhz up to 1Hz accurately.
// 2 - Press SW2 / --->(CAL) pushbutton and hold.
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton. 
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope. 
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 8 - If you read as accurate as possible 1000000 Hz then calibration is done. 
// 9 - Now we must save this calibration value to EEPROM location. 
//     In order to save calibration value, press TX button briefly. TX LED will flash 3 times which indicates that Calibration value is saved.
// 10- Power off ADX.

// **********************************[ DEFINE's ]***********************************************
#define UP 2   //2    // UP Switch
#define DOWN 3  //3  // DOWN Switch
#define TXSW 4  // TX Switch

#define TX 13 //TX LED
#define WSPR  9 //9 WSPR LED 
#define JS8  10 //  10 JS8 LED
#define FT4  11 //  11  FT4 LED
#define FT8 12 //12  FT8 LED

#define RX  8 // RX SWITCH
#define SI5351_REF    25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#define SI5351_ADDR   0x60   // SI5351A I2C address: 0x60 for SI5351A-B-GT, Si5351A-B04771-GT, MS5351M; 0x62 for SI5351A-B-04486-GT; 0x6F for SI5351A-B02075-GT; see here for other variants: https://www.silabs.com/TimingUtility/timing-download-document.aspx?OPN=Si5351A-B02075-GT&OPNRevision=0&FileType=PublicAddendum


//********** Library implementations

// I2C communication starts with a START condition, multiple single byte-transfers (MSB first) followed by an ACK/NACK and stops with a STOP condition;
// during data-transfer SDA may only change when SCL is LOW, during a START/STOP condition SCL is HIGH and SDA goes DOWN for a START and UP for a STOP.
// https://www.ti.com/lit/an/slva704/slva704.pdf
class I2C {
public:
  #define I2C_DELAY   4       // Determines I2C Speed (2=939kb/s (too fast!!); 3=822kb/s; 4=731kb/s; 5=658kb/s; 6=598kb/s).
  #define I2C_DDR     DDRC    // Pins for the I2C bit banging
  #define I2C_PIN     PINC
  #define I2C_PORT    PORTC
  #define I2C_SDA (1 << 4)    // PA4
  #define I2C_SCL (1 << 5)    // PC5
  #define DELAY(n) for(uint8_t i = 0; i != n; i++) asm("nop");
  #define I2C_SDA_GET() I2C_PIN & I2C_SDA
  #define I2C_SCL_GET() I2C_PIN & I2C_SCL
  #define I2C_SDA_HI() I2C_DDR &= ~I2C_SDA;
  #define I2C_SDA_LO() I2C_DDR |=  I2C_SDA;
  #define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY);
  #define I2C_SCL_LO() I2C_DDR |=  I2C_SCL; DELAY(I2C_DELAY);

  I2C(){
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_SDA_LO();
  }
  ~I2C(){
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_DDR &= ~( I2C_SDA | I2C_SCL );
  }  
  inline void start(){
    I2C_PORT &= ~I2C_SDA;
    I2C_SCL_LO();
    I2C_SDA_HI();
  }
  inline void stop(){
    I2C_SDA_LO();   // ensure SDA is LO so STOP-condition can be initiated by pulling SCL HI (in case of ACK it SDA was already LO, but for a delayed ACK or NACK it is not!)
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_DDR &= ~(I2C_SDA | I2C_SCL); // prepare for a start: pull-up both SDA, SCL
    I2C_SDA_LO();
  }
  #define SendBit(data, mask) \
    if(data & mask){ \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();
  inline void SendByte(uint8_t data){
    SendBit(data, 1 << 7);
    SendBit(data, 1 << 6);
    SendBit(data, 1 << 5);
    SendBit(data, 1 << 4);
    SendBit(data, 1 << 3);
    SendBit(data, 1 << 2);
    SendBit(data, 1 << 1);
    SendBit(data, 1 << 0);
    I2C_SDA_HI();  // recv ACK
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SCL_LO();
  }
  inline uint8_t RecvBit(uint8_t mask){
    I2C_SCL_HI();
    uint16_t i = 60000;
    for(;!(I2C_SCL_GET()) && i; i--);  // wait util slave release SCL to HIGH (meaning data valid), or timeout at 3ms
    //if(!i){ lcd.setCursor(0, 1); lcd.print(F("E07 I2C timeout")); }
    uint8_t data = I2C_SDA_GET();
    I2C_SCL_LO();
    return (data) ? mask : 0;
  }
  inline uint8_t RecvByte(uint8_t last){
    uint8_t data = 0;
    data |= RecvBit(1 << 7);
    data |= RecvBit(1 << 6);
    data |= RecvBit(1 << 5);
    data |= RecvBit(1 << 4);
    data |= RecvBit(1 << 3);
    data |= RecvBit(1 << 2);
    data |= RecvBit(1 << 1);
    data |= RecvBit(1 << 0);
    if(last){
      I2C_SDA_HI();  // NACK
    } else {
      I2C_SDA_LO();  // ACK
    }
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SDA_HI();    // restore SDA for read
    I2C_SCL_LO();
    return data;
  }

  void begin(){};
  void beginTransmission(uint8_t addr){ start(); SendByte(addr << 1);  };
  bool write(uint8_t byte){ SendByte(byte); return 1; };
  uint8_t endTransmission(){ stop(); return 0; };
};

I2C i2c;
class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  //volatile uint32_t _mod;
  volatile uint8_t pll_regs[8];

  #define BB0(x) ((uint8_t)(x))           // Bash byte x of int32_t
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define FAST __attribute__((optimize("Ofast")))

  volatile uint32_t fxtal = SI5351_REF;

  inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
  {
    #define _MSC  0x10000
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

    uint16_t msp1 = _msa128min512 + msb128 / _MSC; // = 128 * _msa + msb128 / _MSC - 512;
    uint16_t msp2 = msb128; // = msb128 % _MSC;  assuming MSC is covering exact uint16_t so the mod operation can dissapear (and the upper BB2 byte) // = msb128 - msb128/_MSC * _MSC;

    //pll_regs[0] = BB1(msc);  // 3 regs are constant
    //pll_regs[1] = BB0(msc);
    //pll_regs[2] = BB2(msp1);
    //pll_regs[3] = BB1(msp1);
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4))/*|BB2(msp2)*/; // top nibble MUST be same as top nibble of _MSC !  assuming that BB2(msp2) is always 0 -> so reg is constant
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }

  inline void SendPLLRegisterBulk(){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(26+0*8 + 4);  // Write to PLLA
    //i2c.SendByte(26+1*8 + 4);  // Write to PLLB
    i2c.SendByte(pll_regs[4]);
    i2c.SendByte(pll_regs[5]);
    i2c.SendByte(pll_regs[6]);
    i2c.SendByte(pll_regs[7]);
    i2c.stop();
  }
  
  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    while (n--) i2c.SendByte(*data++);
    i2c.stop();      
  }
  void SendRegister(uint8_t reg, uint8_t val){ SendRegister(reg, &val, 1); }
  int16_t iqmsa; // to detect a need for a PLL reset

  enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };
  
  void ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA, uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0){
    uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
    msa = div_nom / div_denom;     // integer part: msa must be in range 15..90 for PLL, 8+1/1048575..900 for MS
    if(msa == 4) _int = 1;  // To satisfy the MSx_INT=1 requirement of AN619, section 4.1.3 which basically says that for MS divider a value of 4 and integer mode must be used
    msb = (_int) ? 0 : (((uint64_t)(div_nom % div_denom)*_MSC) / div_denom); // fractional part
    msc = (_int) ? 1 : _MSC;
    //lcd.setCursor(0, 0); lcd.print(n); lcd.print(":"); lcd.print(msa); lcd.print(" "); lcd.print(msb); lcd.print(" "); lcd.print(msc); lcd.print(F("    ")); delay(500);
    msp1 = 128*msa + 128*msb/msc - 512;
    msp2 = 128*msb - 128*msb/msc * msc;
    msp3 = msc;
    uint8_t ms_reg2 = BB2(msp1) | (rdiv<<4) | ((msa == 4)*0x0C);
    uint8_t ms_regs[8] = { BB1(msp3), BB0(msp3), ms_reg2, BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };

    SendRegister(n*8+42, ms_regs, 8); // Write to MSx
    if(n < 0){
      SendRegister(n+16+8, 0x80|(0x40*_int)); // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
    } else {
      //SendRegister(n+16, ((pll)*0x20)|0x0C|0|(0x40*_int));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 0=2mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+16, ((pll)*0x20)|0x0C|3|(0x40*_int));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 3=8mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+165, (!_int) * phase * msa / 90);      // when using: make sure to configure MS in fractional-mode, perform reset afterwards
    }
  }


void set_load(uint8_t xtal_load_c)
{
    // Set crystal load capacitance
    #define SI5351_CRYSTAL_LOAD             183
    #define SI5351_CRYSTAL_LOAD_MASK        (3<<6)
    #define SI5351_CRYSTAL_LOAD_0PF         (0<<6)
    #define SI5351_CRYSTAL_LOAD_6PF         (1<<6)
    #define SI5351_CRYSTAL_LOAD_8PF         (2<<6)
    #define SI5351_CRYSTAL_LOAD_10PF        (3<<6)
    SendRegister(SI5351_CRYSTAL_LOAD, (xtal_load_c & SI5351_CRYSTAL_LOAD_MASK) | 0b00010010);
}



  void phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase){ SendRegister(n+165, phase * (div_nom / div_denom) / 90); }  // when using: make sure to configure MS in fractional-mode!, perform reset afterwards

  void reset(){ SendRegister(177, 0xA0); } // 0x20 reset PLLA; 0x80 reset PLLB

  void oe(uint8_t mask){ SendRegister(3, ~mask); } // output-enable mask: CLK2=4; CLK1=2; CLK0=1

  void freq(int32_t fout, uint16_t i, uint16_t q){  // Set a CLK0,1,2 to fout Hz with phase i, q (on PLLA)
      uint8_t rdiv = 0; // CLK pin sees fout/(2^rdiv)
      if(fout > 300000000){ i/=3; q/=3; fout/=3; }  // for higher freqs, use 3rd harmonic
      if(fout < 500000){ rdiv = 7; fout *= 128; } // Divide by 128 for fout 4..500kHz
      uint16_t d; if(fout < 30000000) d = (16 * fxtal) / fout; else d = (32 * fxtal) / fout;  // Integer part  .. maybe 44?
      if(fout < 3500000) d = (7 * fxtal) / fout;  // PLL at 189MHz to cover 160m (freq>1.48MHz) when using 27MHz crystal
      if(fout > 140000000) d = 4; // for f=140..300MHz; AN619; 4.1.3, this implies integer mode
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d += 2; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
      // si5351 spectral purity considerations: https://groups.io/g/QRPLabs/message/42662

      ms(MSNA, fvcoa, fxtal);                   // PLLA in fractional mode
      //ms(MSNB, fvcoa, fxtal);
      ms(MS0,  fvcoa, fout, PLLA, 0, i, rdiv);  // Multisynth stage with integer divider but in frac mode due to phase setting
      ms(MS1,  fvcoa, fout, PLLA, 0, q, rdiv);
#ifdef F_CLK2
      freqb(F_CLK2);
#else
      ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
#endif
      if(iqmsa != (((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90)){ iqmsa = ((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90; reset(); }
      oe(0b00000011);  // output enable CLK0, CLK1

#ifdef x
      ms(MSNA, fvcoa, fxtal);
      ms(MSNB, fvcoa, fxtal);
      #define F_DEV 4
      ms(MS0,  fvcoa, (fout + F_DEV), PLLA, 0, 0, rdiv);
      ms(MS1,  fvcoa, (fout + F_DEV), PLLA, 0, 0, rdiv);
      ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
      reset();
      ms(MS0,  fvcoa, fout, PLLA, 0, 0, rdiv);
      delayMicroseconds(F_MCU/16000000 * 1000000UL/F_DEV);  // Td = 1/(4 * Fdev) phase-shift   https://tj-lab.org/2020/08/27/si5351%e5%8d%98%e4%bd%93%e3%81%a73mhz%e4%bb%a5%e4%b8%8b%e3%81%ae%e7%9b%b4%e4%ba%a4%e4%bf%a1%e5%8f%b7%e3%82%92%e5%87%ba%e5%8a%9b%e3%81%99%e3%82%8b/
      ms(MS1,  fvcoa, fout, PLLA, 0, 0, rdiv);
      oe(0b00000011);  // output enable CLK0, CLK1
#endif
      _fout = fout;  // cache
      _div = d;
      _msa128min512 = fvcoa / fxtal * 128 - 512;
      _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
      //_mod = fvcoa % fxtal;
  }

  void freqb(uint32_t fout){  // Set a CLK2 to fout Hz (on PLLB)
      uint16_t d = (16 * fxtal) / fout;
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz

      ms(MSNB, fvcoa, fxtal);
      ms(MS2,  fvcoa, fout, PLLB, 0, 0, 0);
  }
  

  uint8_t RecvRegister(uint8_t reg){
    i2c.start();  // Data write to set the register address
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.stop();
    i2c.start(); // Data read to retrieve the data from the set address
    i2c.SendByte((SI5351_ADDR << 1) | 1);
    uint8_t data = i2c.RecvByte(true);
    i2c.stop();
    return data;
  }
  void powerDown(){
    SendRegister(3, 0b11111111); // Disable all CLK outputs
    SendRegister(24, 0b00010000); // Disable state: CLK2 HIGH state, CLK0 & CLK1 LOW state when disabled; CLK2 needs to be in HIGH state to make sure that cap to gate is already charged, preventing "exponential pulse is caused by CLK2, which had been at 0v whilst it was disabled, suddenly generating a 5vpp waveform, which is “added to” the 0v filtered PWM output and causing the output fets to be driven with the full 5v pp.", see: https://forum.dl2man.de/viewtopic.php?t=146&p=1307#p1307
    SendRegister(25, 0b00000000); // Disable state: LOW state when disabled
    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b10000000);  // Conserve power when output is disabled
    SendRegister(187, 0);        // Disable fanout (power-safe)
    // To initialise things as they should:
    SendRegister(149, 0);        // Disable spread spectrum enable
    SendRegister(183, 0b11010010);  // Internal CL = 10 pF (default)
  }
  #define SI_CLK_OE 3
};
static SI5351 si5351;

//*******************************[ LIBRARIES ]*************************************************
#include <EEPROM.h>

 
//*******************************[ VARIABLE DECLERATIONS ]*************************************
uint32_t val;
int temp;
uint32_t val_EE; 
int addr = 0;
int mode;
unsigned long freq; 
unsigned long freq1;
uint32_t cal_factor;
int TX_State = 0;

unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long F_FT8;
unsigned long F_FT4; 
unsigned long F_JS8;
unsigned long F_WSPR;
int Band_slot;
int Band = 0;
int UP_State;
int DOWN_State;
int TXSW_State;
int Bdly = 250;

//**********************************[ BAND SELECT ]************************************************
// ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 6 bands.
// To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
// that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode. 
// Now the new selected band bank will flash 3 times and then stored mode LED will be lit. 
// TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.

// Assign your prefered bands to B1,B2,B3 and B4
// Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m

int Band1 = 40; // Band 1 // These are my default bands. Feel free to swap with yours
int Band2 = 30; // Band 2
int Band3 = 20; // Band 3
int Band4 = 17; // Band 4



//*************************************[ SETUP FUNCTION ]************************************** 
void setup()
{
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(TXSW, INPUT);
   
  pinMode(TX, OUTPUT);
  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);
  pinMode(RX, OUTPUT);

  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0
  
INIT();

//------------------------------- SET SI5351 VFO -----------------------------------  
   // The crystal load value needs to match in order to have an accurate calibration

  si5351.set_load(SI5351_CRYSTAL_LOAD_8PF);

/*BARIS
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX 
*/  
 if ( digitalRead(UP) == LOW ) {
Calibration();
 }
 

  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1<<ACIC);  // Analog Comparator Capture Input
  
  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0
          digitalWrite(RX,LOW);

 Mode_assign(); 

}
 
//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop()
{  

       UP_State = digitalRead(UP);
          DOWN_State = digitalRead(DOWN);


if ((UP_State == LOW)&&(DOWN_State == LOW)&&(TX_State == 0)) {
   delay(100); 
 UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);
if ((UP_State == LOW)&&(DOWN_State == LOW)&&(TX_State == 0)) {

    Band_Select();
    
  }
}

  

if ((UP_State == LOW)&&(DOWN_State == HIGH)&&(TX_State == 0)) {
   delay(50); 
     
UP_State = digitalRead(UP);
if ((UP_State == LOW)&&(DOWN_State == HIGH)&&(TX_State == 0)) {
mode = mode - 1;

if (mode < 1){
mode = 4;
}



addr = 40;
 EEPROM.put(addr, mode); 
                Mode_assign();
                
  }} 
   
      DOWN_State = digitalRead(DOWN);

if ((UP_State == HIGH) && (DOWN_State == LOW)&&(TX_State == 0)) {
   delay(50); 
     
DOWN_State = digitalRead(DOWN);
if ((UP_State == HIGH) && (DOWN_State == LOW)&&(TX_State == 0)) {
mode = mode + 1;

if (mode > 4){
mode = 1;
}



addr = 40;
 EEPROM.put(addr, mode); 
                    Mode_assign();
                    
  }} 

  TXSW_State = digitalRead(TXSW);

if ((TXSW_State == LOW) && (TX_State == 0)) {
   delay(50); 
     
TXSW_State = digitalRead(TXSW);
if ((TXSW_State == LOW) && (TX_State == 0)) {
  
Mode_assign();

ManualTX();

           }
  
     }

unsigned int d1,d2;
 int FSK = 10;
 int FSKtx = 0;
 while (FSK>0){
  TCNT1 = 0;
  while (ACSR &(1<<ACO)){
    if (TCNT1>65000) {break;
  }
  }  while ((ACSR &(1<<ACO))==0){
    if (TCNT1>65000) {break;}
  }
  TCNT1 = 0;
  while (ACSR &(1<<ACO)){
    if (TCNT1>65000) {break;}
  }
  d1 = ICR1;  
  while ((ACSR &(1<<ACO))==0){
    if (TCNT1>65000) {break;}
  } 
  while (ACSR &(1<<ACO)){
    if (TCNT1>65000) {break;}
  }
  d2 = ICR1;
  if (TCNT1 < 65000){
  unsigned long codefreq = 1600000000/(d2-d1);
    if (codefreq < 350000){
      if (FSKtx == 0){
        TX_State = 1;
          digitalWrite(TX,HIGH);
          digitalWrite(RX,LOW);
         
          si5351.output_enable(SI5351_CLK1, 0);   //RX off
          si5351.output_enable(SI5351_CLK0, 1);   // TX on
      }
      si5351.set_freq((freq * 100 + codefreq), SI5351_CLK0);  
      
      FSKtx = 1;
    }
  }
  else{
    FSK--;
  }
 }
  digitalWrite(TX,0);
  
    si5351.output_enable(SI5351_CLK0, 0);   //TX off
    si5351.set_freq(freq*100ULL, SI5351_CLK1);
          si5351.output_enable(SI5351_CLK1, 1);   //RX on
    TX_State = 0;
    digitalWrite(RX,HIGH);
  FSKtx = 0;

     
}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************

//********************************************************************************
//******************************** [ FUNCTIONS ] *********************************
//********************************************************************************


//************************************[ MODE Assign ]**********************************

void Mode_assign(){
  
addr = 40;
EEPROM.get(addr,mode);


if ( mode == 1){
  freq1 = F_WSPR;
 digitalWrite(WSPR, HIGH); 
     digitalWrite(JS8, LOW); 
         digitalWrite(FT4, LOW); 
             digitalWrite(FT8, LOW); 

}

if ( mode == 2){
  freq1 = F_JS8;
digitalWrite(WSPR, LOW); 
     digitalWrite(JS8, HIGH); 
         digitalWrite(FT4, LOW); 
             digitalWrite(FT8, LOW); 


  
}

if ( mode == 3){
  freq1 = F_FT4;

digitalWrite(WSPR, LOW); 
     digitalWrite(JS8, LOW); 
         digitalWrite(FT4, HIGH); 
             digitalWrite(FT8, LOW); 

  
}
if ( mode == 4){
  freq1 = F_FT8;
  
digitalWrite(WSPR, LOW); 
     digitalWrite(JS8, LOW); 
         digitalWrite(FT4, LOW); 
             digitalWrite(FT8, HIGH); 

  
}
freq = freq1; 
//freq = freq1 - 1000;

}

//********************************[ END OF MODE ASSIGN ]*******************************

//*********************[ Band dependent Frequency Assign Function ]********************
void Freq_assign(){


//---------- 80m/3.5Mhz 
          if (Band == 80){

            F_FT8 = 3573000;
            F_FT4 = 3575000;
            F_JS8 = 3578000;
            F_WSPR = 3568600;
          }

//---------- 40m/7 Mhz 
          if (Band == 40){

            F_FT8 = 7074000;
            F_FT4 = 7047500;
            F_JS8 = 7078000;
            F_WSPR = 7038600;
          }


//---------- 30m/10 Mhz 
          if (Band == 30){

            F_FT8 = 10136000;
            F_FT4 = 10140000;
            F_JS8 = 10130000;
            F_WSPR = 10138700;
          }


//---------- 20m/14 Mhz 
          if (Band == 20){

            F_FT8 = 14074000;
            F_FT4 = 14080000;
            F_JS8 = 14078000;
            F_WSPR = 14095600;
          }


 //---------- 17m/18 Mhz 
          if (Band == 17){

            F_FT8 = 18100000;
            F_FT4 = 18104000;
            F_JS8 = 18104000;
            F_WSPR = 18104600;
          } 

//---------- 15m/ 21Mhz 
          if (Band == 15){

            F_FT8 = 21074000;
            F_FT4 = 21140000;
            F_JS8 = 21078000;
            F_WSPR = 21094600;
          } 
          
}
//************************[ End of Frequency assign function ]*************************  

//******************************[ Band  Assign Function ]******************************

void Band_assign(){
digitalWrite(WSPR, LOW); 
     digitalWrite(JS8, LOW); 
         digitalWrite(FT4, LOW); 
             digitalWrite(FT8, LOW); 


addr = 50;
EEPROM.get(addr,Band_slot);

if (Band_slot == 1){
  Band = Band1;
  
digitalWrite(FT8, HIGH); 
delay(Bdly);
digitalWrite(FT8, LOW); 
delay(Bdly);
digitalWrite(FT8, HIGH); 
delay(Bdly);
digitalWrite(FT8, LOW); 
delay(Bdly);
digitalWrite(FT8, HIGH); 
delay(Bdly);
digitalWrite(FT8, LOW); 
delay(Bdly);
}

if (Band_slot == 2){
  Band = Band2;
  
digitalWrite(FT4, HIGH); 
delay(Bdly);
digitalWrite(FT4, LOW); 
delay(Bdly);
digitalWrite(FT4, HIGH); 
delay(Bdly);
digitalWrite(FT4, LOW); 
delay(Bdly);
digitalWrite(FT4, HIGH); 
delay(Bdly);
digitalWrite(FT4, LOW); 
delay(Bdly);
}

if (Band_slot == 3){
  Band = Band3;
  
digitalWrite(JS8, HIGH); 
delay(Bdly);
digitalWrite(JS8, LOW); 
delay(Bdly);
digitalWrite(JS8, HIGH); 
delay(Bdly);
digitalWrite(JS8, LOW); 
delay(Bdly);
digitalWrite(JS8, HIGH); 
delay(Bdly);
digitalWrite(JS8, LOW); 
delay(Bdly);
}

if (Band_slot == 4){
  Band = Band4;
  
digitalWrite(WSPR, HIGH); 
delay(Bdly);
digitalWrite(WSPR, LOW); 
delay(Bdly);
digitalWrite(WSPR, HIGH); 
delay(Bdly);
digitalWrite(WSPR, LOW); 
delay(Bdly);
digitalWrite(WSPR, HIGH); 
delay(Bdly);
digitalWrite(WSPR, LOW); 
delay(Bdly);
}


delay(1000);

Freq_assign();

Mode_assign();


}
//***************************[ End of Band assign function ]***************************  


//*******************************[ Manual TX FUNCTION ]********************************
void ManualTX(){
 
 digitalWrite(RX,LOW);
          si5351.output_enable(SI5351_CLK1, 0);   //RX off


TXON:  

TXSW_State = digitalRead(TXSW);

 
digitalWrite(TX,1);
    si5351.set_freq(freq1*100ULL, SI5351_CLK0);
    si5351.output_enable(SI5351_CLK0, 1);   //TX on
    TX_State = 1;
    
  if (TXSW_State == HIGH) {
goto EXIT_TX;

}
   goto TXON;




EXIT_TX:
digitalWrite(TX,0); 
    si5351.output_enable(SI5351_CLK0, 0);   //TX off
    TX_State = 0;

}

//********************************[ END OF Manual TX ]*********************************

//******************************[ BAND SELECT Function]********************************
void Band_Select(){

digitalWrite(TX,1);
addr = 50; 
EEPROM.get(addr,Band_slot);

digitalWrite(WSPR,LOW); 
digitalWrite(JS8, LOW); 
digitalWrite(FT4, LOW); 
digitalWrite(FT8, LOW); 


if (Band_slot == 1){
 
  
digitalWrite(FT8, HIGH); 
delay(Bdly);
digitalWrite(FT8, LOW); 
delay(Bdly);
digitalWrite(FT8, HIGH); 
delay(Bdly);
digitalWrite(FT8, LOW); 
delay(Bdly);
digitalWrite(FT8, HIGH); 
delay(Bdly);
digitalWrite(FT8, LOW); 
delay(Bdly);
}

if (Band_slot == 2){
  
digitalWrite(FT4, HIGH); 
delay(Bdly);
digitalWrite(FT4, LOW); 
delay(Bdly);
digitalWrite(FT4, HIGH); 
delay(Bdly);
digitalWrite(FT4, LOW); 
delay(Bdly);
digitalWrite(FT4, HIGH); 
delay(Bdly);
digitalWrite(FT4, LOW); 
delay(Bdly);
}

if (Band_slot == 3){
  
digitalWrite(JS8, HIGH); 
delay(Bdly);
digitalWrite(JS8, LOW); 
delay(Bdly);
digitalWrite(JS8, HIGH); 
delay(Bdly);
digitalWrite(JS8, LOW); 
delay(Bdly);
digitalWrite(JS8, HIGH); 
delay(Bdly);
digitalWrite(JS8, LOW); 
delay(Bdly);
}

if (Band_slot == 4){
  
digitalWrite(WSPR, HIGH); 
delay(Bdly);
digitalWrite(WSPR, LOW); 
delay(Bdly);
digitalWrite(WSPR, HIGH); 
delay(Bdly);
digitalWrite(WSPR, LOW); 
delay(Bdly);
digitalWrite(WSPR, HIGH); 
delay(Bdly);
digitalWrite(WSPR, LOW); 
delay(Bdly);
}

Band_cont:
 
if (Band_slot == 1){
 
  
digitalWrite(FT8, HIGH); 
digitalWrite(JS8, LOW); 
digitalWrite(FT4, LOW); 
digitalWrite(WSPR, LOW); 
}

if (Band_slot == 2){
  
  
digitalWrite(FT4, HIGH); 
digitalWrite(WSPR, LOW); 
digitalWrite(JS8, LOW); 
digitalWrite(FT8, LOW); 

}

if (Band_slot == 3){
digitalWrite(JS8, HIGH); 
digitalWrite(WSPR, LOW); 
digitalWrite(FT4, LOW); 
digitalWrite(FT8, LOW); 

  

}

if (Band_slot == 4){
digitalWrite(JS8, LOW); 
digitalWrite(WSPR, HIGH); 
digitalWrite(FT4, LOW); 
digitalWrite(FT8, LOW); 
 
  }


 UP_State = digitalRead(UP);
          DOWN_State = digitalRead(DOWN);
    
if ((UP_State == LOW)&&(DOWN_State == HIGH)) {
   delay(100); 
     
UP_State = digitalRead(UP);
if ((UP_State == LOW)&&(DOWN_State == HIGH)) {
Band_slot = Band_slot + 1;

if (Band_slot > 4){
Band_slot = 1;
        }
     }
  } 
   
if ((UP_State == HIGH)&&(DOWN_State == LOW)) {
   delay(100); 
     
DOWN_State = digitalRead(DOWN);
if ((UP_State == HIGH)&&(DOWN_State == LOW)) {
Band_slot = Band_slot - 1;

if (Band_slot < 1){
Band_slot = 4;
       }
    }
 }
                                                

TX_State = digitalRead(TXSW);

if (TX_State == LOW) {
   delay(100); 
 
    TX_State = digitalRead(TXSW);
if (TX_State == LOW) {
  
digitalWrite(TX,0); 

   goto Band_exit;
    
  }
} 

goto Band_cont;

Band_exit:

addr = 50;
 EEPROM.put(addr, Band_slot); 


 Band_assign();


}
  
//*********************************[ END OF BAND SELECT ]*****************************




//************************** [SI5351 VFO Calibration Function] ************************

void Calibration(){

digitalWrite(FT8, LOW);
digitalWrite(FT4, LOW);
digitalWrite(JS8, LOW);
digitalWrite(WSPR, LOW);

digitalWrite(WSPR, HIGH); 
digitalWrite(FT8, HIGH);
delay(100);        
 
digitalWrite(WSPR, LOW); 
digitalWrite(FT8, LOW);
delay(100);                 


digitalWrite(WSPR, HIGH); 
digitalWrite(FT8, HIGH);
delay(100);        
 
digitalWrite(WSPR, LOW); 
digitalWrite(FT8, LOW);
delay(100);                       

 digitalWrite(WSPR, HIGH); 
digitalWrite(FT8, HIGH);
delay(100);        
 
digitalWrite(WSPR, LOW); 
digitalWrite(FT8, LOW);
delay(100);                       

digitalWrite(WSPR, HIGH); 
digitalWrite(FT8, HIGH);
delay(100);        
 
digitalWrite(WSPR, LOW); 
digitalWrite(FT8, LOW);
delay(100);                       

digitalWrite(WSPR, HIGH); 
digitalWrite(FT8, HIGH);

addr = 10;
EEPROM.get(addr, cal_factor); 
//Serial.print("cal factor= ");

Calibrate:

    UP_State = digitalRead(UP);

if (UP_State == LOW) {
   delay(50); 
     
UP_State = digitalRead(UP);
if (UP_State == LOW) {

cal_factor = cal_factor - 100;

 //EEPROM.put(addr, cal_factor); 

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  
  
  // Set CLK2 output
  si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration

  
  }} 
   
    DOWN_State = digitalRead(DOWN);

if (DOWN_State == LOW) {
   delay(50);   
   
         DOWN_State = digitalRead(DOWN);
if (DOWN_State == LOW) {

cal_factor = cal_factor + 100;
//EEPROM.put(addr, cal_factor); 
    
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  
  // Set CLK2 output
  si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for Calibration
  si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable clock2 

  }} 
  
 TXSW_State = digitalRead(TXSW);

if (TXSW_State == LOW) {
   delay(50);   
   
         TXSW_State = digitalRead(TXSW);
if (TXSW_State == LOW) {
  
addr = 10;
EEPROM.put(addr, cal_factor); 

digitalWrite(TX, HIGH); 
delay(Bdly);
digitalWrite(TX, LOW); 
delay(Bdly);
digitalWrite(TX, HIGH); 
delay(Bdly);
digitalWrite(TX, LOW); 
delay(Bdly);
digitalWrite(TX, HIGH); 
delay(Bdly);
digitalWrite(TX, LOW); 

    
  }} 
   goto Calibrate;
      }

//****************************** [ End Of Calibration Function ]****************************************

//*********************************[ INITIALIZATION FUNCTION ]******************************************
void INIT(){

addr = 30;
EEPROM.get(addr,temp);

if (temp != 100){
  
    addr = 10; 
    cal_factor = 100000;
    EEPROM.put(addr, cal_factor); 
    
    addr = 40;
    temp = 4;
    EEPROM.put(addr,temp);
    
    addr = 30;
    temp = 100;
    EEPROM.put(addr,temp);
    
    addr = 50;
    temp = 1;
    EEPROM.put(addr,temp);

        }

else

{
    //--------------- EEPROM INIT VALUES
    addr = 30;
    EEPROM.get(addr,temp);
    
    addr = 10;
    EEPROM.get(addr,cal_factor);
    
    addr = 40;
    EEPROM.get(addr,mode);
    
    addr = 50;
    EEPROM.get(addr,Band_slot);


    }  

Band_assign();

Freq_assign();

Mode_assign();

 si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock

}























//********************************[ END OF INITIALIZATION FUNCTION ]*************************************
