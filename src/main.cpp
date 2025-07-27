// FM передатчик на QN8027

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

//#define USE_HSI_CLOCK 1
// I2C address of the QN8035 tuner.
//#define QN8027_ADDRESS 0x58

// Chip ID related to QN8035 tuner.
//#define QN8027_ID 0x84

#define TX_POWER_MAX 75
#define RDSBUF_SIZE 128 // 8 (bytes) is the smallest value

#define FREQ_DEFAULT 100.0f

#define QN8027_ADDRESS (0x2C)

#define QN8027_REG_SYSTEM (0x00) // SYSTEM Sets device modes, resets.
#define QN8027_REG_CH1 (0x01)    // Lower 8 bits of 10-bit channel index.
#define QN8027_REG_GPLT (0x02)   // Audio controls, gain of TX pilot frequency deviation.
#define QN8027_REG_XTL (0x03)    // XCLK pin control.
#define QN8027_REG_VGA (0x04)    // TX mode input impedance, crystal frequency setting.
#define QN8027_REG_CID1 (0x05)   // Device ID numbers.
#define QN8027_REG_CID2 (0x06)   // Device ID numbers.
#define QN8027_REG_STATUS (0x07) // Device status indicators.
#define QN8027_REG_RDSD0 (0x08)  // RDS data byte 0.
#define QN8027_REG_RDSD1 (0x09)  // RDS data byte 1.
#define QN8027_REG_RDSD2 (0x0A)  // RDS data byte 2.
#define QN8027_REG_RDSD3 (0x0B)  // RDS data byte 3.
#define QN8027_REG_RDSD4 (0x0C)  // RDS data byte 4.
#define QN8027_REG_RDSD5 (0x0D)  // RDS data byte 5.
#define QN8027_REG_RDSD6 (0x0E)  // RDS data byte 6.
#define QN8027_REG_RDSD7 (0x0F)  // RDS data byte 7.
#define QN8027_REG_PAC (0x10)    // PA output power target control.
#define QN8027_REG_FDEV (0x11)   // Specify total TX frequency deviation.
#define QN8027_REG_RDS (0x12)    // Specify RDS frequency deviation, RDS mode selection.

#define QN8027_WRITEREG (0x58)
#define QN8027_READREG (0x59)

// SYSTEM - 0x00
#define QN8027_BIT_RESET (7)
#define QN8027_BIT_RECAL (6)
#define QN8027_BIT_IDLE (5)
#define QN8027_BIT_MONO (4)
#define QN8027_BIT_MUTE (3)
#define QN8027_BIT_RDSRDY (2)
#define QN8027_BIT_CH_HI (0)

#define QN8027_MASK_RESET (1 << QN8027_BIT_RESET) // 1 = RESET
#define QN8027_MASK_RECAL (1 << QN8027_BIT_RECAL)
#define QN8027_MASK_IDLE (1 << QN8027_BIT_IDLE)     // 0 = IDLE
#define QN8027_MASK_MONO (1 << QN8027_BIT_MONO)     // 0 = Stereo
#define QN8027_MASK_MUTE (1 << QN8027_BIT_MUTE)     // 0 = Not Muted
#define QN8027_MASK_RDSRDY (1 << QN8027_BIT_RDSRDY) // 1 = RDS Ready
#define QN8027_MASK_CH_HI (3 << QN8027_BIT_CH_HI)

// GPLT - 0x02
#define QN8027_BIT_PREEMP (7)
#define QN8027_BIT_PRIV (6)
#define QN8027_BIT_T1M (4)

#define QN8027_MASK_PREEMP (1 << QN8027_BIT_PREEMP)
#define QN8027_MASK_PRIV (1 << QN8027_BIT_PRIV)
#define QN8027_MASK_T1M (3 << QN8027_BIT_T1M)
#define QN8027_MASK_TXPLT (15)

// XTL - 0x03
#define QN8027_BIT_XINJ (6)

#define QN8027_MASK_XINJ (3 << QN8027_BIT_XINJ)
#define QN8027_MASK_XISEL (31)

// VGA - 0x04
#define QN8027_BIT_XSEL (7)
#define QN8027_BIT_GVGA (4)
#define QN8027_BIT_GDB (2)
#define QN8027_BIT_RIN (0)

#define QN8027_MASK_XSEL (1 << QN8027_BIT_XSEL) // 0 = 12 MHz; 1 = 24 MHz
#define QN8027_MASK_GVGA (3 << QN8027_BIT_GVGA)
#define QN8027_MASK_GDB (3 << QN8027_BIT_GDB)
#define QN8027_MASK_RIN (3)

// CID1 - 0x05
#define QN8027_BIT_CID1 (2)

#define QN8027_MASK_CID1 (3 << QN8027_BIT_CID1)
#define QN8027_MASK_CID2 (3)

// CID2 - 0x06
#define QN8027_BIT_CID3 (4)

#define QN8027_MASK_CID3 (15 << QN8027_BIT_CID3)
#define QN8027_MASK_CID4 (15)

// STATUS - 0x07
#define QN8027_BIT_AUD_UPK (4)
#define QN8027_BIT_RDS_UPD (3)

#define QN8027_MASK_AUD_UPK (15 << QN8027_BIT_AUD_UPK)
#define QN8027_MASK_RDS_UPD (1 << QN8027_BIT_RDS_UPD)
#define QN8027_MASK_FSM (7)

// PAC - 0x10
#define QN8027_BIT_TXPD_CLR (7)

#define QN8027_MASK_TXPD_CLR (1 << QN8027_BIT_TXPD_CLR)
#define QN8027_MASK_PA_TRGT (127)

// RDS - 0x12
#define QN8027_BIT_RDSEN (7)

#define QN8027_MASK_RDSEN (1 << QN8027_BIT_RDSEN) // 0 - Off; 1 - On
#define QN8027_MASK_RDSFDEV (127)

#define FREQ_FLOAT2CHANNEL(X) (round((X - 76) / 0.05f))

typedef enum
{
  STEREO = 0,
  MONO = 1
} audioMode_e;

typedef enum
{
  PREEMP_50US = 0,
  PREEMP_75US = 1
} preEmphasis_e;

typedef enum
{
  POWERSAVE_58S = 0,
  POWERSAVE_59S = 1,
  POWERSAVE_60S = 2,
  POWERSAVE_NONE = 3
} powerSave_e;

typedef enum
{
  GAIN_TXPILOT_0 = 0,
  GAIN_TXPILOT_1 = 1,
  GAIN_TXPILOT_2 = 2,
  GAIN_TXPILOT_3 = 3
} gainTxPilot_e;

typedef enum
{
  CLOCK_XTAL = 0,
  CLOCK_SQIN = 1,
  CLOCK_OSC = 2,
} clockSource_e;

typedef enum
{
  CLOCKSPEED_12 = 0,
  CLOCKSPEED_24 = 1
} clockSpeed_e;

typedef enum
{
  RIN_0 = 0,
  RIN_1 = 1,
  RIN_2 = 2,
  RIN_3 = 3
} inputResistance_e;

typedef enum
{
  GAIN_0 = 0,
  GAIN_1 = 1,
  GAIN_2 = 2,
  GAIN_3 = 3,
  GAIN_4 = 4,
  GAIN_5 = 5
} inputGain_e;

typedef enum
{
  TXGAIN_0 = 0,
  TXGAIN_1 = 1,
  TXGAIN_2 = 2
} txGain_e;

class QN8027
{
public:
  void begin(float fmFreq);
  void begin(uint8_t sda, uint8_t scl);
  void begin(uint8_t sda, uint8_t scl, float i2cFreq, float fmFreq);

  void reset(void);

  void setFrequency(float freq);
  void setTxPower(uint8_t pwr);
  void enable(boolean enable);

  void rdsBufSet(const char buf[RDSBUF_SIZE], uint16_t length);
  void rdsTick(void);
  void rdsRefresh(void);
  void rdsEnable(boolean enable);

  void getCurrentRds(char rdsBuf[9]);

  // protected:
  // private:
  void _writeRegister(uint8_t addr, uint8_t data);
  void _updateRegister(uint8_t addr, uint8_t data, uint8_t mask);
  uint8_t _readRegister(uint8_t addr);

  void _setFrequency(uint16_t freqChannel);

  void _rotateRdsBuf(uint8_t num);

  void _writeRds();

  uint8_t pages;
  uint8_t pageNumber;
  uint8_t rdsBufPtr;
  uint8_t rdsBufLength;
  uint8_t rdsDisplayPtr;
  char rdsBuf[RDSBUF_SIZE];
};

//---------------------------------------------------------
#define REG_SYSTEM1 0x00   // Device modes.
#define REG_CCA 0x01       // CCA parameters.
#define REG_SNR 0x02       // Estimate RF input CNR value.
#define REG_RSSISIG 0x03   // In-band signal RSSI value.
#define REG_STATUS1 0x04   // System status.
#define REG_CID1 0x05      // Device ID numbers.
#define REG_CID2 0x06      // Device ID numbers.
#define REG_CH 0x07        // Lower 8 bits of 10-bit channel index.
#define REG_CH_START 0x08  // Lower 8 bits of 10-bit channel scan start channel index.
#define REG_CH_STOP 0x09   // Lower 8 bits of 10-bit channel scan stop channel index.
#define REG_CH_STEP 0x0A   // Channel scan frequency step. Highest 2 bits of channel indexes.
#define REG_RDSD0 0x0B     // RDS data byte 0.
#define REG_RDSD1 0x0C     // RDS data byte 1.
#define REG_RDSD2 0x0D     // RDS data byte 2.
#define REG_RDSD3 0x0E     // RDS data byte 3.
#define REG_RDSD4 0x0F     // RDS data byte 4.
#define REG_RDSD5 0x10     // RDS data byte 5.
#define REG_RDSD6 0x11     // RDS data byte 6.
#define REG_RDSD7 0x12     // RDS data byte 7.
#define REG_STATUS2 0x13   // RDS status indicators.
#define REG_VOL_CTL 0x14   // Audio controls.
#define REG_XTAL_DIV0 0x15 // Frequency select of reference clock source.
#define REG_XTAL_DIV1 0x16 // Frequency select of reference clock source.
#define REG_XTAL_DIV2 0x17 // Frequency select of reference clock source.
#define REG_INT_CTRL 0x18  // RDS control.

// Undocumented registers (based on https://github.com/ukrtrip/QN8035-qn8035-FM-chip-library/).
#define REG_CCA_SNR_TH_1 0x39
#define REG_CCA_SNR_TH_2 0x3A
#define REG_NCCFIR3 0x40

// Scanning steps for REG_CH_STEP.
#define REG_CH_STEP_50KHZ 0x00
#define REG_CH_STEP_100KHZ 0x40
#define REG_CH_STEP_200KHZ 0x80

// Bit definitions of REG_SYSTEM1.
#define REG_SYSTEM1_CCA_CH_DIS 0x01 // CH (channel index) selection method. 0 - CH is determined by internal CCA; 1 - CH is determined by the content in CH[9:0].
#define REG_SYSTEM1_CHSC 0x02       // Channel Scan mode enable. 0 - Normal operation; 1 - Channel Scan mode operation.
#define REG_SYSTEM1_FORCE_MO 0x04   // Force receiver in MONO mode, 0 - Auto, 1 - Forced in MONO mode.
#define REG_SYSTEM1_RDSEN 0x08      // RDS enable.
#define REG_SYSTEM1_RXREQ 0x10      // Receiving request. 0 - Non RX mode, 1 - Enter receive mode.
#define REG_SYSTEM1_STNBY 0x20      // Request immediately to enter Standby mode.
#define REG_SYSTEM1_RECAL 0x40      // Reset the state to initial states and recalibrate all blocks.
#define REG_SYSTEM1_SWRST 0x80      // Reset all registers to default values.

// Bit definitions of REG_STATUS1.
#define REG_STATUS1_ST_MO_RX 0x01   // Stereo receiving status.
#define REG_STATUS1_RXAGC 0x02      // AGC error status.
#define REG_STATUS1_RXAGCSET 0x04   // AGC settling status.
#define REG_STATUS1_RXCCA_FAIL 0x08 // RXCCA Status Flag.
#define REG_STATUS1_FSM 0x70        // FSM state indicator.

// RDS group definitions.
#define RDS_GROUP 0xF800
#define RDS_GROUP_A0 0x0000
#define RDS_GROUP_B0 0x0080

// Volume control settings
#define REG_VOL_CTL_MAX_ANALOG_GAIN 0x07
#define REG_VOL_CTL_MIN_ANALOG_GAIN 0x00

// RDS buffering mode.
#define RDS_DOUBLE_BUFFER_ENABLE 0xFF
#define RDS_DOUBLE_BUFFER_DISABLE 0x00

#define FREQ_TO_WORD(f) ((USHORT)((f - 60) / 0.05))

#define UCHAR unsigned char
#define USHORT unsigned short
#define CHAR char
#define INT int
#define DOUBLE double
#define VOID void

#define GET_REG ReadReg

// Size of the RDS buffer.
#define RDS_INFO_MAX_SIZE 16
CHAR rdsInfo[RDS_INFO_MAX_SIZE]; // = {'-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-'};
CHAR tempRDSBuffer[RDS_INFO_MAX_SIZE];
long i; // счетчик общего назначения

int Mode;

long Freq = 108100; // текущая частота

// OLED SSD1306
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // настройки OLED
// Create a U8g2log object

HardwareTimer *MyTim1 = new HardwareTimer(TIM1); //
// HardwareTimer *MyTim3 = new HardwareTimer(TIM3); //

TIM_HandleTypeDef htim3;

int SignalNoise;
int RSSI, RSSIcut;
DOUBLE freq = 102.9; // MHz
USHORT freqWord;
CHAR stereoStatus;
int RDSstate = 1;

void WriteReg(int Addr, int Reg);
int ReadReg(int Addr);
void SetFreq(USHORT Freq);
void powerOff(void);
VOID decodeRDSInfo(unsigned char useDoubleBuffer);
VOID resetRDSInfo();
VOID shutdownTuner();
void SetVol(int vol);

int EncOld = 0, EncNew = 0;
int Stereo;
int triggerUP;
int triggerENC;
int Freq_Volume;
int volume = 7;

int temp1;

static void MX_TIM3_Init(void);

QN8027 fm;
float frequency = 99.5f;

// ========================================================================================================= SETUP =====================================
void setup(void)
{

  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  // RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  // RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  // if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);

  delay(200); // пауза
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // pinMode(PB0, INPUT);     // вход кнопки POWER
  pinMode(PB0, INPUT_PULLUP); // вход кнопки POWER
  pinMode(PB1, INPUT_PULLUP); // вход кнопка DOWN
  pinMode(PB2, INPUT_PULLUP); // вход кнопка UP

  pinMode(PA5, INPUT); // Кнопка энкодера

  pinMode(PA15, OUTPUT);   // RED LED
  pinMode(PB10, OUTPUT);   // BLUE LED
  digitalWrite(PA15, LOW); // светодиод выкл
  digitalWrite(PB10, LOW); // светодиод выкл

  delay(100); // пауза

  MyTim1->setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE, PA8);
  MyTim1->setPrescaleFactor(1);                    // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
  MyTim1->setOverflow(12000000 * 2, HERTZ_FORMAT); // 100000 microseconds = 100 milliseconds
  // MyTim1->attachInterrupt(Update_IT_callback);
  // MyTim1->setInterruptPriority(0, 0);
  MyTim1->resume();

  delay(100);

  // Wire.begin(); // join i2c bus

  fm.begin(frequency);
  delay(10);
  fm.setTxPower(75);
  fm.enable(true);
  fm.rdsEnable(false);

  delay(500);

  // uint8_t readL = fm._readRegister(0x1E);

  // for (uint8_t i = 0; i < 0x24; i++)
  // {
  //   fm._readRegister(i);
  //   delay(10);
  // }

  for (;;)
  {
    // digitalWrite(PA15, LOW); // светодиод выкл
    digitalWrite(PB10, HIGH); // светодиод выкл
    delay(200);
    // digitalWrite(PA15, HIGH); // светодиод выкл
    digitalWrite(PB10, LOW); // светодиод выкл
    delay(200);
  }

  // pinMode(PA6, INPUT_PULLUP); // en
  // pinMode(PB7, INPUT_PULLUP); // en

  //  digitalWrite(PB10, HIGH);   // светодиод вкл
  //   pinMode(PA8, OUTPUT);       // en
  //  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCOSOURCE_LSE, RCC_MCODIV_1);
  //  /*Configure GPIO pin : PA8 */
  // HAL_RCC_MCOConfig(RCC_MCO, RCC_CFGR_MCO_PLLCLK_DIV2, RCC_MCODIV_1);

  // GPIO_InitStruct.Pin = GPIO_PIN_8;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // MyTim1->

  // Timer1.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  // MyTim1->setMode(TIM_CHANNEL_1, TIMER_OUTPUT_COMPARE_TOGGLE); // == TIM_OCMODE_TOGGLE           pin toggles when counter == channel compare

  // pinMode(PA8, PWM);                        // en
  // MyTim1->setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE, PA8);
  // MyTim1->setPrescaleFactor(1);                 // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
  // MyTim1->setOverflow(32768 * 2, HERTZ_FORMAT); // 100000 microseconds = 100 milliseconds
  // // MyTim1->attachInterrupt(Update_IT_callback);
  // // MyTim1->setInterruptPriority(0, 0);
  // MyTim1->resume();

  //__HAL_TIM_SET_COUNTER(&htim3, 327);
  // HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // TIM3->CNT = 23;

  // MyTim3->setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE, PA8);
  // MyTim3->setPrescaleFactor(1); // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
  // // MyTim3->setOverflow(32768 * 2, HERTZ_FORMAT); // 100000 microseconds = 100 milliseconds
  // // MyTim1->attachInterrupt(Update_IT_callback);
  // // MyTim1->setInterruptPriority(0, 0);
  // MyTim3->resume();

  // -------------------------------------------------------------------------
  u8g2.begin();
  int temp = 0;
  // u8g2.print(temp, 16);     // Костыль, убирает один странный баг
  // u8g2.clearBuffer();       // clear the internal memory
  digitalWrite(PB10, LOW);  // светодиод вкл
  digitalWrite(PA15, HIGH); // светодиод вкл

  // for (;;)
  // {
  // }

  // u8g2.clearBuffer(); // clear the internal memory

  // for (;;)
  // {
  // }

  u8g2.setFont(u8g2_font_mercutio_basic_nbp_tf);
  u8g2.drawStr(0, 12, "POWER ON "); // write something to the internal memory
  u8g2.sendBuffer();                // transfer internal memory to the display

  digitalWrite(PB10, HIGH); // светодиод вкл
  digitalWrite(PA15, LOW);  // светодиод вкл
  // for (;;)
  // {
  // }

  delay(200); // пауза
  u8g2.setFont(u8g2_font_mercutio_basic_nbp_tf);
  u8g2.drawStr(0, 24, "Init GPIO ");   // write something to the internal memory
  u8g2.sendBuffer();                   // transfer internal memory to the display
  delay(100);                          // пауза
  u8g2.drawStr(8 * 10, 24, "[OK]");    // write something to the internal memory
  delay(200);                          // пауза
  u8g2.sendBuffer();                   // transfer internal memory to the display
  u8g2.drawStr(0, 36, "Init Timers "); // write something to the internal memory
  u8g2.sendBuffer();                   // transfer internal memory to the display

  // MX_TIM3_Init();

  // delay(50);                          // пауза
  u8g2.drawStr(8 * 10, 36, "[OK]");   // write something to the internal memory
  delay(200);                         // пауза
  u8g2.sendBuffer();                  // transfer internal memory to the display
  u8g2.drawStr(0, 48, "Init TX .. "); // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display

  delay(400); // пауза для загрузки приемника

  // Wire.begin(); // join i2c bus

  // WriteReg(0x00, 0x80); // программный сброс

  // freqWord = FREQ_TO_WORD(freq);
  // SetFreq(freqWord);
  // WriteReg(0x0A, 0x03);       // настройка на канал 320 = 100 МГц
  // WriteReg(0x07, 0x20);       // настройка на канал 320 = 100 МГц
  // WriteReg(0x00, 0b00010001); // включение приема, RDS ON, MONO(1)/STEREO(0), сканирование выкл, ручной выбор канала,
  // WriteReg(0x14, 0b00000111); // Громкость 111 - макс

  // freq = 100.000;
  // freqWord = FREQ_TO_WORD(freq);
  // SetFreq(freqWord);

  u8g2.drawStr(8 * 10, 48, "[OK]"); // write something to the internal memory
  u8g2.sendBuffer();                // transfer internal memory to the display

  // delay(400); // пауза

  // u8g2.setContrast(0); // 0-255

  // for (;;)
  // {
  // }
}

//====================================================================================================== LOOP ========================================
void loop(void)
{
  digitalWrite(PA15, LOW); // светодиод вкл
  delay(20);
  digitalWrite(PA15, HIGH); // светодиод вкл
  /*
   // delay(3);
   // digitalWrite(PA15, HIGH); // светодиод вкл

   SignalNoise = ReadReg(REG_SNR);
   RSSI = ReadReg(REG_RSSISIG) - 43;
   stereoStatus = (ReadReg(REG_STATUS1) & REG_STATUS1_ST_MO_RX) ? 'M' : 'S';

   if (RDSstate == 1)
     decodeRDSInfo(RDS_DOUBLE_BUFFER_ENABLE);

   // digitalWrite(PA15, LOW); // светодиод выкл

   // Freq += 10;
   // if (Freq >= 110000)
   // Freq = 60000;

   EncNew = ((int)(TIM3->CNT) - 32768) / 4;

   if (Freq_Volume == 1) // управление громкостью
   {
     if (EncNew != EncOld)
     {
       volume += EncOld - EncNew;
       if (volume < 0)
         volume = 0;
       if (volume > 7)
         volume = 7;

       SetVol(volume);
     }
     EncOld = EncNew;
   }
   else
   {
     if (EncNew != EncOld)
     {
       freq += (EncOld - EncNew) * 0.05;
       freqWord = FREQ_TO_WORD(freq);
       SetFreq(freqWord);
       resetRDSInfo();
     }
     EncOld = EncNew;
   }
   // ------------- кнопка UP ----------------
   if (digitalRead(PB2) == LOW) // нажата кнопка RESET
   {

     if (digitalRead(PB1) == LOW) // нажата кнопка RESET
     {
       digitalWrite(PB10, HIGH); // светодиод вкл
       delay(100);
       if (digitalRead(PB1) == LOW) // нажата кнопка RESET
         powerOff();
     }

     if (triggerUP == 1)
     {
       triggerUP = 0;
       if (Stereo == 1)
       {
         Stereo = 0;
         WriteReg(0x00, 0b00011001); // включение приема, RDS ON, MONO(1)/STEREO(0), сканирование выкл, ручной выбор канала,
         digitalWrite(PA15, HIGH);   // светодиод вкл
       }
       else
       {
         Stereo = 1;
         WriteReg(0x00, 0b00011101); // включение приема, RDS ON, MONO(1)/STEREO(0), сканирование выкл, ручной выбор канала,
         digitalWrite(PA15, LOW);    // светодиод вкл
       }
     }
   }
   else
   {
     triggerUP = 1;
   }

   // ------------- кнопка DOWN ----------------
   if (digitalRead(PB1) == LOW) // нажата кнопка RESET
   {
     digitalWrite(PA15, HIGH); // светодиод вкл

     if (digitalRead(PB2) == LOW) // нажата кнопка RESET
     {
       digitalWrite(PB10, HIGH); // светодиод вкл
       delay(100);
       if (digitalRead(PB2) == LOW) // нажата кнопка RESET
         powerOff();
     }

     freq -= 0.050;
     freqWord = FREQ_TO_WORD(freq);
     SetFreq(freqWord);

     resetRDSInfo();

     delay(50);

     digitalWrite(PA15, LOW); // светодиод выкл
   }

   if (digitalRead(PA5) == LOW) // нажата кнопка энкодера'
   {
     if (triggerENC == 1)
     {
       triggerENC = 0;
       if (Freq_Volume == 1)
       {
         Freq_Volume = 0;
         digitalWrite(PB10, LOW); // светодиод вкл
       }
       else
       {
         Freq_Volume = 1;
         digitalWrite(PB10, HIGH); // светодиод вкл
       }

       delay(50);
     }
   }
   else
     triggerENC = 1;
 */
  //-------------------------------------------- Отображение на экране ----------------------------
  int temp = 0;
  u8g2.print(temp, 16); // Костыль, убирает один странный баг

  u8g2.clearBuffer(); // clear the internal memory

  // u8g2.setFont(u8g2_font_t0_12b_me); // choose a suitable font
  // u8g2.setFont(u8g2_font_lastapprenticebold_tr); // choose a suitable font
  // u8g2.setFont(u8g2_font_cupcakemetoyourleader_tr); // choose a suitable font
  // u8g2.setFont(u8g2_font_t0_16b_mn); // choose a suitable font
  // u8g2.setFont(u8g2_font_bauhaus2015_tr); // choose a suitable font
  // u8g2.setFont(u8g2_font_Pixellari_tf); // choose a suitable font
  // u8g2.setFont(u8g2_font_t0_17b_tf); // choose a suitable font
  // u8g2.setFont(u8g2_font_tenfatguys_tf);

  u8g2.setFont(u8g2_font_logisoso20_tr);
  u8g2.setCursor(0, 21);
  if (freq < 99.98)
    u8g2.setCursor(13, 21);
  u8g2.print(freq, 2);
  u8g2.setFont(u8g2_font_7x14B_mf);
  u8g2.setCursor(78, 21);
  u8g2.print("MHz V:");
  u8g2.print(volume);

  if (Freq_Volume == 1)
    u8g2.drawRFrame(103, 9, 25, 13, 2);

  u8g2.setFont(u8g2_font_6x13B_mf);
  u8g2.setCursor(100, 10);
  if (stereoStatus == 'S')
  {
    // u8g2.print("ST");
    u8g2.drawCircle(119 - 20, 4, 4);
    u8g2.drawCircle(123 - 20, 4, 4);
  }
  else
    u8g2.drawDisc(121 - 20, 4, 3);

  int VBat = 3800;
  int X_bat = 112;
  int Y_bat = 0;
  u8g2.drawFrame(0 + X_bat, 0 + Y_bat, 14, 7);
  u8g2.drawBox(15 + X_bat, 2 + Y_bat, 1, 3);
  if (VBat >= 3500)
    u8g2.drawBox(2 + X_bat, 2 + Y_bat, 1, 3);
  if (VBat >= 3690)
    u8g2.drawBox(X_bat + 2 + 3, Y_bat + 2, 1, 3);
  if (VBat >= 3780)
    u8g2.drawBox(X_bat + 2 + 3 + 3, Y_bat + 2, 1, 3);
  if (VBat >= 3850)
    u8g2.drawBox(X_bat + 2 + 3 + 3 + 3, Y_bat + 2, 1, 3);

  u8g2.setFont(u8g2_font_7x14B_mf);
  u8g2.setCursor(0, 34);
  u8g2.print("S/N");
  u8g2.setCursor(92 + 6, 34);
  if (SignalNoise < 10)
    u8g2.setCursor(100 + 6, 34);
  u8g2.print(SignalNoise);
  u8g2.setCursor(111 + 4, 34);
  u8g2.print("dB");

  if (SignalNoise > 61)
    SignalNoise = 61;
  u8g2.drawFrame(30, 26, 95 - 30, 6); // уровень SignalNoise
  u8g2.drawBox(32, 28, SignalNoise, 2);

  u8g2.setCursor(0, 35 + 12);
  u8g2.print("RSSI");
  u8g2.setCursor(98, 35 + 12);
  if (RSSI < 100)
    u8g2.setCursor(98, 35 + 12);
  if (RSSI < 10)
    u8g2.setCursor(106, 35 + 12);
  u8g2.print(RSSI);
  u8g2.setCursor(115, 35 + 12);
  u8g2.print("dB");
  RSSIcut = RSSI;
  if (RSSIcut > 61)
    RSSIcut = 61;
  u8g2.drawFrame(30, 27 + 12, 95 - 30, 6); // уровень RSSI
  u8g2.drawBox(32, 29 + 12, RSSIcut, 2);

  if (RDSstate == 1)
  {

    u8g2.setCursor(0, 63);
    u8g2.print("RDS: ");
    u8g2.setFont(u8g2_font_9x18B_mf);

    // u8g2.print(rdsInfo);
    for (i = 0; i < RDS_INFO_MAX_SIZE; i++)
    {
      u8g2.print(rdsInfo[i]);
    }
  }

  // u8g2.drawRFrame(30, 49, 128 - 30, 16, 3);
  u8g2.setCursor(30, 63);
  // u8g2.print(TIM3->CNT); //(&htim3)); // htim3.Instance->CNT);
  // u8g2.print(" ");
  // u8g2.print(temp1); //(&htim3)); // htim3.Instance->CNT);

  u8g2.sendBuffer(); // transfer internal memory to the display
                     // digitalWrite(PA15, LOW);  // светодиод выкл
                     // delay(1);
}

VOID decodeRDSInfo(unsigned char useDoubleBuffer)
{
  UCHAR offset;
  CHAR char1, char2;
  CHAR *buffer;
  USHORT groupB;

  // Construct RDS A,B,C,D packets.
  USHORT rdsA = GET_REG(REG_RDSD1) | GET_REG(REG_RDSD0) << 8;
  USHORT rdsB = GET_REG(REG_RDSD3) | GET_REG(REG_RDSD2) << 8;
  USHORT rdsC = GET_REG(REG_RDSD5) | GET_REG(REG_RDSD4) << 8;
  USHORT rdsD = GET_REG(REG_RDSD7) | GET_REG(REG_RDSD6) << 8;

  // Check for valid group A or B RDS packet(s).
  groupB = rdsB & RDS_GROUP;
  if ((groupB == RDS_GROUP_A0) || (groupB == RDS_GROUP_B0))
  {
    offset = (rdsB & 0x03) << 1;
    char1 = (CHAR)(rdsD >> 8);
    char2 = (CHAR)(rdsD & 0xFF);

    // Fill extracted characters and buffer offsets into primary and secondary arrays.
    if (offset < RDS_INFO_MAX_SIZE)
    {
      if (tempRDSBuffer[offset] == char1)
      {
        // 1st character verification is successful.
        rdsInfo[offset] = char1;
      }
      else if (isprint(char1))
      {
        buffer = useDoubleBuffer ? tempRDSBuffer : rdsInfo;
        buffer[offset] = char1;
      }

      if (tempRDSBuffer[offset + 1] == char2)
      {
        // 2nd character verification is successful.
        rdsInfo[offset + 1] = char2;
      }
      else if (isprint(char2))
      {
        buffer = useDoubleBuffer ? tempRDSBuffer : rdsInfo;
        buffer[offset + 1] = char2;
      }
    }
  }
}

VOID resetRDSInfo()
{
  // Fill primary RDS buffer with whitespaces.
  memset(rdsInfo, ' ', (RDS_INFO_MAX_SIZE - 1));
  rdsInfo[RDS_INFO_MAX_SIZE - 1] = 0x00;

  memset(tempRDSBuffer, ' ', (RDS_INFO_MAX_SIZE - 1));
  tempRDSBuffer[RDS_INFO_MAX_SIZE - 1] = 0x00;
}

void powerOff(void)
{
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.setFont(u8g2_font_mercutio_basic_nbp_tf);
  u8g2.drawStr(0, 12, "STNDBY RX .."); // write something to the internal memory
  u8g2.sendBuffer();                   // transfer internal memory to the display
  shutdownTuner();
  delay(200); // пауза
  u8g2.setFont(u8g2_font_mercutio_basic_nbp_tf);
  u8g2.drawStr(0, 24, "HI-Z GPIO ");  // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display
  delay(100);                         // пауза
  u8g2.drawStr(8 * 10, 24, "[OK]");   // write something to the internal memory
  delay(200);                         // пауза
  u8g2.sendBuffer();                  // transfer internal memory to the display
  u8g2.drawStr(0, 36, "Stop Timer "); // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display
  delay(50);                          // пауза
  u8g2.drawStr(8 * 10, 36, "[OK]");   // write something to the internal memory
  delay(200);                         // пауза
  u8g2.sendBuffer();                  // transfer internal memory to the display
  u8g2.drawStr(0, 48, "POWER OFF ");  // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display

  delay(400);                       // пауза для загрузки приемника
  u8g2.drawStr(8 * 10, 48, "[OK]"); // write something to the internal memory
  u8g2.sendBuffer();                // transfer internal memory to the display

  delay(400);          // пауза
  pinMode(PB0, INPUT); // вход кнопки POWER
}

void SetFreq(USHORT Freq)
{

  WriteReg(REG_CH, (Freq & 0xFF));             // Lo
  WriteReg(REG_CH_STEP, ((Freq >> 8) & 0x03)); // Hi
  // usleep(100);
  delayMicroseconds(100);

  temp1 = 1 << 4 | 1 << 3 | Stereo << 2 | 1 << 0;

  // temp1 = 0b00010101;
  WriteReg(REG_SYSTEM1, temp1);

  // Update global (default) frequency value.
  // currentFreq = Freq;
}

void WriteReg(int Addr, int Reg)
{
  Wire.beginTransmission(QN8027_ADDRESS); // transmit to device
  Wire.write(Addr);
  Wire.write(Reg);
  Wire.endTransmission(); // stop transmitting
}

int ReadReg(int Addr)
{
  int read;
  Wire.beginTransmission(QN8027_ADDRESS); // transmit to device
  Wire.write(Addr);
  Wire.endTransmission(); // stop transmitting

  Wire.beginTransmission(QN8027_ADDRESS); // transmit to device
  Wire.requestFrom(QN8027_ADDRESS, 1);
  while (Wire.available())
  {
    read = Wire.read();
  }
  Wire.endTransmission(); // stop transmitting
  return (read);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
  RCC->APB1ENR = RCC_APB1ENR_TIM3EN;
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM3->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;
  TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  TIM3->ARR = 65535;
  TIM3->CR1 = TIM_CR1_CEN;
  TIM3->CNT = 32768;
}

VOID shutdownTuner()
{
  // Reset and recalibrate the receiver.
  WriteReg(REG_SYSTEM1, REG_SYSTEM1_RECAL | REG_SYSTEM1_SWRST);
  delayMicroseconds(100);

  // Enter tuner into the standby mode.
  WriteReg(REG_SYSTEM1, REG_SYSTEM1_STNBY);
}

void SetVol(int vol)
{
  UCHAR volReg;
  // Update volume control with new value.
  volReg = (GET_REG(REG_VOL_CTL) & 0xF8) | vol;
  WriteReg(REG_VOL_CTL, volReg);
}

// Usually once a few seconds
void QN8027::rdsTick(void)
{
  _rotateRdsBuf(4);
}

void QN8027::rdsRefresh(void)
{
  _writeRds();
}

void QN8027::_rotateRdsBuf(uint8_t num)
{
  if (rdsBufPtr + num < rdsBufLength)
  {
    rdsBufPtr += num;
    pageNumber++;

    if (!(pageNumber % 2))
    {
      rdsDisplayPtr++;
    }
  }
  else
  {
    rdsBufPtr = 0;
    pageNumber = 0;
    rdsDisplayPtr = 0;
  }
}

void QN8027::_writeRegister(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(QN8027_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);
}

uint8_t QN8027::_readRegister(uint8_t addr)
{
  Wire.beginTransmission(QN8027_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(QN8027_ADDRESS, 1);
  return Wire.read();
}

void QN8027::_updateRegister(uint8_t addr, uint8_t data, uint8_t mask)
{
  uint8_t dataOld = _readRegister(addr);
  uint8_t dataNew = (dataOld & ~mask) | (data & mask);
  _writeRegister(addr, dataNew);
}

void QN8027::rdsBufSet(const char buf[RDSBUF_SIZE], uint16_t length)
{
  length -= 1;
  memcpy(rdsBuf, buf, length);
  memset(&rdsBuf[length], ' ', RDSBUF_SIZE - length);
  rdsBufPtr = 0;
  rdsBufLength = length;
  pageNumber = 0;
  rdsDisplayPtr = 0;
  pages = (uint16_t)((length >> 3)); // Division by 8 - fast math
}

void QN8027::begin(float fmFreq)
{
  Wire.begin();
  // Wire.setClock(100000);

  uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(fmFreq)));

  reset();

  _setFrequency(freqChannel);
}

#if (defined(ESP32) || defined(ESP8266))
void QN8027::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);

  uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(FREQ_DEFAULT)));

  reset();

  _setFrequency(freqChannel);
}

void QN8027::begin(uint8_t sda, uint8_t scl, float i2cFreq, float fmFreq)
{
  Wire.begin(sda, scl, i2cFreq);

  uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(fmFreq)));

  reset();

  _setFrequency(freqChannel);

  enable(true);
}
#endif

void QN8027::setTxPower(uint8_t pwr) // 20 - 75
{
  uint8_t data;
  if (pwr < 20)
    data = 20;
  else if (pwr > 75)
    data = 75;
  else
    data = pwr;

  _writeRegister(QN8027_REG_PAC, data);
}

void QN8027::enable(boolean enable)
{
  _updateRegister(QN8027_REG_SYSTEM, (enable << QN8027_BIT_IDLE), QN8027_MASK_IDLE);
}

void QN8027::rdsEnable(boolean enable)
{
  _updateRegister(QN8027_REG_RDS, (enable << QN8027_BIT_RDSEN), QN8027_MASK_RDSEN);
}

void QN8027::_setFrequency(uint16_t freqChannel)
{
  _writeRegister(QN8027_REG_CH1, (uint8_t)freqChannel);
  _updateRegister(QN8027_REG_SYSTEM, (uint8_t)(freqChannel >> 8), QN8027_MASK_CH_HI);
  //_updateRegister(QN8027_REG_SYSTEM, (1 << QN8027_BIT_MONO), QN8027_BIT_MONO);
}

void QN8027::setFrequency(float freq)
{
  uint16_t freqChannel = ((uint16_t)(FREQ_FLOAT2CHANNEL(freq)));

  // if (freqChannel > 640)
  // {
  //   freqChannel = 640;
  // }

  _setFrequency(freqChannel);
}

// If a user is time constrained they can optimize these delay times but it is not really necessary as you initialize the QN8027 only once
void QN8027::reset(void)
{
  _writeRegister(QN8027_REG_SYSTEM, 1 << QN8027_BIT_RESET);

  delay(80);

  _writeRegister(QN8027_REG_XTL, (((uint8_t)CLOCK_SQIN) << QN8027_BIT_XINJ) | 0x0f);

  delay(80);

  //_writeRegister(QN8027_REG_VGA, (((uint8_t)CLOCKSPEED_12) << QN8027_BIT_XSEL) | (((uint8_t)GAIN_0) << QN8027_BIT_GVGA) | (((uint8_t)TXGAIN_2) << QN8027_BIT_GDB) | (((uint8_t)RIN_1) << QN8027_BIT_RIN)); // 0b1010101 //4-2-2

  _writeRegister(QN8027_REG_VGA, 0b00110010);

  delay(80);

  _writeRegister(QN8027_REG_SYSTEM, 1 << QN8027_BIT_RECAL);

  delay(80);

  _writeRegister(QN8027_REG_RDS, 10); // Experimental - Sets N*0,35 kHz as a RDS Freq deviation

  delay(80);

  _writeRegister(QN8027_REG_SYSTEM, 0b00000000);

  delay(80);

  _writeRegister(QN8027_REG_GPLT, 0b00111001);

  delay(80);
}

void QN8027::getCurrentRds(char rdsData[9])
{
  char buf[9];
  buf[8] = '\0';
  memcpy(&buf[0], &rdsBuf[rdsDisplayPtr * 8], 8);
  memcpy(rdsData, buf, 9);
}

void QN8027::_writeRds()
{

  _writeRegister(QN8027_REG_RDSD0, 0b01000000); // 0b00100000
  _writeRegister(QN8027_REG_RDSD1, 0b00000000); // 0b00000000
  _writeRegister(QN8027_REG_RDSD2, 0b00001000); // 0b00100000
  _writeRegister(QN8027_REG_RDSD3, (uint8_t)(0b01000000 | (pageNumber & 0b1111)));

  _writeRegister(QN8027_REG_RDSD4, (uint8_t)rdsBuf[rdsBufPtr]);
  _writeRegister(QN8027_REG_RDSD5, (uint8_t)rdsBuf[rdsBufPtr + 1]);
  _writeRegister(QN8027_REG_RDSD6, (uint8_t)rdsBuf[rdsBufPtr + 2]);
  _writeRegister(QN8027_REG_RDSD7, (uint8_t)rdsBuf[rdsBufPtr + 3]);

  // if (_readRegister(QN8027_REG_STATUS) & (1 << QN8027_BIT_RDS_UPD))
  {

    uint8_t tmp = _readRegister(QN8027_REG_SYSTEM);

    boolean rdsUpdated = (tmp & (1 << QN8027_BIT_RDSRDY)) ? false : true;
    if (!rdsUpdated)
    {
      _writeRegister(QN8027_REG_SYSTEM, (tmp & (0xff - (1 << QN8027_BIT_RDSRDY))));
    }
    else
    {
      _writeRegister(QN8027_REG_SYSTEM, tmp | (1 << QN8027_BIT_RDSRDY));
    }
  }
  // else
  {
    // Serial.printf("Not updating RDS, exit.\r\n");
  }
}