#include "options.h"
#include "stm32l1xx.h"
#include <stdint.h>
#include <stddef.h>

/* -------- defines -------- */
#define OPTIONS_VER 1

#define DATA_EEPROM_START_ADDR  0x08080000
#define DATA_EEPROM_END_ADDR    0x08083FFF

#define OFFSETOF(type, field) ((unsigned long) &(((type *)0)->field))

typedef struct
{
   uint8_t   version;
   uint32_t  console_speed;    // [bps]
   uint32_t  gps_speed;        // [bps]
   uint32_t  AcftID;
   float     TxPower;          // [dBm]
   int16_t   XtalCorr;         // [ppm]
   int32_t   FreqOfs;          // [Hz]
   uint8_t   IWDGDis;          // [0 - IWDG enabled ]
   uint8_t   oper_mode;        // Tracker operation mode
   uint8_t   channel;          // Selected channel for test modes
   uint8_t   gpsdump;          // GPS dump to console
   float     MaxTxPower;       // [dBm]
   uint8_t   gps_always_on;    // Used for GPS without power control
   uint8_t   gps_ant;          // GPS antenna status: 0 - internal, 1 - external
   uint8_t   jam_ratio;        // Jamming ratio: 0 - 100 %
   uint16_t  min_bat_level;    // Minimum battery level
   uint16_t  gps_wdg_time;     // GPS watchdog time [s]
} options_str;

/* -------- variables -------- */
static options_str options;

/* -------- functions -------- */

/**
  * @brief  Function resets all options to default values.
  * @param  void
  * @retval void
  */
void ResetOptions(void)
{ options.version       = OPTIONS_VER;
  options.console_speed = 4800;         // [bps]
  options.gps_speed     = 9600;         // [bps]
  options.AcftID        = 0x07000000 | ((*(uint32_t*)0x1FF80050)&0x00FFFFFF); // set the address to the unique-ID of the CPU
  options.TxPower       = 14.0F;        // [dBm]
  options.XtalCorr      =    0;         // [ppm]
  options.FreqOfs       =    0;         // [Hz]
  options.IWDGDis       =    0;         // [IWDG enabled]
  options.oper_mode     = (uint8_t)MODE_OGN;
  options.channel       =    4;         // 868.4 MHz
  options.gpsdump       =    0;         // Disabled
  options.MaxTxPower    = 14.5F;        // Maximum measured power
  options.gps_always_on =    0;         // Disabled
  options.gps_ant       =    0;         // Internal antenna
  options.jam_ratio     =   10;         // 10%
  options.min_bat_level = 3100;         // 3.1V
  options.gps_wdg_time  = 60;           // 60 sec
}

/**
  * @brief  Function reads all options from EEPROM memory.
  * @param  void
  * @retval void
  */
void ReadOptions(void)
{ uint8_t* eeprom_addr = (uint8_t*)DATA_EEPROM_START_ADDR;
  uint8_t* ram_addr    = (uint8_t*)&options;
  uint8_t  options_len = sizeof(options);
  for (int i=0; i<options_len; i++)
  { *ram_addr++ = *eeprom_addr++; }
}

/**
  * @brief  Function stores all options in EEPROM memory.
  * @param  void
  * @retval verification status
  */
uint8_t WriteOptions(void)
{
   uint16_t i;
   uint32_t eeprom_addr = DATA_EEPROM_START_ADDR;
   uint8_t* ram_addr    = (uint8_t*)&options;
   uint8_t  options_len = sizeof(options);
   uint8_t  ver_status  = 0;
   //FLASH_Status FLASHStatus = FLASH_COMPLETE;

   DATA_EEPROM_Unlock();                                                   // Unlock the FLASH PECR register and Data EEPROM memory
   FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR    // Clear all pending flags
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

   for (i=0; i<options_len; i++)                                           // Erase
     DATA_EEPROM_EraseByte(eeprom_addr+i);
   for (i=0; i<options_len; i++)                                           // Program
     DATA_EEPROM_ProgramByte(eeprom_addr+i, *ram_addr++);
   ram_addr = (uint8_t*)&options;
   for (i=0; i<options_len; i++)                                           // Verify
   { if (*ram_addr++ != *(uint8_t*)(eeprom_addr+i)) ver_status = 1; }

   DATA_EEPROM_Lock();
   return ver_status;
}

/**
  * @brief  Function initializes options structure using data from EEPROM.
  * @brief  Options are initialized to default values if version invalid.
  * @param  void
  * @retval void
  */
void InitOptions(void)
{ ReadOptions();
   if (options.version != OPTIONS_VER)
   { ResetOptions();
     WriteOptions(); }
}

/**
  * @brief  Writes selected block of option structure memory to EEPROM.
  * @param  offset: beginning of block to write, length of block.
  * @retval void
  */
void WriteBlock(uint16_t offset, uint8_t len)
{
   uint8_t i;
   uint32_t eeprom_addr = DATA_EEPROM_START_ADDR + offset;
   uint8_t* ram_addr    = (uint8_t*)&options + offset;

   DATA_EEPROM_Unlock();
   for (i=0; i<len; i++)                                 // Erase
     DATA_EEPROM_EraseByte(eeprom_addr+i);
   for (i=0; i<len; i++)                                 // Program
     DATA_EEPROM_ProgramByte(eeprom_addr+i, *ram_addr++);
   DATA_EEPROM_Lock();
}

/* ------------------------------------------------------ */
/**
  * @brief  Functions accessing option values.
  * @param  Depending on type.
  * @retval Depending on type.
  */
uint32_t* GetConsSpeed(void)
{ return &options.console_speed; }

void SetConsSpeed(uint32_t new_value)
{ options.console_speed = new_value;
  WriteBlock(OFFSETOF(options_str, console_speed), sizeof(options.console_speed)); }

uint32_t* GetGPSSpeed(void)
{ return &options.gps_speed; }

void SetGPSSpeed(uint32_t new_value)
{ options.gps_speed = new_value;
  WriteBlock(OFFSETOF(options_str, gps_speed), sizeof(options.gps_speed)); }

uint32_t* GetAcftID(void)
{ return &options.AcftID; }

void SetAcftID(uint32_t new_value)
{ options.AcftID = new_value;
  WriteBlock(OFFSETOF(options_str, AcftID), sizeof(options.AcftID)); }

float* GetTxPower(void)
{ return &options.TxPower; }

void SetTxPower(float new_value)
{ options.TxPower = new_value;
  WriteBlock(OFFSETOF(options_str, TxPower), sizeof(options.TxPower)); }
  
float* GetMaxTxPower(void)
{ return &options.MaxTxPower; }

void SetMaxTxPower(float new_value)
{ options.MaxTxPower = new_value;
  WriteBlock(OFFSETOF(options_str, MaxTxPower), sizeof(options.MaxTxPower)); }

int16_t *GetXtalCorr(void)
{ return &options.XtalCorr; }

void SetXtalCorr(int16_t new_value)
{ options.XtalCorr = new_value;
  WriteBlock(OFFSETOF(options_str, XtalCorr), sizeof(options.XtalCorr)); }

int32_t *GetFreqOfs(void)
{ return &options.FreqOfs; }

void SetFreqOfs(int32_t new_value)
{ options.FreqOfs = new_value;
  WriteBlock(OFFSETOF(options_str, FreqOfs), sizeof(options.FreqOfs)); }
  
uint8_t* GetIWDGDis(void)
{ return &options.IWDGDis; }

void SetIWDGDis(uint8_t new_value)
{ options.IWDGDis = new_value;
  WriteBlock(OFFSETOF(options_str, IWDGDis), sizeof(options.IWDGDis)); }

uint8_t* GetOperMode(void)
{ return &options.oper_mode; }

void SetOperMode(uint8_t new_value)
{ options.oper_mode = new_value;
  WriteBlock(OFFSETOF(options_str, oper_mode), sizeof(options.oper_mode)); }

uint8_t* GetChannel(void)
{ return &options.channel; }

void SetChannel(uint8_t new_value)
{ options.channel = new_value;
  WriteBlock(OFFSETOF(options_str, channel), sizeof(options.channel)); }
  
uint8_t* GetGPSDump(void)
{ return &options.gpsdump; }

void SetGPSDump(uint8_t new_value)
{ options.gpsdump = new_value;
  WriteBlock(OFFSETOF(options_str, gpsdump), sizeof(options.gpsdump)); }

uint8_t* GetGPSAlwON(void)
{ return &options.gps_always_on; }

void SetGPSAlwON(uint8_t new_value)
{ options.gps_always_on = new_value;
  WriteBlock(OFFSETOF(options_str, gps_always_on), sizeof(options.gps_always_on)); }
  
uint8_t* GetGPSAnt(void)
{ return &options.gps_ant; }

void SetGPSAnt(uint8_t new_value)
{ options.gps_ant = new_value;
  WriteBlock(OFFSETOF(options_str, gps_ant), sizeof(options.gps_ant)); }

uint8_t* GetJamRatio(void)
{ return &options.jam_ratio; }
  
void SetJamRatio(uint8_t new_value)
{ options.jam_ratio = new_value;
  WriteBlock(OFFSETOF(options_str, jam_ratio), sizeof(options.jam_ratio)); }
  
uint16_t* GetMinBatLvl(void)
{ return &options.min_bat_level; }
  
void SetMinBatLvl(uint16_t new_value)
{ options.min_bat_level = new_value;
  WriteBlock(OFFSETOF(options_str, min_bat_level), sizeof(options.min_bat_level)); }
  
uint16_t* GetGPSWdgTime(void)
{ return &options.gps_wdg_time; }
 
void SetGPSWdgTime(uint16_t new_value)
{ options.gps_wdg_time = new_value;
  WriteBlock(OFFSETOF(options_str, gps_wdg_time), sizeof(options.gps_wdg_time)); }

 
/* ------------------------------------------------------ */
/**
  * @brief  Entry point function for accessing option values.
  * @param  Selected option code.
  * @retval void* - should be casted to appr. type.
  */
void* GetOption(option_types opt_code)
{ void* ret_val = NULL;
  switch (opt_code)
  { case OPT_CONS_SPEED: { ret_val = GetConsSpeed(); break; }
    case OPT_GPS_SPEED:  { ret_val = GetGPSSpeed();  break; }
    case OPT_ACFT_ID:    { ret_val = GetAcftID();    break; }
    case OPT_TX_POWER:   { ret_val = GetTxPower();   break; }
    case OPT_XTAL_CORR:  { ret_val = GetXtalCorr();  break; }
    case OPT_FREQ_OFS:   { ret_val = GetFreqOfs();   break; }
    case OPT_IWDG:       { ret_val = GetIWDGDis();   break; }
    case OPT_OPER_MODE:  { ret_val = GetOperMode();  break; }
    case OPT_CHANNEL:    { ret_val = GetChannel();   break; }
    case OPT_GPSDUMP:    { ret_val = GetGPSDump();   break; }
    case OPT_MAX_TX_PWR: { ret_val = GetMaxTxPower(); break;}
    case OPT_GPS_ALW_ON: { ret_val = GetGPSAlwON();  break; }
    case OPT_GPS_ANT:    { ret_val = GetGPSAnt();    break; }
    case OPT_JAM_RATIO:  { ret_val = GetJamRatio();  break; }
    case OPT_MIN_BAT_LVL:{ ret_val = GetMinBatLvl(); break; }
    case OPT_GPS_WDG_TIME:{ ret_val = GetGPSWdgTime(); break; }
    default: break; }
  return ret_val; }

void SetOption(option_types opt_code, void* value)
{ switch (opt_code)
  { case OPT_CONS_SPEED: { SetConsSpeed(*(uint32_t *) value); break; }
    case OPT_GPS_SPEED:  { SetGPSSpeed (*(uint32_t *) value); break; }
    case OPT_ACFT_ID:    { SetAcftID   (*(uint32_t *) value); break; }
    case OPT_TX_POWER:   { SetTxPower  (*(float    *) value); break; }
    case OPT_XTAL_CORR:  { SetXtalCorr (*(int16_t  *) value); break; }
    case OPT_FREQ_OFS:   { SetFreqOfs  (*(int32_t  *) value); break; }
    case OPT_IWDG:       { SetIWDGDis  (*(uint8_t  *) value); break; }
    case OPT_OPER_MODE:  { SetOperMode (*(uint8_t  *) value); break; }
    case OPT_CHANNEL:    { SetChannel  (*(uint8_t  *) value); break; }
    case OPT_GPSDUMP:    { SetGPSDump  (*(uint8_t  *) value); break; }
    case OPT_MAX_TX_PWR: { SetMaxTxPower  (*(float *) value); break; }
    case OPT_GPS_ALW_ON: { SetGPSAlwON (*(uint8_t  *) value); break; }
    case OPT_GPS_ANT:    { SetGPSAnt   (*(uint8_t  *) value); break; }
    case OPT_JAM_RATIO:  { SetJamRatio (*(uint8_t  *) value); break; }
    case OPT_MIN_BAT_LVL:{ SetMinBatLvl(*(uint16_t *) value); break; }
    case OPT_GPS_WDG_TIME:{SetGPSWdgTime(*(uint16_t *) value); break;}
    default: break; }
}

