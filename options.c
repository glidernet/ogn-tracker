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
   uint32_t  console_speed;
   uint32_t  gps_speed;
   uint32_t  AcftID;
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
{
   options.version       = OPTIONS_VER;
   options.console_speed = 4800;
   options.gps_speed     = 9600;
   options.AcftID        = 0x00000000;
}

/**
  * @brief  Function reads all options from EEPROM memory.
  * @param  void
  * @retval void
  */
void ReadOptions(void)
{
   uint16_t i;
   uint8_t* eeprom_addr = (uint8_t*)DATA_EEPROM_START_ADDR;
   uint8_t* ram_addr    = (uint8_t*)&options;
   uint8_t  options_len = sizeof(options);

   for (i=0; i<options_len; i++)
   {
      *ram_addr++ = *eeprom_addr++;
   }
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

   /* Unlock the FLASH PECR register and Data EEPROM memory */
   DATA_EEPROM_Unlock();
   /* Clear all pending flags */
   FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

   /* Erase */
   for (i=0; i<options_len; i++)
   {
      DATA_EEPROM_EraseByte(eeprom_addr+i);
   }
   /* Program */
   for (i=0; i<options_len; i++)
   {
      DATA_EEPROM_ProgramByte(eeprom_addr+i, *ram_addr++);
   }
   ram_addr = (uint8_t*)&options;
   /* Verify */
   for (i=0; i<options_len; i++)
   {
      if (*ram_addr++ != *(uint8_t*)(eeprom_addr+i))
      {
         ver_status = 1;
      }
   }

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
{
   ReadOptions();
   if (options.version != OPTIONS_VER)
   {
      ResetOptions();
      WriteOptions();
   }
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
   /* Erase */
   for (i=0; i<len; i++)
   {
      DATA_EEPROM_EraseByte(eeprom_addr+i);
   }
   /* Program */
   for (i=0; i<len; i++)
   {
      DATA_EEPROM_ProgramByte(eeprom_addr+i, *ram_addr++);
   }
   DATA_EEPROM_Lock();
}

/* ------------------------------------------------------ */
/**
  * @brief  Functions accessing option values.
  * @param  Depending on type.
  * @retval Depending on type.
  */
uint32_t* GetConsSpeed(void)
{
   return &options.console_speed;
}

void SetConsSpeed(uint32_t new_value)
{
   options.console_speed = new_value;
   WriteBlock(OFFSETOF(options_str, console_speed), sizeof(options.console_speed));
}

uint32_t* GetGPSSpeed(void)
{
   return &options.gps_speed;
}

void SetGPSSpeed(uint32_t new_value)
{
   options.gps_speed = new_value;
   WriteBlock(OFFSETOF(options_str, gps_speed), sizeof(options.gps_speed));
}

uint32_t* GetAcftID(void)
{
   return &options.AcftID;
}

void SetAcftID(uint32_t new_value)
{
   options.AcftID = new_value;
   WriteBlock(OFFSETOF(options_str, AcftID), sizeof(options.AcftID));
}


/* ------------------------------------------------------ */
/**
  * @brief  Entry point function for accessing option values.
  * @param  Selected option code.
  * @retval void* - should be casted to appr. type.
  */
void* GetOption(option_types opt_code)
{
   void* ret_val = NULL;

   switch (opt_code)
   { case OPT_CONS_SPEED: { ret_val = GetConsSpeed(); break; }
     case OPT_GPS_SPEED:  { ret_val = GetGPSSpeed();  break; }
     case OPT_ACFT_ID:    { ret_val = GetAcftID();    break; }
     default: break;
   }
   return ret_val;
}

void SetOption(option_types opt_code, void* value)
{
   switch (opt_code)
   {
     case OPT_CONS_SPEED:
     {
        SetConsSpeed(*(uint32_t*) value);
        break;
     }

     case OPT_GPS_SPEED:
     {
        SetGPSSpeed(*(uint32_t*) value);
        break;
     }
     default:
        break;
   }
}

