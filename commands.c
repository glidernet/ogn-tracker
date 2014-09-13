#include "commands.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>

/* -------- defines -------- */
/* -------- variables -------- */
/* -------- constants -------- */
static const char * const pcVersion = "0.0.1\r\n";
/* -------- functions -------- */

/**
  * @brief  Command ver: prints version string.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvVerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
   strcpy((char*)pcWriteBuffer, pcVersion);
   return pdFALSE;
}

/* -------- additional command constants -------- */

static const CLI_Command_Definition_t VerCommand =
{
    "ver",
    "ver: version number\r\n",
    prvVerCommand,
    0
};

/**
  * @brief  Function registers all console commands.
  * @param  None
  * @retval None
  */
void RegisterCommands(void)
{
   /* The commands are displayed in help in the order provided here */
   FreeRTOS_CLIRegisterCommand(&VerCommand);
}
