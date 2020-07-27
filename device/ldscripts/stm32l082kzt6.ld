/*
Linker subscript for STM32L082 definitions with 192K Flash and 20K RAM
*/

/* Memory Spaces Definitions */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 192K
  RAM  (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* highest address of the user mode stack */
_estack = 0x20005000;
