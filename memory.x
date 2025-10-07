/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 96K
RAM2 (xrw)      : ORIGIN = 0x10000000, LENGTH = 32K
FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 1024K
}

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of RAM */
