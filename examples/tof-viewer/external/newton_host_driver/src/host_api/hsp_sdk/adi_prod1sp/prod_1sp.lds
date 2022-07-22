OUTPUT_ARCH( "riscv" )

MEMORY
{
  ROM (rx) : ORIGIN = 0xFFFF0000, LENGTH = 0x4000
  RAM (rwx) : ORIGIN = 0xFFF00000, LENGTH = 0x8000
}

ENTRY(_start)

_stack_size = 0x400;
_hob_size = 0x100;
_1sp_image_offset = 4 * 1024;
_1sp_image_start = ORIGIN(RAM) + _1sp_image_offset;
_1sp_text_offset = 8 * 1024;

SECTIONS
{
    . = ORIGIN(RAM);
	.stack :
	{
		_start_stack = .;
		. = . + _stack_size;
		PROVIDE( _sp = . );
		_end_stack = .;
	} > RAM

    . = ALIGN(4);

    .hob :
    {
        PROVIDE(_hob_addr = .);
        . = . + _hob_size;
    } > RAM

    . = ALIGN(4);

    .bss :
    {
        _start_bss = .;
        *(.bss* .sbss*)
        *(COMMON)
        . = ALIGN(4);
        _end_bss = .;
    } > RAM

    . = ALIGN(4);

    .data (_1sp_image_start) :
    {
        _start_data = .;
        *(.data.fuse) /* Put fuse data at the known position(start of data RAM) */
        . = _start_data + 0x6c; /* Reserve size for fuse data */ 
        *(.rodata .rodata.*)
        *(.data .data.*)
        *(.sdata*)
        . = ALIGN(4);
        _end_data = .;
    } > RAM

    .text (ORIGIN(RAM) + _1sp_text_offset) :
    {
        _start_text = .;
        KEEP(*(.text.init))
        KEEP(*(_1sp_trap_vector))
        /* KEEP(*(_trap_vector)) */
        *(.text .text.*)
        . = ALIGN(4);
        _end_text = .;
    } > RAM
}
