/* GPL-3.0-or-later. Offline link layout for the experimental module format. */
ENTRY(start)
SECTIONS
{
    . = DEFINED(MODULE_BASE) ? MODULE_BASE : 0x10000;
    .text : { *(.text .text.*) }
    .rodata : { *(.rodata .rodata.*) }
    .ARM.exidx : { *(.ARM.exidx .ARM.exidx.*) }
    . = ALIGN(4096);
    .data : { *(.data .data.*) *(.data.rel.ro .data.rel.ro.*) }
    /* Private .ap_rust_import_<hash> input sections intentionally remain
       separate orphan RW sections. Both pinned linkers place these with data;
       the packer validates their placement, size and placeholder. No KEEP:
       unused wrappers and their imports must be garbage-collected. */
    .got : { *(.got) }
    .bss (NOLOAD) : { *(.bss .bss.*) *(COMMON) }
    /DISCARD/ : { *(.eh_frame*) *(.comment) *(.note*) }
}
