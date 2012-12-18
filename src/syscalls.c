/*
 * syscalls.c
 *
 *  Created on: Dec 18, 2012
 *      Author: XPMUser
 */
//
//      RM0090 Reference manual - STM32F4xx advanced ARM-based 32-bit MCUs
//      PM0056 Programming manual - STM32Fxxx Cortex-M3 programming manual
//      UM1472 Users manual - STM32F4DISCOVERY discovery board
//      http://sourceware.org/newlib/libc.html
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <errno.h>
#include "syscalls.h"
#include "allinclude.h"

#define MARKS(x) (*(uint32_t *) 0x10000000) |= (1<<(x))
// register char *stack_ptr asm ("sp");
// extern char stack_ptr asm ("_endof_sram"); // Defined by the linker
// extern char _end       asm ("_end"); // Defined by the linker
//extern char _endof_sram;
extern char _end;
static char *heap_end=0;
extern uint32_t _Min_Heap_Size;
caddr_t _sbrk(size_t incr){                     // this is !! //
        char *prev_heap_end;
        char *stack;
//      stack= (char *)__get_MSP();
//      stack=  &_endof_sram;
        stack= & _end + _Min_Heap_Size;

        //MARKS(13);
#ifdef SBRK_DEBUG
        TIM4->CCR4=0xff;
        putstr("\r\nheap_end=");
        printLONG((uint32_t)heap_end);
#endif
        if( heap_end == NULL )heap_end = & _end;
        prev_heap_end = heap_end;

#ifdef SBRK_DEBUG
        putstr("...");
        printLONG((uint32_t)heap_end);
#endif

        if(( heap_end + incr ) > stack){
                return (caddr_t)(-1);   // errno= ENOMEM;
        };
        heap_end += incr;

#ifdef SBRK_DEBUG
        putstr(" ... heap_return=");
        printLONG((uint32_t)prev_heap_end);
#endif
        return( (caddr_t) prev_heap_end );
}
