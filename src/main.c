#include "main.h"

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

int main(void)
{
    while (true)
    {
    }
}

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
