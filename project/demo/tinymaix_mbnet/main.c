#include "common/framework/platform_init.h"
#include "kernel/os/os.h"
#include <stdio.h>

extern int tinymaix_loader();

int main(void)
{
    platform_init();
    tinymaix_loader();
    return 0;
}