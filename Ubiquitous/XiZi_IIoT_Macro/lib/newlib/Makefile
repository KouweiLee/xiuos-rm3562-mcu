SRC_FILES := stdio.c fs_syscalls.c time_syscalls.c mem_syscalls.c task_syscalls.c

$(eval _SDIDINIT_EXISTS := $(shell echo \
	'#include <sys/reent.h>\nint main() { (void)_GLOBAL_REENT->__sdidinit; return 0; }' \
	| $(CROSS_COMPILE)gcc $(CFLAGS) -x c -o /dev/null -c - 2>/dev/null && echo 1 || echo 0))

ifeq ($(_SDIDINIT_EXISTS), 1)
CFLAGS += -D_SDIDINIT_EXISTS
endif

include $(KERNEL_ROOT)/compiler.mk
