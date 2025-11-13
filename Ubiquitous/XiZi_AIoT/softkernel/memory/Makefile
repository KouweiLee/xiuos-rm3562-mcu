#SRC_FILES:= kalloc.c pagetable.c pagetable_level2.c buddy.c object_allocator.c share_page.c

#include $(KERNEL_ROOT)/compiler.mk

ifneq ($(findstring $(BOARD), 3588), )
SRC_FILES := kalloc.c pagetable.c pagetable_level3.c buddy.c object_allocator.c share_page.c
endif
ifneq ($(findstring $(BOARD), 3568), )
SRC_FILES := kalloc.c pagetable.c pagetable_level3.c buddy.c object_allocator.c share_page.c
endif
ifneq ($(findstring $(BOARD), imx6q-sabrelite zynq7000-zc702), )
SRC_FILES:= kalloc.c pagetable.c pagetable_level2.c buddy.c object_allocator.c share_page.c
endif
ifneq ($(findstring $(BOARD), jh7110), )
SRC_FILES := kalloc.c pagetable_riscv.c pagetable_riscv_level3.c buddy.c object_allocator.c share_page.c
endif


include $(KERNEL_ROOT)/compiler.mk
