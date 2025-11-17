# PikaPython Makefile for XiUOS

# IMPORTANT:
# This build system expects SRC_FILES (list of .c files) or recurses into sub-makefiles.
# To ensure files are compiled, enumerate source files explicitly.

SRC_FILES :=

ifeq ($(CONFIG_LIB_USING_PIKAPYTHON),y)
    # PikaPython core files
    SRC_FILES += pikascript-core/PikaVM.c
    SRC_FILES += pikascript-core/PikaObj.c
    SRC_FILES += pikascript-core/PikaParser.c
    SRC_FILES += pikascript-core/PikaCompiler.c
    SRC_FILES += pikascript-core/BaseObj.c
    SRC_FILES += pikascript-core/dataArg.c
    SRC_FILES += pikascript-core/dataArgs.c
    SRC_FILES += pikascript-core/dataLink.c
    SRC_FILES += pikascript-core/dataLinkNode.c
    SRC_FILES += pikascript-core/dataMemory.c
    SRC_FILES += pikascript-core/dataQueue.c
    SRC_FILES += pikascript-core/dataQueueObj.c
    SRC_FILES += pikascript-core/dataStack.c
    SRC_FILES += pikascript-core/dataString.c
    SRC_FILES += pikascript-core/dataStrs.c
    SRC_FILES += pikascript-core/TinyObj.c
    SRC_FILES += pikascript-core/PikaPlatform.c
    
    # Pre-compiled Python API files
    SRC_FILES += pikascript-api/pikaScript.c
    SRC_FILES += pikascript-api/__pikaBinding.c
    SRC_FILES += pikascript-api/__asset_pikaModules_py_a.c
    
    # PikaStdLib files (basic functionality)
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdLib_RangeObj.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdLib_StringObj.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdLib_SysObj.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_ByteArray.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_Dict.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_FILEIO.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_List.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_String.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_Tuple.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdData_Utils.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdTask_Task.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaDebuger_Debuger.c
    SRC_FILES += pikascript-lib/PikaStdLib/PikaStdLib_MemChecker.c
endif

include $(KERNEL_ROOT)/compiler.mk
