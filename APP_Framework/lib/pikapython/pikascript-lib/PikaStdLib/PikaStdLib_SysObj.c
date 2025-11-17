#include "../../pikascript-api/PikaStdLib_SysObj.h"
#include "../../pikascript-api/PikaStdData_FILEIO.h"
#include "../../pikascript-api/PikaStdLib_REPL.h"
#include "../../pikascript-core/PikaVM.h"
#include "../../pikascript-core/dataStrs.h"

void PikaStdLib_REPL_setEcho(PikaObj* self, pika_bool echo) {
    pikaShellSetEcho(echo);
}
