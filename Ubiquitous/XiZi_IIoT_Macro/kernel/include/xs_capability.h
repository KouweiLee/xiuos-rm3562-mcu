/*
* Copyright (c) 2020 AIIT XUOS Lab
* XiUOS is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*        http://license.coscl.org.cn/MulanPSL2
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
* See the Mulan PSL v2 for more details.
*/

/**
* @file:    xs_capability.h
* @brief:   capability header file for XiZi-IIoT
* @version: 1.0
* @author:  AIIT XUOS Lab
* @date:    2024/10/14
*
*/

#ifndef XS_CAPABILITY_H
#define XS_CAPABILITY_H

/*  import  board special configuration */
#include <xsconfig.h>
#include <xs_base.h>
#include <xs_memory.h>
#include <xs_assign.h>


// assume the number of capability types is limited to 8*MAX_NUM_TA
typedef enum {
    XS_CAP = 0,
    XS_CAP_ROOT,
    XS_CAP_TASK,
    XS_CAP_MEM,
    XS_CAP_RESOURCES,
    XS_CAP_MAX  // the number of capability types
} xs_capability_type;

typedef struct {
    uint8 flags[XS_CAP_MAX/8+1];
} xs_capability, *xs_capability_t;

typedef struct {
    int32 pid;
    xs_capability cap;
} xs_task_capability, *xs_task_capability_t;

typedef struct capability_node{
    xs_task_capability inner;
    struct capability_node* next;
} capability_node, *capability_node_t;


// functions to manipulate capability flags
int CheckCapability(xs_capability *cap, xs_capability_type type);

// set_capability and clear_capability are used to manipulate capability flags
void SetCapability(xs_capability *cap, xs_capability_type type);

// clear_capability is used to manipulate capability flags
void ClearCapability(xs_capability *cap, xs_capability_type type);

// functions to manipulate task capabilities
xs_capability* FindTaskCapability(int32 pid);

// add_task_capability and remove_task_capability are used to manipulate task capabilities
void AddTaskCapability(int32 pid, xs_capability *cap);

// remove_task_capability is used to manipulate task capabilities
void RemoveTaskCapability(int32 pid);

// add_root_task_capability is used to manipulate task capabilities
void AddRootTaskCapability(int32 pid);

// Capability_Copy is used to deep copy capabilitie 
void CapabilityCopy(xs_capability *cap, xs_capability *cap_copy_from);

// check_task_capability is used to check if a task has a certain capability
x_bool CheckTaskCapability(int32 pid, xs_capability_type type);

// set_task_capability and clear_task_capability are used to manipulate task capabilities
void SetTaskCapability(int32 pid, xs_capability_type type);

// clear_task_capability is used to manipulate task capabilities
void ClearTaskCapability(int32 pid, xs_capability_type type);

// test
void PrintTaskCapability();

typedef struct CapsOps
{
    int (*CheckCap) (xs_capability *cap, xs_capability_type type);
    void (*SetCap) (xs_capability *cap, xs_capability_type type);
    void (*ClearCap) (xs_capability *cap, xs_capability_type type);
    xs_capability* (*GetTaskCaps) (int32 pid);
    void (*AddTaskCaps) (int32 pid, xs_capability *cap);
    void (*RemoveTaskCaps) (int32 pid);
    void (*AddRootTaskCaps) (int32 pid);
    void (*Copy)(xs_capability *cap, xs_capability *cap_copy_from);
    x_bool (*CheckTaskCap)(int32 pid, xs_capability_type type);
    void (*SetTaskCap) (int32 pid, xs_capability_type type);
    void (*ClearTaskCap) (int32 pid, xs_capability_type type);
    void (*Print) ();
} CapsOps;

#endif /* XS_CAPABILITY_H */ 


