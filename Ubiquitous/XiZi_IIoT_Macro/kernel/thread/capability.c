#include "xs_capability.h"

#ifdef __cplusplus
#define NULL 0L
#else
#define NULL ((void*)0)
#endif

// global task capability list, used to manage task capabilities
static capability_node task_capabilities={
    .next = NULL
};

CapsOps CapsOperations = {
    CheckCapability,
    SetCapability,
    ClearCapability,
    FindTaskCapability,
    AddTaskCapability,
    RemoveTaskCapability,
    AddRootTaskCapability,
    CapabilityCopy,
    CheckTaskCapability,
    SetTaskCapability,
    ClearTaskCapability,
    PrintTaskCapability,
};

// functions to manipulate capability flags
int CheckCapability(xs_capability *cap, xs_capability_type type){
    return cap->flags[type/8] & (1 << (type%8));
}
// set_capability and clear_capability are used to manipulate capability flags
void SetCapability(xs_capability *cap, xs_capability_type type){
    cap->flags[type/8] |= (1 << (type%8));
}

// clear_capability is used to manipulate capability flags
void ClearCapability(xs_capability *cap, xs_capability_type type){
    cap->flags[type/8] &= ~(1 << (type%8));
}

// functions to manipulate task capabilities
xs_capability* FindTaskCapability(int32 pid){
    capability_node* node = task_capabilities.next;
    while(node){
        if(node->inner.pid == pid){
            return &node->inner.cap;
        }
        node = node->next;
    }
    return NULL;
}

// test print
void PrintTaskCapability(){
    x_base lock = CriticalAreaLock();
    capability_node* node = task_capabilities.next;
    KPrintf("All cap:\n");
    while(node){
        KPrintf("pid : %d,  : %d \n", node->inner.pid, node->inner.cap.flags[0]);
        node = node->next;
    }
    KPrintf("\n");
    CriticalAreaUnLock(lock);
}


// add_task_capability and remove_task_capability are used to manipulate task capabilities
void AddTaskCapability(int32 pid, xs_capability *cap){
    x_base lock = CriticalAreaLock();
    capability_node* node = (capability_node*)x_malloc(sizeof(capability_node));
    node->inner.pid = pid;
    node->inner.cap = *cap;
    node->next = task_capabilities.next;
    task_capabilities.next = node;
    CriticalAreaUnLock(lock);
}

// add_root_task_capability is used to manipulate task capabilities
void AddRootTaskCapability(int32 pid){
    x_base lock = CriticalAreaLock();
    capability_node* node = (capability_node*)x_malloc(sizeof(capability_node));
    xs_capability *cap = (xs_capability*)x_malloc(sizeof(xs_capability));
    for(int i = 0; i < XS_CAP_MAX/8+1; i++) cap->flags[i] = 0xFF;
    node->inner.pid = pid;
    node->inner.cap = *cap;
    node->next = task_capabilities.next;
    task_capabilities.next = node;
    CriticalAreaUnLock(lock);
}

// Capability_Copy is used to deep copy capabilitie 
void CapabilityCopy(xs_capability *cap, xs_capability *cap_copy_from){
    for(int i = 0; i < XS_CAP_MAX/8+1; i++){
        cap->flags[i] = cap_copy_from->flags[i];
    }
} 

// remove_task_capability is used to manipulate task capabilities
void RemoveTaskCapability(int32 pid){
    x_base lock = CriticalAreaLock();
    capability_node* node = &task_capabilities;
    while(node->next){
        if(node->next->inner.pid == pid){
            capability_node* removing = node->next;
            node->next = node->next->next;
            x_free(removing);
            CriticalAreaUnLock(lock);
            return;
        }
        node = node->next;
    }
    CriticalAreaUnLock(lock);
}

// check_task_capability is used to check if a task has a certain capability
x_bool CheckTaskCapability(int32 pid, xs_capability_type type){
    x_base lock = CriticalAreaLock();
    xs_capability* cap = FindTaskCapability(pid);
    x_bool result = cap && CheckCapability(cap, type);
    CriticalAreaUnLock(lock);
    return result;
}

// set_task_capability and clear_task_capability are used to manipulate task capabilities
void SetTaskCapability(int32 pid, xs_capability_type type){
    int32 current = GetKTaskDescriptor()->id.id;
    xs_capability* cap = FindTaskCapability(current);
    if(CheckCapability(cap, XS_CAP_ROOT)){
        x_base lock = CriticalAreaLock();
        xs_capability* cap = FindTaskCapability(pid);
        if(cap){
            SetCapability(cap, type);
        }
        CriticalAreaUnLock(lock);
    } else {
        KPrintf("No permission to SetTaskCapability\n");
    }

}

// clear_task_capability is used to manipulate task capabilities
void ClearTaskCapability(int32 pid, xs_capability_type type){
    int32 current = GetKTaskDescriptor()->id.id;
    xs_capability* cap = FindTaskCapability(current);
    if(CheckCapability(cap, XS_CAP_ROOT) || current == pid){
        x_base lock = CriticalAreaLock();
        xs_capability* cap = FindTaskCapability(pid);
        if(cap){
            ClearCapability(cap, type);
        }
        CriticalAreaUnLock(lock);
    } else {
        KPrintf("No permission to ClearTaskCapability\n");
    }
}



