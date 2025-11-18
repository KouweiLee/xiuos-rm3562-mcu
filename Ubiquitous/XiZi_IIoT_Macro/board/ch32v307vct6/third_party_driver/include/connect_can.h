





#ifndef CONNECT_CAN_H
#define CONNECT_CAN_H


#include <bus_can.h>
#include <dev_can.h>


#include <ch32v30x.h>
#include <ch32v30x_can.h>


struct CanDev
{
    CAN_TypeDef *instance;

    char *bus_name;

    // CAN_InitTypeDef init;

    uint8 can_flag;
    struct CanBus can_bus;
};

int InitHwCan(void);

#endif