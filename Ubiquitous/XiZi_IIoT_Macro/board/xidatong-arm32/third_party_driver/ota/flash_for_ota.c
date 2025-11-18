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
* @file flash_for_ota.c
* @brief support flash function
* @version 2.0 
* @author AIIT XUOS Lab
* @date 2023-11-20
*/
 
#include <stdio.h>
#include "flash_for_ota.h"
#include "MIMXRT1052.h"

uint8_t NorFlash_BUFFER[4096];         //4K buffer cache
uint8_t buffer[FLASH_PAGE_SIZE];       //256 bytes buffer cache

/*******************************************************************************
* 函 数 名: Flash_Erase
* 功能描述: 以扇区为擦除单位擦除Flash指定长度的空间,最终擦除的字节可能大于imageSize
* 形    参: start_addr:擦除区域起始地址
            imageSize:要擦除的字节数
* 返 回 值: 如果函数执行成功,状态值为 kStatus_Success,否则状态值为其他错误码 
*******************************************************************************/
status_t Flash_Erase(uint32_t start_addr, uint32_t imageSize)
{
    uint16_t i;
    status_t status;
    uint32_t sectorNum = (imageSize%SECTOR_SIZE != 0)? (imageSize/SECTOR_SIZE + 1):(imageSize/SECTOR_SIZE);
    
    for(i=0;i<sectorNum;i++)
    {
        status = FLASH_EraseSector(start_addr+i*SECTOR_SIZE);

        if(status != kStatus_Success)
        {
            KPrintf("Erase_Sector 0x%x faild!\r\n",i*SECTOR_SIZE);
            return status;
        }
    }
    return (status_t)kStatus_Success;
}

/*******************************************************************************
* 函 数 名: Flash_Write
* 功能描述: 写入W25QXX在指定地址开始写入指定长度的数据 
* 形    参: pBuffer:数据存储区
            WriteAddr:开始写入的地址
            NumByteToWrite:要写入的字节数(最大65535)  
* 返 回 值: 如果函数执行成功,状态值为 kStatus_Success,否则状态值为其他错误码 
* 注    释: 该函数带擦除操作
*******************************************************************************/
status_t Flash_Write(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite)
{ 
    uint32_t secPos;
    uint16_t secOff;
    uint16_t secRemain;
    uint16_t i;    
    uint8_t *NorFlash_BUF = 0;
    status_t status;

    NorFlash_BUF = NorFlash_BUFFER;//RAM缓冲区4K

    WriteAddr &= 0x0FFFFFFF;
    
    secPos = WriteAddr/SECTOR_SIZE;//扇区地址
    secOff = WriteAddr%SECTOR_SIZE;//在扇区内的偏移
    secRemain = SECTOR_SIZE - secOff;//扇区剩余空间大小
    
    if(NumByteToWrite <= secRemain)
    {
        secRemain = NumByteToWrite;//不大于4096个字节
    }
    while(1) 
    {
        status = FLASH_ReadBuf(CHIP_FLAH_BASE + secPos*SECTOR_SIZE, (void *)NorFlash_BUF, SECTOR_SIZE);//读出整个扇区的内容
        if(status != kStatus_Success)
        {
            return status;
        }
        for(i=0;i<secRemain;i++)//校验数据
        {
            if(NorFlash_BUF[secOff+i] != 0xFF)
            {
                break;//需要擦除
            }
        }
        if(i < secRemain)//需要擦除
        {
            status = FLASH_EraseSector(CHIP_FLAH_BASE + secPos*SECTOR_SIZE);
            if(status != kStatus_Success)
            {
                return status;
            }
            for(i=0;i<secRemain;i++)//复制
            {
                NorFlash_BUF[i+secOff] = pBuffer[i];                       
            }
            status = NorFlash_Write_NoCheck(NorFlash_BUF,CHIP_FLAH_BASE + secPos*SECTOR_SIZE,SECTOR_SIZE);//写入整个扇区
            if(status != kStatus_Success)
            {
                return status;
            }  
        }
        else
        {
            status = NorFlash_Write_NoCheck(pBuffer,CHIP_FLAH_BASE + WriteAddr,secRemain);//写已经擦除了的,直接写入扇区剩余区间. 	
            if(status != kStatus_Success)
            {
                return status;
            } 	
        }
                           
        if(NumByteToWrite == secRemain)
        {
            break;//写入结束了
        }
        else//写入未结束
        {
            secPos++;//扇区地址增1
            secOff=0;//偏移位置为0

            pBuffer += secRemain;//指针偏移
            WriteAddr += secRemain;//写地址偏移
            NumByteToWrite -= secRemain;//字节数递减
            if(NumByteToWrite > SECTOR_SIZE)
            {
                secRemain = SECTOR_SIZE;//下一个扇区还是写不完	
            }
            else
            {
                secRemain = NumByteToWrite;//下一个扇区可以写完了	
            }
        }
    }

    return (status_t)kStatus_Success;
}


/*******************************************************************************
* 函 数 名: Flash_Read
* 功能描述: 读Flash内容
* 形    参: addr:读取区域起始地址
            buf:数据存储区
            len:要读取的字节数
* 返 回 值: 如果函数执行成功，状态值为 kStatus_Success，否则状态值为其他错误码 
*******************************************************************************/
status_t Flash_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{   
    /* For FlexSPI Memory ReadBack, use IP Command instead of AXI command for security */
    if((addr >= 0x60000000) && (addr < 0x61000000))
    {
        return FLASH_ReadBuf(addr, (void *)buf, len);
    }
    else
    {
        void* result = memcpy(buf, (void*)addr, len);
        if(result == NULL)
        {
            return (status_t)kStatus_Fail;  
        } 
        else 
        {
            return (status_t)kStatus_Success;  
        }  
    }
}


/*******************************************************************************
* 函 数 名: Flash_Copy
* 功能描述: 实现flash数据在分区之间的拷贝
* 形    参: srcAddr:源flash的起始地址
            dstAddr:目标flash的起始地址;
            imageSize:要拷贝的flash空间大小,单位为字节
* 返 回 值: 如果函数执行成功，状态值为 kStatus_Success，否则状态值为其他错误码 
*******************************************************************************/
status_t Flash_Copy(uint32_t srcAddr,uint32_t dstAddr, uint32_t imageSize)
{   
    uint32_t PageNum, Remain, i;
    status_t status;

    if((srcAddr == dstAddr) || imageSize > APP_FLASH_SIZE)
    {
        return (status_t)kStatus_Fail;
    }

    status = Flash_Erase(dstAddr,imageSize);
    if(status != kStatus_Success)
    {
        KPrintf("Erase flash 0x%08x failure !\r\n",dstAddr);
        return status;
    }

    PageNum = imageSize/FLASH_PAGE_SIZE;
    Remain = imageSize%FLASH_PAGE_SIZE;

    for(i=0;i<PageNum;i++)
    {
        memset(buffer, 0, sizeof(buffer));
        status = Flash_Read(srcAddr + i*FLASH_PAGE_SIZE, buffer, sizeof(buffer));
        if(status != kStatus_Success)
        {
            KPrintf("Read flash 0x%08x failure !\r\n", srcAddr + i*FLASH_PAGE_SIZE);
            return status;
        }
        status = Flash_Write(dstAddr+ i*FLASH_PAGE_SIZE, buffer, FLASH_PAGE_SIZE);
        if(status != kStatus_Success)
        {
            KPrintf("Write flash 0x%08x failure !\r\n", dstAddr + i*FLASH_PAGE_SIZE);
            return status;
        }
    }

    if(Remain)
    {
        memset(buffer, 0, sizeof(buffer));
        status = Flash_Read(srcAddr + i*FLASH_PAGE_SIZE, buffer, Remain);
        if(status != kStatus_Success)
        {
            KPrintf("Read flash 0x%08x failure !\r\n", srcAddr + i*FLASH_PAGE_SIZE);
            return status;
        }
        status = Flash_Write(dstAddr+ i*FLASH_PAGE_SIZE, buffer, Remain);
        if(status != kStatus_Success)
        {
            KPrintf("Write flash 0x%08x failure !\r\n", dstAddr + i*FLASH_PAGE_SIZE);
            return status;
        }
    }

    return (status_t)kStatus_Success; 
}

/*******************************************************************************
* 函 数 名: NOR_FLASH_Write
* 功能描述: 写入W25QXX在指定地址开始写入指定长度的数据 
* 形    参: FlashAddress:用于存储当前写入Flash地址的指针，写入过程中会移动
            Data:要写入数据存储区
            DataLength:要写入的字节数
* 返 回 值: 如果函数执行成功,状态值为 kStatus_Success,否则状态值为其他错误码  
*******************************************************************************/
#ifndef  USE_HIGHT_SPEED_TRANS
status_t NOR_FLASH_Write(uint32_t* FlashAddress, uint8_t* Data ,uint16_t DataLength)
{
    status_t status;
    uint32_t WriteAddr;
    WriteAddr = *FlashAddress;
    status = Flash_Write(WriteAddr,Data,DataLength);
    if(status != kStatus_Success)
    {
        return status;
    }
    *FlashAddress += DataLength;
    return (status_t)kStatus_Success;
}
#else
uint8_t packetNum = 0;
uint32_t dataLen = 0;
uint32_t WriteAddr;
uint8_t dataBuff[5*1024];
status_t NOR_FLASH_Write(uint32_t* FlashAddress, uint8_t* Data ,uint16_t DataLength,uint8_t doneFlag)
{
    status_t status;
    if(!doneFlag)
    {  
        memcpy(&dataBuff[dataLen],Data,DataLength);
        dataLen += DataLength;
        packetNum ++;
        if(1 == packetNum)
        {
            WriteAddr = *FlashAddress;
        }

        if(dataLen>=SECTOR_SIZE)
        {
            status = Flash_Write(WriteAddr,dataBuff,dataLen);
            if(status != kStatus_Success)
            {
                return status;
            } 
            packetNum = 0;
            dataLen = 0; 
        }
        *FlashAddress += DataLength;
    }
    else
    {
        status = Flash_Write(WriteAddr,dataBuff,dataLen);
        if(status != kStatus_Success)
        {
            return status;
        }
        packetNum = 0;
        dataLen = 0; 
    }
   return (status_t)kStatus_Success;;
}
#endif
