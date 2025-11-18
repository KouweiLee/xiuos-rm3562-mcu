#include "board.h"
#include <device.h>
#include "connect_can.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

/* ===================== CAN1 ===================== */
#ifdef BSP_USING_CAN1

static FDCAN_HandleTypeDef hfdcan1;

static struct CanDriverConfigure can1_config =
{
    .tsjw = 1,
    .tbs1 = 13,
    .tbs2 = 2,
    .brp  = 4,
    .mode = FDCAN_MODE_NORMAL
    // .mode = FDCAN_MODE_INTERNAL_LOOPBACK
};

void FDCAN1_IT0_IRQHandler(int irq_num, void *arg)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
DECLARE_HW_IRQ(FDCAN1_IT0_IRQn, FDCAN1_IT0_IRQHandler, NONE);

#endif



/* ===================== CAN2 ===================== */
#ifdef BSP_USING_CAN2

FDCAN_HandleTypeDef hfdcan2;

// volatile uint8_t g_can2_rx_buf[8];
// volatile uint8_t g_can2_rx_len = 0;

static struct CanDriverConfigure can2_config =
{
    .tsjw = 1,
    .tbs1 = 13,
    .tbs2 = 2,
    .brp  = 4,
    .mode = FDCAN_MODE_NORMAL
};

void FDCAN2_IT0_IRQHandler(int irq_num, void *arg)
{
    HAL_FDCAN_IRQHandler(&hfdcan2);
}
DECLARE_HW_IRQ(FDCAN2_IT0_IRQn, FDCAN2_IT0_IRQHandler, NONE);

#endif



/* ===================== CAN3 ===================== */
#ifdef BSP_USING_CAN3

FDCAN_HandleTypeDef hfdcan3;

// volatile uint8_t g_can3_rx_buf[8];
// volatile uint8_t g_can3_rx_len = 0;

static struct CanDriverConfigure can3_config =
{
    .tsjw = 1,
    .tbs1 = 13,
    .tbs2 = 2,
    .brp  = 4,
    .mode = FDCAN_MODE_NORMAL
};

void FDCAN3_IT0_IRQHandler(int irq_num, void *arg)
{
    HAL_FDCAN_IRQHandler(&hfdcan3);
}
DECLARE_HW_IRQ(FDCAN3_IT0_IRQn, FDCAN3_IT0_IRQHandler, NONE);

#endif



/* ===================== FDCAN 外设时钟选择（PLLQ） ===================== */
void FDCANClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;  // 来自 PLLQ
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ===================== MSP：GPIO/IRQ 与时钟源 ===================== */

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
        __HAL_RCC_FDCAN_CLK_ENABLE();
    }

#ifdef BSP_USING_CAN1
    if (fdcanHandle->Instance == FDCAN1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* PB8: FDCAN1_RX, PB9: FDCAN1_TX */
        GPIO_InitStruct.Pin       = GPIO_CAN1_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIO_CAN1_RX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_CAN1_TX_PIN;
        HAL_GPIO_Init(GPIO_CAN1_TX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
#endif

#ifdef BSP_USING_CAN2
    if (fdcanHandle->Instance == FDCAN2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_CAN2_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIO_CAN2_RX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_CAN2_TX_PIN;
        HAL_GPIO_Init(GPIO_CAN2_TX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
#endif

#ifdef BSP_USING_CAN3
    if (fdcanHandle->Instance == FDCAN3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_CAN3_RX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_FDCAN3;
        HAL_GPIO_Init(GPIO_CAN3_RX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_CAN3_TX_PIN;
        HAL_GPIO_Init(GPIO_CAN3_TX_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }
#endif
}



/* ===================== XiUOS 设备 I/O 实现 ===================== */

static uint32 CanOpen(void *dev)
{
    KPrintf("CAN open\n");
    return EOK;
}

static uint32 CanClose(void *dev)
{
    KPrintf("CAN close\n");
    return EOK;
}

static uint32 CanSendMsg(void *dev, struct BusBlockWriteParam *wp)
{
    if (!dev || !wp || !wp->buffer || wp->size == 0) {
        KPrintf("CanSendMsg: invalid parameters (dev=%p, wp=%p, buffer=%p, size=%lu)\n",
                dev, wp, wp ? wp->buffer : NULL, wp ? (unsigned long)wp->size : 0);
        return ERROR;
    }
    struct CanHardwareDevice *cdev = (struct CanHardwareDevice *)dev;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)cdev->private_data;
    if (!hfdcan) {
        KPrintf("CanSendMsg: NULL hfdcan pointer, device private_data not set\n");
        return ERROR;
    }
    /* —— 按 CAN 实例选择该路的默认发送参数 —— */
    uint32_t id        = 0;
    uint32_t idtype    = FDCAN_STANDARD_ID;
    uint32_t fdformat  = FDCAN_CLASSIC_CAN;     // Classic or FD
    uint32_t dlc_field = FDCAN_DLC_BYTES_8;
    uint8_t  max_bytes = 8;

    if (hfdcan->Instance == FDCAN1) {
        id = CAN1_TX_ID; 
        idtype = CAN1_TX_IDTYPE; 
        fdformat = CAN1_TX_FORMAT; 
        dlc_field = CAN1_TX_DLC;
        max_bytes = CAN1_TX_BYTES;
    } else if (hfdcan->Instance == FDCAN2) {
        id = CAN2_TX_ID; 
        idtype = CAN2_TX_IDTYPE; 
        fdformat = CAN2_TX_FORMAT; 
        dlc_field = CAN2_TX_DLC;
        max_bytes = CAN2_TX_BYTES;
    } else if (hfdcan->Instance == FDCAN3) {
        id = CAN3_TX_ID; 
        idtype = CAN3_TX_IDTYPE; 
        fdformat = CAN3_TX_FORMAT; 
        dlc_field = CAN3_TX_DLC;
        max_bytes = CAN3_TX_BYTES;
    } else {
        KPrintf("CanSendMsg: unsupported CAN instance %p\n", hfdcan->Instance);
        return ERROR;
    }

    /* Classic CAN 下确保 ≤ 8 字节（避免 DLC 设成 >8 时硬件报错） */
    if (fdformat == FDCAN_CLASSIC_CAN && max_bytes > 8) {
        max_bytes = 8;
        dlc_field = FDCAN_DLC_BYTES_8;
    }

    /* 根据该路固定字节数截断 */
    uint8_t len = (wp->size > max_bytes) ? max_bytes : (uint8_t)wp->size;

    FDCAN_TxHeaderTypeDef txh = {0};
    txh.Identifier    = id;
    txh.IdType        = idtype;
    txh.TxFrameType   = FDCAN_DATA_FRAME;   // 数据帧；如需远程帧改为 FDCAN_REMOTE_FRAME
    txh.FDFormat      = fdformat;           // FDCAN_CLASSIC_CAN 或 FDCAN_FD_CAN（来自 CANx_TX_FORMAT）
    txh.BitRateSwitch = FDCAN_BRS_OFF;      // 如某路用 FD+BRS，可在对应分支改为 FDCAN_BRS_ON
    txh.DataLength    = dlc_field;          // 固定 DLC（例如 FDCAN_DLC_BYTES_8）

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, (uint8_t *)wp->buffer) != HAL_OK){
        KPrintf("CanSendMsg: HAL_FDCAN_AddMessageToTxFifoQ failed (CAN%ld)\n",
            (hfdcan->Instance == FDCAN1) ? 1 :
            (hfdcan->Instance == FDCAN2) ? 2 : 3);
        return ERROR;
    }
        

    return len;
}

static uint32 CanRecvMsg(void *dev, struct BusBlockReadParam *rp)
{
    if (!dev || !rp || !rp->buffer || rp->size == 0) {
        KPrintf("CanRecvMsg: invalid parameters (dev=%p, rp=%p, buffer=%p, size=%lu)\n",
                dev, rp, rp ? rp->buffer : NULL, rp ? (unsigned long)rp->size : 0);
        return ERROR;
    }
    struct CanHardwareDevice *cdev = (struct CanHardwareDevice *)dev;
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)cdev->private_data;
    if (!hfdcan) {
        KPrintf("CanRecvMsg: NULL hfdcan pointer, device private_data not set\n");
        return ERROR;
    }
   /* —— 按 CAN 实例选择该路接收 FIFO 与固定字节数 —— */
    uint32_t rx_fifo   = FDCAN_RX_FIFO0;
    uint8_t  max_bytes = 8;

    if (hfdcan->Instance == FDCAN1) {
        rx_fifo   = CAN1_RX_FIFO;
        max_bytes = CAN1_RX_BYTES;
    } else if (hfdcan->Instance == FDCAN2) {
        rx_fifo   = CAN2_RX_FIFO;
        max_bytes = CAN2_RX_BYTES;
    } else if (hfdcan->Instance == FDCAN3) {
        rx_fifo   = CAN3_RX_FIFO;
        max_bytes = CAN3_RX_BYTES;
    } else {
        return ERROR;
    }

    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, rx_fifo) == 0) return ERROR;

    /* 缓冲区大小：经典 CAN 8；若后续用 FD，可改成 64 并相应调整 CANx_RX_BYTES */
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, rx_fifo, &rxh, data) != HAL_OK) return ERROR;

    /* 按本路固定字节数与上层缓冲区做截断 */
    uint8_t len = max_bytes;
    if (len > sizeof(data)) len = sizeof(data);
    if (len > rp->size)     len = rp->size;

    memcpy(rp->buffer, data, len);
    rp->size = len;
    return EOK;
}

/* ===================== 驱动/设备回调表 ===================== */

struct CanDevDone can_dev_done = {
    .open  = CanOpen,
    .close = CanClose,
    .write = CanSendMsg,
    .read  = CanRecvMsg,
};

/* 
*    在 有些 HAL 版本/系列（包含 G4 的一部分包）里，Rx 头的 DataLength 并不是左移 16 位的编码值，而是直接给 DLC 的数值 0..15。
*   所以再右移 16 位就变成 0 了；8 字节也会被读成 0。

*   如果值小于等于 15，就当作“原始 DLC”；否则按“高 16 位编码”处理，再用 LUT 映射到真实字节数（兼容 CAN FD 的 12/16/...）
*/
static inline uint8_t can_dlc_to_bytes_rx(uint32_t v)
{
    // v<=15: 多数 HAL 在 RxHeader 直接填 DLC 数值(0..15)
    uint32_t dlc = (v <= 0xF) ? (v & 0xF) : ((v >> 16) & 0xF);
    static const uint8_t lut[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return lut[dlc];
}

/* HAL FDCAN 数据到达回调（如果开启了 RX 通知） */
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// {
//     if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

//     uint32_t rx_fifo   = FDCAN_RX_FIFO0;
//     uint8_t  max_bytes = 8;
//     long     can_idx   = 0;

//     if (hfdcan->Instance == FDCAN1) {

//         rx_fifo = CAN1_RX_FIFO; max_bytes = CAN1_RX_BYTES; can_idx = 1;
//     } else if (hfdcan->Instance == FDCAN2) {
//         rx_fifo = CAN2_RX_FIFO; max_bytes = CAN2_RX_BYTES; can_idx = 2;
//     } else if (hfdcan->Instance == FDCAN3) {
//         rx_fifo = CAN3_RX_FIFO; max_bytes = CAN3_RX_BYTES; can_idx = 3;
//     } else {
//         KPrintf("RXCB: unsupported FDCAN instance %p\n", hfdcan->Instance);
//         return;
//     }

//     /* 把 FIFO 现有报文都取走，避免丢帧 */
//     while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, rx_fifo) > 0) {
//         FDCAN_RxHeaderTypeDef rxh;
//         uint8_t data[8];                        /* 经典 CAN -> 8 字节缓冲 */

//         if (HAL_FDCAN_GetRxMessage(hfdcan, rx_fifo, &rxh, data) != HAL_OK) {
//             KPrintf("CAN%ld RX: HAL_FDCAN_GetRxMessage failed\n", can_idx);
//             break;
//         }

//         /* DLC 的实际字节数（经典 CAN：高 16 位 0..8） */
//         uint8_t len = can_dlc_to_bytes_rx(rxh.DataLength);
//         if (len > max_bytes) len = max_bytes;
//         if (len > sizeof(data)) len = sizeof(data);

//         /* 存到对应全局缓冲，并打印（简易测试版，ISR 里打印也OK） */
//         if (hfdcan->Instance == FDCAN1) {
//             memcpy((void*)g_can1_rx_buf, data, len);
//             g_can1_rx_len = len;
//         }else if (hfdcan->Instance == FDCAN2) {
//             memcpy((void*)g_can2_rx_buf, data, len);
//             g_can2_rx_len = len;
//         } else if (hfdcan->Instance == FDCAN3) {
//             memcpy((void*)g_can3_rx_buf, data, len);
//             g_can3_rx_len = len;
//         }

//         KPrintf("CAN%ld RX %s ID=0x%lX LEN=%u :",
//                 can_idx,
//                 (rxh.IdType == FDCAN_STANDARD_ID) ? "STD" : "EXT",
//                 (unsigned long)rxh.Identifier,
//                 (unsigned)len);
//         for (uint8_t i = 0; i < len; i++) KPrintf(" %02X", data[i]);
//         KPrintf("\n");
//     }
// }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

    long can_idx = (hfdcan->Instance == FDCAN1) ? 1 :
                   (hfdcan->Instance == FDCAN2) ? 2 : 3;

    // 临时设备壳：复用已有 read()
    struct CanHardwareDevice shim = {0};
    shim.dev_done     = &can_dev_done;
    shim.private_data = hfdcan;

    uint8_t tmp[64];                          // 经典 CAN 用前8字节；FD 可到 64
    struct BusBlockReadParam rp = { .buffer = tmp, .size = sizeof(tmp) };

    if (shim.dev_done->read((void*)&shim, &rp) == EOK) {
        KPrintf("CAN%ld RX LEN=%u:", can_idx, (unsigned)rp.size);
        for (uint8_t i = 0; i < rp.size; ++i) KPrintf(" %02X", tmp[i]);
        KPrintf("\n");
    }
}

static uint32 HalCanInit(struct CanDriver *drv, struct BusConfigureInfo *info)
{
    if (!drv || !drv->private_data) return ERROR;

    struct CanDriverConfigure *cfg = (struct CanDriverConfigure *)drv->private_data;

    /* 用配置对象身份来区分 CANx（与不改结构体的前提相容） */
#ifdef BSP_USING_CAN1
    if (cfg == &can1_config){
        FDCAN_HandleTypeDef *hfdcan = &hfdcan1;    
        hfdcan->Instance                 = FDCAN1;
            /* 基本模式与位时序 */
        hfdcan->Init.ClockDivider         = FDCAN_CLOCK_DIV1;
        hfdcan->Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
        hfdcan->Init.Mode                 = cfg->mode;
        hfdcan->Init.AutoRetransmission   = ENABLE;
        hfdcan->Init.TransmitPause        = DISABLE;
        hfdcan->Init.ProtocolException    = DISABLE;

        hfdcan->Init.NominalPrescaler     = cfg->brp;
        hfdcan->Init.NominalSyncJumpWidth = cfg->tsjw;
        hfdcan->Init.NominalTimeSeg1      = cfg->tbs1;
        hfdcan->Init.NominalTimeSeg2      = cfg->tbs2;

        /* （未用 CAN FD，这里数据时序占位） */
        hfdcan->Init.DataPrescaler        = 1;
        hfdcan->Init.DataSyncJumpWidth    = 1;
        hfdcan->Init.DataTimeSeg1         = 1;
        hfdcan->Init.DataTimeSeg2         = 1;

        /* 过滤器数量预留 */
        hfdcan->Init.StdFiltersNbr        = 1;
        hfdcan->Init.ExtFiltersNbr        = 0;
        hfdcan->Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

        if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
            return ERROR;

        /* 过滤器（默认全接收，投递到 RXFIFO0） */
        FDCAN_FilterTypeDef filter = {
            .IdType      = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType  = FDCAN_FILTER_MASK,
            .FilterConfig= FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1   = 0x000,
            .FilterID2   = 0x000,
        };
        if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
            return ERROR;
    }
#endif

#ifdef BSP_USING_CAN2
    if (cfg == &can2_config){
        FDCAN_HandleTypeDef *hfdcan = &hfdcan2;    
        hfdcan->Instance                 = FDCAN2;
            /* 基本模式与位时序 */
        hfdcan->Init.ClockDivider         = FDCAN_CLOCK_DIV1;
        hfdcan->Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
        hfdcan->Init.Mode                 = cfg->mode;
        hfdcan->Init.AutoRetransmission   = ENABLE;
        hfdcan->Init.TransmitPause        = DISABLE;
        hfdcan->Init.ProtocolException    = DISABLE;

        hfdcan->Init.NominalPrescaler     = cfg->brp;
        hfdcan->Init.NominalSyncJumpWidth = cfg->tsjw;
        hfdcan->Init.NominalTimeSeg1      = cfg->tbs1;
        hfdcan->Init.NominalTimeSeg2      = cfg->tbs2;

        /* （未用 CAN FD，这里数据时序占位） */
        hfdcan->Init.DataPrescaler        = 1;
        hfdcan->Init.DataSyncJumpWidth    = 1;
        hfdcan->Init.DataTimeSeg1         = 1;
        hfdcan->Init.DataTimeSeg2         = 1;

        /* 过滤器数量预留 */
        hfdcan->Init.StdFiltersNbr        = 1;
        hfdcan->Init.ExtFiltersNbr        = 0;
        hfdcan->Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

        if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
            return ERROR;

        /* 过滤器（默认全接收，投递到 RXFIFO0） */
        FDCAN_FilterTypeDef filter = {
            .IdType      = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType  = FDCAN_FILTER_MASK,
            .FilterConfig= FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1   = 0x000,
            .FilterID2   = 0x000,
        };
        if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
            return ERROR;
    }
#endif

#ifdef BSP_USING_CAN3
    if (cfg == &can3_config){
        FDCAN_HandleTypeDef *hfdcan = &hfdcan3;    
        hfdcan->Instance                 = FDCAN3;
            /* 基本模式与位时序 */
        hfdcan->Init.ClockDivider         = FDCAN_CLOCK_DIV1;
        hfdcan->Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
        hfdcan->Init.Mode                 = cfg->mode;
        hfdcan->Init.AutoRetransmission   = ENABLE;
        hfdcan->Init.TransmitPause        = DISABLE;
        hfdcan->Init.ProtocolException    = DISABLE;

        hfdcan->Init.NominalPrescaler     = cfg->brp;
        hfdcan->Init.NominalSyncJumpWidth = cfg->tsjw;
        hfdcan->Init.NominalTimeSeg1      = cfg->tbs1;
        hfdcan->Init.NominalTimeSeg2      = cfg->tbs2;

        /* （未用 CAN FD，这里数据时序占位） */
        hfdcan->Init.DataPrescaler        = 1;
        hfdcan->Init.DataSyncJumpWidth    = 1;
        hfdcan->Init.DataTimeSeg1         = 1;
        hfdcan->Init.DataTimeSeg2         = 1;

        /* 过滤器数量预留 */
        hfdcan->Init.StdFiltersNbr        = 1;
        hfdcan->Init.ExtFiltersNbr        = 0;
        hfdcan->Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

        if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
            return ERROR;

        /* 过滤器（默认全接收，投递到 RXFIFO0） */
        FDCAN_FilterTypeDef filter = {
            .IdType      = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType  = FDCAN_FILTER_MASK,
            .FilterConfig= FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1   = 0x000,
            .FilterID2   = 0x000,
        };
        if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
            return ERROR;
    }
#endif

    return EOK;
}

static uint32 HalCanConfigure(struct CanDriver *drv, int cmd)
{
    if (!drv || !drv->private_data) return ERROR;

    struct CanDriverConfigure *cfg = (struct CanDriverConfigure *)drv->private_data;
    FDCAN_HandleTypeDef *hfdcan = NULL;

#ifdef BSP_USING_CAN1
    if (cfg == &can1_config) hfdcan = &hfdcan1;
#endif
#ifdef BSP_USING_CAN2
    if (cfg == &can2_config) hfdcan = &hfdcan2;
#endif
#ifdef BSP_USING_CAN3
    if (cfg == &can3_config) hfdcan = &hfdcan3;
#endif
    if (!hfdcan) return ERROR;

    switch (cmd) {
    case OPER_SET_INT:
    {
        /* 1) 把需要的事件明确路由到 Line0（你只开了 IT0 的 NVIC） */
        HAL_FDCAN_ConfigInterruptLines(hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_FULL |
            FDCAN_IT_ERROR_WARNING,
            FDCAN_INTERRUPT_LINE0);

        /* 2) Start 之后再开通知 */
        HAL_FDCAN_ActivateNotification(hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_FULL |
            FDCAN_IT_ERROR_WARNING,
            0);
    }
    break;


    case OPER_CLR_INT:
        HAL_FDCAN_DeactivateNotification(hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_FULL |
            FDCAN_IT_ERROR_WARNING);
        break;

    default:
        break;
    }

    return EOK;
}

static uint32 HalCanDrvConfigure(void *drv, struct BusConfigureInfo *info)
{
    if (!info) return ERROR;
    if (info->configure_cmd == OPE_INT)
        return HalCanInit((struct CanDriver *)drv, info);
    else if (info->configure_cmd == OPE_CFG)
        return HalCanConfigure((struct CanDriver *)drv, *(int *)info->private_data);
    return EOK;
}



/* ===================== Bus/Driver/Device 装配工具 ===================== */

static int BoardCanBusInit(struct CanBus* can_bus,
                           struct CanDriver* can_driver,
                           const char* bus_name,
                           const char* drv_name)
{
    x_err_t ret = EOK;
    ret = CanBusInit(can_bus, bus_name);
    if (ret != EOK) return ERROR;

    ret = CanDriverInit(can_driver, drv_name);
    if (ret != EOK) return ERROR;

    ret = CanDriverAttachToBus(drv_name, bus_name);
    if (ret != EOK) return ERROR;

    return ret;
}

/* 设备挂到 Bus：设备 private_data 保存 FDCAN 句柄，用于收发 */
static x_err_t HwCanDeviceAttach(const char *bus_name,
                                 const char *device_name,
                                 FDCAN_HandleTypeDef *hfdcan)
{
    NULL_PARAM_CHECK(bus_name);
    NULL_PARAM_CHECK(device_name);
    x_err_t ret;

    struct CanHardwareDevice *can_device =
        (struct CanHardwareDevice *)x_malloc(sizeof(struct CanHardwareDevice));
    if (!can_device) return ERROR;

    memset(can_device, 0, sizeof(struct CanHardwareDevice));
    can_device->dev_done     = &can_dev_done;
    can_device->private_data = hfdcan;

    ret = CanDeviceRegister(can_device, NONE, device_name);
    if (ret != EOK) { x_free(can_device); return ERROR; }   

    ret = CanDeviceAttachToBus(device_name, bus_name);
    if (ret != EOK) { x_free(can_device); return ERROR; }   

    return ret;
}

/* ===================== 顶层初始化 ===================== */
int InitCAN(void)
{
    x_err_t ret = EOK;
    FDCANClock_Config();

#ifdef BSP_USING_CAN1
    /* 驱动对象：private_data = 配置结构 */
    static struct CanBus can_bus_1;
    static struct CanDriver can_driver_1;

    memset(&can_driver_1, 0, sizeof(struct CanDriver));

    can_driver_1.configure    = HalCanDrvConfigure;
    can_driver_1.driver.configure     = HalCanDrvConfigure;
    can_driver_1.private_data = &can1_config;
    can_driver_1.driver.private_data  = can_driver_1.private_data;

    ret = BoardCanBusInit(&can_bus_1, &can_driver_1, CAN1_BUS_NAME, CAN1_DRV_NAME);
    if (ret != EOK) return ERROR;

    /* 设备对象：private_data = 句柄 */
    ret = HwCanDeviceAttach(CAN1_BUS_NAME, CAN1_DEVICE_NAME, &hfdcan1);
    if (ret != EOK) return ERROR;

    /* 走驱动 configure，完成 HAL_FDCAN_Init + 过滤器配置 */
    struct BusConfigureInfo bi1 = { .configure_cmd = OPE_INT, .private_data = NULL };
    can_driver_1.configure(&can_driver_1, &bi1);

    HAL_FDCAN_Start(&hfdcan1);

    /* 可选：开启接收中断通知（如需） */
    int set_int_cmd = OPER_SET_INT;
    struct BusConfigureInfo bi1_int = { .configure_cmd = OPE_CFG, .private_data = &set_int_cmd };
    can_driver_1.configure(&can_driver_1, &bi1_int);

    
#endif

#ifdef BSP_USING_CAN2

    static struct CanBus can_bus_2;
    static struct CanDriver can_driver_2;

    memset(&can_driver_2, 0, sizeof(struct CanDriver));
    can_driver_2.configure    = HalCanDrvConfigure;
    can_driver_2.private_data = &can2_config;

    ret = BoardCanBusInit(&can_bus_2, &can_driver_2, CAN2_BUS_NAME, CAN2_DRV_NAME);
    if (ret != EOK) return ERROR;

    ret = HwCanDeviceAttach(CAN2_BUS_NAME, CAN2_DEVICE_NAME, &hfdcan2);
    if (ret != EOK) return ERROR;

    struct BusConfigureInfo bi2 = { .configure_cmd = OPE_INT, .private_data = NULL };
    can_driver_2.configure(&can_driver_2, &bi2);

    HAL_FDCAN_Start(&hfdcan2);

    int set_int_cmd2 = OPER_SET_INT;
    struct BusConfigureInfo bi2_int = { .configure_cmd = OPE_CFG, .private_data = &set_int_cmd2 };
    can_driver_2.configure(&can_driver_2, &bi2_int);

    
#endif

#ifdef BSP_USING_CAN3

    static struct CanBus can_bus_3;
    static struct CanDriver can_driver_3;

    memset(&can_driver_3, 0, sizeof(struct CanDriver));
    can_driver_3.configure    = HalCanDrvConfigure;
    can_driver_3.private_data = &can3_config;

    ret = BoardCanBusInit(&can_bus_3, &can_driver_3, CAN3_BUS_NAME, CAN3_DRV_NAME);
    if (ret != EOK) return ERROR;

    ret = HwCanDeviceAttach(CAN3_BUS_NAME, CAN3_DEVICE_NAME, &hfdcan3);
    if (ret != EOK) return ERROR;

    struct BusConfigureInfo bi3 = { .configure_cmd = OPE_INT, .private_data = NULL };
    can_driver_3.configure(&can_driver_3, &bi3);

    HAL_FDCAN_Start(&hfdcan3);

    int set_int_cmd3 = OPER_SET_INT;
    struct BusConfigureInfo bi3_int = { .configure_cmd = OPE_CFG, .private_data = &set_int_cmd3 };
    can_driver_3.configure(&can_driver_3, &bi3_int);

#endif

    return ret;
}


static void Test_SendByDev(void)
{
    static const uint8_t payload[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};

    struct CanHardwareDevice shim = {0};
    shim.dev_done     = &can_dev_done;     
    shim.private_data = &hfdcan3;         

    // 2) 组写入参数
    struct BusBlockWriteParam wp = {
        .buffer = (void*)payload,
        .size   = sizeof(payload)          // 经典 CAN 建议 ≤8
    };

    // 3) 通过操作表 .write 调用 CanSendMsg()
    uint32 ret = shim.dev_done->write((void*)&shim, &wp);
    if ((int)ret <= 0) {
        KPrintf("CAN3 write failed, ret=%ld\n", (long)ret);
    } 
    else {
        KPrintf("CAN3 write ok, %lu bytes\n", (unsigned long)ret);
    }
}


void TestCAN(void){
    Test_SendByDev();
}


SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    TestCAN, TestCAN, test can);