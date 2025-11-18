/**
 * @file ping.c
 * @brief  该文件所有函数无效，仅作ping命令迁移参考
 * @author Huo Yujia (huoyujia081@126.com)
 * @version 1.0
 * @date 2024-08-19
 */
#include "ping.h"

#include "connect_ether.h"
#include "eth_driver.h"
#include "shell.h"
#include "string.h"
#include "wchnet.h"
#include "xs_base.h"


u8 sendErrCnt;
u8 unreachCnt;
u8 timeoutCnt;
u8 sucRecCnt;
u8 sendBuf[100];
u8 ICMPCnt = 0;
u8 ICMPSeq = 0;
u8 ICMPSuc = 0;
u8 ICMPTmpVal = 0;
extern u32 timeCnt;
int pingSendCount = 0;  // the times of ping data sending

u8 DESIP[4] = {192, 168, 1, 100};  // destination IP address
u8 IPRawProto = 1;

extern u8 SocketId;                                            // socket id
extern u8 SocketRecvBuf[WCHNET_MAX_SOCKET_NUM][RECE_BUF_LEN];  // socket receive buffer
extern u8 MyBuf[RECE_BUF_LEN];
/*********************************************************************
 * @fn      WCHNET_CreateIPRawSocket
 *
 * @brief   Create raw IP Socket
 *
 * @return  none
 */
void WCHNET_CreateIPRawSocket(void) {
    u8 i;
    SOCK_INF TmpSocketInf;
    memset((void *)&TmpSocketInf, 0, sizeof(SOCK_INF));
    memcpy((void *)TmpSocketInf.IPAddr, DESIP, 4);
    TmpSocketInf.SourPort = IPRawProto;  // In IPRAW mode, SourPort is the protocol type
    TmpSocketInf.ProtoType = PROTO_TYPE_IP_RAW;
    TmpSocketInf.RecvStartPoint = (u32)(SocketRecvBuf[0]);
    TmpSocketInf.RecvBufLen = RECE_BUF_LEN;
    i = WCHNET_SocketCreat(&SocketId, &TmpSocketInf);
    memset(SocketRecvBuf[0], 0, RECE_BUF_LEN);
    mStopIfError(i);
}

/*********************************************************************
 * @fn      InitParameter
 *
 * @brief   Initializes parameters.
 *
 * @return  none
 */
void InitParameter(void) {
    unreachCnt = 0;
    timeoutCnt = 0;
    sucRecCnt = 0;
    ICMPSuc = ICMP_SOKE_CON;
    pingSendCount = 0;
}

/*********************************************************************
 * @fn      InitPING
 *
 * @brief   Initializes PING.
 *
 * @return  none
 */
void InitPING(void) {
    IcmpHeader head;
    u32 check_sum = 0;
    u8 i;

    ICMPCnt++;
    ICMPSeq += 1;
    head.i_type = ICMP_HEAD_TYPE;
    head.i_code = ICMP_HEAD_CODE;
    head.i_id = ICMP_HEAD_ID;
    head.i_seq = ICMP_HEAD_SEQ + ICMPSeq;
    memset(head.i_data, 0, sizeof(head.i_data));
    for (i = 0; i < ICMP_DATA_BYTES; i++) {
        if (i < 26)
            head.i_data[i] = i + 'a';
        else
            head.i_data[i] = i + 'a' - 26;
        if (i % 2 == 0)
            check_sum += head.i_data[i] << 8;
        else
            check_sum += head.i_data[i];
    }
    check_sum += head.i_type << 8;
    check_sum += head.i_code;
    check_sum += head.i_id;
    check_sum += head.i_seq;
    head.i_cksum = check_sum >> 16;
    head.i_cksum += check_sum & 0xffff;
    head.i_cksum = 0xffff - head.i_cksum;
    memset(sendBuf, 0, sizeof(sendBuf));
    memcpy(sendBuf, &head, sizeof(head));
    sendBuf[2] = head.i_cksum >> 8;
    sendBuf[3] = head.i_cksum & 0xff;
    sendBuf[4] = head.i_id >> 8;
    sendBuf[5] = head.i_id & 0xff;
    sendBuf[6] = head.i_seq >> 8;
    sendBuf[7] = head.i_seq & 0xff;
}

/*********************************************************************
 * @fn      Respond_PING
 *
 * @brief   Respond to PING request.
 *
 * @param   pDat - ACK data.
 *
 * @return  none
 */
void Respond_PING(u8 *pDat) {
    IcmpHeader head;
    u32 check_sum = 0;
    u8 i;

    head.i_type = ICMP_HEAD_REPLY;
    head.i_code = pDat[1];
    head.i_id = (pDat[4] << 8) + pDat[5];
    head.i_seq = (pDat[6] << 8) + pDat[7];

    check_sum += head.i_type << 8;
    check_sum += head.i_code;
    check_sum += head.i_id;
    check_sum += head.i_seq;
    for (i = 0; i < 32; i++) {
        head.i_data[i] = pDat[i + 8];
        if (i % 2 == 0)
            check_sum += head.i_data[i] << 8;
        else
            check_sum += head.i_data[i];
    }
    head.i_cksum = check_sum >> 16;
    head.i_cksum += check_sum & 0xffff;
    head.i_cksum = 0xffff - head.i_cksum;
    memset(sendBuf, 0, sizeof(sendBuf));
    memcpy(sendBuf, &head, sizeof(head));
    sendBuf[2] = head.i_cksum >> 8;
    sendBuf[3] = head.i_cksum & 0xff;
    sendBuf[4] = head.i_id >> 8;
    sendBuf[5] = head.i_id & 0xff;
    sendBuf[6] = head.i_seq >> 8;
    sendBuf[7] = head.i_seq & 0xff;
}

/*********************************************************************
 * @fn      WCHNET_ICMPRecvData
 *
 * @brief   receive and parse data.
 *
 * @param   len - data length.
 *          pDat - data buff
 *
 * @return  none
 */
void WCHNET_ICMPRecvData(u32 len, u8 *pDat) {
    u16 tmp = 0;

    ICMPTmpVal = ICMPSuc;
    ICMPSuc = 3;
    if (len == 40) {
        if (pDat[0] == ICMP_HEAD_REPLY) {
            if (pDat[1] == ICMP_HEAD_CODE) {
                tmp = pDat[4];
                tmp = tmp << 8;
                tmp += pDat[5];
                if (tmp == ICMP_HEAD_ID) {
                    tmp = pDat[6];
                    tmp = (tmp << 8);
                    tmp += pDat[7] - ICMPSeq;
                    if (tmp == ICMP_HEAD_SEQ) {
                        ICMPSuc = ICMP_RECV_SUC;
                    }
                }
            }
        }
        if (pDat[0] == ICMP_HEAD_TYPE) {
            if (pDat[1] == ICMP_HEAD_CODE) {
                Respond_PING(pDat);
                ICMPSuc = ICMP_REPLY;
            }
        }
    } else {
        if (pDat[0] == 3) {
            if (pDat[1] == 1) {
                ICMPSuc = ICMP_UNRECH;
            }
        }
    }
}

/*********************************************************************
 * @fn      WCHNET_PINGSendData
 *
 * @brief   send PING command.
 *
 * @param   PSend - data buff.
 *          Len - data length
 *          id - socket id
 *
 * @return  none
 */
void WCHNET_PINGSendData(u8 *PSend, u32 Len, u8 id) {
    u32 length;
    u8 i, count = 0;

    length = Len;
    count = 0;

    while (1) {
        Len = length;
        i = WCHNET_SocketSend(id, PSend, &Len);
        pingSendCount++;
        mStopIfError(i);
        if (Len == 0) {
            count++;
            if (count > 2) {
                return;
            }
        }
        length -= Len;
        PSend += Len;          // offset buffer pointer
        if (length) continue;  // If the data is not sent, continue to send
        break;
    }
    ICMPSuc++;
}

/*********************************************************************
 * @fn      WCHNET_PINGCmd
 *
 * @brief   Query status and execute corresponding commands.
 *
 * @return  none
 */
void WCHNET_PINGCmd(void) {
    if (ICMPSuc < ICMP_KEEP_NO) {
        switch (ICMPSuc) {
            case ICMP_SOKE_CON:
                ICMPSuc = 1;
                timeCnt = 0;
                KPrintf("Ping %d.%d.%d.%d with %d bytes of data.\r\n", (u16)DESIP[0], (u16)DESIP[1], (u16)DESIP[2],
                        (u16)DESIP[3], (u16)ICMP_DATA_BYTES);
                WCHNET_PINGSendData(sendBuf, 40, SocketId);
                break;
            case ICMP_SEND_ERR:
                if (sendErrCnt > 10) {
                    KPrintf(
                        "Ping statistics for %d.%d.%d.%d:\r\nPackets: Sent = "
                        "%d,Received = %d,Lost = %d<%d%% loss>.\r\n",
                        (u16)DESIP[0], (u16)DESIP[1], (u16)DESIP[2], (u16)DESIP[3], pingSendCount, (u16)sucRecCnt,
                        (u16)(pingSendCount - sucRecCnt), (u16)((pingSendCount - sucRecCnt) * 100 / pingSendCount));
                    ICMPSuc = ICMP_KEEP_NO;
                    sendErrCnt = 0;
                }
                if (timeCnt > 250) {
                    KPrintf("send data fail!\n");
                    timeCnt = 0;
                    sendErrCnt++;
                    WCHNET_PINGSendData(sendBuf, 40, SocketId);
                }
                break;
            case ICMP_SEND_SUC:
                if (timeCnt > 100) {
                    KPrintf("Request timeout.\n");
                    timeoutCnt++;
                    if (ICMPCnt < PING_SEND_CNT) {
                        ICMPSuc = 1;
                        InitPING();
                        timeCnt = 0;
                        WCHNET_PINGSendData(sendBuf, 40, SocketId);
                    } else {
                        KPrintf("PING end!\r\n");
                        KPrintf("send = %d Received = %d\n", PING_SEND_CNT, sucRecCnt);
                        ICMPSuc = ICMP_KEEP_NO;
                    }
                }
                break;
            case ICMP_RECV_ERR:  // error unknown
                KPrintf("receive unknown data.\r\n");
                ICMPSuc = ICMP_KEEP_NO;
                break;
            case ICMP_RECV_SUC:  // success
                KPrintf("Reply from %d.%d.%d.%d: bytes=%d time<4ms\r\n", (u16)DESIP[0], (u16)DESIP[1], (u16)DESIP[2],
                        (u16)DESIP[3], (u16)ICMP_DATA_BYTES);
                sucRecCnt++;
                if (ICMPCnt < PING_SEND_CNT) {
                    ICMPSuc = 1;
                    InitPING();
                    timeCnt = 0;
                    WCHNET_PINGSendData(sendBuf, 40, SocketId);
                } else {
                    KPrintf(
                        "Ping statistics for %d.%d.%d.%d:\r\nPackets: Sent = "
                        "%d,Received = %d,Lost = %d<%d%% loss>.\r\n",
                        (u16)DESIP[0], (u16)DESIP[1], (u16)DESIP[2], (u16)DESIP[3], pingSendCount, (u16)sucRecCnt,
                        (u16)(pingSendCount - sucRecCnt), (u16)((pingSendCount - sucRecCnt) * 100 / pingSendCount));
                    ICMPSuc = ICMP_KEEP_NO;
                }
                break;
            case ICMP_UNRECH:  // unreachable
                KPrintf("Reply from %d.%d.%d.%d: Destination host unreachable.\r\n", (u16)DESIP[0], (u16)DESIP[1],
                        (u16)DESIP[2], (u16)DESIP[3]);
                unreachCnt++;
                if (ICMPCnt < PING_SEND_CNT) {
                    ICMPSuc = 1;
                    InitPING();
                    timeCnt = 0;
                    WCHNET_PINGSendData(sendBuf, 40, SocketId);
                } else {
                    KPrintf("PING end!\r\n");
                    KPrintf("send = %d Received = %d", PING_SEND_CNT, sucRecCnt);
                    ICMPSuc = ICMP_KEEP_NO;
                }
                break;
            case ICMP_REPLY:
                WCHNET_PINGSendData(sendBuf, 40, SocketId);
                break;
            case ICMP_REPLY_SUC:
                KPrintf("Reply ping.\r\n");
                ICMPSuc = ICMPTmpVal;
                break;
            case ICMP_KEEP_NO:
                break;
            default:
                break;
        }
    }
}
void parseDesIP(char *argv[]) {
    char charDesIP[4][4];
    int i;
    int j = 0;
    int k = 0;
    for (i = 0;; i++) {
        // KPrintf("%c\n", argv[1][i]);
        if (argv[1][i] == '\0') {
            charDesIP[j][k++] = '\0';
            break;
        } else if (argv[1][i] == '.') {
            charDesIP[j++][k] = '\0';
            k = 0;
        } else {
            charDesIP[j][k++] = argv[1][i];
        }
    }
    for (i = 0; i < 4; i++) {
        DESIP[i] = atoi(charDesIP[i]);
    }
    // KPrintf("%s.%s.%s.%s\n", charDesIP[0], charDesIP[1], charDesIP[2],
    // charDesIP[3]); KPrintf("%d.%d.%d.%d\n", DESIP[0], DESIP[1], DESIP[2],
    // DESIP[3]);
}
int ping(int argc, char *argv[]) {
    if (argc != 2) {
        KPrintf("Usage: ping <ip>\r\n");
        return 1;
    }
    parseDesIP(argv);
    WCHNET_CreateIPRawSocket();
    InitParameter();
    InitPING();
    while (1) {
        /*Ethernet library main task function,
         * which needs to be called cyclically*/
        WCHNET_MainTask();
        /*Query the Ethernet global interrupt,
         * if there is an interrupt, call the global interrupt handler*/
        if (WCHNET_QueryGlobalInt()) {
            WCHNET_HandleGlobalInt(NULL);
        }
        WCHNET_PINGCmd();
        if (ICMPSuc == ICMP_KEEP_NO) {
            break;
        }
    }
    WCHNET_SocketClose(SocketId, TCP_CLOSE_NORMAL);
    ICMPCnt = 0;  // restore the original ICMPCnt
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), ping, ping, test ping command);