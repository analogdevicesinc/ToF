/**
 * @file jtagmailbox_hostside.h
 * @brief JTAG Mailbox functionality, which is subject to the mailbox message definitions
 */

#ifndef __JTAGMAILBOX_HOSTSIDE_H
#define __JTAGMAILBOX_HOSTSIDE_H
#include <stdint.h>
#include <stddef.h>
#include "inttypes.h"
#include "stdio.h"
#include "libtdr.h"

#define OLIMEX_ID_VENDOR 0x15BA
#define OLIMEX_ID_ARM_USB_TINY_H_PRODUCT 0x002A
#define IR_WIDTH_5 5

typedef struct _Hsp_Generic_Register_t
{
    volatile uint32_t u;
} Hsp_Generic_Register_t;
typedef Hsp_Generic_Register_t volatile *Ptr_Hsp_Generic_Register_t;

#define CREG_OFFSET_MISC_J2P_MBOX0 0x2f000088
#define CREG_OFFSET_MISC_J2P_MBOX1 0x2f00008c
#ifndef __cplusplus
#define bool _Bool
#define true (1)
#define false (0)
#endif

#define CREG_BASE (0x2F000000)

#define CREG_MISC (CREG_BASE + 0x80)
#define J2P_MBOX0 CREG_OFFSET_MISC_J2P_MBOX0
#define J2P_MBOX1 CREG_OFFSET_MISC_J2P_MBOX1

#define MAX_JTAG_COMMANDS 1 //MAx JTAG commands in single instance

static inline void hsp_write32(volatile uint32_t *addr, uint32_t data)
{
    *addr = data;
}
static inline uint32_t hsp_read32(volatile uint32_t *addr) { return *addr; }

typedef union _Jtag_Task_Reqeust_t {
    struct
    {
        uint8_t Request : 5;     // [4:0]
        uint8_t ACK : 1;         // [5]
        uint8_t Done : 1;        // [6]
        uint8_t Pass1_Fail0 : 1; // [7]
    };
    volatile uint8_t u;
} Jtag_Task_Reqeust_t;
typedef Jtag_Task_Reqeust_t volatile *Ptr_Jtag_Task_Reqeust_t;

typedef union _Jtag_Mailbox_Fuse_Command_Struct_t {
    struct
    {
        Jtag_Task_Reqeust_t Task_Request; // [7:0]
        uint32_t Fuse_Word_Address : 8;   // [15:8]
        uint32_t reserved_30_16 : 15;     // [30:16]
        uint32_t Last_Access_Request : 1; // [31]
    };
    volatile uint32_t u;
} Jtag_Mailbox_Fuse_Command_Struct_t;
typedef Jtag_Mailbox_Fuse_Command_Struct_t volatile *Ptr_Jtag_Mailbox_Fuse_Command_Struct_t;

typedef struct _Jtag_Mailbox_Fuse_RW_Message_t
{
    Hsp_Generic_Register_t data;
    Jtag_Mailbox_Fuse_Command_Struct_t CommandStruct;
} Jtag_Mailbox_Fuse_RW_Message_t;
typedef Jtag_Mailbox_Fuse_RW_Message_t volatile *Ptr_Jtag_Mailbox_Fuse_RW_Message_t;

typedef union _Jtag_Mailbox_SS_Transition_Command_Struct_t {
    struct
    {
        Jtag_Task_Reqeust_t Task_Request; // [7:0]
        uint32_t reserved_31_8 : 24;      // [31:8]
    };
    volatile uint32_t u;
} Jtag_Mailbox_SS_Transition_Command_Struct_t;
typedef Jtag_Mailbox_SS_Transition_Command_Struct_t volatile *Ptr_Jtag_Mailbox_SS_Transition_Command_Struct_t;

#define JTAG_MAILBOX_MAX_COMMANDS_IN_REG 4
#define JTAG_MAILBOX_MAX_BYTES_IN_REG 4

typedef union _Jtag_Mailbox_Generic_Command_Struct_t {
    struct
    {
        Jtag_Task_Reqeust_t Task_Request[JTAG_MAILBOX_MAX_COMMANDS_IN_REG];
    };
    volatile uint32_t u;
} Jtag_Mailbox_Generic_Command_Struct_t;
typedef Jtag_Mailbox_Generic_Command_Struct_t volatile *Ptr_Jtag_Mailbox_Generic_Command_Struct_t;

typedef struct _Jtag_Mailbox_SS_Transition_Message_t
{
    Jtag_Mailbox_SS_Transition_Command_Struct_t CommandStruct;
    Hsp_Generic_Register_t unused;
} Jtag_Mailbox_SS_Transition_Message_t;
typedef Jtag_Mailbox_SS_Transition_Message_t volatile *Ptr_Jtag_Mailbox_SS_Transition_Message_t;

typedef enum
{
    JTAG_MAILBOX_COMMAND_CHANGE_TO_TEST_STATE = 1,
    JTAG_MAILBOX_COMMAND_CHANGE_TO_PROD_STATE = 2,
    JTAG_MAILBOX_COMMAND_CHANGE_TO_SECURE_STATE = 3,
    JTAG_MAILBOX_COMMAND_CHANGE_TO_RETEST_OR_EOL_STATE = 4,
    JTAG_MAILBOX_COMMAND_READ_SECURITY_STATE,
    JTAG_MAILBOX_COMMAND_FUSE_READ,
    JTAG_MAILBOX_COMMAND_FUSE_WRITE,
    JTAG_MAILBOX_COMMAND_UNKNOWN,
} Jtag_Mailbox_Command_t;

typedef enum
{
    JTAG_MAILBOX_COMMAND_STATUS_ACK = 0,
    JTAG_MAILBOX_COMMAND_STATUS_DONE = 1,
    JTAG_MAILBOX_COMMAND_STATUS_PASS1_FAIL0 = 2,
} Jtag_Mailbox_Command_Status_t;

typedef enum
{
    HSP_SUCCESS,               /**< 0x0 = Success */
    HSP_UNKNOWN_ERROR,         /**< 0x1 = Unknown error */
    HSP_INVALID_PARAMETER,     /**< 0x2 = Invalid parameter */
    HSP_NOT_IMPLEMENTED,       /**< 0x3 = Not implemented */
    HSP_COMMAND_ERROR,         /**< 0x4 = Command error */
    HSP_BUS_ERROR,             /**< 0x5 = Bus error */
    HSP_FAULT_ERROR,           /**< 0x6 = Fault error */
    HSP_NOT_OWNER_ERROR,       /**< 0x7 = Not owner error */
    HSP_BUSY_STATE,            /**< 0x8 = HSP is busy */
    HSP_SIGNATURE_MISMATCH,    /**< 0x9 = Signature does not match */
    HSP_AUTHENTICATION_FAILED, /**< 0xA = Signature does not match */
    HSP_IRQ_STATE,             /**< 0xB = trapped in IRQ */
    HSP_FIQ_STATE,             /**< 0xC = trapped in FIQ */

    //When program command is issued to a fuse slot (often key slot) that has ECC, and any data or
    //ECC bit in the slot is already programmed (not zero)
    HSP_KEY_NOT_ZERO_ERROR,  /**< 0xD = Key not zero error */
    HSP_BLANK_CHECK_FAIL,    /**< 0xE = Fuse word check is not blank; this might not be error */
    HSP_ECC_CORRECTION_FAIL, /**< 0xF = Ecc correction fails, having more than 1 errors */

    //Mailbox status
    HSP_MBX_INVALID_ATTRIBUTE_ERROR,      /**< 0x10 = Invalid Command Packet Attribute field */
    HSP_MBX_REGISTER_ACCESS_ERROR,        /**< 0x11 = Invalid register access */
    HSP_MBX_INVALID_CMD_ERROR,            /**< 0x12 = Invalid Command Packet Command field */
    HSP_MBX_INVALID_SIZE_ERROR,           /**< 0x13 = Invalid Command Packet Size field */
    HSP_MBX_EXCEED_ENCRYPT_ENTRIES_ERROR, /**< 0x14 = Exceeded Number of Mailbox Encryption Entries */
} Hsp_Status_t;

Hsp_Status_t GetJtagMailBoxCmds(uint32_t mailBoxIndex, uint32_t *commands, uint32_t *num, uint32_t *commandStatuses);
Hsp_Status_t GetJtagMailBoxTasks(uint32_t mailBoxIndex, Jtag_Task_Reqeust_t *tasks, uint32_t *num);
Hsp_Status_t SetJtagMailBoxCommandStatus(uint32_t mailBoxIndex, uint32_t cmdIndex,
                                         Jtag_Mailbox_Command_Status_t statusType, uint32_t commandStatus);
Hsp_Status_t SetJtagMailBox1stCommandStatus(Jtag_Mailbox_Command_Status_t statusType, uint32_t commandStatus);
Hsp_Status_t SetJtagMailBoxCommandErrorCode(uint32_t mailBoxIndex, uint32_t cmdIndex, uint32_t errorCode);
static void JtagMailboxWaitForAckClear(uint32_t mailBoxIndex, uint32_t cmdIndex);
static void JtagMailboxWaitFor1stCmdAckClear(void);

/**
 * @brief Get jtag mail box commands
 * \b Description:
 *      1. Fetch the commands from a jtag mail box.
 * 
 * @param uint32_t mailBoxIndex, the mail box index, 0 for J2P_MBX0, 1 FOR J2P_MBX1
 * @param uint32_t* commands, pointer to a storage large enough to hold the command list.  
 *                  The exact command definitions are tbd for the specific product
 * @param uint32_t* num, number of commands fetched
 * @param uint32_t* commandStatuses, each bit can stand for status of a command, such as valid, ack, processed.  The exact bit definition 
 *                  is subject to the jtag mailbox message format defined.
 * 
 * @return Status for reading the jtag mailbox register 
 */

/**
 * @brief Set jtag mail box commands status
 * \b Description:
 *      1. Set jtag mail box commands status, such as ack, complete status bits, but does not modify the commands themselves.
 * 
 * @param uint32_t mailBoxIndex, the mail box index, 0 for J2P_MBX0, 1 FOR J2P_MBX1
 * @param uint32_t commandStatuses, each bit can stand for status of a command, such as valid, ack, processed.  The exact bit definition 
 *                  is subject to the jtag mailbox message format defined.
 * 
 * @return Status for writing the jtag mailbox register 
 */
Hsp_Status_t SetJtagMailBoxCmdsStatus(uint32_t mailBoxIndex, uint32_t commandStatuses);

/**
 * @brief Write jtag mail box output data
 * \b Description:
 *      1. Write jtag mail box output data.  The usage is subject to how the jtag protocal is defined on
 *          communicating with the outside host. For example, when hsp is in the state of fetching mailbox commands,
 *          there shouldn't be any output data in the mailbox. If a mailbox command directs hsp to spew some info, hsp will
 *          proceed to write the relevant info to the mailbox, and potentially wiping out the previous command message.
 * 
 * @param uint32_t mailBoxIndex, the mail box index, 0 for J2P_MBX0, 1 FOR J2P_MBX1
 * @param uint8_t* data, storage point to output data for the outside host to process, such as certificate spew, etc.
 * @param uint32_t length, byte length of output data
 * 
 * @return Status for writing data to the jtag mailbox register 
 */
Hsp_Status_t WriteJtagMailBoxOutputData(uint32_t mailBoxIndex, uint8_t *data, uint32_t length);

static void JtagMailboxHandleFault(void)
{
    while (1)
        ;
}
static void JtagMailboxWaitForReset(void)
{
    while (1)
        ;
}
static uint32_t jtagMailboxContent[2];

static uint32_t *GetJtagMailBoxPtr(uint32_t mailboxIndex)
{
    //CRITICAL_VALIDATE_PARAMETER(mailboxIndex <= 1);
    if (mailboxIndex > 1)
    {
        JtagMailboxHandleFault();
        return NULL;
    }
    return (mailboxIndex == 0) ? (uint32_t *)J2P_MBOX0 : (uint32_t *)J2P_MBOX1;
}

static uint32_t GetJtagMailBoxContent(uint32_t mailboxIndex)
{
    return (mailboxIndex == 0) ? rdJtagMbox0() : rdJtagMbox1();
}

static void SetJtagMailBoxContent(uint32_t mailboxIndex, uint32_t content)
{
    if (mailboxIndex == 0)
        wrJtagMbox0(content);
    else
        wrJtagMbox1(content);
}

static void GetJtagMailBoxContentAll(uint32_t *content)
{
    content[0] = GetJtagMailBoxContent(0);
    content[1] = GetJtagMailBoxContent(1);
}

static void SetJtagMailBoxContentAll(uint32_t *content)
{
    SetJtagMailBoxContent(0, content[0]);
    SetJtagMailBoxContent(1, content[1]);
}

static void FetchJtagMailBoxContentAll(void)
{
    GetJtagMailBoxContentAll(jtagMailboxContent);
}

static void UpdateJtagMailBoxContentAll(void)
{
    SetJtagMailBoxContentAll(jtagMailboxContent);
}

Hsp_Status_t SetJtagMailBox1stCommandStatus(Jtag_Mailbox_Command_Status_t statusType, uint32_t commandStatus)
{
    return SetJtagMailBoxCommandStatus(0, 0, statusType, commandStatus);
}

static void JtagMailboxSetCmd(uint32_t mailBoxIndex, uint32_t cmdIndex,
                              Jtag_Mailbox_Command_t cmd)
{
    if (mailBoxIndex > 1 || cmdIndex > (JTAG_MAILBOX_MAX_COMMANDS_IN_REG - 1))
    {
        JtagMailboxHandleFault();
        return;
    }
    Jtag_Task_Reqeust_t task;
    task.u = 0;
    task.Request = cmd;

    uint32_t mailboxContent;
    Ptr_Jtag_Mailbox_Generic_Command_Struct_t ptrCmds;
    Ptr_Jtag_Task_Reqeust_t ptrTask;
    mailboxContent = GetJtagMailBoxContent(mailBoxIndex);
    ptrCmds = (Ptr_Jtag_Mailbox_Generic_Command_Struct_t)(&mailboxContent);
    ptrTask = &ptrCmds->Task_Request[cmdIndex];
    ptrTask->u = task.u;
    SetJtagMailBoxContent(mailBoxIndex, mailboxContent);
}

static void JtagMailboxSetTransitionCmd(uint32_t mailBoxIndex, uint32_t cmdIndex,
                                        Jtag_Mailbox_Command_t cmd)
{
    if (mailBoxIndex > 1 || cmdIndex > (JTAG_MAILBOX_MAX_COMMANDS_IN_REG - 1) || cmd < JTAG_MAILBOX_COMMAND_CHANGE_TO_TEST_STATE || cmd > JTAG_MAILBOX_COMMAND_CHANGE_TO_RETEST_OR_EOL_STATE)
    {
        JtagMailboxHandleFault();
        return;
    }
    uint32_t mailboxContent[2];
    mailboxContent[0] = mailboxContent[1] = 0;
    Ptr_Jtag_Mailbox_SS_Transition_Message_t ptrCmds;
    ptrCmds = (Ptr_Jtag_Mailbox_SS_Transition_Message_t)(&mailboxContent);
    ptrCmds->CommandStruct.Task_Request.Request = cmd;
    SetJtagMailBoxContentAll(mailboxContent);
}

static void JtagMailboxWaitForStatus(uint32_t mailBoxIndex, uint32_t cmdIndex,
                                     Jtag_Mailbox_Command_Status_t statusType, bool waitForVal)
{
    if (mailBoxIndex > 1 || cmdIndex > (JTAG_MAILBOX_MAX_COMMANDS_IN_REG - 1))
    {
        JtagMailboxHandleFault();
        return;
    }
    uint32_t mailboxContent;
    Ptr_Jtag_Mailbox_Generic_Command_Struct_t ptrCmds;
    Ptr_Jtag_Task_Reqeust_t ptrTask;
    bool status;

    do
    {
        mailboxContent = GetJtagMailBoxContent(mailBoxIndex);
        ptrCmds = (Ptr_Jtag_Mailbox_Generic_Command_Struct_t)(&mailboxContent);
        ptrTask = &ptrCmds->Task_Request[cmdIndex];

        if (ptrTask->Request >= JTAG_MAILBOX_COMMAND_UNKNOWN || ptrTask->Request == 0)
        {
            JtagMailboxHandleFault();
        }
        switch (statusType)
        {
        case JTAG_MAILBOX_COMMAND_STATUS_ACK:
            status = ptrTask->ACK;
            break;
        case JTAG_MAILBOX_COMMAND_STATUS_DONE:
            status = ptrTask->Done;
            break;
        case JTAG_MAILBOX_COMMAND_STATUS_PASS1_FAIL0:
            status = ptrTask->Pass1_Fail0;
            break;
        default:
            JtagMailboxHandleFault();
            return;
        }
    } while (status != waitForVal);
}

static void JtagMailboxWaitForAckClear(uint32_t mailBoxIndex, uint32_t cmdIndex)
{
    JtagMailboxWaitForStatus(mailBoxIndex, cmdIndex, JTAG_MAILBOX_COMMAND_STATUS_ACK, 0);
}

static void JtagMailboxWaitForAckSet(uint32_t mailBoxIndex, uint32_t cmdIndex)
{
    JtagMailboxWaitForStatus(mailBoxIndex, cmdIndex, JTAG_MAILBOX_COMMAND_STATUS_ACK, 1);
}

static void JtagMailboxWaitFor1stCmdAckClear(void)
{
    JtagMailboxWaitForAckClear(0, 0);
}

static void JtagMailboxWaitFor1stCmdAckSet(void)
{
    JtagMailboxWaitForAckSet(0, 0);
}

Hsp_Status_t GetJtagMailBoxCmds(uint32_t mailBoxIndex, uint32_t *commands, uint32_t *num, uint32_t *commandStatuses)
{
    return HSP_SUCCESS;
}

bool WaitForFirstJtagMailboxTask(int32_t counterMinusOne)
{
    uint32_t mailboxContent;
    Jtag_Task_Reqeust_t task;
    bool taskFlag = false;
    bool infinite = (counterMinusOne < 0) ? true : false;
    do
    {
        mailboxContent = GetJtagMailBoxContent(0);
        task.u = ((Ptr_Jtag_Mailbox_Generic_Command_Struct_t)(&mailboxContent))->Task_Request[0].u;
        taskFlag = (task.Request > 0 && task.Request < JTAG_MAILBOX_COMMAND_UNKNOWN && task.ACK == 0 && task.Done == 0);
        if (!infinite && counterMinusOne > 0)
        {
            counterMinusOne--;
        }
    } while (!taskFlag && counterMinusOne != 0);
    return taskFlag;
}

Hsp_Status_t GetJtagMailBoxTasks(uint32_t mailBoxIndex, Jtag_Task_Reqeust_t *tasks, uint32_t *num)
{
    if (mailBoxIndex > 1 || tasks == NULL || num == NULL)
    {
        JtagMailboxHandleFault();
        return HSP_INVALID_PARAMETER;
    }
    uint32_t mailboxContent = GetJtagMailBoxContent(mailBoxIndex);
    Ptr_Jtag_Mailbox_Generic_Command_Struct_t ptrCmds = (Ptr_Jtag_Mailbox_Generic_Command_Struct_t)(&mailboxContent);
    *num = 0;
    uint32_t index = 0;
    if (ptrCmds->Task_Request[index].Request < JTAG_MAILBOX_COMMAND_UNKNOWN &&
        ptrCmds->Task_Request[index].Request != 0)
    {
        tasks[index].u = ptrCmds->Task_Request[index].u;
        index++;
        (*num)++;
    }
    return HSP_SUCCESS;
}

Hsp_Status_t SetJtagMailBoxCmdsStatus(uint32_t mailBoxIndex, uint32_t commandStatuses)
{
    return HSP_SUCCESS;
}

Hsp_Status_t SetJtagMailBoxCommandStatus(uint32_t mailBoxIndex, uint32_t cmdIndex,
                                         Jtag_Mailbox_Command_Status_t statusType, uint32_t commandStatus)
{
    if (mailBoxIndex > 1 || cmdIndex > (JTAG_MAILBOX_MAX_COMMANDS_IN_REG - 1) || commandStatus > 1)
    {
        JtagMailboxHandleFault();
        return HSP_INVALID_PARAMETER;
    }
    uint32_t mailboxContent = GetJtagMailBoxContent(mailBoxIndex);
    Ptr_Jtag_Mailbox_Generic_Command_Struct_t ptrCmds = (Ptr_Jtag_Mailbox_Generic_Command_Struct_t)(&mailboxContent);
    Ptr_Jtag_Task_Reqeust_t ptrTask = &ptrCmds->Task_Request[cmdIndex];
    switch (statusType)
    {
    case JTAG_MAILBOX_COMMAND_STATUS_ACK:
        ptrTask->ACK = commandStatus;
        break;
    case JTAG_MAILBOX_COMMAND_STATUS_DONE:
        ptrTask->Done = commandStatus;
        break;
    case JTAG_MAILBOX_COMMAND_STATUS_PASS1_FAIL0:
        ptrTask->Pass1_Fail0 = commandStatus;
        break;
    default:
        JtagMailboxHandleFault();
        return HSP_UNKNOWN_ERROR;
        break;
    }
    SetJtagMailBoxContent(mailBoxIndex, mailboxContent);
    return HSP_SUCCESS;
}

Hsp_Status_t SetJtagMailBoxCommandErrorCode(uint32_t mailBoxIndex, uint32_t cmdIndex, uint32_t errorCode)
{
    if (mailBoxIndex > 1 || cmdIndex > (JTAG_MAILBOX_MAX_COMMANDS_IN_REG - 1) || errorCode > 0x1F)
    {
        JtagMailboxHandleFault();
        return HSP_INVALID_PARAMETER;
    }
    uint32_t mailboxContent = GetJtagMailBoxContent(mailBoxIndex);
    Ptr_Jtag_Mailbox_Generic_Command_Struct_t ptrCmds = (Ptr_Jtag_Mailbox_Generic_Command_Struct_t)(&mailboxContent);
    Ptr_Jtag_Task_Reqeust_t ptrTask = &ptrCmds->Task_Request[cmdIndex];
    if (ptrTask->ACK != 1 || ptrTask->Done != 1 || ptrTask->Pass1_Fail0 != 0)
    {
        JtagMailboxHandleFault();
        return HSP_UNKNOWN_ERROR;
    }

    ptrTask->Request = errorCode;
    SetJtagMailBoxContent(mailBoxIndex, mailboxContent);
    return HSP_SUCCESS;
}

Hsp_Status_t WriteJtagMailBoxOutputData(uint32_t mailBoxIndex, uint8_t *data, uint32_t length)
{
    if (mailBoxIndex > 1 || length > JTAG_MAILBOX_MAX_BYTES_IN_REG)
    {
        JtagMailboxHandleFault();
        return HSP_INVALID_PARAMETER;
    }
    uint32_t mailBoxContent = GetJtagMailBoxContent(mailBoxIndex);
    uint8_t *ptrDataDest = (uint8_t *)(&mailBoxContent);
    for (uint32_t i = 0; i < length; i++)
    {
        ptrDataDest[i] = data[i];
    }
    SetJtagMailBoxContent(mailBoxIndex, mailBoxContent);
    return HSP_SUCCESS;
}
void WaitForALongTime(void)
{
    for (uint32_t i = 0; i < 100000000; i++)
    {
    }
}

void DoReadSSFlow()
{
    setHSPReset();

    uint32_t content[2];
    content[1] = content[0] = 0;
    Ptr_Jtag_Task_Reqeust_t ptrTask;
    ptrTask = (Ptr_Jtag_Task_Reqeust_t)(content);
    ptrTask->Request = JTAG_MAILBOX_COMMAND_READ_SECURITY_STATE;
    printf("JTAG: sending jtag message c0=%x,  c1=%x \n", content[0], content[1]);
    SetJtagMailBoxContentAll(content);

    releaseReset();

    //   printf("JTAG: Wait for JtagMailboxWaitFor1stCmdAckSet\n");
    //   JtagMailboxWaitForStatus(0, 0, JTAG_MAILBOX_COMMAND_STATUS_ACK, 1);

    //   printf("After waiting for JtagMailboxWaitFor1stCmdAckSet\n");
    //   GetJtagMailBoxContentAll(content);
    //   printf("JTAG: ptrTask->ACK= %d\n", ptrTask->ACK);
    //   printf("JTAG: Read state = %d\n", content[1]);

    //   printf("JTAG: Wait for JtagMailbox Wait For 1st Cmd DONE Set\n");
    //   JtagMailboxWaitForStatus(0, 0, JTAG_MAILBOX_COMMAND_STATUS_DONE, 1);

    //   printf("JTAG: After Wait for JtagMailbox Wait For 1st Cmd DONE Set\n");

    printf("Wait for a long time...\n");
    WaitForALongTime();
    GetJtagMailBoxContentAll(content);

    printf("JTAG: ptrTask->ACK= %d\n", ptrTask->ACK);
    printf("JTAG: ptrTask->Done= %d\n", ptrTask->Done);
    printf("JTAG: ptrTask->Pass1_Fail0= %d\n", ptrTask->Pass1_Fail0);
    printf("JTAG: Read state = %d\n", content[1]);

    //   printf("JTAG: === Setting JTAG_TDR_MBX1 to 0xc001c0de\n");
    //   SetJtagMailBoxContent(1, 0xc001c0de);
}

void DoTransitionFlow(Jtag_Mailbox_Command_t jtagTransitionCmd)
{
    setHSPReset();

    if (jtagTransitionCmd == JTAG_MAILBOX_COMMAND_CHANGE_TO_RETEST_OR_EOL_STATE)
    {
        printf("JTAG: === Setting JTAG_TDR_SS_RETEST_EN to 1\n");
        enRetestChange();
    }
    uint32_t content[2];
    content[1] = content[0] = 0;
    Ptr_Jtag_Mailbox_SS_Transition_Message_t ptrCmds;
    ptrCmds = (Ptr_Jtag_Mailbox_SS_Transition_Message_t)(content);
    ptrCmds->CommandStruct.Task_Request.Request = jtagTransitionCmd;
    printf("JTAG: sending jtag message c0=%x,  c1=%x \n", content[0], content[1]);
    SetJtagMailBoxContentAll(content);
    releaseReset();

    // printf("JTAG: Wait for JtagMailboxWaitFor1stCmdAckSet\n");
    // JtagMailboxWaitFor1stCmdAckSet();

    // printf("JTAG: After waiting for JtagMailboxWaitFor1stCmdAckSet\n");
    // GetJtagMailBoxContentAll(content);
    // printf("JTAG: ptrCmds->CommandStruct.Task_Request.ACK= %d\n", ptrCmds->CommandStruct.Task_Request.ACK);

    // printf("JTAG: Wait for JtagMailbox Wait For 1st Cmd DONE Set\n");
    // JtagMailboxWaitForStatus(0, 0, JTAG_MAILBOX_COMMAND_STATUS_DONE, 1);

    // printf("JTAG: After Wait for JtagMailbox Wait For 1st Cmd DONE Set\n");

    printf("Wait for a long time...\n");
    WaitForALongTime();

    GetJtagMailBoxContentAll(content);
    printf("JTAG: ptrCmds->CommandStruct.Task_Request.ACK= %d\n", ptrCmds->CommandStruct.Task_Request.ACK);
    printf("JTAG: ptrCmds->CommandStruct.Task_Request.Done= %d\n", ptrCmds->CommandStruct.Task_Request.Done);
    printf("JTAG: ptrCmds->CommandStruct.Task_Request.Pass1_Fail0= %d\n", ptrCmds->CommandStruct.Task_Request.Pass1_Fail0);

    // printf("JTAG: === Setting JTAG_TDR_MBX1 to 0xc001c0de\n");
    // SetJtagMailBoxContent(1, 0xc001c0de);
}

void JtagSetup(bool testState)
{
    uint16_t vid = OLIMEX_ID_VENDOR;
    uint16_t pid = OLIMEX_ID_ARM_USB_TINY_H_PRODUCT;
    const char *vendor_str = "Olimex OpenOCD JTAG ARM-USB-TINY-H";

    // Open JTAG USB port
    openJtag(vid, pid, vendor_str, VERBOSE1);

    // Set Frequency
    setFrequency(1000000);

    // Setup Chain
    if (testState)
    {
        addTap(IR_WIDTH_5, "HSP_RISCV"); //Uncomment for TEST state
    }
    addTap(IR_WIDTH_5, "TDR");

    // This is required before executing any TDR Commands
    setTdrTap((testState ? 1 : 0), VERBOSE1); //set tdrTap = 1 for TEST state

    tmsReset();

    readIDCode();
}

void JtagSetupAndReset(bool testState)
{
    JtagSetup(testState);
    // TDR HSP reset
    setHSPReset();
    releaseReset();
}

#endif //__JTAGMAILBOX_HOSTSIDE_H
