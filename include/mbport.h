/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbport.h,v 1.19 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include "port.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"


#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif



/* ----------------------- Type definitions ---------------------------------*/

typedef enum
{
    EV_READY,                   /*!< Startup finished. */
    EV_FRAME_RECEIVED,          /*!< Frame received. */
    EV_EXECUTE,                 /*!< Execute function. */
    EV_FRAME_SENT,               /*!< Frame sent. */
    EV_MASTER_READY,
    EV_MASTER_FRAME_RECEIVED,
    EV_MASTER_EXECUTE,
    EV_MASTER_FRAME_SENT,
    EV_MASTER_ERROR_PROCESS,
    EV_MASTER_PROCESS_SUCCESS,
    EV_MASTER_ERROR_RESPOND_TIMEOUT,
    EV_MASTER_RECEIVE_DATA,
    EV_MASTER_EXECUTE_FUNCTION
} eMBEventType;
//typedef enum
//{
//    EV_MASTER_READY                    = 1<<0,  /*!< Startup finished. */
//    EV_MASTER_FRAME_RECEIVED           = 1<<1,  /*!< Frame received. */
//    EV_MASTER_EXECUTE                  = 1<<2,  /*!< Execute function. */
//    EV_MASTER_FRAME_SENT               = 1<<3,  /*!< Frame sent. */
//    EV_MASTER_ERROR_PROCESS            = 1<<4,  /*!< Frame error process. */
//    EV_MASTER_PROCESS_SUCESS           = 1<<5,  /*!< Request process success. */
//    EV_MASTER_ERROR_RESPOND_TIMEOUT    = 1<<6,  /*!< Request respond timeout. */
//    EV_MASTER_ERROR_RECEIVE_DATA       = 1<<7,  /*!< Request receive data error. */
//    EV_MASTER_ERROR_EXECUTE_FUNCTION   = 1<<8,  /*!< Request execute function error. */
//} eMBMasterEventType;
typedef enum
{
    EV_ERROR_RESPOND_TIMEOUT,         /*!< Slave respond timeout. */
    EV_ERROR_RECEIVE_DATA,            /*!< Receive frame data erroe. */
    EV_ERROR_EXECUTE_FUNCTION,        /*!< Execute function error. */
} eMBMasterErrorEventType;

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
 */
typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Odd parity. */
    MB_PAR_EVEN                 /*!< Even parity. */
} eMBParity;

/*! \ingroup modbus
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    MB_ENOERR,                  /*!< no error. */
    MB_ENOREG,                  /*!< illegal register address. */
    MB_EINVAL,                  /*!< illegal argument. */
    MB_EPORTERR,                /*!< porting layer error. */
    MB_ENORES,                  /*!< insufficient resources. */
    MB_EIO,                     /*!< I/O error. */
    MB_EILLSTATE,               /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode;

enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
};


typedef BOOL (*cbFunction)(void);
typedef void    ( *pvMBFrameStart ) ( void );
typedef void    ( *pvMBFrameStop ) ( void );
typedef eMBErrorCode( *peMBFrameReceive ) ( UCHAR * pucRcvAddress,
                                            UCHAR ** pucFrame,
                                            USHORT * pusLength );

typedef eMBErrorCode( *peMBFrameSend ) ( UCHAR slaveAddress,
                                         const UCHAR * pucFrame,
                                         USHORT usLength );

typedef void( *pvMBFrameClose ) ( void );

typedef enum{
  TCP_CLIENT_IDLE = 0x1,
  TCP_CLIENT_CONNECTING,
  TCP_CLIENT_CONNECTED,
  TCP_CLIENT_DISCONNECTING,
  TCP_CLIENT_DISCONNECTED
}tcp_connection_state_t;

typedef struct serial_config_s{
  SerialDriver *driver;
  bool bEnableTx;
  bool bEnableRx;
  char rxBuf[520];
  char txBuf[260];
  unsigned short usRcvBufferPos;
  unsigned short usSndBufferPos;
  unsigned short usRcvBufferCount;
  unsigned short usSndBufferCount;
  bool bFrameRcvd;
  bool bFrameSend;
  uint32_t ulTimeoutMs;
  BOOL bFrameReceived;
}serial_config_t;

typedef struct tcp_config_s{
  uint16_t port;
  uint16_t remoteport;
  uint8_t remoteIp[6];
  tcp_connection_state_t connectionState;
  struct tcp_pcb *pxPCBClient;
  struct tcp_pcb *pxPCBServer;
  unsigned short usRcvBufferPos;
  unsigned short usSndBufferPos;
  unsigned short usRcvBufferCount;
  unsigned short usSndBufferCount;
  char rxBuf[520];
  char txBuf[520];
}tcp_config_t;
  
typedef struct {
  uint16_t port;
  uint16_t remoteport;
  uint8_t remoteIp[6];
  tcp_connection_state_t connectionState;
  SerialDriver *driver;
  struct tcp_pcb *pxPCBClient;
  struct tcp_pcb *pxPCBServer;
  unsigned short usRcvBufferPos;
  unsigned short usSndBufferPos;
  unsigned short usRcvBufferCount;
  unsigned short usSndBufferCount;
}modbus_stream_t;

typedef struct {
  uint8_t state;
  bool taskPending;
  bool isBroadcast;
  bool waitResponse;
  uint32_t timeout;
  uint8_t error;
}mb_master_t;


typedef struct mb_struct_s{
  struct mb_struct_s *next;
  uint8_t role; // MASTER/SLAVE
  uint8_t mode; // RTU/TCP/ASCII
  //serial_config_t *serial;
  eMBEventType eQueuedEvent;
  BOOL  bEventInQueue;
  uint8_t state;
  unsigned char recvState;
  unsigned char sndState;
  uint8_t address;
  uint8_t errorType;
  virtual_timer_t vt;
  uint32_t ulTimeoutInUs;
  uint32_t ulTimeoutMs;
  char c;
  mb_master_t masterState;
  //tcp_config_t *tcp;
  // stream definition
  uint16_t port;
  uint16_t remoteport;
  uint8_t remoteIp[6];
  tcp_connection_state_t connectionState;
  SerialDriver *driver;
  struct tcp_pcb *pxPCBClient;
  struct tcp_pcb *pxPCBServer;
  unsigned short usRcvBufferPos;
  unsigned short usSndBufferPos;
  unsigned short usRcvBufferCount;
  unsigned short usSndBufferCount;  
  bool bEnableTx;
  bool bEnableRx;
  bool bFrameRcvd;
  bool bFrameSend;
  uint8_t buffer[520];
}mb_struct_t;


/* ----------------------- Supporting functions -----------------------------*/
BOOL            xMBPortEventInit(  mb_struct_t *mb );

BOOL            xMBPortEventPost( eMBEventType eEvent, mb_struct_t *mb );

BOOL            xMBPortEventGet(  /*@out@ */ eMBEventType * eEvent , mb_struct_t *mb);

/* ----------------------- Serial port functions ----------------------------*/

BOOL            xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate,
                                   UCHAR ucDataBits, eMBParity eParity,mb_struct_t *mb );

void            vMBPortClose(mb_struct_t *mb);

void            xMBPortSerialClose( void );

void            vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable,mb_struct_t *mb );

BOOL            xMBPortSerialGetByte( CHAR * pucByte,mb_struct_t *mb );

BOOL            xMBPortSerialPutByte( CHAR ucByte,mb_struct_t *mb );

BOOL            xMBPortSerialWrite(mb_struct_t *mb);

BOOL            xMBPortSerialPoll(mb_struct_t *mb);

BOOL            xMBPortSerialSetTimeout(uint32_t timeoutMS, mb_struct_t *mb);

/* ----------------------- Timers functions ---------------------------------*/
BOOL            xMBPortTimersInit( USHORT usTimeOut50us, mb_struct_t *mb );

void            xMBPortTimersClose( mb_struct_t *mb );

void            vMBPortTimersEnable( mb_struct_t *mb );

void            vMBPortTimersDisable( mb_struct_t *mb );

void            vMBPortTimersDisableI( mb_struct_t *mb );

void            vMBPortTimersDelay( USHORT usTimeOutMS );

void            vMBPortTimerPoll(mb_struct_t *mb);

/* ----------------------- Callback for the protocol stack ------------------*/

/*!
 * \brief Callback function for the porting layer when a new byte is
 *   available.
 *
 * Depending upon the mode this callback function is used by the RTU or
 * ASCII transmission layers. In any case a call to xMBPortSerialGetByte()
 * must immediately return a new character.
 *
 * \return <code>TRUE</code> if a event was posted to the queue because
 *   a new byte was received. The port implementation should wake up the
 *   tasks which are currently blocked on the eventqueue.
 */
extern          BOOL( *pxMBFrameCBByteReceived ) ( void );

extern          BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );

extern          BOOL( *pxMBPortCBTimerExpired ) ( void );

/* ----------------------- TCP port functions -------------------------------*/
BOOL            xMBTCPPortInit( USHORT usTCPPort,mb_struct_t *mb );

void            vMBTCPPortClose( mb_struct_t *mb );

void            vMBTCPPortStart(mb_struct_t *mb);

void            vMBTCPPortDisable( mb_struct_t *mb );

BOOL            xMBTCPPortGetRequest( UCHAR **ppucMBTCPFrame, USHORT * usTCPLength,mb_struct_t *mb );

BOOL            xMBTCPPortSendResponse( const UCHAR *pucMBTCPFrame, USHORT usTCPLength,mb_struct_t *mb );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
