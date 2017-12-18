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
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT,              /*!< Transmitter is in transfer state. */
    STATE_TX_XFED
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
//static volatile eMBSndState eSndState;
//static volatile eMBRcvState eRcvState;

//volatile UCHAR  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

static volatile USHORT usRcvBufferPos;


/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBRTUInit( UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity, mb_struct_t *mb )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ULONG           usTimerT35_50us;

    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );   

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity,mb ) != TRUE )
    {
        eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
            usTimerT35_50us = 35;       /* 1800us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            usTimerT35_50us = ( 7UL * 20000UL ) / ( 2UL * ulBaudRate )*10;
          //usTimerT35_50us = 35;       /* 1800us. */
        }
        if( xMBPortTimersInit( ( USHORT ) usTimerT35_50us ,mb) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
    }
    
    if(mb->role == MB_MASTER){
      mb->masterState.error = 0;
      mb->masterState.isBroadcast = false;
      mb->masterState.state = 0;
      mb->masterState.taskPending = false;
      mb->masterState.timeout = 1000;   // default for 1 seconds
      mb->masterState.waitResponse = false;
    }
    
    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

//ulReloadValue = ( ACLK * ( ULONG )usTim1Timeout50us ) / MB_TIMER_TICKS; 1.5版本中的公式，

void
eMBRTUStart( mb_struct_t *mb )
{
    ENTER_CRITICAL_SECTION(  );
    
    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    
    if(mb->role == MB_SLAVE){
      mb->recvState = STATE_RX_INIT; 
      vMBPortSerialEnable( TRUE, FALSE,mb );   
    }
    else{
      mb->recvState = STATE_RX_IDLE;
      mb->sndState = STATE_TX_IDLE;
      vMBPortSerialEnable(FALSE,FALSE,mb);      // disable all function
    }
    ( void )xMBPortEventPost( EV_READY, mb );

    EXIT_CRITICAL_SECTION(  );
}

void
eMBRTUStop( mb_struct_t *mb )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE,mb );
    vMBPortTimersDisable( mb );
    EXIT_CRITICAL_SECTION(  );
}

eMBErrorCode
eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength,mb_struct_t *mb )
{
    BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;

    ENTER_CRITICAL_SECTION(  );
    assert( mb->usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

    /* Length and CRC check */
    
//    if( ( mb->serial->usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
//        && ( usMBCRC16( ( UCHAR * ) mb->serial->rxBuf, mb->serial->usRcvBufferPos ) == 0 ) )
//    {
    if( ( mb->usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( UCHAR * ) mb->buffer, mb->usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = mb->buffer[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( mb->usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & mb->buffer[MB_SER_PDU_PDU_OFF];
        mb->bFrameRcvd = TRUE;
    }
    else
    {
        eStatus = MB_EIO;
    }

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

eMBErrorCode
eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength, mb_struct_t *mb)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;

    ENTER_CRITICAL_SECTION(  );

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( mb->recvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        mb->usSndBufferCount = 1;
        mb->usSndBufferPos = 0;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        mb->usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, mb->usSndBufferCount );
        mb->buffer[mb->usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        mb->buffer[mb->usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        mb->sndState = STATE_TX_XMIT;
        vMBPortSerialEnable( FALSE, TRUE,mb );
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
xMBRTUReceiveFSM( mb_struct_t *mb )
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

    assert( mb->sndState  == STATE_TX_IDLE );

    /* Always read the character. */
    ( void )xMBPortSerialGetByte( ( CHAR * ) & ucByte,mb );
    // a character already put into mb->rxBuf;
    switch ( mb->recvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
      if(mb->role == MB_MASTER)
        mb->masterState.waitResponse = false;
        vMBPortTimersEnable( mb );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
      if(mb->role == MB_MASTER)
        mb->masterState.waitResponse = false;
        vMBPortTimersEnable( mb );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
      if(mb->role == MB_MASTER)
        mb->masterState.waitResponse = false;

      mb->usRcvBufferPos = 0;
        mb->buffer[mb->usRcvBufferPos++] = ucByte;
        mb->recvState = STATE_RX_RCV;
        /* Enable t3.5 timers. */
        vMBPortTimersEnable( mb );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( mb->usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
          mb->buffer[mb->usRcvBufferPos++] = ucByte;
        }
        else
        {
            //eRcvState = STATE_RX_ERROR;
            mb->recvState = STATE_RX_ERROR;
        }
        vMBPortTimersEnable(mb  );
        break;
    }
    return xTaskNeedSwitch;
}

BOOL
xMBRTUTransmitFSM( mb_struct_t *mb )
{
    BOOL            xNeedPoll = FALSE;

    assert(mb->recvState == STATE_RX_IDLE);

    switch ( mb->sndState)
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE,mb );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( mb->usSndBufferCount  != 0 )
        {
            xMBPortSerialPutByte( ( CHAR )mb->buffer[mb->usSndBufferPos++], mb );
            //xMBPortSerialWrite(mb);
            mb->usSndBufferCount--;
//            pucSndBufferCur++;  /* next byte in sendbuffer. */
//            usSndBufferCount--;
        }
        else
        {
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT,mb );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( TRUE, FALSE,mb);
            mb->sndState = STATE_TX_IDLE;
            if(mb->role == MB_MASTER){
              mb->sndState = STATE_TX_XFED;
            }
        }
        break;
    }

    return xNeedPoll;
}

BOOL
xMBRTUTimerT35Expired( mb_struct_t *mb )
{
    BOOL            xNeedPoll = FALSE;

    switch ( mb->recvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY,mb );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED,mb);
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }
    
    switch(mb->sndState){
    case STATE_TX_XFED:
      if(mb->masterState.waitResponse){
        if(!mb->masterState.isBroadcast){
          mb->masterState.error = EV_ERROR_RESPOND_TIMEOUT;
          xNeedPoll = xMBPortEventPost(EV_MASTER_ERROR_PROCESS,mb);
        }
      }
      break;
    default:
      assert(( mb->sndState == STATE_TX_XFED ) || ( mb->sndState == STATE_TX_IDLE ));
    }

    mb->sndState = STATE_TX_IDLE;
    mb->recvState = STATE_RX_IDLE;
    //vMBPortTimersDisableI( mb );

    return xNeedPoll;
}
