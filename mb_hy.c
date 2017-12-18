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
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"

#include "mbport.h"

#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif

#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif

#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

/* ----------------------- Static variables ---------------------------------*/

//static UCHAR    ucMBAddress;
//static eMBMode  eMBCurrentMode;


static UCHAR eMBState = STATE_NOT_INITIALIZED;



#define NOF_INSTANCE    2

mb_struct_t *mbRoot = 0;
mb_struct_t *activeMb;


mb_struct_t *addNewMbStruct(){
  mb_struct_t *mb = (mb_struct_t*)chHeapAlloc(NULL,sizeof(mb_struct_t));
  mb->next = 0;
  
  return mb;
}

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */
//static peMBFrameSend peMBFrameSendCur;
//static pvMBFrameStart pvMBFrameStartCur;
//static pvMBFrameStop pvMBFrameStopCur;
//static peMBFrameReceive peMBFrameReceiveCur;
//static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived ) ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired ) ( void );

BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
xMBFunctionHandler *pHandler;
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};

static xMBFunctionHandler xMasterFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBMasterFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBMasterFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBMasterFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBMasterFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBMasterFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBMasterFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBMasterFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBMasterFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBMasterFuncReadDiscreteInputs},
#endif
};


/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit( eMBRole eRole, eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    mb_struct_t *curMb;
    if(!mbRoot){
      mbRoot = addNewMbStruct();
      curMb = mbRoot;
    }else{
      curMb = mbRoot;
      while(curMb->next){
        curMb = curMb->next;
      }
      curMb->next = addNewMbStruct();
      curMb = curMb->next;
    }
    curMb->address = ucSlaveAddress;
    curMb->role = eRole;
    curMb->mode = eMode;
    curMb->bEnableRx = curMb->bEnableTx = false;
    curMb->usRcvBufferPos = curMb->usSndBufferPos = 0;
    curMb->usRcvBufferCount = curMb->usSndBufferCount = 0;
    /* check preconditions */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        switch ( eMode )
        {
#if MB_RTU_ENABLED > 0
        case MB_RTU:
              switch(ucPort){
              case 1:
                curMb->driver = &SD1;
                break;
              case 2:
                curMb->driver = &SD2;
                break;
              }
            if(curMb->role == MB_SLAVE){
              eStatus = eMBRTUInit( curMb->address, ucPort, ulBaudRate, eParity,curMb );
            }
            else{
              eStatus = eMBRTUInit(0, ucPort, ulBaudRate, eParity,curMb );
              curMb->masterState.error = 0;
              curMb->masterState.isBroadcast = false;
              curMb->masterState.state = 0;
              curMb->masterState.taskPending = false;
              curMb->masterState.timeout = 0;
            }
            break;
#endif
#if MB_ASCII_ENABLED > 0
        case MB_ASCII:
              switch(ucPort){
              case 1:
                curMb->driver = &SD1;
                break;
              case 2:
                curMb->driver = &SD2;
                break;
              }
            if(curMb->role == MB_SLAVE){
              eStatus = eMBASCIIInit( curMb->address, ucPort, ulBaudRate, eParity ,curMb);
            }else{
              eStatus = eMBASCIIInit( curMb->address, ucPort, ulBaudRate, eParity ,curMb);
              curMb->masterState.error = 0;
              curMb->masterState.isBroadcast = false;
              curMb->masterState.state = 0;
              curMb->masterState.taskPending = false;
              curMb->masterState.timeout = 0;
            }
            break;
#endif
        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            if( !xMBPortEventInit(curMb) )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                curMb->state = STATE_DISABLED;
            }
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit(  eMBRole eRole, USHORT ucTCPPort)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    
    mb_struct_t *curMb;
    if(!mbRoot){
      mbRoot = addNewMbStruct();
      curMb = mbRoot;
    }else{
      curMb = mbRoot;
      while(curMb->next){
        curMb = curMb->next;
      }
      curMb->next = addNewMbStruct();
      curMb = curMb->next;
    }
    curMb->mode = MB_TCP;
    curMb->role = eRole;
    curMb->bEnableRx = curMb->bEnableTx = false;
    curMb->usRcvBufferPos = curMb->usSndBufferPos = 0;
    curMb->usRcvBufferCount = curMb->usSndBufferCount = 0;
    
    if( ( eStatus = eMBTCPDoInit( ucTCPPort,curMb ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit( curMb ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        curMb->state = STATE_DISABLED;
        curMb->role = eRole;
        curMb->mode = MB_TCP;
      if(curMb->role == MB_MASTER){
        curMb->masterState.error = 0;
        curMb->masterState.isBroadcast = false;
        curMb->masterState.state = 0;
        curMb->masterState.taskPending = false;
        curMb->masterState.timeout = 1000;   // default for 1 seconds
        curMb->masterState.waitResponse = false;
      }
    }
    return eStatus;
}
#endif

eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose(mb_struct_t *mb)
{
    eMBErrorCode    eStatus = MB_ENOERR;
 
    if( mb->state == STATE_DISABLED )
    {
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBEnable( mb_struct_t *mb )
{
    eMBErrorCode    eStatus = MB_ENOERR;


    //eMBState: STATE_ENABLED,STATE_DISABLED,STATE_NOT_INITIALIZED
    
    if( mb->state == STATE_DISABLED )            
    {
        /* Activate the protocol stack. */
      switch(mb->mode){
      case MB_RTU:
        eMBRTUStart(mb);
        break;
      case MB_ASCII:
        eMBASCIIStart(mb);
        break;
      case MB_TCP:
        eMBTCPStart(mb);
        break;
      }
        mb->state = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( mb_struct_t *mb )
{
    eMBErrorCode    eStatus;
    if( mb->state == STATE_ENABLED )
    {
      switch(mb->mode){
      case MB_RTU:
        eMBRTUStop(mb);
        break;
      case MB_ASCII:
        eMBASCIIStop(mb);
        break;
      case MB_TCP:
        eMBTCPStop(mb);
        break;
      }
        mb->state = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBPoll( mb_struct_t *mb )
{
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static UCHAR    ucFunctionCode;
    static USHORT   usLength;
    static eMBException eException;

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if( mb->state != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }

    activeMb = mb;
    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet( &eEvent,mb ) == TRUE )
    {
        switch ( eEvent )
        {
        case EV_READY:
            vMBPortTimersDisable(mb);
          break;

        case EV_FRAME_RECEIVED:
          vMBPortTimersDisable(mb);
          // todo: below code seems not required
          //====================================================================
          switch(mb->mode){
          case MB_RTU:
            eStatus = eMBRTUReceive(&ucRcvAddress, &ucMBFrame, &usLength, mb);
            break;
          case MB_ASCII:
            eStatus = eMBASCIIReceive(&ucRcvAddress,&ucMBFrame,&usLength,mb);
            break;
          case MB_TCP:
            eStatus = eMBTCPReceive(&ucRcvAddress,&ucMBFrame,&usLength,mb);
            break;
          }
          
          //====================================================================

          if( eStatus == MB_ENOERR )
          {
              /* Check if the frame is for us. If not ignore the frame. */
            switch(mb->mode){
            case MB_RTU:
            case MB_ASCII:
              if( ( ucRcvAddress == mb->address ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
              {
                  ( void )xMBPortEventPost( EV_EXECUTE,mb);
              }
              break;
            case MB_TCP:
              ( void )xMBPortEventPost( EV_EXECUTE,mb);
              break;
            }
          }else{
            mb->masterState.error = EV_ERROR_RECEIVE_DATA;
            (void)xMBPortEventPost(EV_MASTER_ERROR_PROCESS,mb);
          }
          break;

        case EV_MASTER_FRAME_RECEIVED:
          switch(mb->mode){
          case MB_RTU:
            eStatus = eMBRTUReceive(&ucRcvAddress, &ucMBFrame, &usLength, mb);
            break;
          case MB_ASCII:
            eStatus = eMBASCIIReceive(&ucRcvAddress, &ucMBFrame, &usLength, mb);
            break;
          }
            if( eStatus == MB_ENOERR )
            {
                /* Check if the frame is for us. If not ignore the frame. */
              if((eStatus == MB_ENOERR) && (ucRcvAddress == mb->address)){
                (void)xMBPortEventPost(EV_MASTER_EXECUTE,mb);
              }
            }
        
            break;

        case EV_EXECUTE:
          
          if(mb->mode == MB_TCP){
            ucMBFrame = (UCHAR*)(mb->buffer+7);
          }else{
            ucMBFrame = (UCHAR*)(mb->buffer+1);
          }
          ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;
          if(mb->role == MB_SLAVE)
            pHandler = xFuncHandlers;
          else
            pHandler = xMasterFuncHandlers;
          
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                /* No more function handlers registered. Abort. */
                if( pHandler[i].ucFunctionCode == 0 )
                {
                    break;
                }
                else if( pHandler[i].ucFunctionCode == ucFunctionCode )
                {
                    eException = pHandler[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
            if( ucRcvAddress != MB_ADDRESS_BROADCAST )
            {
                if( eException != MB_EX_NONE )
                {
                    /* An exception occured. Build an error frame. */
                    usLength = 0;
                    ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                    ucMBFrame[usLength++] = eException;
                }
                if( ( mb->mode == MB_ASCII ) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS )
                {
                    vMBPortTimersDelay( MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS );
                }                
                switch(mb->mode){
                case MB_RTU:
                  if(mb->role == MB_SLAVE)
                    eStatus = eMBRTUSend(mb->address, ucMBFrame, usLength, mb);
                  else{
                    mb->masterState.error = 0;
                    mb->masterState.state = 0;
                    mb->masterState.taskPending = false;
                    mb->masterState.waitResponse = false;
                  }
                  break;
                case MB_ASCII:
                  if(mb->role == MB_SLAVE)
                    eStatus = eMBASCIISend(mb->address,ucMBFrame,usLength,mb);
                  else{
                    mb->masterState.error = 0;
                    mb->masterState.state = 0;
                    mb->masterState.taskPending = false;
                    mb->masterState.waitResponse = false;
                  }
                  break;
                case MB_TCP:
                  if(mb->role == MB_SLAVE){
                    eStatus = eMBTCPSend(0,ucMBFrame,usLength,mb);
                  }
                  else{
                    mb->masterState.error = 0;
                    mb->masterState.state = 0;
                    mb->masterState.taskPending = false;
                    mb->masterState.waitResponse = false;
                  }
                  break;
                }
            }
            break;

        case EV_FRAME_SENT:
            if(mb->role == MB_MASTER){
              mb->masterState.waitResponse = true;
              vMBPortSerialEnable( TRUE, FALSE,mb );
              vMBPortTimersEnable(mb);
            }
            break;
        case EV_MASTER_EXECUTE:
          ucFunctionCode = mb->buffer[MB_PDU_FUNC_OFF];
          eException = MB_EX_ILLEGAL_FUNCTION;
          if(ucFunctionCode >> 7){
            eException = (eMBException)mb->buffer[MB_PDU_DATA_OFF];
          }
          else{
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                /* No more function handlers registered. Abort. */
                if( xMasterFuncHandlers[i].ucFunctionCode == 0 )
                {
                    break;
                }
                else if( xMasterFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    eException = xMasterFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }

            if( eException != MB_EX_NONE )
            {
              mb->masterState.error = EV_ERROR_EXECUTE_FUNCTION;
              xMBPortEventPost(EV_MASTER_ERROR_PROCESS,mb);
            }
            else{
              mb->masterState.taskPending = false;
            }
          }

        case EV_MASTER_FRAME_SENT:
            switch(mb->mode){
            case MB_RTU:
              eStatus = eMBRTUSend(mb->address, mb->buffer+1, mb->usSndBufferCount, mb);
              break;
            case MB_ASCII:
              eStatus = eMBASCIISend(mb->address,mb->buffer+1,mb->usSndBufferCount,mb);
              break;
            case MB_TCP:
              eStatus = eMBTCPSend(mb->address,mb->buffer+7,mb->usSndBufferCount,mb);
              break;
            }
            break;
          case EV_MASTER_ERROR_PROCESS:
            mb->masterState.taskPending = false;
            mb->masterState.waitResponse = false;
            vMBPortTimersDisable(mb);
//            ucMBFrame = mb->serial->rxBuf;
//            switch(mb->masterState.error){
//            case EV_ERROR_RESPONSE_TIMEOUT:
//             
//              break;
//            case EV_ERROR_RECEIVE_DATA:
//              break;
//            case EV_ERROR_EXECUTE_FUNCTION:
//              break;
//            }
            break;
        }
    }
    return MB_ENOERR;
}

mb_struct_t *xMBGetRootMb(void)
{
  return mbRoot;
}

mb_struct_t *xMBGetActiveMb(void)
{
  return activeMb;
}
