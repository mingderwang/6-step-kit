
/**
  ******************************************************************************
  * @file    mc_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the MC Interface component of the Motor Control SDK:
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCInterface
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "speed_ctrl.h"
#include "mc_interface.h"
#include "motorcontrol.h"

#define ROUNDING_OFF

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCInterface Motor Control Interface
  * @brief MC Interface component of the Motor Control SDK
  *
  * @todo Document the MC Interface "module".
  *
  * @{
  */
/* Private macros ------------------------------------------------------------*/

#define round(x) ((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))

/* Functions -----------------------------------------------*/

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. It is also used to assign the
  *         state machine object, the speed and torque controller, and the FOC
  *         drive object to be used by MC Interface.
  * @param  pHandle pointer on the component instance to initialize.
  * @param  pSTM the state machine object used by the MCI.
  * @param  pSTC the speed and torque controller used by the MCI.
  * @param  pFOCVars pointer to FOC vars to be used by MCI.
  * @retval none.
  */
__weak void MCI_Init( MCI_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTC, pSixStepVars_t pSixStepVars, PWMC_Handle_t *pPWMHandle )
{
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pSTC = pSTC;
    pHandle->pSixStepVars = pSixStepVars;
    pHandle->pPWM = pPWMHandle;

    /* Buffer related initialization */
    pHandle->lastCommand = MCI_NOCOMMANDSYET;
    pHandle->hFinalSpeed = 0;
    pHandle->hFinalTorque = 0;
    pHandle->hDurationms = 0;
    pHandle->CommandState = MCI_BUFFER_EMPTY;
    pHandle->DirectCommand = MCI_NO_COMMAND;
    pHandle->State = IDLE;
    pHandle->CurrentFaults = MC_NO_FAULTS;
    pHandle->PastFaults = MC_NO_FAULTS;
#ifdef NULL_PTR_MC_INT
  }
#endif
}

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the
  *         value.
  * @retval none.
  */
__weak void MCI_ExecSpeedRamp(MCI_Handle_t *pHandle,  int16_t hFinalSpeed, uint16_t hDurationms)
{
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif

    if ((hFinalSpeed * pHandle->hFinalSpeed) >= 0)
    {
      pHandle->lastCommand = MCI_CMD_EXECSPEEDRAMP;
      pHandle->hFinalSpeed = hFinalSpeed;
      pHandle->hDurationms = hDurationms;
      pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
      pHandle->LastModalitySetByUser = MCM_SPEED_MODE;
    }
    else
    {
      pHandle->DirectCommand = MCI_STOP;
      pHandle->hFinalSpeed = hFinalSpeed;
      pHandle->hDurationms = hDurationms;
      pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
      pHandle->LastModalitySetByUser = MCM_SPEED_MODE;
    }
#ifdef NULL_PTR_MC_INT
  }
#endif
}

/**
  * @brief  This is a buffered command to set a motor speed ramp. This commands
  *         don't become active as soon as it is called but it will be executed
  *         when the pSTM state is START_RUN or RUN. User can check the status
  *         of the command calling the MCI_IsCommandAcknowledged method.
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  FinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in rpm.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the
  *         value.
  * @retval none.
  */
__weak void MCI_ExecSpeedRamp_F( MCI_Handle_t * pHandle, const float FinalSpeed, uint16_t hDurationms )
{
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (FinalSpeed * pHandle->pSTC->SpeedRefUnitExt >= 0)
    {
      int16_t hFinalSpeed = (int16_t) ((FinalSpeed * SPEED_UNIT) / U_RPM);
      MCI_ExecSpeedRamp(pHandle, hFinalSpeed, hDurationms);
	}
	else
	{
      pHandle->DirectCommand = MCI_STOP;
      int16_t hFinalSpeed = (int16_t) ((FinalSpeed * SPEED_UNIT) / U_RPM);
      MCI_ExecSpeedRamp(pHandle, hFinalSpeed, hDurationms);
    }
#ifdef NULL_PTR_MC_INT
  }
#endif
}

/**
  * @brief  This is a user command used to begin the start-up procedure.
  *         If the state machine is in IDLE state the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  *         Before calling MCI_StartMotor it is mandatory to execute one of
  *         these commands:\n
  *         MCI_ExecSpeedRamp\n
  *         MCI_ExecTorqueRamp\n
  *         MCI_SetCurrentReferences\n
  *         Otherwise the behavior in run state will be unpredictable.\n
  *         <B>Note:</B> The MCI_StartMotor command is used just to begin the
  *         start-up procedure moving the state machine from IDLE state to
  *         IDLE_START. The command MCI_StartMotor is not blocking the execution
  *         of project until the motor is really running; to do this, the user
  *         have to check the state machine and verify that the RUN state (or
  *         any other state) has been reached.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
__weak bool MCI_StartMotor(MCI_Handle_t *pHandle)
{
  bool RetVal;

  if ((IDLE == MCI_GetSTMState(pHandle)) &&
      (MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
      (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
  {
    pHandle->DirectCommand = MCI_START;
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    RetVal = true;
  }
  else
  {
    /* reject the command as the condition are not met */
    RetVal = false;
  }

  return (RetVal);
}

/**
  * @brief  This is a user command used to begin the stop motor procedure.
  *         If the state machine is in RUN or START states the command is
  *         executed instantaneously otherwise the command is discarded. User
  *         must take care of this possibility by checking the return value.\n
  *         <B>Note:</B> The MCI_StopMotor command is used just to begin the
  *         stop motor procedure moving the state machine to ANY_STOP.
  *         The command MCI_StopMotor is not blocking the execution of project
  *         until the motor is really stopped; to do this, the user have to
  *         check the state machine and verify that the IDLE state has been
  *         reached again.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
__weak bool MCI_StopMotor(MCI_Handle_t * pHandle)
{
  bool RetVal;
  bool status;
  MCI_State_t State;

  State = MCI_GetSTMState(pHandle);
  if (IDLE == State  || ICLWAIT == State)
  {
    status = false;
  }
  else
  {
    status = true;
  }

  if ((MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
      (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)) &&
       status == true )
  {
    pHandle->DirectCommand = MCI_STOP;
    RetVal = true;
  }
  else
  {
    /* reject the command as the condition are not met */
    RetVal = false;
  }

  return (RetVal);
}

/**
  * @brief  This is a user command used to indicate that the user has seen the
  *         error condition. If is possible, the command is executed
  *         instantaneously otherwise the command is discarded. User must take
  *         care of this possibility by checking the return value.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is successfully executed
  *         otherwise it return false.
  */
__weak bool MCI_FaultAcknowledged(MCI_Handle_t *pHandle)
{
  bool RetVal;

  if ((FAULT_OVER == MCI_GetSTMState(pHandle)) && (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
  {
    pHandle->PastFaults = MC_NO_FAULTS;
    pHandle->DirectCommand = MCI_ACK_FAULTS;
    RetVal = true;
  }
  else
  {
    /* reject the command as the conditions are not met */
    RetVal = false;
  }
  return (RetVal);
}

/**
 * @brief It clocks both HW and SW faults processing and update the state
 *        machine accordingly with hSetErrors, hResetErrors and present state.
 *        Refer to State_t description for more information about fault states.
 * @param pHanlde pointer of type  STM_Handle_t
 * @param hSetErrors Bit field reporting faults currently present
 * @param hResetErrors Bit field reporting faults to be cleared
 * @retval State_t New state machine state after fault processing
 */
__weak void MCI_FaultProcessing(MCI_Handle_t *pHandle, uint16_t hSetErrors, uint16_t hResetErrors)
{
  /* Set current errors */
  pHandle->CurrentFaults = (pHandle->CurrentFaults | hSetErrors ) & (~hResetErrors);
  pHandle->PastFaults |= hSetErrors;

  return;
}

/**
  * @brief  This is usually a method managed by task. It must be called
  *         periodically in order to check the status of the related pSTM object
  *         and eventually to execute the buffered command if the condition
  *         occurs.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval none.
  */
__weak void MCI_ExecBufferedCommands(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if ( pHandle->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED )
    {
      bool commandHasBeenExecuted = false;
      switch (pHandle->lastCommand)
      {
        case MCI_CMD_EXECSPEEDRAMP:
        {
          pHandle->pSixStepVars->bDriveInput = INTERNAL;
          STC_SetControlMode(pHandle->pSTC, MCM_SPEED_MODE);
          commandHasBeenExecuted = STC_ExecRamp(pHandle->pSTC, pHandle->hFinalSpeed, pHandle->hDurationms);
          break;
        }

        default:
          break;
      }

      if (commandHasBeenExecuted)
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
      }
      else
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
      }
    }
#ifdef NULL_PTR_MC_INT
  }
#endif
}

/**
  * @brief  It returns information about the state of the last buffered command.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval CommandState_t  It can be one of the following codes:
  *         - MCI_BUFFER_EMPTY if no buffered command has been called.
  *         - MCI_COMMAND_NOT_ALREADY_EXECUTED if the buffered command
  *         condition hasn't already occurred.
  *         - MCI_COMMAND_EXECUTED_SUCCESFULLY if the buffered command has
  *         been executed successfully. In this case calling this function reset
  *         the command state to BC_BUFFER_EMPTY.
  *         - MCI_COMMAND_EXECUTED_UNSUCCESFULLY if the buffered command has
  *         been executed unsuccessfully. In this case calling this function
  *         reset the command state to BC_BUFFER_EMPTY.
  */
__weak MCI_CommandState_t  MCI_IsCommandAcknowledged(MCI_Handle_t *pHandle)
{
  MCI_CommandState_t retVal;
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    retVal = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
  }
  else
  {
#endif
    retVal = pHandle->CommandState;

    if ((MCI_COMMAND_EXECUTED_SUCCESFULLY == retVal) || (MCI_COMMAND_EXECUTED_UNSUCCESFULLY == retVal) )
    {
      pHandle->CommandState = MCI_BUFFER_EMPTY;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval State_t It returns the current state of the related pSTM object.
  */
__weak MCI_State_t  MCI_GetSTMState(MCI_Handle_t *pHandle)
{
  return (pHandle->State);
}

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        historically occurred since the state machine has been moved into
  *        FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about the faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
__weak uint16_t MCI_GetOccurredFaults(MCI_Handle_t *pHandle)
{
  return ((uint16_t)pHandle->PastFaults);
}

/**
  * @brief It returns a 16 bit fields containing information about faults
  *        currently present.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  * @param pHandle Pointer on the component instance to work on.
  * @retval uint16_t  16 bit fields with information about about currently
  *         present faults.
  * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
  */
__weak uint16_t MCI_GetCurrentFaults(MCI_Handle_t *pHandle)
{
  return ((uint16_t)pHandle->CurrentFaults);
}

/**
  * @brief It returns two 16 bit fields containing information about both faults
  *        currently present and faults historically occurred since the state
  *        machine has been moved into state
  * @param pHanlde pointer of type  STM_Handle_t.
  * @retval uint32_t  Two 16 bit fields: in the most significant half are stored
  *         the information about currently present faults. In the least
  *         significant half are stored the information about the faults
  *         historically occurred since the state machine has been moved into
  *         FAULT_NOW state
  */
__weak uint32_t MCI_GetFaultState(MCI_Handle_t *pHandle)
{
  uint32_t LocalFaultState;

  LocalFaultState = (uint32_t)(pHandle->PastFaults);
  LocalFaultState |= (uint32_t)(pHandle->CurrentFaults) << 16;

  return (LocalFaultState);
}

/**
  * @brief  It returns the modality of the speed and torque controller.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval MC_ControlMode_t It returns the modality of STC. It can be one of
  *         these two values: MCM_TORQUE_MODE or MCM_SPEED_MODE.
  */
__weak MC_ControlMode_t MCI_GetControlMode(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  return ((MC_NULL == pHandle) ? MCM_TORQUE_MODE : pHandle->LastModalitySetByUser);
#else
  return (pHandle->LastModalitySetByUser);
#endif
}

/**
  * @brief  It returns the motor direction imposed by the last command
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed,
  *         hFinalTorque or Iqdref.q of the last command.
  */
__weak int16_t MCI_GetImposedMotorDirection(MCI_Handle_t *pHandle)
{
  int16_t retVal = 1;

#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    switch (pHandle->lastCommand)
    {
      case MCI_CMD_EXECSPEEDRAMP:
        if (pHandle->hFinalSpeed < 0)
        {

          retVal = -1;
        }
        break;
        break;
      default:
        break;
    }
#ifdef NULL_PTR_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  It returns information about the last ramp final speed sent by the
  *         user expressed in tenths of HZ.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t last ramp final speed sent by the user expressed in tenths
  *         of HZ.
  */
__weak int16_t MCI_GetLastRampFinalSpeed(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  int16_t retVal = 0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    retVal = pHandle->hFinalSpeed;
  }
  return (retVal);
#else
  return (pHandle->hFinalSpeed);
#endif
}

/**
  * @brief  It returns information about the last ramp Duration sent by the
  *         user .
  * @param  pHandle Pointer on the component instance to work on.
  * @retval uint16_t last ramp final torque sent by the user expressed in digit
  */
__weak uint16_t MCI_GetLastRampFinalDuration(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  uint16_t retVal = 0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    retVal = pHandle->hDurationms;
  }
  return (retVal);
#else
  return (pHandle->hDurationms);
#endif
}

/**
  * @brief  It returns last ramp final speed expressed in rpm.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval float last ramp final speed sent by the user expressed in rpm.
  */
__weak float MCI_GetLastRampFinalSpeed_F(MCI_Handle_t *pHandle)
{
  float RetVal = 0.0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    RetVal = (float)((pHandle->hFinalSpeed * U_RPM) / SPEED_UNIT);
  }
  return (RetVal);
}

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
__weak bool MCI_RampCompleted(MCI_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (RUN == MCI_GetSTMState(pHandle))
    {
      retVal = STC_RampCompleted(pHandle->pSTC);
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is executed, false otherwise.
  *
  * @deprecated This function is deprecated and should not be used anymore. It will be
  *             removed in a future version of the MCSDK. Use MCI_StopRamp() instead.
  */
__weak bool MCI_StopSpeedRamp(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  return ((MC_NULL == pHandle) ? false : STC_StopSpeedRamp(pHandle->pSTC));
#else
  return (STC_StopSpeedRamp(pHandle->pSTC));
#endif
}

/**
  * @brief  Stop the execution of ongoing ramp.
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak void MCI_StopRamp(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STC_StopRamp(pHandle->pSTC);
#ifdef NULL_PTR_MC_INT
  }
#endif
}

/**
  * @brief  It returns speed sensor reliability with reference to the sensor
  *         actually used for reference frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the speed sensor utilized for reference
  *         frame transformation and (in speed control mode) for speed
  *         regulation is reliable, false otherwise
  */
__weak bool MCI_GetSpdSensorReliability(MCI_Handle_t *pHandle)
{
  bool status;
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    status = false;
  }
  else
  {
#endif
    SpeednPosFdbk_Handle_t *SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);
    status = SPD_Check(SpeedSensor);
#ifdef NULL_PTR_MC_INT
  }
#endif

  return (status);
}

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT and related to the sensor actually
  *         used by FOC algorithm
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak int16_t MCI_GetAvrgMecSpeedUnit(MCI_Handle_t *pHandle)
{
  int16_t temp_speed;
#ifdef NULL_PTR_MC_INT
  if (MC_NULL == pHandle)
  {
    temp_speed = 0;
  }
  else
  {
#endif
    SpeednPosFdbk_Handle_t * SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);
    temp_speed = SPD_GetAvrgMecSpeedUnit(SpeedSensor);
#ifdef NULL_PTR_MC_INT
  }
#endif
  return (temp_speed);
}

/**
  * @brief  Returns the last computed average mechanical speed, expressed in rpm
  *         and related to the sensor actually used by FOC algorithm.
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak float MCI_GetAvrgMecSpeed_F(MCI_Handle_t *pHandle)
{
  SpeednPosFdbk_Handle_t *SpeedSensor = STC_GetSpeedSensor(pHandle->pSTC);

  return ((float)((SPD_GetAvrgMecSpeedUnit(SpeedSensor) * U_RPM) / SPEED_UNIT));
}

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in the unit defined by #SPEED_UNIT
  *
  * @param  pHandle Pointer on the component instance to work on.
  *
  */
__weak int16_t MCI_GetMecSpeedRefUnit(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_MC_INT
  return ((MC_NULL == pHandle) ? 0 : STC_GetMecSpeedRefUnit(pHandle->pSTC));
#else
  return (STC_GetMecSpeedRefUnit(pHandle->pSTC));
#endif
}

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in rpm.
  *
  * @param  pHandle Pointer on the component instance to work on.
  *
  */
__weak float MCI_GetMecSpeedRef_F(MCI_Handle_t *pHandle)
{
  return ((float)((STC_GetMecSpeedRefUnit( pHandle->pSTC ) * U_RPM) / SPEED_UNIT));
}

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval uint16_t Teref
  */
__weak uint16_t MCI_GetDutyCycleRef( MCI_Handle_t * pHandle )
{
  return ( pHandle->pSixStepVars->DutyCycleRef );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
