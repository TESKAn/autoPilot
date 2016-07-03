/*
 * functions.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */
#include "allinclude.h"
#include <string.h>
#include <stdlib.h>
#include "sensors/altimeter.h"

// Check RC inputs timing
int16_t CheckRCInputTimeouts()
{
	int i = 0;
	int goodChannels = 0;
	for(i=0; i<8;i++)
	{
		RCData.ch[i].PWM_Timeout++;
		if(100 < RCData.ch[i].PWM_Timeout)
		{
			RCData.ch[i].PWM_Timeout = 100;
			RCData.ch[i].PWM_Good = 0;
		}
		else
		{
			RCData.ch[i].PWM_Good = 1;
			goodChannels++;
		}
	}

	if(6 <= goodChannels)
	{
		RCData.inputs_ok = 1;
	}
	else
	{
		RCData.inputs_ok = 0;
	}

	return 0;
}

// Send data
int16_t SendCommData()
{
	int i = 0;
	// From sensors

	UART_QueueMessagef(VAR_GYRO_X, fusionData._gyro.vector.x);
	UART_QueueMessagef(VAR_GYRO_Y, fusionData._gyro.vector.y);
	UART_QueueMessagef(VAR_GYRO_Z, fusionData._gyro.vector.z);
/*
	UART_QueueMessagef(VAR_GYRO_X, fusionData._gyro.vectorRaw.x);
	UART_QueueMessagef(VAR_GYRO_Y, fusionData._gyro.vectorRaw.y);
	UART_QueueMessagef(VAR_GYRO_Z, fusionData._gyro.vectorRaw.z);
*/
	UART_QueueMessagef(VAR_ACC_X, fusionData._accelerometer.vector.x);
	UART_QueueMessagef(VAR_ACC_Y, fusionData._accelerometer.vector.y);
	UART_QueueMessagef(VAR_ACC_Z, fusionData._accelerometer.vector.z);


	UART_QueueMessagef(VAR_MAG_X, fusionData._mag.vector.x);
	UART_QueueMessagef(VAR_MAG_Y, fusionData._mag.vector.y);
	UART_QueueMessagef(VAR_MAG_Z, fusionData._mag.vector.z);

	// DCM matrix
	UART_QueueMessagef(VAR_DCM_AX, fusionData._fusion_DCM.a.x);
	UART_QueueMessagef(VAR_DCM_AY, fusionData._fusion_DCM.a.y);
	UART_QueueMessagef(VAR_DCM_AZ, fusionData._fusion_DCM.a.z);

	UART_QueueMessagef(VAR_DCM_BX, fusionData._fusion_DCM.b.x);
	UART_QueueMessagef(VAR_DCM_BY, fusionData._fusion_DCM.b.y);
	UART_QueueMessagef(VAR_DCM_BZ, fusionData._fusion_DCM.b.z);

	UART_QueueMessagef(VAR_DCM_CX, fusionData._fusion_DCM.c.x);
	UART_QueueMessagef(VAR_DCM_CY, fusionData._fusion_DCM.c.y);
	UART_QueueMessagef(VAR_DCM_CZ, fusionData._fusion_DCM.c.z);
	// 6*3*6=162

	// PWM inputs
	for(i=0; i < 8; i++)
	{
		UART_QueueMessageui16(VAR_PWMIN_1 + i, RCData.ch[i].PWMIN);
	}
	// 8*7=56

	for(i=0; i < 8; i++)
	{
		UART_QueueMessagef(VAR_PWMIN_1_ZERO + i, RCData.ch[i].PWMIN_Zero);
	}
	// 8*9=72
	for(i=0; i < 12; i++)
	{
		UART_QueueMessageui16(VAR_PWMOUT_1 + i, RCData.ch[i].PWMOUT);
	}

	// 12*7=84
	// 374
	UART_QueueMessageui16(VAR_MOTOR_FR_ARMED, (UInt16)RS485Motor_FR.REGS.ui8Armed);
	UART_QueueMessageui16(VAR_MOTOR_FR_USEPWM, (UInt16)RS485Motor_FR.REGS.ui8UsePWMIN);
	UART_QueueMessageui16(VAR_MOTOR_FR_REVERSE, (UInt16)RS485Motor_FR.REGS.ui8ReverseRotation);
	UART_QueueMessageui16(VAR_MOTOR_FR_PARK, (UInt16)RS485Motor_FR.REGS.ui8Park);
	UART_QueueMessagei16(VAR_MOTOR_FR_PARKPOS, RS485Motor_FR.REGS.i16ParkPosition);
	UART_QueueMessagei16(VAR_MOTOR_FR_PWMMIN, RS485Motor_FR.REGS.i16PWMMin);
	UART_QueueMessagei16(VAR_MOTOR_FR_PWMMAX, RS485Motor_FR.REGS.i16PWMMax);
	UART_QueueMessagei16(VAR_MOTOR_FR_CURRENTPWM, RS485Motor_FR.REGS.i16CurrentPWM);
	UART_QueueMessagei16(VAR_MOTOR_FR_SETRPM, RS485Motor_FR.REGS.i16SetRPM);
	UART_QueueMessagei16(VAR_MOTOR_FR_CURRENTRPM, RS485Motor_FR.REGS.i16RPM);
	// 10*7 = 70
	UART_QueueMessageui16(VAR_MOTOR_FL_ARMED, (UInt16)RS485Motor_FL.REGS.ui8Armed);
	UART_QueueMessageui16(VAR_MOTOR_FL_USEPWM, (UInt16)RS485Motor_FL.REGS.ui8UsePWMIN);
	UART_QueueMessageui16(VAR_MOTOR_FL_REVERSE, (UInt16)RS485Motor_FL.REGS.ui8ReverseRotation);
	UART_QueueMessageui16(VAR_MOTOR_FL_PARK, (UInt16)RS485Motor_FL.REGS.ui8Park);
	UART_QueueMessagei16(VAR_MOTOR_FL_PARKPOS, RS485Motor_FL.REGS.i16ParkPosition);
	UART_QueueMessagei16(VAR_MOTOR_FL_PWMMIN, RS485Motor_FL.REGS.i16PWMMin);
	UART_QueueMessagei16(VAR_MOTOR_FL_PWMMAX, RS485Motor_FL.REGS.i16PWMMax);
	UART_QueueMessagei16(VAR_MOTOR_FL_CURRENTPWM, RS485Motor_FL.REGS.i16CurrentPWM);
	UART_QueueMessagei16(VAR_MOTOR_FL_SETRPM, RS485Motor_FL.REGS.i16SetRPM);
	UART_QueueMessagei16(VAR_MOTOR_FL_CURRENTRPM, RS485Motor_FL.REGS.i16RPM);
	// 10*7 = 70
	UART_QueueMessageui16(VAR_MOTOR_R_ARMED, (UInt16)RS485Motor_R.REGS.ui8Armed);
	UART_QueueMessageui16(VAR_MOTOR_R_USEPWM, (UInt16)RS485Motor_R.REGS.ui8UsePWMIN);
	UART_QueueMessageui16(VAR_MOTOR_R_REVERSE, (UInt16)RS485Motor_R.REGS.ui8ReverseRotation);
	UART_QueueMessageui16(VAR_MOTOR_R_PARK, (UInt16)RS485Motor_R.REGS.ui8Park);
	UART_QueueMessagei16(VAR_MOTOR_R_PARKPOS, RS485Motor_R.REGS.i16ParkPosition);
	UART_QueueMessagei16(VAR_MOTOR_R_PWMMIN, RS485Motor_R.REGS.i16PWMMin);
	UART_QueueMessagei16(VAR_MOTOR_R_PWMMAX, RS485Motor_R.REGS.i16PWMMax);
	UART_QueueMessagei16(VAR_MOTOR_R_CURRENTPWM, RS485Motor_R.REGS.i16CurrentPWM);
	UART_QueueMessagei16(VAR_MOTOR_R_SETRPM, RS485Motor_R.REGS.i16SetRPM);
	UART_QueueMessagei16(VAR_MOTOR_R_CURRENTRPM, RS485Motor_R.REGS.i16RPM);
	// 10*7 = 70
	// 584
	UART_QueueMessageui16(VAR_SERVO_FR_TORQ_ON, (UInt16)RS485Servo_FR.REGS.ui8TorqueEnabled);
	UART_QueueMessageui16(VAR_SERVO_FR_GOAL_POS, RS485Servo_FR.REGS.ui16GoalPosition);
	UART_QueueMessageui16(VAR_SERVO_FR_POS, RS485Servo_FR.REGS.ui16PresentPosition);
	UART_QueueMessageui16(VAR_SERVO_FR_VOLTAGE, (UInt16)RS485Servo_FR.REGS.ui8PresentVoltage);
	UART_QueueMessageui16(VAR_SERVO_FR_TEMPERATURE, (UInt16)RS485Servo_FR.REGS.ui8PresentTemperature);
	// 5*7=35
	UART_QueueMessageui16(VAR_SERVO_FL_TORQ_ON, (UInt16)RS485Servo_FL.REGS.ui8TorqueEnabled);
	UART_QueueMessageui16(VAR_SERVO_FL_GOAL_POS, RS485Servo_FL.REGS.ui16GoalPosition);
	UART_QueueMessageui16(VAR_SERVO_FL_POS, RS485Servo_FL.REGS.ui16PresentPosition);
	UART_QueueMessageui16(VAR_SERVO_FL_VOLTAGE, (UInt16)RS485Servo_FL.REGS.ui8PresentVoltage);
	UART_QueueMessageui16(VAR_SERVO_FL_TEMPERATURE, (UInt16)RS485Servo_FL.REGS.ui8PresentTemperature);
	// 5*7=35
	UART_QueueMessageui16(VAR_SERVO_R_TORQ_ON, (UInt16)RS485Servo_R.REGS.ui8TorqueEnabled);
	UART_QueueMessageui16(VAR_SERVO_R_GOAL_POS, RS485Servo_R.REGS.ui16GoalPosition);
	UART_QueueMessageui16(VAR_SERVO_R_POS, RS485Servo_R.REGS.ui16PresentPosition);
	UART_QueueMessageui16(VAR_SERVO_R_VOLTAGE, (UInt16)RS485Servo_R.REGS.ui8PresentVoltage);
	UART_QueueMessageui16(VAR_SERVO_R_TEMPERATURE, (UInt16)RS485Servo_R.REGS.ui8PresentTemperature);
	// 5*7=35
	// 689
	UART_QueueMessageui32(VAR_UI32FLIGHTSTATEMACHINE, FCFlightData.ui32FlightStateMachine);
	UART_QueueMessageui32(VAR_UI32FLIGHTINITSTATE, FCFlightData.ui32FlightInitState);
	// 2*9=18

	UART_QueueMessageui16(VAR_MOTOR_FR_TEMPERATURE, (UInt16)RS485Motor_FR.REGS.ui8PresentTemperature);
	UART_QueueMessageui16(VAR_MOTOR_FL_TEMPERATURE, (UInt16)RS485Motor_FL.REGS.ui8PresentTemperature);
	UART_QueueMessageui16(VAR_MOTOR_R_TEMPERATURE, (UInt16)RS485Motor_R.REGS.ui8PresentTemperature);
	// 3*7=21
	// 728

	UART_QueueMessageui32(VAR_UI32FLIGHTDEINITSTATE, FCFlightData.ui32FlightDeInitStates);
	// 1*9=9
	// 737
	UART_QueueMessageui16(VAR_UI16REQUESTEDPOSITION_FR, FCFlightData.TILT_SERVOS.FR.ui16RequestedPosition);
	UART_QueueMessageui16(VAR_UI16REQUESTEDPOSITION_FL, FCFlightData.TILT_SERVOS.FL.ui16RequestedPosition);
	UART_QueueMessageui16(VAR_UI16REQUESTEDPOSITION_R, FCFlightData.TILT_SERVOS.R.ui16RequestedPosition);
	// 3*7 = 21
	// 758

	UART_QueueMessagef(VAR_F32NACELLETILT_FR, FCFlightData.f32NacelleTilt_FR);
	UART_QueueMessagef(VAR_F32NACELLETILT_FL, FCFlightData.f32NacelleTilt_FL);
	UART_QueueMessagef(VAR_F32NACELLETILT_R, FCFlightData.f32NacelleTilt_R);
	// 3*9=27
	// 785
	UART_QueueMessagef(VAR_PIDALTITUDE, FCFlightData.PIDAltitude.s);
	UART_QueueMessagef(VAR_PIDPITCH, FCFlightData.PIDPitch.s);
	UART_QueueMessagef(VAR_PIDROLL, FCFlightData.PIDRoll.s);
	UART_QueueMessagef(VAR_PIDYAW, FCFlightData.PIDYaw.s);
	// 4*9=36
	// 821
	UART_QueueMessagef(VAR_ORIENTATIONALTITUDE, FCFlightData.ORIENTATION.f32Altitude);
	UART_QueueMessagef(VAR_ORIENTATIONZEROALTITUDE, FCFlightData.ORIENTATION.f32ZeroAltitude);
	UART_QueueMessagef(VAR_ORIENTATIONREQUIREDALTITUDE, FCFlightData.ORIENTATION_REQUIRED.f32Altitude);
	// 3*9=27
	// 848

	UART_QueueMessagef(VAR_PIDALTITUDEKP, FCFlightData.PIDAltitude.Kp);
	UART_QueueMessagef(VAR_PIDALTITUDEKI, FCFlightData.PIDAltitude.Ki);
	UART_QueueMessagef(VAR_PIDALTITUDEKD, FCFlightData.PIDAltitude.Kd);

	UART_QueueMessagef(VAR_PIDROLLKP, FCFlightData.PIDRoll.Kp);
	UART_QueueMessagef(VAR_PIDROLLKI, FCFlightData.PIDRoll.Ki);
	UART_QueueMessagef(VAR_PIDROLLKD, FCFlightData.PIDRoll.Kd);

	UART_QueueMessagef(VAR_PIDPITCHKP, FCFlightData.PIDPitch.Kp);
	UART_QueueMessagef(VAR_PIDPITCHKI, FCFlightData.PIDPitch.Ki);
	UART_QueueMessagef(VAR_PIDPITCHKD, FCFlightData.PIDPitch.Kd);

	UART_QueueMessagef(VAR_PIDYAWKP, FCFlightData.PIDYaw.Kp);
	UART_QueueMessagef(VAR_PIDYAWKI, FCFlightData.PIDYaw.Ki);
	UART_QueueMessagef(VAR_PIDYAWKD, FCFlightData.PIDYaw.Kd);
	// 12*9=108
	// 956
	UART_QueueMessagef(VAR_RC_AILERON_DIFF, RCData.ch[RC_AILERON].PWMDiff);
	UART_QueueMessagef(VAR_RC_ELEVATOR_DIFF, RCData.ch[RC_ELEVATOR].PWMDiff);
	UART_QueueMessagef(VAR_RC_THROTTLE_DIFF, RCData.ch[RC_THROTTLE].PWMDiff);
	UART_QueueMessagef(VAR_RC_RUDDER_DIFF, RCData.ch[RC_RUDDER].PWMDiff);
	// 4*9=36
	// 992




	UART_QueueMessageui32(VAR_UI32TESTVAR, ui32TestVar);


	UART_SendBuffer();
	return 0;
}


int16_t SendCommData1()
{
	UART_QueueMessagef(VAR_YAW_ACT, FCFlightData.ORIENTATION.f32Yaw);
	UART_QueueMessagef(VAR_YAW_REQ, FCFlightData.ORIENTATION_REQUIRED.f32Yaw);
	UART_QueueMessagef(VAR_YAW_ERR, FCFlightData.PIDYaw.em);
	// 3*9=27

	UART_QueueMessagei16(VAR_MOTOR_FR_DCLINK, RS485Motor_FR.REGS.i16UIn);
	UART_QueueMessagei16(VAR_MOTOR_FL_DCLINK, RS485Motor_FL.REGS.i16UIn);
	UART_QueueMessagei16(VAR_MOTOR_R_DCLINK, RS485Motor_R.REGS.i16UIn);
	// 3*7=21
	// 48

	UART_QueueMessagei16(VAR_MOTOR_FR_POSITION, RS485Motor_FR.REGS.i16Position);
	UART_QueueMessagei16(VAR_MOTOR_FL_POSITION, RS485Motor_FL.REGS.i16Position);
	UART_QueueMessagei16(VAR_MOTOR_R_POSITION, RS485Motor_R.REGS.i16Position);
	// 3*7=21
	// 69

	UART_QueueMessagei16(VAR_MOTOR_FR_SETPARKPOSITION, FCFlightData.MOTORS.FR.i16SetParkPosition);
	UART_QueueMessagei16(VAR_MOTOR_FL_SETPARKPOSITION, FCFlightData.MOTORS.FL.i16SetParkPosition);
	UART_QueueMessagei16(VAR_MOTOR_R_SETPARKPOSITION, FCFlightData.MOTORS.R.i16SetParkPosition);
	// 3*7=21
	// 90

	UART_QueueMessagef(VAR_THROTTLE_INPUT, RCData.f32ThrottleValue);
	// 9
	// 99

	UART_QueueMessagef(VAR_RC_AILERON_MAX, RCData.ch[RC_AILERON].PWMMax);
	UART_QueueMessagef(VAR_RC_AILERON_MIN, RCData.ch[RC_AILERON].PWMMin);

	UART_QueueMessagef(VAR_RC_ELEVATOR_MAX, RCData.ch[RC_ELEVATOR].PWMMax);
	UART_QueueMessagef(VAR_RC_ELEVATOR_MIN, RCData.ch[RC_ELEVATOR].PWMMin);

	UART_QueueMessagef(VAR_RC_THROTTLE_MAX, RCData.ch[RC_THROTTLE].PWMMax);
	UART_QueueMessagef(VAR_RC_THROTTLE_MIN, RCData.ch[RC_THROTTLE].PWMMin);

	UART_QueueMessagef(VAR_RC_RUDDER_MAX, RCData.ch[RC_RUDDER].PWMMax);
	UART_QueueMessagef(VAR_RC_RUDDER_MIN, RCData.ch[RC_RUDDER].PWMMin);

	UART_QueueMessagef(VAR_ROLL_REQ, FCFlightData.ORIENTATION_REQUIRED.f32Roll);
	UART_QueueMessagef(VAR_PITCH_REQ, FCFlightData.ORIENTATION_REQUIRED.f32Pitch);

	UART_QueueMessagef(VAR_PWM_FR, RCData.ch[RC_MOTOR_FR].PWMOUT_Val);
	UART_QueueMessagef(VAR_PWM_FL, RCData.ch[RC_MOTOR_FL].PWMOUT_Val);
	UART_QueueMessagef(VAR_PWM_R, RCData.ch[RC_MOTOR_R].PWMOUT_Val);
	// 13*9=117
	// 216


	UART_QueueMessagei16(VAR_MOTOR_FR_IIN, RS485Motor_FR.REGS.i16IIn);
	UART_QueueMessagei16(VAR_MOTOR_FL_IIN, RS485Motor_FL.REGS.i16IIn);
	UART_QueueMessagei16(VAR_MOTOR_R_IIN, RS485Motor_R.REGS.i16IIn);
	// 3*7=21
	// 237

	UART_QueueMessagef(VAR_PIDROLLMAX, FCFlightData.PIDRoll.outMax);
	UART_QueueMessagef(VAR_PIDPITCHMAX, FCFlightData.PIDPitch.outMax);
	UART_QueueMessagef(VAR_PIDYAWMAX, FCFlightData.PIDYaw.outMax);

	UART_QueueMessagef(VAR_PIDROLLMIN, FCFlightData.PIDRoll.outMin);
	UART_QueueMessagef(VAR_PIDPITCHMIN, FCFlightData.PIDPitch.outMin);
	UART_QueueMessagef(VAR_PIDYAWMIN, FCFlightData.PIDYaw.outMin);
	// 6*9=54
	// 291

	UART_QueueMessagef(VAR_PIDGYROX, fusionData._gyroErrorPID.x.s);
	UART_QueueMessagef(VAR_PIDGYROY, fusionData._gyroErrorPID.y.s);
	UART_QueueMessagef(VAR_PIDGYROZ, fusionData._gyroErrorPID.z.s);
	// 3*9=27
	// 318

	UART_QueueMessagef(VAR_GYROERRORX, fusionData._gyro.gyroError.x);
	UART_QueueMessagef(VAR_GYROERRORY, fusionData._gyro.gyroError.y);
	UART_QueueMessagef(VAR_GYROERRORZ, fusionData._gyro.gyroError.z);
	// 3*9=27
	// 345

	UART_QueueMessagef(VAR_PIDROLLIM, FCFlightData.PIDRoll.im);
	UART_QueueMessagef(VAR_PIDPITCHIM, FCFlightData.PIDPitch.im);
	UART_QueueMessagef(VAR_PIDYAWIM, FCFlightData.PIDYaw.im);

	UART_QueueMessageui32(VAR_UI32SENSACQELAPSTIME, ui32ElapsedTime);

	UART_QueueMessagef(VAR_PIDROLLD, FCFlightData.PIDRoll.d);
	UART_QueueMessagef(VAR_PIDPITCHD, FCFlightData.PIDPitch.d);
	UART_QueueMessagef(VAR_PIDYAWD, FCFlightData.PIDYaw.d);

	UART_QueueMessagef(VAR_PIDROLLI, FCFlightData.PIDRoll.i);
	UART_QueueMessagef(VAR_PIDPITCHI, FCFlightData.PIDPitch.i);
	UART_QueueMessagef(VAR_PIDYAWI, FCFlightData.PIDYaw.i);

	UART_QueueMessagef(VAR_PIDROLLP, FCFlightData.PIDRoll.p);
	UART_QueueMessagef(VAR_PIDPITCHP, FCFlightData.PIDPitch.p);
	UART_QueueMessagef(VAR_PIDYAWP, FCFlightData.PIDYaw.p);

	UART_QueueMessageui32(VAR_UI32SENSORUPDATEINTERVAL, ui32SensorUpdateInterval);
	// 14*9=126
	// 471

	UART_QueueMessageui16(VAR_BATMON_CV0, RS485BatMon.REGS.ui16CellVoltage0);
	UART_QueueMessageui16(VAR_BATMON_CV1, RS485BatMon.REGS.ui16CellVoltage1);
	UART_QueueMessageui16(VAR_BATMON_CV2, RS485BatMon.REGS.ui16CellVoltage2);
	UART_QueueMessageui16(VAR_BATMON_CV3, RS485BatMon.REGS.ui16CellVoltage3);
	UART_QueueMessageui16(VAR_BATMON_CV4, RS485BatMon.REGS.ui16CellVoltage4);
	UART_QueueMessageui16(VAR_BATMON_CV5, RS485BatMon.REGS.ui16CellVoltage5);
	UART_QueueMessageui16(VAR_BATMON_CV6, RS485BatMon.REGS.ui16CellVoltage6);
	UART_QueueMessageui16(VAR_BATMON_CV7, RS485BatMon.REGS.ui16CellVoltage7);
	UART_QueueMessageui16(VAR_BATMON_CV8, RS485BatMon.REGS.ui16CellVoltage8);
	UART_QueueMessageui16(VAR_BATMON_CV9, RS485BatMon.REGS.ui16CellVoltage9);
	UART_QueueMessageui16(VAR_BATMON_CV10, RS485BatMon.REGS.ui16CellVoltage10);
	UART_QueueMessageui16(VAR_BATMON_CV11, RS485BatMon.REGS.ui16CellVoltage11);

	UART_QueueMessageui16(VAR_BATMON_STACKVOLTAGE, RS485BatMon.REGS.ui16StackVoltage);
	UART_QueueMessageui16(VAR_BATMON_OUTPUTVOLTAGE, RS485BatMon.REGS.ui16OutputVoltage);
	UART_QueueMessagei16(VAR_BATMON_OUTPUTCURRENT, RS485BatMon.REGS.i16OutputCurrent);
	UART_QueueMessageui16(VAR_BATMON_PCBTEMPERATURE, (uint16_t)RS485BatMon.REGS.ui8PCBTemperature);
	UART_QueueMessageui16(VAR_BATMON_MAHUSED, RS485BatMon.REGS.ui16MAhUsed);
	// 17*7 = 119
	// 590

	while(0 != UART2_Transferring)
	{

	}
	UART_SendBuffer();
	return 0;
}

// Calibrate sensors
void calibrateI2CSensors(void)
{

}

// Update RS485 data
void Refresh485()
{
#ifndef RS485_DEBUG
	//**************************************
	// Check servos
	CheckServo(&RS485Servo_FR);
	CheckServo(&RS485Servo_FL);
	CheckServo(&RS485Servo_R);
	//**************************************
	// Check motors
	CheckMotor(&RS485Motor_FR);
	CheckMotor(&RS485Motor_FL);
	CheckMotor(&RS485Motor_R);
	//**************************************
	// Check batmon
	CheckBatmon(&RS485BatMon);
	//**************************************
#endif
}

int16_t CheckMotor(RS485MOTOR* motor)
{
	//float32_t f32Temp = 0.0f;

	int16_t i16Temp;

	FLIGHT_MOTOR* FMotorData;

	// Get relevant data
	if(motor->REGS.ui8ID == RS485Motor_FR.REGS.ui8ID)
	{
		FMotorData = &FCFlightData.MOTORS.FR;
	}
	else if(motor->REGS.ui8ID == RS485Motor_FL.REGS.ui8ID)
	{
		FMotorData = &FCFlightData.MOTORS.FL;
	}
	else if(motor->REGS.ui8ID == RS485Motor_R.REGS.ui8ID)
	{
		FMotorData = &FCFlightData.MOTORS.R;
	}

	if(1 == motor->ui8FreshData)
	{
		FMotorData->i16PWMMin = motor->REGS.i16PWMMin;
		FMotorData->i16PWMMax = motor->REGS.i16PWMMax;
		FMotorData->ui8Enabled = motor->REGS.ui8Armed;
		FMotorData->ui8Parked = motor->REGS.ui8Park;
		FMotorData->ui8Reversed = motor->REGS.ui8ReverseRotation;
		FMotorData->ui8UsingPWM = motor->REGS.ui8UsePWMIN;
		FMotorData->ui8MeasuringPWMMin = motor->REGS.ui8MeasurePWMMin;
		FMotorData->ui8MeasuringPWMMax = motor->REGS.ui8MeasurePWMMax;
		FMotorData->i16CurrentRPM = motor->REGS.i16RPM;
		// Check park position
		i16Temp = motor->REGS.i16ParkPosition - motor->REGS.i16Position;
		if((60 > i16Temp)&&(-60 < i16Temp))
		{
			FMotorData->ui8ParkPosition = 1;
		}
		else
		{
			FMotorData->ui8ParkPosition = 0;
		}


		//***********************************
		// Check PWM measurement

		if(1 == motor->REGS.ui8MeasurePWMMin)
		{
			if(0 == FMotorData->ui8MeasPWMMin)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_MEAS_MIN_PWM, 0);
				motor->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FMotorData->ui8MeasPWMMin)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_MEAS_MIN_PWM, 1);
				motor->ui8FreshData = 0;
			}
		}
		if(1 == motor->REGS.ui8MeasurePWMMax)
		{
			if(0 == FMotorData->ui8MeasPWMMax)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_MEAS_MAX_PWM, 0);
				motor->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FMotorData->ui8MeasPWMMax)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_MEAS_MAX_PWM, 1);
				motor->ui8FreshData = 0;
			}
		}

		//***********************************

		//***********************************
		// Check enable - only if not working in PWM mode
		// Check local reg and motor reg to be sure
		if((0 == FMotorData->ui8UsePWM)&&(0 == motor->REGS.ui8UsePWMIN))
		{
			if(1 == motor->REGS.ui8Armed)
			{
				// Check
				if(0 == FMotorData->ui8Enable)
				{
					RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_ARMED, 0);
					motor->ui8FreshData = 0;
				}
			}
			else
			{
				if(1 == FMotorData->ui8Enable)
				{
					RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_ARMED, 1);
					motor->ui8FreshData = 0;
				}
			}
		}

		//***********************************

		//***********************************
		// Check park

		if(1 == motor->REGS.ui8Park)
		{
			// Check
			if(0 == FMotorData->ui8Park)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_PARK, 0);
				motor->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FMotorData->ui8Park)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_PARK, 1);
				motor->ui8FreshData = 0;
			}
		}

		//***********************************

		//***********************************
		// Check use PWM

		if(1 == motor->REGS.ui8UsePWMIN)
		{
			// Check
			if(0 == FMotorData->ui8UsePWM)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_USE_PWMIN, 0);
				motor->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FMotorData->ui8UsePWM)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_USE_PWMIN, 1);
				motor->ui8FreshData = 0;
			}
		}

		//***********************************

		//***********************************
		// Check reverse rotation

		if(1 == motor->REGS.ui8ReverseRotation)
		{
			// Check
			if(0 == FMotorData->ui8ReverseRotation)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_REVERSE, 0);
				motor->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FMotorData->ui8ReverseRotation)
			{
				RS485_WriteSlaveReg8(motor->REGS.ui8ID, MOTORREG_REVERSE, 1);
				motor->ui8FreshData = 0;
			}
		}

		//***********************************

		//***********************************
		// Check RPM

		if(0 == motor->REGS.ui8UsePWMIN)
		{
			// Check
			if(FMotorData->i16SetRPM != motor->REGS.i16SetRPM)
			{
				RS485_WriteSlaveReg16(motor->REGS.ui8ID, MOTORREG_SETRPM, FMotorData->i16SetRPM);
				motor->ui8FreshData = 0;
			}
		}
		//***********************************

		//***********************************
		// Check park position

		// Check
		if(FMotorData->i16SetParkPosition != motor->REGS.i16ParkPosition)
		{
			RS485_WriteSlaveReg16(motor->REGS.ui8ID, MOTORREG_PARKPOSITION, FMotorData->i16SetParkPosition);
			motor->ui8FreshData = 0;
		}

		//***********************************
	}
	//***********************************

	return 0;
}

int16_t CheckServo(RS485SERVO * servo)
{
	FLIGHT_SERVO* FServoData;
	float32_t f32Temp = 0.0f;

	// Get relevant data
	if(servo->REGS.ui8ID == RS485Servo_FR.REGS.ui8ID)
	{
		FServoData = &FCFlightData.TILT_SERVOS.FR;
	}
	else if(servo->REGS.ui8ID == RS485Servo_FL.REGS.ui8ID)
	{
		FServoData = &FCFlightData.TILT_SERVOS.FL;
	}
	else if(servo->REGS.ui8ID == RS485Servo_R.REGS.ui8ID)
	{
		FServoData = &FCFlightData.TILT_SERVOS.R;
	}

	//***********************************
	// Check servos
	// Fresh data?
	if(1 == servo->ui8FreshData)
	{
		FServoData->ui8Enabled = servo->REGS.ui8TorqueEnabled;
		// Calculate position from current servo data
		// 4096 = 360 deg
		// Reverse position?
		if(0 == FServoData->ui8Reverse)
		{
			f32Temp = (float32_t)servo->REGS.ui16PresentPosition;
		}
		else
		{
			f32Temp = 4096.0f - (float32_t)servo->REGS.ui16PresentPosition;
		}

		f32Temp *= 0.087890625;
		f32Temp -= FServoData->f32ServoZero;
		FServoData->f32ServoAngle = f32Temp;

		FServoData->ui16MaxTorque = servo->REGS.ui16MaxTorque;

		//***********************************

		//***********************************
		// Torque enabled?

		if(1 == servo->REGS.ui8TorqueEnabled)
		{
			// Check
			if(0 == FServoData->ui8Enable)
			{
				RS485_WriteSlaveReg8(servo->REGS.ui8ID, SERVOREG_ENABLE_TORQUE, 0);
				servo->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FServoData->ui8Enable)
			{
				RS485_WriteSlaveReg8(servo->REGS.ui8ID, SERVOREG_ENABLE_TORQUE, 1);
				servo->ui8FreshData = 0;
			}
		}

		//***********************************

		//***********************************
		// Max torque?
		if(FServoData->ui16RequestedMaxTorque != servo->REGS.ui16MaxTorque)
		{
			RS485_WriteSlaveReg16(servo->REGS.ui8ID, SERVOREG_MAX_TORQUE_REG, FServoData->ui16RequestedMaxTorque);
			servo->ui8FreshData = 0;
		}
		//***********************************

		//***********************************
		// Check position

		if(FServoData->ui16RequestedPosition != servo->REGS.ui16GoalPosition)
		{
			// Set new position
			RS485_WriteSlaveReg16(servo->REGS.ui8ID, SERVOREG_POSITION, FServoData->ui16RequestedPosition);
			servo->ui8FreshData = 0;
		}
	}
	//***********************************
	return 0;
}

int16_t CheckBatmon(RS485BATMON * batmon)
{
	float32_t f32Temp = 0.0f;
	FLIGHT_BATMON * FBatMon = &FCFlightData.batMon;

	//***********************************
	// Check servos
	// Fresh data?
	if(1 == batmon->ui8FreshData)
	{
		FBatMon->ui8OutEnabled = batmon->REGS.ui8OutputEnabled;
		//***********************************

		//***********************************
		// Out enabled?

		if(1 == batmon->REGS.ui8OutputEnabled)
		{
			// Check
			if(0 == FBatMon->ui8OutEnable)
			{
				RS485_WriteSlaveReg8(batmon->REGS.ui8ID, BATMONREG_ENABLE_OUTPUT, 0);
				batmon->ui8FreshData = 0;
			}
		}
		else
		{
			if(1 == FBatMon->ui8OutEnabled)
			{
				RS485_WriteSlaveReg8(batmon->REGS.ui8ID, BATMONREG_ENABLE_OUTPUT, 1);
				batmon->ui8FreshData = 0;
			}
		}

		//***********************************

	}
	//***********************************
	return 0;
}

// Update PWM out values
void refreshPWMOutputs(void)
{
	TIM_SetCompare4(TIM1, RCData.ch[0].PWMOUT);
	TIM_SetCompare3(TIM1, RCData.ch[1].PWMOUT);
	TIM_SetCompare2(TIM1, RCData.ch[2].PWMOUT);
	TIM_SetCompare1(TIM1, RCData.ch[3].PWMOUT);
	TIM_SetCompare4(TIM3, RCData.ch[4].PWMOUT);
	TIM_SetCompare3(TIM3, RCData.ch[5].PWMOUT);
	TIM_SetCompare2(TIM3, RCData.ch[6].PWMOUT);
	TIM_SetCompare1(TIM3, RCData.ch[7].PWMOUT);
	TIM_SetCompare4(TIM2, RCData.ch[8].PWMOUT);
	TIM_SetCompare3(TIM2, RCData.ch[9].PWMOUT);
	TIM_SetCompare2(TIM2, RCData.ch[10].PWMOUT);
	TIM_SetCompare1(TIM2, RCData.ch[11].PWMOUT);
}

// Returns system time, resolution is 10 usec
uint32_t getSystemTime(void)
{
	uint32_t time = 0;
	uint32_t timeFrac = 0;
	time = systemTime * 100;	// 1 time tick is 10 usec
	timeFrac = (uint32_t)(TIM14->CNT);
	timeFrac = timeFrac / 10;	// Period is 1000 usec
	time = time + timeFrac;
	return time;
}


float32_t getFTime(void)
{
	float32_t time = 0;
	float32_t timeFrac = 0;
	time = systemTime;
	timeFrac = (float32_t)(TIM14->CNT);
	timeFrac = timeFrac / 1000;
	time = time + timeFrac;
	return time;
}

ErrorStatus FS_Initialize(void)
{
	if(!SD_INITIALIZED)
	{
		// Try to initialize SD card
		if(disk_initialize(0) == RES_OK)
		{
			SD_INITIALIZED = 1;
			return SUCCESS;
		}
	}
	else
	{
		return SUCCESS;
	}
	return ERROR;
}

void storeAHRSAngles(FUSION_CORE *data)
{
	// Store angles

	fusionData.ROLLPITCHYAW.roll = atan2f(data->_fusion_DCM.c.y, data->_fusion_DCM.c.z);

	fusionData.ROLLPITCHYAW.pitch = -asinf(data->_fusion_DCM.c.x);

	fusionData.ROLLPITCHYAW.yaw = atan2f( data->_fusion_DCM.b.x, data->_fusion_DCM.a.x);

}

void openLog(void)
{
	unsigned int bytesWritten;

	// Check that SD card is mounted
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
		#ifdef DEBUG_USB
			sendUSBMessage("SD card mounted");
		#endif
		}
		else
		{
		#ifdef DEBUG_USB
			sendUSBMessage("SD card not mounted!");
		#endif
			return;
		}
	}

	// Generate file name
	// File name = "/LOG_ddmmyyyy_hhmmss.txt"

	// Fill buffer
	FSBuffer[0] = '/';
	FSBuffer[1] = 'L';
	FSBuffer[2] = 'O';
	FSBuffer[3] = 'G';
	FSBuffer[4] = '_';/*
	FSBuffer[5] = charFromNumber(fusionData._gps. / 10);
	FSBuffer[6] = charFromNumber(GPS_DAY % 10);
	FSBuffer[7] = charFromNumber(GPS_MONTH / 10);
	FSBuffer[8] = charFromNumber(GPS_MONTH % 10);*/
	FSBuffer[9] = '2';
	FSBuffer[10] = '0';
	FSBuffer[11] = '1';
	FSBuffer[12] = '2';
	FSBuffer[13] = '_';/*
	FSBuffer[14] = charFromNumber(GPS_HOURS / 10);
	FSBuffer[15] = charFromNumber(GPS_HOURS % 10);
	FSBuffer[16] = charFromNumber(GPS_MINUTES / 10);
	FSBuffer[17] = charFromNumber(GPS_MINUTES % 10);
	FSBuffer[18] = charFromNumber(GPS_SECONDS / 10);
	FSBuffer[19] = charFromNumber(GPS_SECONDS % 10);*/
	FSBuffer[20] = '.';
	FSBuffer[21] = 'c';
	FSBuffer[22] = 's';
	FSBuffer[23] = 'v';
	FSBuffer[24] = 0;
	// Open file
	if(f_open(&logFile, FSBuffer, FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		//flag error
	#ifdef DEBUG_USB
		sendUSBMessage("Open file error");
	#endif
		return;
	}

	// Write first line
	f_write(&logFile, "Time;GPS Lock;Lat;;Lon;;Alt;Speed;Track angle;HDOP;GG;AccX;AccY;AccZ;GyroX;GyroY;GyroZ;MagX;MagY;MagZ;Baro;AirSpeed\r\n", 136, &bytesWritten);
#ifdef DEBUG_USB
	sendUSBMessage("Log opened");
#endif

	LOG_ISOPEN = 1;
}

void closeLog(void)
{
	// Flush buffers to SD
	uint32_t BufferCount = SD_Buf1Count;
	unsigned int temp = 0;
	// Make pointer to buffer
	char* Buffer = &SD_Buffer1[0];
	// If we are using buffer 2, change pointer
	if(SD_BUF_IN_USE)
	{
		Buffer = &SD_Buffer2[0];
		BufferCount = SD_Buf2Count;
	}
	f_write(&logFile, Buffer, BufferCount, &temp);

	// Close.
	LOG_ISOPEN = 0;
	SD_WRITE_LOG = 0;
	f_close(&logFile);
#ifdef DEBUG_USB
	sendUSBMessage("Log closed");
#endif
}

void write_toLog(void)
{
	/*
	//uint16_t temp1, temp2;
	//uint32_t temp = 0;
	// Make pointer to buffer place
	uint32_t* BufferPointer = &SD_Buf1Count;
	// Make pointer to buffer
	char* Buffer = &SD_Buffer1[0];
	// If we are using buffer 2, change pointers
	if(SD_BUF_IN_USE)
	{
		Buffer = &SD_Buffer2[0];
		BufferPointer = &SD_Buf2Count;
		// Check that we are not writing buffer 2
		if(SD_WRITING_BUF2) return;
	}
	else
	{
		// Check that we are not writing buffer 1
		if(SD_WRITING_BUF1) return;
	}
	// Store time
	//BufferPointer += sprintf (&Buffer[*BufferPointer], "%d:%d:%d;", GPS_HOURS, GPS_MINUTES, GPS_SECONDS);
	// Store GPS data
	// GPS lock
	if ((GPS_VALID & 32768) != 0)
	{
		Buffer[*BufferPointer] = '1';
	}
	else
	{
		Buffer[*BufferPointer] = '0';
	}
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = ';';
	*BufferPointer = *BufferPointer + 1;
	// Latitude
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_LATITUDE, GPS_LATITUDE_FRAC);
	// N/S
	if((GPS_NS_EW & 1) != 0)
	{
		Buffer[*BufferPointer] = 'S';
	}
	else
	{
		Buffer[*BufferPointer] = 'N';
	}
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = ';';
	*BufferPointer = *BufferPointer + 1;
	// Longitude
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_LONGITUDE, GPS_LONGITUDE_FRAC);
	// E/W
	if((GPS_NS_EW & 2) != 0)
	{
		Buffer[*BufferPointer] = 'W';
	}
	else
	{
		Buffer[*BufferPointer] = 'E';
	}
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = ';';
	*BufferPointer = *BufferPointer + 1;
	// Altitude
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_ALTITUDE, GPS_ALTITUDE_FRAC);
	// Speed
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_SPEED, GPS_SPEED_FRAC);
	// Track angle
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_TRACKANGLE, GPS_TRACKANGLE_FRAC);
	// HDOP
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_HDOP, GPS_HDOP_FRAC);
	// GG
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d,%d;", GPS_GG, GPS_GG_FRAC);

	// Store accelerometer X, Y, Z
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;%d;%d;", (int16_t)ACC_X, (int16_t)ACC_Y, (int16_t)ACC_Z);
	// Store gyro X, Y, Z
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;%d;%d;", (int16_t)GYRO_X, (int16_t)GYRO_Y, (int16_t)GYRO_Z);
	// Store magnetometer X, Y, Z
	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;%d;%d;", (int16_t)MAG_X, (int16_t)MAG_Y, (int16_t)MAG_Z);

	// Store barometer pressure
	temp1 = (uint16_t)(fusionData._altimeter.pressure / 1000);
	temp2 = (uint16_t)(fusionData._altimeter.pressure % 1000);

	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d%d", temp1, temp2);

	temp1 = (uint16_t)(fusionData._altimeter.pressure_frac);

	*BufferPointer += sprintf (&Buffer[*BufferPointer], ",%d;", temp1);

	*BufferPointer += sprintf (&Buffer[*BufferPointer], "%d;", AIN3);

	// Store power Uin, Iin, mAh used, T1, T2, T3 - always positive 16 bit
	*BufferPointer = *BufferPointer + storeNumber(VOLTAGE, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(CURRENT, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(MAH, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(T1, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(T2, Buffer, *BufferPointer);
	*BufferPointer = *BufferPointer + storeNumber(T3, Buffer, *BufferPointer);

	// Write \r\n
	Buffer[*BufferPointer] = 0x0d;
	*BufferPointer = *BufferPointer + 1;
	Buffer[*BufferPointer] = 0x0a;
	*BufferPointer = *BufferPointer + 1;
	// Check value
	if(*BufferPointer > SD_BUF_MESSAGE_LIMIT)
	{
		if(!SD_BUF_IN_USE)
		{
			// Was filling buffer 1, write it and move to buffer 2
			SD_WRITING_BUF1 = 1;
			SD_BUF_IN_USE = 1;
			Buffer[*BufferPointer] = 0;
		}
		else
		{
			// Was filling buffer 2, write it and move to buffer 1
			SD_WRITING_BUF2 = 1;
			SD_BUF_IN_USE = 0;
			Buffer[*BufferPointer] = 0;
		}

	}*/
}

int storeNumber(uint16_t number, char* buffer, int offset)
{
	// Stores 16 bit number to buffer, starting at offset, ending with ;
	char temp = 0;
	int charWritten = offset;
	if(number > 10000)
	{
		temp = charFromNumber(number / 10000);
		buffer[offset] = temp;
		offset++;
	}
	if(number > 1000)
	{
		temp = charFromNumber(number / 1000);
		buffer[offset] = temp;
		offset++;
	}
	if(number > 100)
	{
		temp = charFromNumber((number / 100) % 10);
		buffer[offset] = temp;
		offset++;
	}
	if(number > 10)
	{
		temp = charFromNumber((number / 10) % 10);
		buffer[offset] = temp;
		offset++;
	}
	temp = charFromNumber(number % 10);
	buffer[offset] = temp;
	offset++;
	buffer[offset] = ';';
	offset++;
	return offset - charWritten;
}
int storeNegativeNumber(uint16_t number, char* buffer, int offset)
{
	// Stores 16 bit number to buffer, starting at offset, ending with ;
	char temp = 0;
	int charWritten = offset;
	int16_t negNumber = (int16_t)number;
	if(negNumber < 0)
	{
		// Write -
		buffer[offset] = '-';
		offset++;
		// Make positive
		negNumber = negNumber * -1;
	}
	if(negNumber > 10000)
	{
		temp = charFromNumber(negNumber / 10000);
		buffer[offset] = temp;
		offset++;
	}
	if(negNumber > 1000)
	{
		temp = charFromNumber(negNumber / 1000);
		buffer[offset] = temp;
		offset++;
	}
	if(negNumber > 100)
	{
		temp = charFromNumber((negNumber / 100) % 10);
		buffer[offset] = temp;
		offset++;
	}
	if(negNumber > 10)
	{
		temp = charFromNumber((negNumber / 10) % 10);
		buffer[offset] = temp;
		offset++;
	}
	temp = charFromNumber(negNumber % 10);
	buffer[offset] = temp;
	offset++;
	buffer[offset] = ';';
	offset++;
	return offset - charWritten;
}

//
ErrorStatus int16ToStr(int16_t value, char* text, char* str)
{
	int n = strlen(text);
	strcpy (str, text);
	n += sprintf (str+n, "%d", value);
	return SUCCESS;
}

ErrorStatus uint32ToStr(uint32_t value, char* text, char* str)
{
	//int n = strlen(text);
	strcpy (str, text);
	//n += sprintf (str+n, "%d", value);
	return SUCCESS;
}

// value - floating point value to convert
// text - text to append before value
// str - buffer for converted string

ErrorStatus float32ToStr(float32_t value, char* text, char* str)
{

	int n = strlen(text);
	int i = 0;
	float multi = 0;
	int num = 0;
	int exp = 0;
	strcpy (str, text);

	if(value != 0)
	{
		// Store +/-
		if(value < 0)
		{
			// Negative number
			// Store - sign
			strcat(str, "-");
			n++;
			// Make positive
			value *= -1;
		}
		// Move value up or down
		exp = 0;
		// If larger or equal to 10, divide by 10
		while(value >= 10.0f)
		{
			value /= 10;
			exp = exp + 1;
		}
		// If smaller than 1, multiply by 10
		while(value < 1)
		{
			value *= 10;
			exp = exp - 1;
		}

		num = (int)value;
		n += sprintf (str+n, "%d", num);
		strcat(str, ".");
		n++;
		value = value - (float)num;
		multi = 1000000000;
		for(i=0; i < 9; i++)
		{
			value = value * 10;
			multi = multi / 10;
			num = (int)value;
			if(num == 0)
			{
				strcat(str, "0");
				n++;
			}
			else
			{
				value = value * multi;
				num = (int)value;
				n += sprintf (str+n, "%d", num);
				break;
			}
		}
		// Store e
		strcat(str, "e");
		n++;
		// Store exp
		n += sprintf (str+n, "%d", exp);
		}
	else
	{
		sprintf (str+n, "%d", 0);
	}
	return SUCCESS;
}

ErrorStatus strToFloat32(float32_t* result, char* file, char* str)
{
	char* strBeginning = 0;
	char* strEnd = 0;
	char* numBeginning = 0;
	uint8_t strBeginLen = strlen(str);
	float32_t convertedValue = 0;

	// file - file that contains data
	// str - string that marks data, in form of val1=1234;

	strBeginning = strstr(file, str);
	// Check that it is not null
	if(strBeginning != NULL)
	{
		// Get pointer to where in file string ends
		strEnd = strstr(strBeginning, ";");

		numBeginning = strBeginning + strBeginLen;

		// Check that we have ending
		if(strEnd == NULL)
		{
			// Return error
			return ERROR;
		}
		// Convert
		convertedValue = strtof(numBeginning, (char**)strEnd);
	}
	else
	{
		return ERROR;
	}
	*result = convertedValue;
	return SUCCESS;
}

uint16_t strTouint16(char* file, char* str)
{
	char* strBeginning = 0;
	char* strEnd = 0;
	uint8_t convert = 0;
	uint8_t strBeginLen = strlen(str);
	uint16_t convertedValue = 0;
	uint8_t i = 0;
	//uint32_t value = 0;
	char currentChar = 0;
	// file - file that contains data
	// str - string that marks data, in form of val1=1234;
	strBeginning = strstr(file, str);
	// Check that it is not null
	if(strBeginning != NULL)
	{
		// Get pointer to where in file string ends
		strEnd = strstr(strBeginning, ";");
		// Count how many chars to convert
		convert = (uint8_t)((uint32_t)(strEnd - strBeginning) - strBeginLen);
		// Convert
		for(i = 0; i < convert; i++)
		{
			convertedValue = convertedValue * 10;
			// Get value
			currentChar = strBeginning[5 + i];
			convertedValue = convertedValue + (uint16_t)numberFromChar(currentChar);
		}
	}
	else
	{
		convertedValue = 0;
	}
	return convertedValue;
}



char charFromNumber(uint8_t number)
{
	return number + 48;
}
uint8_t numberFromChar(char c)
{
	return c - 48;
}

// Function to mount SD card
ErrorStatus mountSDCard(void)
{
	// Check if card is mounted
	if(SD_MOUNTED)
	{
		// If it is, return
		#ifdef DEBUG_USB
			sendUSBMessage("FS already mounted");
		#endif
		return ERROR;
	}
	// Load drive
	if(f_mount(0, &FileSystemObject)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("FS mount error");
		#endif
		// Flag error
		f_mount(0,0);
		return ERROR;
	}
	driveStatus = disk_initialize(0);
	if((driveStatus & STA_NOINIT) ||
		   (driveStatus & STA_NODISK) ||
		   (driveStatus & STA_PROTECT))
	{
		#ifdef DEBUG_USB
			sendUSBMessage("Drive Status error");
		#endif
		// Flag error.
		f_mount(0,0);
		return ERROR;
	}
	// Mark SD card is mounted
	SD_MOUNTED = 1;
	return SUCCESS;
}

// Function to unmount SD card
ErrorStatus unmountSDCard(void)
{
	if(SD_MOUNTED)
	{
		// Close all open files
		f_mount(0,0);
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

ErrorStatus loadSingleSetting(char* name, float32_t* storeLocation)
{
	FIL settingsFile;
	unsigned int bytesToRead = 0;
	unsigned int readBytes = 0;
	unsigned int bytesProcessed = 0;
	unsigned int fileSize = 0;

	// Check that SD card is mounted
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card mounted");
			#endif
		}
		else
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card not mounted!");
			#endif
			return ERROR;
		}
	}
	// Open file
	if(f_open(&settingsFile, "/settings.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("File open error");
		#endif
		// Flag error
		f_close(&settingsFile);
		return ERROR;
	}

	fileSize = f_size(&settingsFile);

	do
	{
		bytesToRead = 255;
		if((bytesToRead + bytesProcessed) > fileSize)
		{
			bytesToRead = fileSize - bytesProcessed;
		}
		// Read file to buffer
		if(f_read (&settingsFile, &FSBuffer[0], bytesToRead, &readBytes) != FR_OK)
		{
	#ifdef DEBUG_USB
			sendUSBMessage("File read error");
	#endif
			// Close and unmount.
			f_close(&settingsFile);
			return ERROR;
		}
		// Store null character to last place
		FSBuffer[254] = '\0';
		// Check if required data is in settings file
		if(strToFloat32(storeLocation, &FSBuffer[0], name) == SUCCESS)
		{
			// Return success
			//Close and unmount.
			f_close(&settingsFile);
			return SUCCESS;
		}
		else
		{
			// Set processed bytes
			bytesProcessed = bytesProcessed + 200;
			// Else seek for next section
			f_lseek(&settingsFile, bytesProcessed);
		}
	}
	while(bytesProcessed < fileSize);

	// Close
	f_close(&settingsFile);

	return ERROR;
}

// Store parameters calculated in software
// Like PID coefficients etc.
ErrorStatus storeRunningValues(void)
{
	FIL settingsFile;
#ifdef DEBUG_USB
	sendUSBMessage("Begin saving parameters");
#endif
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card mounted");
			#endif
		}
		else
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card not mounted!");
			#endif
			return ERROR;
		}
	}
	// Open file
	if(f_open(&settingsFile, "/settings1.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("File open error");
		#endif
		// Flag error
		f_close(&settingsFile);
		return ERROR;
	}
	/*
	// Store number
	float32ToStr(ahrs_data.accRate, "accRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);
	*/
	// Close file
	f_close(&settingsFile);

#ifdef DEBUG_USB
	sendUSBMessage("Parameters saved");
#endif

	return SUCCESS;
}

// Store program parameters
ErrorStatus storeSettings(void)
{
	FIL settingsFile;
#ifdef DEBUG_USB
	sendUSBMessage("Begin saving settings");
#endif
	if(!SD_MOUNTED)
	{
		// Try to mount SD card
		if(mountSDCard() == SUCCESS)
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card mounted");
			#endif
		}
		else
		{
			#ifdef DEBUG_USB
				sendUSBMessage("SD card not mounted!");
			#endif
			return ERROR;
		}
	}
	// Open file
	if(f_open(&settingsFile, "/settings1.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
	{
		#ifdef DEBUG_USB
			sendUSBMessage("File open error");
		#endif
		// Flag error
		f_close(&settingsFile);
		return ERROR;
	}
	// Store settings
	// Acc rate
	// Store number
	//float32ToStr(ahrs_data.accRate, "accRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Acceleration rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Gyro rate
	// Store number
	//float32ToStr(ahrs_data.gyroRate, "gyroRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Gyro rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Mag rate
	// Store number
	//float32ToStr(ahrs_data.magRate, "magRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Magnetometer rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Mag inclination
	// Store number
	float32ToStr(DEFAULT_MAG_INCLINATION, "magInclination=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Magnetic field inclination\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Mag declination
	// Store number
	float32ToStr(DEFAULT_MAG_DECLINATION, "magDeclination=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Magnetic field declination\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// PID threshold
	// Store number
	//float32ToStr(ahrs_data.PIDErrorThreshold, "PIDThreshold=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Minimal PID error to use\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Minimal rotation rate that is detected
	// Store number
	//float32ToStr(ahrs_data.MinRotationRate, "MinRotationRate=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Minimal detectable rotation rate\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Soft mag matrix
	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rxx], "softMagRxx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Softmag matrix coefficients\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Ryx], "softMagRyx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rzx], "softMagRzx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rxy], "softMagRxy=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Ryy], "softMagRyy=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rzy], "softMagRzy=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rxz], "softMagRxz=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Ryz], "softMagRyz=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.magRotationMatrix.vector.pData[Rzz], "softMagRzz=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Hard mag vector
	// Store number
	//float32ToStr(ahrs_data.MagOffsetVector.vector.pData[VECT_X], "hardMagX=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Hard mag vector\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.MagOffsetVector.vector.pData[VECT_Y], "hardMagY=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.MagOffsetVector.vector.pData[VECT_Z], "hardMagZ=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Hard mag scale
	// Store number
	float32ToStr(SOFTMAG_SCALE, "hardMagScale=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Hard mag scale\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// PID parameters
	// Store number
	//float32ToStr(ahrs_data.PIData.Kix, "Kix=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("PID parameters\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.Kpx, "Kpx=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.eMax, "PID_MaxErr=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.eMin, "PID_MinErr=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.maxIx, "PID_MaxI=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.minIx, "PID_MinI=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.rMax, "PID_MaxR=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.PIData.rMin, "PID_MinR=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);


	// Roll, pitch, yaw correction factors
	// Store number
	//float32ToStr(ahrs_data.RollPitchCorrectionScale, "rollPitchCorrectionScale=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Roll, pitch, yaw correction factors\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Store number
	//float32ToStr(ahrs_data.YawCorrectionScale, "yawCorrectionScale=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts (FSBuffer, &settingsFile);

	// Samples to discard on startup
	// Store number
	//float32ToStr(ahrs_data.sampleDiscardCount, "discardCount=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Samples to discard on startup\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Minimum GPS speed to use GPS for yaw
	// Store number
	//float32ToStr(ahrs_data.MinGPSSpeed, "useGPSSpeed=", FSBuffer);
	// Store end of line
	strcat(FSBuffer, ";\r\n");
	// Write
	f_puts ("Minimum GPS speed to use GPS for yaw\r\n", &settingsFile);
	f_puts (FSBuffer, &settingsFile);

	// Close file
	f_close(&settingsFile);

#ifdef DEBUG_USB
	sendUSBMessage("Settings saved");
#endif
	return SUCCESS;
}

ErrorStatus loadSettings(void)
{
	// Load all settings, one at a time
	/*
	// Acceleration rate
	if(loadSingleSetting("accRate=", &(ahrs_data.accRate)) != SUCCESS)
	{
		// Load default value
		ahrs_data.accRate = DEFAULT_ACC_RATE;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.accRate, "accRate=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Gyroscope rate
	if(loadSingleSetting("gyroRate=", &(ahrs_data.gyroRate)) != SUCCESS)
	{
		// Load default value
		ahrs_data.gyroRate = DEFAULT_GYRO_RATE;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.gyroRate, "gyroRate=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Magnetometer rate
	if(loadSingleSetting("magRate=", &(ahrs_data.magRate)) != SUCCESS)
	{
		// Load default value
		ahrs_data.magRate = DEFAULT_MAG_RATE;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.magRate, "magRate=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Ki factor
	if(loadSingleSetting("Kix=", &(ahrs_data.PIData.Kix)) != SUCCESS)
	{
		// Load default value
		ahrs_data.PIData.Kix = DEFAULT_KI;
	}
	// Store
	ahrs_data.PIData.Kiy = ahrs_data.PIData.Kix;
	ahrs_data.PIData.Kiz = ahrs_data.PIData.Kix;
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.PIData.Kix, "Kix=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif

	// Kp factor
	if(loadSingleSetting("Kpx=", &(ahrs_data.PIData.Kpx)) != SUCCESS)
	{
		// Load default value
		ahrs_data.PIData.Kpx = DEFAULT_KP;
	}
	// Store
	ahrs_data.PIData.Kpy = ahrs_data.PIData.Kpx;
	ahrs_data.PIData.Kpz = ahrs_data.PIData.Kpx;
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.PIData.Kpx, "Kpx=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif
	// Load mag matrix
	// Rxx
	if(loadSingleSetting("softMagRxx=", &(ahrs_data.rotationMatrix.vector.pData[Rxx])) != SUCCESS)
	{
		// Load default value
		ahrs_data.rotationMatrix.vector.pData[Rxx] = SOFTMAG_DEFAULT_RXX;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.rotationMatrix.vector.pData[Rxx], "softMagRxx=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif

	// Ryx
	if(loadSingleSetting("softMagRyx=", &(ahrs_data.rotationMatrix.vector.pData[Ryx])) != SUCCESS)
	{
		// Load default value
		ahrs_data.rotationMatrix.vector.pData[Ryx] = SOFTMAG_DEFAULT_RYX;
	}
#ifdef DEBUG_USB
	float32ToStr(ahrs_data.rotationMatrix.vector.pData[Ryx], "softMagRyx=", StringBuffer);
	sendUSBMessage(StringBuffer);
#endif

*/

#ifdef DEBUG_USB
	sendUSBMessage("Settings loaded successfully");
#endif
	return SUCCESS;
}


void extPeripheralInit(void)
{
	// Turn sensor power ON
	SENSOR_POWER_ON;
	// Initialize SD card
	FS_Initialize();

	// Setup GPS
	GPSSetDataOutput(getSystemTime());

	Delayms(100);
	// Setup sensors

	// set PS busy
	PSBUSY = 1;
	// Short delay
	Delaynus(5000);

	sensorInit();

	ADC_ENABLED = 1;
	// Mark sensors initiated
	EXTSENS_INIT_DONE = 1;
	// Mark null sensor
	//EXTSENS_NULLING_GYRO = 1;

	Delayms(100);

}

void Delayms(uint32_t ms)
{
	uint32_t time = systemTime - 1;
	uint32_t deltaTime = 0;
	while(deltaTime < ms)
	{
		Delaynus(1500);
		if(systemTime == time)
		{
			// Error - time is not counting, break the loop
			Delaynus(ms * 1000);
			break;
		}
		deltaTime = systemTime - time;
	}
}

/*
 * Function Name  : Delaynus
 * Description    : Inserts a delay time abort nus.
 * Input          :nus
 * Output         : None
 * Return         : None
 */
void Delaynus(vu32 nus)
{
    u8 nCount;

    while (nus--)
    {
        for (nCount = 6; nCount != 0; nCount--);
    }
}

void transferDMA_USART1(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART1 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART1);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_USART1_CH;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;//    USART1_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART1, &DMAInitStructure);

	//Enable DMA2 stream 7 - USART1 TX
	DMA_Cmd(DMA_USART1, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART1, DMA_IT_TC, ENABLE);
}

void transferDMA_USART2(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART2 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART2);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_4;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;//    USART2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART2, &DMAInitStructure);

	//Enable DMA1 stream 0 - USART2 TX
	DMA_Cmd(DMA_USART2, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART2, DMA_IT_TC, ENABLE);
}

void transferDMA_USART3(uint8_t *data, int length)
{
	// Mark GPS is sending data
	GPS_Sending(1);
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART3 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART3);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;//    USART3_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART3, &DMAInitStructure);

	//Enable DMA1 stream 4 - USART3 TX
	DMA_Cmd(DMA_USART3, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART3, DMA_IT_TC, ENABLE);
}

float32_t intToFloat(uint16_t whole, uint16_t frac)
{
	float32_t result = 0;
	float32_t temp = 0;
	temp = (float32_t)frac;
	while(temp > 1.0f)
	{
		temp = temp / 10;
	}
	result = (float32_t)whole + temp;
	return result;
}

// Ring buffer functions

int16_t RB_full(RING_BUFFER* rb)
{
    if(rb->count == rb->size) return 0;
    else return -1;
}

int16_t RB_Init(RING_BUFFER* rb, uint8_t *buf, int16_t size)
{
	rb->buffer = buf;

    rb->buffer_end = rb->buffer + size;
    rb->size = size;
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;

	return 0;
}

int16_t RB_push(RING_BUFFER* rb, uint8_t data)
{
	if(rb->count < rb->size)
	{
		*rb->data_end = data;
		rb->data_end++;
		if (rb->data_end == rb->buffer_end)
		{
			rb->data_end = rb->buffer;
		}
		rb->count++;
	}
	else
	{
		// Return error
		return -1;
	}
	return 0;
}

uint8_t RB_pop(RING_BUFFER* rb)
{
	if(0 < rb->count)
	{
		uint8_t data = *rb->data_start;
		rb->data_start++;
		rb->count--;
		if (rb->data_start == rb->buffer_end)
		{
			rb->data_start = rb->buffer;
		}
		// Check buffer
		if(0 == rb->count)
		{
			// start equal end?
			if(rb->data_start != rb->data_end)
			{
				// If not, fix it.
				rb->count++;
				//rb->data_start = rb->data_end;
			}
		}
		return data;
	}
	return 0;
}

int16_t RB_flush(RING_BUFFER* rb)
{
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;
    return 0;
}

// 32 bit ring buffer

int16_t RB32_full(RING_BUFFER32* rb)
{
    if(rb->count == rb->size) return 0;
    else return -1;
}

int16_t RB32_Init(RING_BUFFER32* rb, uint32_t *buf, int16_t size)
{
	rb->buffer = buf;
    rb->buffer_end = rb->buffer + size;
    rb->size = size;
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;

	return 0;
}

int16_t RB32_push(RING_BUFFER32* rb, uint32_t data)
{
	if(rb->count < rb->size)
	{
		*rb->data_end = data;
		rb->data_end++;
		if (rb->data_end == rb->buffer_end)
		{
			rb->data_end = rb->buffer;
		}
		rb->count++;
	}
	else
	{
		// Return error
		return -1;
	}
	return 0;
}

uint32_t RB32_pop(RING_BUFFER32* rb)
{
	if(0 < rb->count)
	{
		uint32_t data = *rb->data_start;
		rb->data_start++;
		rb->count--;
		if (rb->data_start == rb->buffer_end)
		{
			rb->data_start = rb->buffer;
		}
		// Check buffer
		if(0 == rb->count)
		{
			// start equal end?
			if(rb->data_start != rb->data_end)
			{
				// If not, fix it.
				rb->count++;
				//rb->data_start = rb->data_end;
			}
		}
		return data;
	}
    return 0;
}

int16_t RB32_flush(RING_BUFFER32* rb)
{
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;
    return 0;
}

