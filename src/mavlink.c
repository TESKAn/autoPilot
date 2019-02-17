/*
 * mavlink.c
 *
 *  Created on: 17. feb. 2019
 *      Author: Jure
 */

#include "allinclude.h"

// Mavlink
const uint8_t ui8MavlinkHash[] = {0xfa, 0xfd, 0xb8, 0xa7, 0xda, 0x3b, 0x99, 0x0d};
const char cMavLinkParam1[] = "param1\0";
const uint8_t ui8UId2[] = "02\0";

uint16_t MavlinkTransfer()
{
	uint16_t ui16Temp = 0;
	//int32_t i32Temp = 0;
	uint64_t ui64Temp = 0;
	uint32_t ui32Temp = 0;

	if(rb32MavlinkTXQueue.count)
	{
		i32CurrentMavlinkMessage = RB32_pop(&rb32MavlinkTXQueue);
		if(-1 != i32CurrentMavlinkMessage)
		{
			switch(i32CurrentMavlinkMessage)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					ui16Temp = mavlink_msg_heartbeat_pack(1, 1, &mavlinkMessageDataTX, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					// V2
					//ui16Temp = mavlink_msg_battery_status_pack(1, 1, &mavlinkMessageDataTX, 0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 2500, FCFlightData.batMon.ui16MavlinkBatteryVoltages, FCFlightData.batMon.i16MavLinkCurrent, FCFlightData.batMon.i32MavLinkCurrentConsumed, -1, -1, -1, MAV_BATTERY_CHARGE_STATE_OK);
					// V1
					ui16Temp = mavlink_msg_battery_status_pack(1, 1, &mavlinkMessageDataTX, 0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 2500, FCFlightData.batMon.ui16MavlinkBatteryVoltages, FCFlightData.batMon.i16MavLinkCurrent, FCFlightData.batMon.i32MavLinkCurrentConsumed, -1, -1);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_ATTITUDE:
				{
					ui16Temp = mavlink_msg_attitude_pack(1, 1,&mavlinkMessageDataTX, (uint32_t)getFTime(), fusionData.ROLLPITCHYAW.roll, fusionData.ROLLPITCHYAW.pitch, fusionData.ROLLPITCHYAW.yaw, fusionData.updateRotation.x, fusionData.updateRotation.y, fusionData.updateRotation.z);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				/*
				// V2
				case MAVLINK_MSG_ID_PROTOCOL_VERSION:
				{
					ui16Temp = mavlink_msg_protocol_version_pack(1, 1, &mavlinkMessageDataTX, 200, 200, 200, ui8MavlinkHash, ui8MavlinkHash);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}*/
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				{
					ui16Temp = mavlink_msg_param_value_pack(1, 1, &mavlinkMessageDataTX, cMavLinkParam1, 1.23f, MAVLINK_TYPE_FLOAT, 1, 0);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_SYSTEM_TIME:
				{
					ui64Temp = getSystemTime();
					ui32Temp = (uint32_t)(ui64Temp / 100);
					ui64Temp = ui64Temp * 10;
					ui16Temp = mavlink_msg_system_time_pack(1, 1, &mavlinkMessageDataTX, ui64Temp, ui32Temp);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
				{
					// V2
					//ui16Temp = mavlink_msg_autopilot_version_pack(1, 1, &mavlinkMessageDataTX, 1, 1, 1, 1, 2, ui8MavlinkHash, ui8MavlinkHash, ui8MavlinkHash, 666, 999, 0, ui8UId2);
					// V1
					ui16Temp = mavlink_msg_autopilot_version_pack(1, 1, &mavlinkMessageDataTX, 1, 1, 1, 1, 2, ui8MavlinkHash, ui8MavlinkHash, ui8MavlinkHash, 1, 1, 1);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG:
				{
					break;
				}
				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					ui16Temp = mavlink_msg_sys_status_pack(1, 1, &mavlinkMessageDataTX, 1, 1, 1, 1, FCFlightData.batMon.ui16MavlinkTotalBatteryVoltage, FCFlightData.batMon.i16MavLinkCurrent, -1, 0, 0, 0, 0, 0, 0);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					ui64Temp = getSystemTime();
					ui32Temp = (uint32_t)(ui64Temp / 100);
					ui64Temp = ui64Temp * 10;
					ui16Temp = mavlink_msg_global_position_int_pack(1, 1, &mavlinkMessageDataTX, ui32Temp, GPSInfo.latitude, GPSInfo.longitude, GPSInfo.altitude, FCFlightData.ORIENTATION.i32AltitudeAboveGround, GPSInfo.velnorth, GPSInfo.veleast, GPSInfo.veltop, FCFlightData.ORIENTATION.ui16Hdg);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_VFR_HUD:
				{
					ui16Temp = mavlink_msg_vfr_hud_pack(1, 1, &mavlinkMessageDataTX, 0, 0, (int16_t)FCFlightData.ORIENTATION.ui16Hdg, FCFlightData.MOTORS.ui16Throttle, FCFlightData.ORIENTATION.f32Altitude, 0.0f);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{
					ui64Temp = getSystemTime();
					ui32Temp = (uint32_t)(ui64Temp / 100);
					ui64Temp = ui64Temp * 10;
					ui16Temp = mavlink_msg_gps_raw_int_pack(1, 1, &mavlinkMessageDataTX, ui64Temp, GPSInfo.satfix, GPSInfo.latitude, GPSInfo.longitude, GPSInfo.altitude, UINT16_MAX, UINT16_MAX, GPSInfo.velground, UINT16_MAX, GPSInfo.satnum);
					ui16Temp = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessageDataTX);
					transferDMA_USART1(mavlinkBuffer, (int)ui16Temp);
					mavlinkSendBusy = 1;
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}
	return 0;
}
