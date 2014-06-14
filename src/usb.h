/*
 * usb.h
 *
 *  Created on: Dec 13, 2012
 *      Author: XPMUser
 */

#ifndef USB_H_
#define USB_H_
#ifdef __cplusplus
 extern "C" {
#endif
/*
#define USB_OTG_HS_CORE
#define USE_USB_OTG_HS
#define USE_EMBEDDED_PHY
*/
void usb_initVars(void);

#ifdef __cplusplus
}
#endif



 // Pointer arrays for transferring data
 // 64 byte messages
 extern float64_t* USBFloat64Vars1[7];

 extern float32_t* USBFloat32Vars1[15];
 extern float32_t* USBFloat32Vars2[15];
 extern float32_t* USBFloat32Vars3[15];

 // int32
 extern int32_t* USBInt32Vars1[15];

 // uint32
 extern uint32_t* USBUint32Vars1[15];

 // Array of int16_t pointers
 extern int16_t* USBInt16Vars1[30];

 // uint16
 extern uint16_t* USBUint16Vars1[30];

 // int8
 extern int8_t* USBInt8Vars1[60];

 // uint8
 extern uint8_t* USBUint8Vars1[60];


#endif /* USB_H_ */
