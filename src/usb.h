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

#define USB_OTG_HS_CORE
#define USE_USB_OTG_HS
#define USE_EMBEDDED_PHY

void usb_init(void);

#ifdef __cplusplus
}
#endif
#endif /* USB_H_ */
