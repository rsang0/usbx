/**
 * @file    usbd_cdc_vcom.h
 * @author  CIP Application Team
 * @brief   mp32gm51x HAL Version definitions.
 * @version 0.1
 * @date    2023-4-3
 *
 ******************************************************************************
 * @copyright
 *
 * <h2><center>&copy; Copyright (c) 2023 CIP United Co.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by CIP under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#ifndef __USBX_DEVICE_CDC_VCOM__
#define __USBX_DEVICE_CDC_VCOM__

#include "usb.h"

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief  Test main function
 * @retval Status.
 */
int usbx_device_cdc_vcom_test(void);

#ifdef __cplusplus
}
#endif

#endif //__USBD_CDC_VCOM__