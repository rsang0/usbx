/**
 * @file    usbd_cdc_vcom.c
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

#include <string.h>
#include "usbx_device_cdc_vcom.h"

#include "dbg_log.h"
#include "mp32gm51fxx.h"
#include "mp32gm51_hal_config.h"
#include "mp32gm51_hal_rcm.h"
#include "mp32gm51_hal_usb.h"
#include "common_include/usb.h"
#include "ux_dcd_musb.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_api.h"
#include "usb_device_musb.h"

#define STATE_DEVICE_NO            0x01
#define STATE_DEVICE_CONN          0x02
#define STATE_DEVICE_READY         0x04
#define STATE_DEVICE_READ          0x08
#define STATE_DEVICE_WRITE         0x10
#define STATE_DEVICE_IDLE          0x20
#define STATE_POWER_FAULT          0x40
#define STATE_USB_DMA_HOST_TX_COMP 0X50
#define STATE_USB_DMA_DEV_TX_COMP  0X51

void EnableIRQ(IRQn_t irq_num);
uint32_t _cdc_flag = STATE_DEVICE_NO;
usb_device_handle deviceHandle;
uint32_t actual_length;
uint32_t requested_length;
UCHAR buffer[20];

uint32_t contrl_handle(void *data, uint32_t event)
{
    switch (event) {
        case 3:
            USB0->TSTIDXR = ((USB0->TSTIDXR & 0xff00) | 1);
            USB0->IEP.CSR.TXEPN |= USB_OTG_IEP_TXCSR_MODE;
            //USB0->INDEX = 2;
            USB0->TSTIDXR = ((USB0->TSTIDXR & 0xff00) | 3);
            USB0->IEP.CSR.TXEPN |= USB_OTG_IEP_TXCSR_MODE;

            _cdc_flag   = STATE_DEVICE_CONN;
            break;
        case USB_EVENT_RX_AVAILABLE:
            _cdc_flag = STATE_DEVICE_READ;
            break;
        case USB_EVENT_DATA_REMAINING:
            break;
        case USB_EVENT_REQUEST_BUFFER:
            break;
        case USB_EVENT_TX_COMPLETE:
//            printf("IN cb 3\r\n");
            break;
        default:
            _cdc_flag = STATE_DEVICE_NO;
            break;
    }

    return 0;
}

#define UX_DEMO_STACK_SIZE      (1024 * 2)

#ifndef USBX_MEMORY_SIZE
#define USBX_MEMORY_SIZE         (1024 * 8)
#endif

UX_SLAVE_CLASS_CDC_ACM_PARAMETER parameter;
UX_SLAVE_CLASS_CDC_ACM *cdc;

static uint32_t usb_memory[USBX_MEMORY_SIZE / sizeof(uint32_t)];

#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED      (sizeof(device_framework_full_speed))
unsigned char device_framework_full_speed[] = {

    /* Device descriptor     18 bytes
       0x02 bDeviceClass:    CDC class code
       0x00 bDeviceSubclass: CDC class sub code
       0x00 bDeviceProtocol: CDC Device protocol

       idVendor & idProduct - http://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x10, 0x01, 0xEF, 0x02, 0x01, 0x40, 0xC9, 0x1F,
    0xB4, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,

    /* Configuration 1 descriptor 9 bytes */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0x40, 0x00,

    /* Interface association descriptor. 8 bytes.  */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement. 9 bytes.   */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor 4 bytes */
    0x04, 0x24, 0x02, 0x0f,

    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor 5 bytes */
    0x05, 0x24, 0x01, 0x03, 0x01, /* Data interface   */

    /* Endpoint 1 descriptor 7 bytes */
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x08,

    /* Data Class Interface Descriptor Requirement 9 bytes */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* First alternate setting Endpoint 1 descriptor 7 bytes*/
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,

    /* Endpoint 2 descriptor 7 bytes */
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
};

#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED      (sizeof(device_framework_high_speed))
unsigned char device_framework_high_speed[] = {

    /* Device descriptor
       0x02 bDeviceClass:    CDC class code
       0x00 bDeviceSubclass: CDC class sub code
       0x00 bDeviceProtocol: CDC Device protocol

       idVendor & idProduct - http://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x40, 0xC9, 0x1F,
    0xB4, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,

    /* Device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x01, 0x00,

    /* Configuration 1 descriptor */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0x40, 0x00,

    /* Interface association descriptor. */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x0f,

    /* Union Functional Descriptor */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x00, 0x01,

    /* Endpoint 1 descriptor */
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x08,

    /* Data Class Interface Descriptor Requirement */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* First alternate setting Endpoint 1 descriptor */
    0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00,

    /* Endpoint 2 descriptor */
    0x07, 0x05, 0x81, 0x02, 0x00, 0x02, 0x00,
};

#define STRING_FRAMEWORK_LENGTH             (sizeof(string_framework))
unsigned char string_framework[] = {
    /* Manufacturer string descriptor : Index 1 */
    0x09, 0x04, 0x01, 18U,
    'N', 'X', 'P', ' ',
    'S', 'E', 'M', 'I', 'C', 'O', 'N', 'D', 'U', 'C', 'T', 'O', 'R', 'S',

    /* Product string descriptor : Index 2 */
    0x09, 0x04, 0x02, 16U,
    'U', 'S', 'B', ' ',
    'C', 'D', 'C', ' ', 'A', 'C', 'M', ' ', 'D', 'E', 'M', 'O',

    /* Serial Number string descriptor : Index 3 */
    0x09, 0x04, 0x03, 0x04, 0x30, 0x30, 0x30, 0x31
};


/*
 * Multiple languages are supported on the device, to add
 * a language besides english, the unicode language code must
 * be appended to the language_id_framework array and the length
 * adjusted accordingly.
 */
#define LANGUAGE_ID_FRAMEWORK_LENGTH        (sizeof(language_id_framework))
unsigned char language_id_framework[] = {

    /* English. */
    0x09, 0x04
};

/**
 * @brief  USB0 device interrupt handler.
 * @retval None
 */
void __attribute__((interrupt)) USBOTG0_INT_IRQHandler(void)
{
    /* Handle the USB interrupt */
    usb0_device_int_handler(deviceHandle);
}

static VOID demo_cdc_instance_activate(VOID *cdc_instance)
{
    printf("CDC device activate\r\n");

    /* Save the CDC instance.  */
    cdc = (UX_SLAVE_CLASS_CDC_ACM *)cdc_instance;
}

static VOID demo_cdc_instance_deactivate(VOID *cdc_instance)
{
    /* Reset the CDC instance.  */
    cdc = UX_NULL;
}

void EnableIRQ(IRQn_t irq_num)
{
    uint32_t offset = ((uint32_t)irq_num >> 5);

    REG_WRITE(VIC->ISER[offset], (0x01 << ((uint32_t)irq_num & 0x1F)));
}

void tx_application_define(void)
{
    UINT status;

    /* Initialize USBX Memory */
    ux_system_initialize((VOID *)usb_memory, USBX_MEMORY_SIZE, UX_NULL, 0);

    /* The code below is required for installing the device portion of USBX. No call back for
       device status change in this example. */
    status = ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                        device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                        string_framework, STRING_FRAMEWORK_LENGTH, language_id_framework,
                                        LANGUAGE_ID_FRAMEWORK_LENGTH, UX_NULL);
    if (status != UX_SUCCESS)
        return;

    /* Set the parameters for callback when insertion/extraction of a CDC device.  */
    parameter.ux_slave_class_cdc_acm_instance_activate   = demo_cdc_instance_activate;
    parameter.ux_slave_class_cdc_acm_instance_deactivate = demo_cdc_instance_deactivate;

    /* Initialize the device CDC class. This class owns both interfaces starting with 0. */
    status = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry, 1, 0,
                                            &parameter);
    if (status != UX_SUCCESS)
        return;

    printf("Enable usb0 high speed\r\n");
    HAL_USB_HighSpeedEnable(USB0, true);
    // Enable the USB interrupt.
    printf("Enable the USB interrupt\r\n");
    EnableIRQ(OTG_HS0_INT_IRQ);

    //usb_device_setup();
    _ux_dcd_musb_initialize(kUSB_ControllerMUSB0, &deviceHandle, contrl_handle);

    if (status != UX_SUCCESS)
        return;

    return;
}


/**
 * @brief  Test main function
 * @retval Status.
 */
int usbx_device_cdc_vcom_test(void)
{
    printf("Start USBX device CDC ACM example...\r\n");
    int i;
    USB_OTG_t *USB = (USB_OTG_t *)USB0_ADDR_BASE;

    // sel PLL_MAIN
    HAL_RCM_SYSCLK_SWITCH(SYSTEM_CLOCK_SOURCE);
    // PHY clock
    HAL_RCM_HSE_Config(HSE_CLK_ON);
    i = 0;
    while (i == 0) {
        i = HAL_RCM_HSE_IsReady();
    }
//    printf("crm_sysclk_HSE, hse ready==%d...\n", i);

    // Reset and Clock, clock enable first then release reset
    HAL_RCM_ClkEnable(PERIPHERAL_USBOTG0);
    HAL_RCM_Reset(PERIPHERAL_USBOTG0);

    tx_application_define();

    while (1) {
        switch (_cdc_flag) {
        case STATE_DEVICE_CONN:
            printf("\rCDC Ready!\n\r");
            _cdc_flag = STATE_DEVICE_READY;
            break;
        case STATE_DEVICE_READ:
            actual_length = HAL_USB_EpDataAvail(USB, 2);
            printf("\r\nRecv size 0x%lx\n\r", actual_length);
            HAL_USB_EpDataGet(USB, 2, buffer, &actual_length);
            HAL_USB_DevEpDataAck(USB, 2, 1);
            for (i = 0; i < actual_length; ++i)
                printf("0x%x ", buffer[i]);

            printf("\r\nSend data to host\r\n");
            buffer[0] = 'a'; buffer[1] = 'b'; buffer[2] = 'c'; buffer[3] = 'd';
            buffer[4] = 'e'; buffer[5] = 'f'; buffer[6] = 'g'; buffer[7] = 'h';

            HAL_USB_EpDataPut(USB, 1, buffer, 8);
            HAL_USB_EpDataSend(USB, 1, USB_TRANS_IN);
            _cdc_flag = STATE_DEVICE_READY;
            break;

        case STATE_DEVICE_READY:
        case STATE_DEVICE_NO:
        case STATE_POWER_FAULT:
            break;
        default:
            break;
        }
    }

    return 0;
}
