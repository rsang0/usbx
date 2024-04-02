/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb_device_dci.h"
#include "usb_device.h"
#include "usb_device_musb.h"
#include "usb.h"
#include "usb_misc.h"
#include "mp32gm51_hal_usb.h"

/* define the reserved buffer for endpoint max packet copy */
#define SETUP_TRANSFER_DATA_SIZE     (64U) /* The actual size is 8. Here use 64 aligned to 64-byte boundaries. */
#define CONTROL_TRANSFER_DATA_SIZE   (64U) /* The maximum size in the control data stage. */
#define ZERO_TRANSFER_DATA_SIZE      (64U) /* The actual size is 4. Here use 64 aligned to 64-byte boundaries. */
#define SETUP_TRANSFER_DATA_OFFSET   (0U)

static volatile uint8_t usb_ep0_state = KUSB_EP0_STATE_STATUS;

static usb_device_musbip_state_struct_t
    s_UsbDeviceMUSBIpState[kUSB_ControllerMUSB0 + kUSB_ControllerMUSB1];

__attribute__((aligned(4))) static uint8_t setupBuf[SETUP_TRANSFER_DATA_SIZE];

static void usbd_ep0_tx(usb_device_controller_handle controllerHandle, uint32_t endpointAddress);
static void usbd_ep0_rx(usb_device_controller_handle controllerHandle, uint32_t endpointAddress);

#define USB_MUSBIP_ENDPOINT_DES_INDEX(endpointAddress) \
    (((((endpointAddress)) & 0x0FU) << 1) +               \
     ((0U != ((endpointAddress)&USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK)) ? (1U) : (0U)))

static usb_device_musbip_endpoint_state_struct_t *USB_DeviceMUSBIpGetEndpointStateStruct(
    usb_device_musbip_state_struct_t *musbIpState, uint8_t endpointAddress)
{
    uint8_t endpointIndex = USB_MUSBIP_ENDPOINT_DES_INDEX(endpointAddress);
    if (endpointIndex <= ((uint32_t)USB_DEVICE_MUSB_ENDPOINTS_NUM * 2))
    {
        return &(musbIpState->endpointState[endpointIndex]);
    }

    return NULL;
}
/***********************************************************************
 *
 **********************************************************************/

/* Config and Enable endpoint */
static usb_status_t USB_DeviceMUSBIpEndpointInit(usb_device_musbip_state_struct_t *musbIpState,
                                                    usb_device_endpoint_init_struct_t *epInit)
{
    uint8_t endpointNumber = epInit->endpointAddress & 0x0f;
    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, epInit->endpointAddress);
    uint16_t maxPacketSize = epInit->maxPacketSize;

    /* clear the endpoint status bits */
    epState->stateUnion.state = 0x00000000U;

    /* Enable EP interrupt */
    musbIpState->registerBase->TXIER |= (1 << endpointNumber);
    if (endpointNumber !=0 )
    {
        musbIpState->registerBase->RXIER |= (1 << endpointNumber);
    }

    /* Save the max packet size of the endpoint */
    epState->stateUnion.stateBitField.maxPacketSize = maxPacketSize;
    /* Set the ZLT field */
    epState->stateUnion.stateBitField.zlt          = epInit->zlt;
    epState->stateUnion.stateBitField.endpointType = epInit->transferType;

    /* get the endpoint default control value */
    epState->stateUnion.stateBitField.epControlDefault = 0x00U;

    epState->transferBuffer         = NULL;
    epState->transferLength         = 0;
    epState->remainLength           = 0;
    epState->transferDone           = 0;
    epState->CurrentTransferLength  = 0;
    epState->epPacketBuffer         = NULL;
    epState->stateUnion.stateBitField.transferring  = 0;
    epState->epPacketBuffer = NULL;
    return kStatus_USB_Success;
}

/*!
 * @brief De-initialize a specified endpoint.
 *
 * The function is used to de-initialize a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be disabled.
 *
 * @param musbIpState      Pointer of the controller state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMUSBIpEndpointDeinit(usb_device_musbip_state_struct_t *musbIpState, uint8_t endpointAddress)
{
    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    /* Cancel the transfer of the endpoint */
    // (void)USB_DeviceMUSBIpCancel(musbIpState, ep);
    epState->transferBuffer         = NULL;
    epState->transferLength         = 0;
    epState->remainLength           = 0;
    epState->transferDone           = 0;
    epState->CurrentTransferLength  = 0;
    epState->epPacketBuffer         = NULL;
    epState->stateUnion.stateBitField.transferring  = 0;

    /* Clear the max packet size */
    epState->stateUnion.stateBitField.maxPacketSize = 0U;

    return kStatus_USB_Success;
}

usb_status_t USB_DeviceMUSBIpCancel(usb_device_controller_handle controllerHandle, uint8_t endpointAddress)
{
    usb_device_musbip_state_struct_t *musbIpState = (usb_device_musbip_state_struct_t *)controllerHandle;
    usb_device_callback_message_struct_t message;
    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    epState->stateUnion.stateBitField.transferring = 0U;
    message.length  = USB_CANCELLED_TRANSFER_LENGTH;
    message.buffer  = epState->transferBuffer;
    message.code    = endpointAddress & 0x0f;
    message.isSetup = 0U;
    (void)USB_DeviceNotificationTrigger(musbIpState->deviceHandle, &message);

    return kStatus_USB_Success;
}

/*!
 * @brief Un-stall a specified endpoint.
 *
 * The function is used to un-stall a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be un-stalled.
 *
 * @param musbIpState      Pointer of the controller state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMUSBIpEndpointUnstall(usb_device_musbip_state_struct_t *musbIpState, uint8_t endpointAddress)
{
    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    /* Clear the endpoint stall state, the hardware resets the endpoint
     * toggle to one for both directions when a setup token is received */
    epState->stateUnion.stateBitField.stalled = 0U;

    (void)USB_DeviceMUSBIpCancel(musbIpState, endpointAddress);

    return kStatus_USB_Success;
}

/*!
 * @brief Stall a specified endpoint.
 *
 * The function is used to stall a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be stalled.
 *
 * @param musbIpState      Pointer of the controller state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceMUSBIpEndpointStall(usb_device_musbip_state_struct_t *musbIpState, uint8_t endpointAddress)
{
    usb_device_musbip_endpoint_state_struct_t *epState;

    /* stall the endpoint */
    if ((endpointAddress & USB_ENDPOINT_NUMBER_MASK) == USB_CONTROL_ENDPOINT)
    {
        epState                                   = USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, 0);
        epState->stateUnion.stateBitField.stalled = 1U;
        epState                                   = USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, 1);
        epState->stateUnion.stateBitField.stalled = 1U;

        /* cancel the transfer in the endpoint */
        (void)USB_DeviceMUSBIpCancel(musbIpState, 0);
        /* cancel the transfer in the endpoint */
        (void)USB_DeviceMUSBIpCancel(musbIpState, 0x80);
    }
    else
    {
        epState = USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);
        /* Set endpoint stall flag. */
        epState->stateUnion.stateBitField.stalled = 1U;
    }
    /* cancel the transfer in the endpoint */
    (void)USB_DeviceMUSBIpCancel(musbIpState, endpointAddress);
    return kStatus_USB_Success;
}

/*!
 * @brief Handle the USB bus reset interrupt.
 *
 * The function is used to handle the USB bus reset interrupt.
 *
 * @param musbIpState       Pointer of the controller state structure.
 *
 */
static void USB_DeviceMusbIpInterruptReset(usb_device_musbip_state_struct_t *musbIpState)
{
    usb_device_callback_message_struct_t message;

    /* Set reset flag */
    //usb_ep0_state = KUSB_EP0_STATE_STATUS;
    musbIpState->isResetting = 1U;
    musbIpState->deviceSpeed = USB_SPEED_HIGH;

    message.buffer  = (uint8_t *)NULL;
    message.code    = (uint8_t)kUSB_DeviceNotifyBusReset;
    message.length  = 0U;
    message.isSetup = 0U;
    /* Notify up layer the USB bus reset signal detected. */
    (void)USB_DeviceNotificationTrigger(musbIpState->deviceHandle, &message);
}

/*!
 * @brief Initializes the USB device controller instance.
 *
 * This function initializes the USB device controller module specified by the controllerId.
 *
 * @param[in] controllerId      The controller ID of the USB IP. See the enumeration type usb_controller_index_t.
 * @param[in] handle            Pointer of the device handle used to identify the device object belongs to.
 * @param[out] controllerHandle An out parameter used to return the pointer of the device controller handle to the
 * caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMUSBInit(uint8_t controllerId,
                                usb_device_handle handle,
                                usb_device_controller_handle *controllerHandle)
{
    usb_device_musbip_state_struct_t *musbIpState = NULL;
    uint32_t musbBases[] = {USB0_ADDR_BASE, USB1_ADDR_BASE};

    musbIpState = &s_UsbDeviceMUSBIpState[controllerId - (uint8_t)kUSB_ControllerMUSB0];
    
    /* get the ip base address */
    musbIpState->registerBase = (USB_OTG_t *)musbBases[controllerId - (uint8_t)kUSB_ControllerMUSB0];
    
    musbIpState->controllerId = controllerId;
    musbIpState->setupData    = setupBuf;
    
    /* reset registers */
    /* Clear USB interrupt status */
    musbIpState->registerBase->USBISER;
    /* Clear USB TX EP interrupt status */
    musbIpState->registerBase->TXISR;
    /* Clear USB RX EP interrupt status */
    musbIpState->registerBase->RXISR;

    /*Enable reset, connect interrupt*/
    musbIpState->registerBase->USBISER = USB_OTG_USBISER_IE_RESET | \
                                         USB_OTG_USBISER_IE_CONN;
    /*Enable all of EP TX interrupt*/
    musbIpState->registerBase->TXIER   = 0xF;
    /*Enable all of EP RX interrupt*/
    musbIpState->registerBase->RXIER   = 0xe;
    
    /* Clear device address. */
    musbIpState->registerBase->PMFAR = 0x0;

    /* Enable USB high speed mode. */
    musbIpState->registerBase->PMFAR |= USB_OTG_PMFAR_HSEN;
    
    musbIpState->deviceHandle = handle;

    musbIpState->ep0_state = KUSB_EP0_STATE_STATUS;

    *controllerHandle         = musbIpState;
    HAL_USB_OtgSessionRequest(musbIpState->registerBase, true);

    return kStatus_USB_Success;
}

/*!
 * @brief Deinitializes the USB device controller instance.
 *
 * This function deinitializes the USB device controller module.
 *
 * @param[in] controllerHandle   Pointer of the device controller handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMUSBDeinit(usb_device_controller_handle controllerHandle)
{
    return kStatus_USB_Success;
}

static void usbd_ep0_tx(usb_device_controller_handle controllerHandle, uint32_t endpointAddress)
{
    uint8_t *buf;
    uint32_t len;

    usb_device_musbip_state_struct_t *musbIpState = 
                    (usb_device_musbip_state_struct_t *)controllerHandle;

    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    USB_OTG_t *USB = musbIpState->registerBase;

    /* Set EP0 state as TX */
    musbIpState->ep0_state = KUSB_EP0_STATE_TX;

    len = epState->remainLength;

    /* Adjust the length */
    len = len > EP0_MAX_PACKET_SIZE ? EP0_MAX_PACKET_SIZE : len;
    /* Create a pointer to the data */
    buf = (uint8_t *)epState->transferBuffer + epState->transferDone;

    /* Put the data into the FIFO */
    HAL_USB_EpDataPut(USB, USB_EP_0, buf, len);

    /* Check the length */
    if (len == EP0_MAX_PACKET_SIZE) {
        /* Send the data */
        HAL_USB_EpDataSend(USB, USB_EP_0, USB_TRANS_IN);
    } else {
        /* Set the EP0 state as STATUS */
        musbIpState->ep0_state = KUSB_EP0_STATE_STATUS;
        epState->stateUnion.stateBitField.transferring = 0;
        /* Send last packet */
        HAL_USB_EpDataSend(USB, USB_EP_0, USB_TRANS_IN_LAST);
    }

    /* Update the data pointer and position */
    epState->remainLength -= len;
    epState->transferDone += len;
}

static void usbd_ep0_rx(usb_device_controller_handle controllerHandle, uint32_t endpointAddress)
{
    uint32_t len;
    uint8_t *buffer;
    usb_device_callback_message_struct_t message;

    usb_device_musbip_state_struct_t *musbIpState = 
                    (usb_device_musbip_state_struct_t *)controllerHandle;

    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    USB_OTG_t *USB = musbIpState->registerBase;

    musbIpState->ep0_state = KUSB_EP0_STATE_RX;

    buffer = epState->transferBuffer + epState->transferDone;

    /* Gets the size of the available data. */
    len = HAL_USB_EpDataAvail(USB, USB_EP_0);

    /* Get the data from EP0 */
    HAL_USB_EpDataGet(USB, USB_EP_0, buffer, &len);

    epState->remainLength -= len;

    if (epState->remainLength == 0)
    {
        /* ACK the data on EP0, this is the last packet */
        HAL_USB_DevEpDataAck(USB, USB_EP_0, 1);
        epState->transferDone += len;
        musbIpState->ep0_state = KUSB_EP0_STATE_STATUS;
        epState->stateUnion.stateBitField.transferring = 0;

        message.length = epState->transferDone;
        message.buffer = (musbIpState->setupData);
        message.isSetup = 0;
        message.code    = ((uint8_t)USB_EP_0 | (uint8_t)(0x00U << 0x07U));
        (void)USB_DeviceNotificationTrigger(musbIpState->deviceHandle, &message);
    }
    else
    {
        /* ACK the data on EP0, more data is coming */
        HAL_USB_DevEpDataAck(USB, USB_EP_0, 0);
        epState->transferDone += len;
    }
}

/*!
 * @brief Sends data through a specified endpoint.
 *
 * This function sends data through a specified endpoint.
 *
 * @param[in] controllerHandle Pointer of the device controller handle.
 * @param[in] endpointAddress  Endpoint index.
 * @param[in] buffer           The memory address to hold the data need to be sent.
 * @param[in] length           The data length need to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value indicates whether the sending request is successful or not. The transfer completion is
 * notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for a specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is obtained through the
 * endpoint
 * callback).
 */
usb_status_t USB_DeviceMUSBSend(usb_device_controller_handle controllerHandle,
                                uint8_t endpointAddress,
                                uint8_t *buffer,
                                uint32_t length)
{
    usb_status_t status = kStatus_USB_Success;
//    uint32_t endpointNumber = endpointAddress & 0x0f;

    usb_device_musbip_state_struct_t *musbIpState = 
                    (usb_device_musbip_state_struct_t *)controllerHandle;

    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    if ((length == 0) || (buffer == NULL))
    {
        return 0;
    }

    if (0U == epState->stateUnion.stateBitField.transferring)
    {
        /* Save the transfer information */
        epState->transferBuffer       = buffer;
        epState->transferLength       = length;
        epState->remainLength         = length;
        epState->transferDone         = 0U;
        epState->CurrentTransferLength       = 0U;
        epState->stateUnion.stateBitField.transferring = 1;
    }

    usbd_ep0_tx(controllerHandle, endpointAddress);
    return status;
}

/*!
 * @brief Receives data through a specified endpoint.
 *
 * This function receives data through a specified endpoint.
 *
 * @param[in] controllerHandle Pointer of the device controller handle.
 * @param[in] endpointAddress  Endpoint index.
 * @param[in] buffer           The memory address to save the received data.
 * @param[in] length           The data length to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value indicates whether the receiving request is successful or not. The transfer completion is
 * notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for a specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is obtained through the
 * endpoint
 * callback).
 */
usb_status_t USB_DeviceMUSBRecv(usb_device_controller_handle controllerHandle,
                                uint8_t endpointAddress,
                                uint8_t *buffer,
                                uint32_t length)
{
    int status = kStatus_USB_Success;

    usb_device_musbip_state_struct_t *musbIpState = 
                    (usb_device_musbip_state_struct_t *)controllerHandle;

    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    if ((length == 0) || (buffer == NULL))
    {
        return 0;
    }

    if (0U == epState->stateUnion.stateBitField.transferring)
    {
        /* Save the transfer information */
        epState->transferLength           = length;
        epState->transferBuffer           = buffer;
        epState->remainLength             = length;
        epState->transferDone             = 0U;
        epState->CurrentTransferLength    = 0U;
        epState->stateUnion.stateBitField.transferring = 1;
        musbIpState->ep0_state = KUSB_EP0_STATE_RX;
    }

//    usbd_ep0_rx(controllerHandle, endpointAddress);
    return status;
}

/*!
 * @brief Cancels the pending transfer in a specified endpoint.
 *
 * The function is used to cancel the pending transfer in a specified endpoint.
 *
 * @param[in] controllerHandle  ointer of the device controller handle.
 * @param[in] ep                Endpoint address, bit7 is the direction of endpoint, 1U - IN, abd 0U - OUT.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMUSBCancel(usb_device_controller_handle controllerHandle, uint8_t ep)
{
    return kStatus_USB_Success;
}

/*!
 * @brief Controls the status of the selected item.
 *
 * The function is used to control the status of the selected item.
 *
 * @param[in] controllerHandle      Pointer of the device controller handle.
 * @param[in] type             The selected item. Please refer to enumeration type usb_device_control_type_t.
 * @param[in,out] param            The parameter type is determined by the selected item.
#include <es32f36xx.h>
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMUSBControl(usb_device_controller_handle controllerHandle,
                                   usb_device_control_type_t type,
                                   void *param)
{
    usb_device_musbip_state_struct_t *musbIpState = (usb_device_musbip_state_struct_t *)controllerHandle;
    usb_status_t error                            = kStatus_USB_Error;
    uint32_t tmp32Value;
    uint8_t tmp8Value;

    usb_device_musbip_endpoint_state_struct_t *epState;
    USB_OTG_t *USB = musbIpState->registerBase;

    if (controllerHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    switch (type)
    {
        case kUSB_DeviceControlRun:
            /* Get the current interrupt status and clear all interrupts */
            HAL_USB_IntStatusGet(USB);
            HAL_USB_IntStatusEpGet(USB);
            musbIpState->registerBase->PMFAR |= (USB_OTG_PMFAR_SFTCNN);
            HAL_USB_IntEnable(USB, USB_OTG_USBISER_IE_RESET_MSK | USB_OTG_USBISER_IE_CONN_MSK | USB_OTG_USBISER_IE_RESUME_MSK | \
                              USB_OTG_USBISER_IE_SUSPEND_MSK | USB_OTG_USBISER_IE_SOF_MSK);
            /* Enable all andpoint interrupt */
            HAL_USB_IntEnableEp(USB, USB_INTEP_ALL);
            /* Enable software connect */
            HAL_USB_DevConnect(USB);
            break;

        case kUSB_DeviceControlStop:
            musbIpState->registerBase->PMFAR &= (~USB_OTG_PMFAR_SFTCNN);
            break;

        case kUSB_DeviceControlEndpointInit:
            if (NULL != param)
            {
                error = USB_DeviceMUSBIpEndpointInit(musbIpState, (usb_device_endpoint_init_struct_t *)param);
            }
            break;

        case kUSB_DeviceControlEndpointDeinit:
            if (NULL != param)
            {
                tmp8Value = *((uint8_t *)param);
                error     = USB_DeviceMUSBIpEndpointDeinit(musbIpState, tmp8Value);
            }
            break;

        case kUSB_DeviceControlEndpointStall:
            if (NULL != param)
            {
                tmp8Value = *((uint8_t *)param);
                error     = USB_DeviceMUSBIpEndpointStall(musbIpState, tmp8Value);
            }
            break;

        case kUSB_DeviceControlEndpointUnstall:
            if (NULL != param)
            {
                tmp8Value = *((uint8_t *)param);
                error     = USB_DeviceMUSBIpEndpointUnstall(musbIpState, tmp8Value);
            }
            break;

        case kUSB_DeviceControlGetDeviceStatus:
            if (NULL != param)
            {
                *((uint16_t *)param) =
                    (USB_DEVICE_CONFIG_SELF_POWER << (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT));
                error = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceControlGetEndpointStatus:
            if (NULL != param)
            {
                usb_device_endpoint_status_struct_t *endpointStatus = (usb_device_endpoint_status_struct_t *)param;

                if ((((endpointStatus->endpointAddress) & USB_ENDPOINT_NUMBER_MASK)) <
                    (uint8_t)USB_DEVICE_MUSB_ENDPOINTS_NUM)
                {
                    epState = USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointStatus->endpointAddress);
                    endpointStatus->endpointStatus =
                        (uint16_t)((epState->stateUnion.stateBitField.stalled == 1U) ? kUSB_DeviceEndpointStateStalled :
                                                                                       kUSB_DeviceEndpointStateIdle);
                    error = kStatus_USB_Success;
                }
            }
            break;

        case kUSB_DeviceControlPreSetDeviceAddress:
            //usb_echo("\r\n Set device address \r\n");
            tmp8Value  = *((uint8_t *)param);
            musbIpState->registerBase->PMFAR = (musbIpState->registerBase->PMFAR & (~USB_OTG_PMFAR_FADD_MSK)) | \
                                                (tmp8Value & USB_OTG_PMFAR_FADD_MSK);
            //ald_usb_dev_ep_data_ack(0, false);
            break;

        case kUSB_DeviceControlSetDeviceAddress:
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceControlGetSynchFrame:
            break;

        case kUSB_DeviceControlSetDefaultStatus:
            for (tmp32Value = 0U; tmp32Value < (uint32_t)USB_DEVICE_MUSB_ENDPOINTS_NUM; tmp32Value++)
            {
                (void)USB_DeviceMUSBIpEndpointDeinit(musbIpState, (uint8_t)(tmp32Value | (USB_IN << 0x07U)));
                (void)USB_DeviceMUSBIpEndpointDeinit(musbIpState, (uint8_t)(tmp32Value | (USB_OUT << 0x07U)));
            }
            //USB_DeviceMUSBIpSetDefaultState(musbIpState);
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceControlGetSpeed:
            if (NULL != param)
            {
                *((uint8_t *)param) = musbIpState->deviceSpeed;
                error               = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceControlGetOtgStatus:
            break;
        case kUSB_DeviceControlSetOtgStatus:
            break;
        default:
            /*no action*/
            break;
    }

    return error;
    
    return kStatus_USB_Success;
}

void usbd_ep0_handle(usb_device_musbip_state_struct_t *musbIpState)
{
//   printf("usbd_ep0_handle\r\n");
    uint32_t status, len;
    usb_device_callback_message_struct_t message;
    usb_setup_struct_t *setup;
    USB_OTG_t *USB = musbIpState->registerBase;

    /* Get the EP0 status */
    status = HAL_USB_EpStatus(USB, USB_EP_0);

#if 0
    if (status & USB_OTG_IEP_CSR0_RXSTALL)
    {
        USB->IEP.CSR.EP0 &= ~USB_OTG_IEP_CSR0_RXSTALL;
        usb_ep0_state = KUSB_EP0_STATE_STATUS;
        return;
    }

    if (status & USB_OTG_IEP_CSR0_STPEND)
    {
        USB->IEP.CSR.EP0 |= USB_OTG_IEP_CSR0_CTXRDY;
    }
#endif
    //usb_echo("\r\nEP0 CSR status = 0x%x:", status);
    //usb_echo("\r\nusb_ep0_state = 0x%x:", usb_ep0_state);
    switch (musbIpState->ep0_state)
    {
        case KUSB_EP0_STATE_STATUS:
            if (status & USB_OTG_IEP_RXCSR_RXRDY)
            {
                ////usb_echo("\r\nReceive setup request:");
                ////usb_echo("\r\nstatus = 0x%x  \r\n", status);
                len = HAL_USB_EpDataAvail(USB, USB_EP_0);
                if (len != 8)
                {
                    return;
                }

                message.length = 8U;
                message.buffer = (musbIpState->setupData);
                message.isSetup = 1;
                message.code    = ((uint8_t)USB_EP_0 | (uint8_t)(0x01U << 0x07U));
                setup = (usb_setup_struct_t *)message.buffer;
                HAL_USB_EpDataGet(USB, USB_EP_0, (uint8_t *)setup, (uint32_t *)&len);
                if (setup->wLength) {
                    HAL_USB_DevEpDataAck(USB, USB_EP_0, 0);
                } else {
                    HAL_USB_DevEpDataAck(USB, USB_EP_0, 1);
                }
                (void)USB_DeviceNotificationTrigger(musbIpState->deviceHandle, &message);
            }
            break;

        case KUSB_EP0_STATE_TX:
            usbd_ep0_tx(musbIpState, ((uint8_t)USB_EP_0 | USB_REQUEST_TYPE_DIR_IN));
            break;

        case KUSB_EP0_STATE_RX:
            usbd_ep0_rx(musbIpState, ((uint8_t)USB_EP_0 | USB_REQUEST_TYPE_DIR_OUT));
            break;

        case KUSB_EP0_STATE_STALL:    /* Handle the STALL state */
            /* If we send a stall signal, then entry this interrupt */
            if (status & USB_OTG_IEP_CSR0_TXSTALL_MSK) {
                /* Clear the EP0 status */
                HAL_USB_DevEpStatusClear(USB, USB_EP_0, USB_OTG_IEP_CSR0_TXSTALL_MSK);
            }
            musbIpState->ep0_state = KUSB_EP0_STATE_STATUS;
            break;
        default:
            break;
    }
}

static void USB_DeviceEndpointHandler(usb_device_musbip_state_struct_t *musbIpState,
                                      uint8_t endpointAddress)
{
    uint32_t ep_status;
    usb_device_callback_message_struct_t message;
    uint32_t length;
    //usb_setup_struct_t *setupPacket;
    uint8_t epNum;
    uint8_t direction;
    USB_OTG_t *USB = musbIpState->registerBase;

    usb_device_musbip_endpoint_state_struct_t *epState =
        USB_DeviceMUSBIpGetEndpointStateStruct(musbIpState, endpointAddress);

    epNum = endpointAddress & 0x0f;
    direction = (endpointAddress & 0x80) >> 0x7;
    ep_status = HAL_USB_EpStatus(USB, epNum);

    /* USB_IN, Send completed */
    if (direction == USB_IN)
    {
//        printf("\r\nIN 1\r\n");
        message.isSetup = 0;
        message.code    = (epNum) | (uint8_t)(USB_IN << 0x07U);
//        printf("IN ep_status = 0x%x\r\n", ep_status);

        if (ep_status & 0x04) {
            //HAL_USB_EpDataSend(USB, epNum, USB_TRANS_IN);
            //HAL_USB_FifoFlush(USB, epNum, uint32_t flags)
            USB->TSTIDXR = ((USB->TSTIDXR & 0xff00) | epNum);
            USB->IEP.CSR.TXEPN |= USB_OTG_IEP_TXCSR_FLSFIFO_MSK; //USB_OTG_IEP_TXCSR_CLRDAT
        }

        HAL_USB_DevEpStatusClear(USB, epNum, ep_status);
        (void)USB_DeviceNotificationTrigger(musbIpState->deviceHandle, &message);
    } else {
//        printf("OUT 1\r\n");
        message.isSetup = 0;
        message.code    = (epNum) | (uint8_t)(USB_OUT << 0x07U);

        // USB->TSTIDXR = ((USB->TSTIDXR & 0xff00) | epNum);
        // USB->IEP.RXCSR |= USB_OTG_IEP_RXCSR_DATTGGL_MSK;
        /* Clear the endpoint status bits */
        HAL_USB_DevEpStatusClear(USB, epNum, ep_status);
        /* Has a packet been received */
        if ((ep_status >> 16) & USB_OTG_IEP_RXCSR_RXRDY)
        {
//            printf("OUT 2\r\n");
            length = HAL_USB_EpDataAvail(USB, epNum);
            message.buffer  = epState->epPacketBuffer;
            message.length  = length;
            (void)USB_DeviceNotificationTrigger(musbIpState->deviceHandle, &message);
        }
        else
        {
            /* No packet was received */
//            if (ep_status & MUSB_RX_ERROR_FLAGS)
            {

            }
        }
    }
}

void usb0_device_int_handler(void *deviceHandle)
{
//    printf("\r\nusb0 isr\r\n");
    usb_device_struct_t *handle = (usb_device_struct_t *)deviceHandle;
    usb_device_musbip_state_struct_t *musbIpState;
    uint32_t status;
    uint32_t index;
    uint32_t epNum;

    if (NULL == deviceHandle)
    {
        return;
    }

    musbIpState = (usb_device_musbip_state_struct_t *)(handle->controllerHandle);
    
    USB_OTG_t *USB = musbIpState->registerBase;

    /* Get the USB interrupt status */
    status = HAL_USB_IntStatusGet(USB);
//    printf("Usb isr status = 0x%x:\r\n", status);
    /* Receive a reset signal from the USB bus */
    if (status & USB_OTG_USBISER_IS_RESET)
    {
//        printf("Usb reset:\r\n");
        //usb_echo("\r\nUsb reset: \r\n");
        USB_DeviceMusbIpInterruptReset(musbIpState);
    }
    
    /* Get the endpoint interrupt status */
    status = HAL_USB_IntStatusEpGet(USB);
    
    if (status & USB_INTEP_0) /* EP0 interrupt */
    {
//        printf("Usb EP0 interrupt:\r\n");
        /* Handle EP0 interrupt */
        //USB0->INDEX = 0;
        usbd_ep0_handle(musbIpState);
        status &= ~USB_INTEP_0;
    }

    if (status != 0)
    {
        /* check the endpoint interrupt */
        for (index = 1; index < 11; ++index)
        {
            if (status & (0x01UL << index))
            {
                epNum = index;
                printf("Usb EP%d IN interrupt:\r\n", epNum);
                USB_DeviceEndpointHandler(musbIpState, (epNum | (USB_IN << 0x7)));
            }
        }

        for (index = 17; (index < 22); ++index)
        {
            if (status & (0x01UL << index))
            {
                epNum = index - 16;
                printf("Usb EP%d OUT interrupt:\r\n", epNum);
                USB_DeviceEndpointHandler(musbIpState, (epNum | (USB_OUT << 0x7)));
            }
        }
    }
}


void usb0_dma_int_handler(void *deviceHandle)
{
    
}

