/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_MUSB_H__
#define __USB_DEVICE_MUSB_H__

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_dci.h"
#include "mp32gm51fxx.h"
#include "dbg_log.h"

/*!
 * @addtogroup usb_device_controller_musb_driver
 * @{
 */
/* USB module features */

/* @brief Size of the USB dedicated RAM */
// #define FSL_FEATURE_USB_USB_RAM (0x00004000)
/* @brief Base address of the USB dedicated RAM */
// #define FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS (0x40100000)
/* @brief USB version */
#define FSL_FEATURE_USB_VERSION (200)
/* @brief Number of the endpoint in USB FS */
#define FSL_FEATURE_USB_EP_NUM (11)

/**
  * @defgroup Device_Core_Public_Macros Public Macros
  * @{
  */
#define USB_EP_IN			0U
#define USB_EP_OUT			1U
#define USB_MAX_INTERFACES_PER_DEVICE	8U
#define EP0_MAX_PACKET_SIZE		64U
#define DEV_ADDR_PENDING		0x80000000U
#define DEFAULT_CONFIG_ID		1U
#define REMOTE_WAKEUP_PULSE_MS		10U
#define REMOTE_WAKEUP_READY_MS		20U
#define USBLIB_LPM_STATE_DISABLED	0x0U
#define USBLIB_LPM_STATE_AWAKE		0x1U
#define USBLIB_LPM_STATE_SLEEP		0x2U

/**
  * @defgroup Device_Core_Public_Types Public Types
  * @{
  */

#define USB_IRQS {USB_INT_IRQn}
#define USB_DMA_IRQS {USB_DMA_IRQn}


/**
  * @brief Endpoint 0 status
  */
typedef enum {
    KUSB_EP0_STATE_IDLE      = 0x0,    /**< Idle status */
    KUSB_EP0_STATE_TX        = 0x1,    /**< Send data status */
    KUSB_EP0_STATE_TX_CONFIG = 0x2,    /**< Send configure descriptor status*/
    KUSB_EP0_STATE_RX        = 0x3,    /**< Receive status */
    KUSB_EP0_STATE_STATUS    = 0x4,    /**< Before idle status */
    KUSB_EP0_STATE_STALL     = 0x5,    /**< Stall status */
} usb_ep0_state_t;

#define MUSB_RX_ERROR_FLAGS      ((USB_OTG_IEP_RXCSR_DATERR << 16) | (USB_OTG_IEP_RXCSR_OVRRN << 16) |\
                                 (USB_OTG_IEP_RXCSR_FIFOFL << 16))

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Prime all the double endpoint buffer at the same time, if the transfer length is larger than max packet size.
 */
#define USB_DEVICE_MUSB_DOUBLE_BUFFER_ENABLE (0U)

#define USB_MUSB_Type              USB_OTG_t
#define USB_DEVICE_MUSB_ENDPOINTS_NUM FSL_FEATURE_USB_EP_NUM

/* if FSL_FEATURE_USBHSD_HAS_EXIT_HS_ISSUE is true:
 * Enable this macro to exit HS mode automatically if the user case is:
 *   host and device keep cable connected, and host turn off vbus to simulate detachment.
 * If user disconnects the cable, there is no issue and don't need enable this macro.
 * There is one delay in the isr if enable this macro.
 */
#define USB_DEVICE_MUSBHS_FORCE_EXIT_HS_MODE_ENABLE (0u)

typedef enum {
    USB_EP0_STATE_SETUP = 0x0,      /**< SETUP DATA */
    USB_EP0_STATE_IN_DATA = 0x1,    /**< IN DATA */
    USB_EP0_STATE_OUT_DATA = 0x3,   /**< OUT DATA */
    USB_EP0_STATE_IN_STATUS = 0x4,  /**< IN status */
    USB_EP0_STATE_OUT_STATUS = 0x5, /**< OUT status */
    USB_EP0_STATE_IN_ZLP = 0x6,     /**< OUT status */
    USB_EP0_STATE_STALL = 0x7,      /**< STALL status */
} ep0_state_t;

/*! @brief Endpoint state structure */
typedef struct _usb_device_musb_endpoint_state_struct
{
    uint8_t *transferBuffer;       /*!< Address of buffer containing the data to be transmitted */
    uint32_t transferLength;       /*!< Length of data to transmit. */
    uint32_t transferDone;         /*!< The data length has been transferred*/
	uint32_t remainLength;		   /*!< The data length remain */
    uint32_t CurrentTransferLength;/*!< Record the current transmission length*/
    uint32_t requestTransferActualLebgth;
    uint8_t *epPacketBuffer;       /*!< The max packet buffer for copying*/
    union
    {
        uint32_t state; /*!< The state of the endpoint */
        struct
        {
            uint32_t maxPacketSize : 12U; /*!< The maximum packet size of the endpoint */
            uint32_t stalled : 1U;        /*!< The endpoint is stalled or not */
            uint32_t transferring : 3U;   /*!< The endpoint is transferring */
            uint32_t zlt : 1U;            /*!< zlt flag */
            uint32_t stallPrimed : 1U;
            uint32_t epPacketCopyed : 1U;   /*!< whether use the copy buffer */
            uint32_t epControlDefault : 3u; /*!< The EP command/status 26~30 bits */
            uint32_t doubleBufferBusy : 2U; /*!< How many buffers are primed, for control endpoint it is not used */
            uint32_t producerOdd : 1U;      /*!< When priming one transaction, prime to this endpoint buffer */
            uint32_t consumerOdd : 1U;      /*!< When transaction is done, read result from this endpoint buffer */
            uint32_t endpointType : 2U;
#if (defined(USB_DEVICE_CONFIG_ROOT2_TEST) && (USB_DEVICE_CONFIG_ROOT2_TEST > 0U))
            uint32_t isOpened : 1U; /*!< whether the endpoint is initialized */
            uint32_t reserved1 : 3U;
#else
            uint32_t reserved1 : 4U;
#endif
        } stateBitField;
    } stateUnion;
    union
    {
        uint16_t epBufferStatus;
        /* If double buff is disable, only epBufferStatusUnion[0] is used;
           For control endpoint, only epBufferStatusUnion[0] is used. */
        struct
        {
            uint16_t transactionLength : 15U;
            uint16_t epPacketCopyed : 1U;
        } epBufferStatusField;
    } epBufferStatusUnion[2];
} usb_device_musbip_endpoint_state_struct_t;

/*! @brief USB controller (MUSB) state structure */
typedef struct _usb_device_musb_state_struct
{
    volatile ep0_state_t ep0_state;             /**< EP0 status */
    /*!< control data buffer, must align with 64 */
    uint8_t *controlData;
    /*!< 8 bytes' setup data, must align with 64 */
    uint8_t *setupData;
    /*!< 4 bytes for zero length transaction, must align with 64 */
    uint8_t *zeroTransactionData;
    volatile uint32_t dev_addr;
    /* Endpoint state structures */
    usb_device_musbip_endpoint_state_struct_t endpointState[(USB_DEVICE_MUSB_ENDPOINTS_NUM * 2)];
    usb_device_handle deviceHandle;   /*!< (4 bytes) Device handle used to identify the device object belongs to */
    USB_OTG_t *registerBase; /*!< (4 bytes) ip base address */
    volatile uint32_t *epCommandStatusList; /* endpoint list */

    uint8_t controllerId; /*!< Controller ID */
    uint8_t isResetting;  /*!< Is doing device reset or not */
    uint8_t deviceSpeed;  /*!< some controller support the HS */
} usb_device_musbip_state_struct_t;

/*!
 * @name USB device controller (MUSB) functions
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

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
                                     usb_device_controller_handle *controllerHandle);

/*!
 * @brief Deinitializes the USB device controller instance.
 *
 * This function deinitializes the USB device controller module.
 *
 * @param[in] controllerHandle   Pointer of the device controller handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMUSBDeinit(usb_device_controller_handle controllerHandle);

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
                                uint32_t length);

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
                                uint32_t length);

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
usb_status_t USB_DeviceMUSBCancel(usb_device_controller_handle controllerHandle, uint8_t ep);

/*!
 * @brief Controls the status of the selected item.
 *
 * The function is used to control the status of the selected item.
 *
 * @param[in] controllerHandle      Pointer of the device controller handle.
 * @param[in] type             The selected item. Please refer to enumeration type usb_device_control_type_t.
 * @param[in,out] param            The parameter type is determined by the selected item.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMUSBControl(usb_device_controller_handle controllerHandle,
                                   usb_device_control_type_t type,
                                   void *param);


void usb0_device_int_handler(void *deviceHandle);


void usb0_dma_int_handler(void *deviceHandle);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __USB_DEVICE_MUSB_H__ */
