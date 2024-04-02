/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   MUSB Controller Driver                                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_dcd_musb.h                                     PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */ 
/*    This file contains the MUSB USB device controller definitions.    */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  xx-xx-xxxx     Chaoqiong Xiao           Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/

#ifndef UX_DCD_MUSB_H
#define UX_DCD_MUSB_H

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_dci.h"
#include "ux_api.h"

/* Define MUSB generic equivalences.  */

#define UX_DCD_MUSB_SLAVE_CONTROLLER                           0x80
#ifndef UX_DCD_MUSB_MAX_ED
#define UX_DCD_MUSB_MAX_ED                                     11
#endif /* UX_DCD_MUSB_MAX_ED */
#define UX_DCD_MUSB_IN_FIFO                                    3


#define UX_DCD_MUSB_FLUSH_RX_FIFO                              0x00000010
#define UX_DCD_MUSB_FLUSH_TX_FIFO                              0x00000020
#define UX_DCD_MUSB_FLUSH_FIFO_ALL                             0x00000010
#define UX_DCD_MUSB_ENDPOINT_SPACE_SIZE                        0x00000020
#define UX_DCD_MUSB_ENDPOINT_CHANNEL_SIZE                      0x00000020


/* Define USB MUSB physical endpoint status definition.  */

#define UX_DCD_MUSB_ED_STATUS_UNUSED                            0
#define UX_DCD_MUSB_ED_STATUS_USED                              1
#define UX_DCD_MUSB_ED_STATUS_TRANSFER                          2
#define UX_DCD_MUSB_ED_STATUS_STALLED                           4

/* Define USB MUSB physical endpoint state machine definition.  */

#define UX_DCD_MUSB_ED_STATE_IDLE                               0
#define UX_DCD_MUSB_ED_STATE_DATA_TX                            1
#define UX_DCD_MUSB_ED_STATE_DATA_RX                            2
#define UX_DCD_MUSB_ED_STATE_STATUS_TX                          3
#define UX_DCD_MUSB_ED_STATE_STATUS_RX                          4

/* Define USB MUSB endpoint transfer status definition.  */

#define UX_DCD_MUSB_ED_TRANSFER_STATUS_IDLE                     0
#define UX_DCD_MUSB_ED_TRANSFER_STATUS_SETUP                    1
#define UX_DCD_MUSB_ED_TRANSFER_STATUS_IN_COMPLETION            2
#define UX_DCD_MUSB_ED_TRANSFER_STATUS_OUT_COMPLETION           3

/* Define USB MUSB physical endpoint structure.  */

typedef struct UX_DCD_MUSB_ED_STRUCT 
{

    UCHAR           ux_dcd_musb_ed_status;
    UCHAR           ux_dcd_musb_ed_state;
    UCHAR           ux_dcd_musb_ed_index;
    UCHAR           ux_dcd_musb_ed_direction;
    struct UX_SLAVE_ENDPOINT_STRUCT             
                    *ux_dcd_musb_ed_endpoint;
} UX_DCD_MUSB_ED;


/* Define USB MUSB DCD structure definition.  */

typedef struct UX_DCD_MUSB_STRUCT
{

    struct UX_SLAVE_DCD_STRUCT
                        *ux_dcd_musb_dcd_owner;
    struct UX_DCD_MUSB_ED_STRUCT
                        ux_dcd_musb_ed[UX_DCD_MUSB_MAX_ED];
    usb_device_handle   handle;
} UX_DCD_MUSB;


/* Define USB MUSB DCD prototypes.  */

UINT            _ux_dcd_musb_endpoint_create(UX_DCD_MUSB *dcd_musb, UX_SLAVE_ENDPOINT *endpoint);
UINT            _ux_dcd_musb_endpoint_destroy(UX_DCD_MUSB *dcd_musb, UX_SLAVE_ENDPOINT *endpoint);
UINT            _ux_dcd_musb_endpoint_reset(UX_DCD_MUSB *dcd_musb, UX_SLAVE_ENDPOINT *endpoint);
UINT            _ux_dcd_musb_endpoint_stall(UX_DCD_MUSB *dcd_musb, UX_SLAVE_ENDPOINT *endpoint);
UINT            _ux_dcd_musb_endpoint_status(UX_DCD_MUSB *dcd_musb, ULONG endpoint_index);
UINT            _ux_dcd_musb_frame_number_get(UX_DCD_MUSB *dcd_musb, ULONG *frame_number);
UINT            _ux_dcd_musb_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);
UINT            _ux_dcd_musb_initialize_complete(VOID);
UINT            _ux_dcd_musb_transfer_request(UX_DCD_MUSB *dcd_musb, UX_SLAVE_TRANSFER *transfer_request);
UINT            _ux_dcd_musb_initialize(ULONG controller_id, VOID** handle_ptr, void *contrl_handle);
UINT            _ux_dcd_musb_uninitialize(ULONG controller_id, VOID** handle_ptr);
usb_status_t    _ux_dcd_musb_callback(usb_device_handle handle, uint32_t event, void *param);
usb_status_t    _ux_dcd_musb_control_callback(usb_device_handle handle,
                                              usb_device_endpoint_callback_message_struct_t *message,
                                              void *callbackParam);
usb_status_t    _ux_dcd_musb_transfer_callback(usb_device_handle handle,
                                              usb_device_endpoint_callback_message_struct_t *message,
                                              void *callbackParam);
UINT            _ux_dcd_musb_change_state(usb_device_handle handle, ULONG parameter);

#endif

