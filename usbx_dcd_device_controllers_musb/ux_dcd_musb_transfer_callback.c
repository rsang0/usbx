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

#define UX_SOURCE_CODE
#define UX_DCD_MUSB_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "common_include/usb.h"
#include "ux_dcd_musb.h"
#include "ux_device_stack.h"
#include "ux_utility.h"
#include "ux_device_class_cdc_acm.h"
#include "mp32gm51_hal_usb.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_musb_transfer_callback                    PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function handles callback from the USB driver.                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    handle                                Pointer to device handle      */
/*    message                               Message for callback          */
/*    callbackParam                         Parameter for callback        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    _ux_utility_semaphore_put             Put semaphore                 */
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    MUSB Driver                                                       */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  xx-xx-xxxx     Chaoqiong Xiao           Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
usb_status_t _ux_dcd_musb_transfer_callback(usb_device_handle handle,
                                              usb_device_endpoint_callback_message_struct_t *message,
                                              void *callbackParam)
{
//printf("EP CB\r\n");
UX_DCD_MUSB_ED         *ed;
UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

    ed = (UX_DCD_MUSB_ED*) callbackParam;

    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)ed -> ux_dcd_musb_ed_endpoint ->ux_slave_endpoint_interface ->ux_slave_interface_class_instance;

    /* Get the pointer to the transfer request.  */
    transfer_request =  &(ed -> ux_dcd_musb_ed_endpoint -> ux_slave_endpoint_transfer_request);

    if (message->length != USB_UNINITIALIZED_VAL_32)
    {

        /* Set the completion code to no error.  */
        transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

        /* The transfer is completed.  */
        transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;

        if (ed -> ux_dcd_musb_ed_direction == UX_ENDPOINT_OUT)
        {
//            printf("UX_ENDPOINT_OUT\r\n");
            /* Update the length of the data sent in previous transaction.  */
            transfer_request -> ux_slave_transfer_request_actual_length =  message->length;

            /* start read run */
            cdc_acm->ux_device_class_cdc_acm_read_state = 0;

            _ux_dcd_musb_change_state(handle, USB_EVENT_RX_AVAILABLE);
        }
        else
        {
//            printf("IN 2\r\n");
            /* start write run */
            cdc_acm ->ux_device_class_cdc_acm_write_state = 0;
            _ux_dcd_musb_change_state(handle, USB_EVENT_TX_COMPLETE);
        }

        /* Non control endpoint operation, use semaphore.  */
       // _ux_utility_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);
    }

    return kStatus_USB_Success;
}


