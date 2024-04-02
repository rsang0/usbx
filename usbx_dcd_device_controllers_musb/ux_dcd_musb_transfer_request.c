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
#include "ux_dcd_musb.h"
#include "ux_utility.h"
#include "ux_device_stack.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_musb_transfer_request                     PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will initiate a transfer to a specific endpoint.      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_musb                            Pointer to device controller  */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    USB_DeviceSendRequest                 Transmit data                 */
/*    USB_DeviceRecvRequest                 Receive data                  */
/*    _ux_utility_semaphore_get             Get semaphore                 */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    MUSB Controller Driver                                            */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  xx-xx-xxxx     Chaoqiong Xiao           Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_musb_transfer_request(UX_DCD_MUSB *dcd_musb, UX_SLAVE_TRANSFER *transfer_request)
{

UX_SLAVE_ENDPOINT       *endpoint;


    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Check for transfer direction.  Is this a IN endpoint ? */
    if (transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT)
    {

        /* Transmit data.  */
        USB_DeviceSendRequest(dcd_musb -> handle,
                            endpoint->ux_slave_endpoint_descriptor.bEndpointAddress,
                            transfer_request->ux_slave_transfer_request_data_pointer,
                           transfer_request->ux_slave_transfer_request_requested_length);
    }
    else
    {

        /* We have a request for a SETUP or OUT Endpoint.  */
        /* Receive data.  */
        USB_DeviceRecvRequest(dcd_musb -> handle,
                            endpoint->ux_slave_endpoint_descriptor.bEndpointAddress,
                            transfer_request->ux_slave_transfer_request_data_pointer,
                            transfer_request->ux_slave_transfer_request_requested_length);
    }

    /* Return to caller with success.  */
    return(UX_STATE_WAIT);
}

