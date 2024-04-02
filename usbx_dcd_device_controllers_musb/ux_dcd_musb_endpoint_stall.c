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
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_musb_endpoint_stall                       PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will stall a physical endpoint.                       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_musb                            Pointer to device controller  */
/*    endpoint                              Pointer to endpoint container */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    USB_DeviceStallEndpoint               Set STALL condition           */
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
UINT  _ux_dcd_musb_endpoint_stall(UX_DCD_MUSB *dcd_musb, UX_SLAVE_ENDPOINT *endpoint)
{

UX_DCD_MUSB_ED     *ed;


    /* Get the physical endpoint address in the endpoint container.  */
    ed =  (UX_DCD_MUSB_ED *) endpoint -> ux_slave_endpoint_ed;

    /* Set the endpoint to stall.  */
    ed -> ux_dcd_musb_ed_status |=  UX_DCD_MUSB_ED_STATUS_STALLED;

    /* Stall the endpoint.  */
    USB_DeviceStallEndpoint(dcd_musb -> handle, endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress);

    /* Check if it is the Control endpoint.  */
    if(endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress == 0)
    {

        /* Stall both direction of the Control endpoint.  */
        USB_DeviceStallEndpoint(dcd_musb -> handle, 0x80);
    }

    /* This function never fails.  */
    return(UX_SUCCESS);
}
