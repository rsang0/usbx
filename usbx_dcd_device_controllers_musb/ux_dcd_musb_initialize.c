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
/*    _ux_dcd_musb_initialize                           PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the MUSB USB device controller.         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    controller_id                         Controller ID                 */
/*    handle_ptr                            Pointer to driver handle      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    USB_DeviceInit                        Initialize USB driver         */
/*    USB_DeviceRun                         Enable device                 */
/*    _ux_utility_memory_allocate           Allocate memory               */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    USBX Device Stack                                                   */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  xx-xx-xxxx     Chaoqiong Xiao           Initial Version 6.1           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_musb_initialize(ULONG controller_id, VOID** handle_ptr, void *chage_state_cablk)
{
//    printf("_ux_dcd_musb_initialize\r\n");

UX_SLAVE_DCD            *dcd;
UX_DCD_MUSB            *dcd_musb;
usb_status_t             status;


    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* The controller initialized here is of MUSB type.  */
    dcd -> ux_slave_dcd_controller_type =  UX_DCD_MUSB_SLAVE_CONTROLLER;

    /* Allocate memory for this MUSB DCD instance.  */
    dcd_musb =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_MUSB));

    /* Check if memory was properly allocated.  */
    if(dcd_musb == UX_NULL) {
        printf("Allocate memory for this MUSB DCD instance error!!!\r\n");
        return(UX_MEMORY_INSUFFICIENT);
    }

    /* Set the pointer to the MUSB DCD.  */
    dcd -> ux_slave_dcd_controller_hardware =  (VOID *) dcd_musb;

    /* Set the generic DCD owner for the MUSB DCD.  */
    dcd_musb -> ux_dcd_musb_dcd_owner =  dcd;

    /* Initialize the function collector for this DCD.  */
    dcd -> ux_slave_dcd_function =  _ux_dcd_musb_function;

    status = USB_DeviceInit((uint8_t)controller_id, _ux_dcd_musb_callback, &dcd_musb -> handle);

    *handle_ptr =  dcd_musb -> handle;

    usb_device_struct_t *p_handle = (usb_device_struct_t *)*handle_ptr;

    p_handle->config_change = chage_state_cablk;

    USB_DeviceRun(dcd_musb -> handle);

    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;

    /* Return completion status.  */
    return(status);
}

