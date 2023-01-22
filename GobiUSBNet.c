
/*===========================================================================
FILE:
   GobiUSBNet.c

DESCRIPTION:
   QTI USB Network device

FUNCTIONS:
   GatherEndpoints
   GobiSuspend
   GobiResume
   GobiNetDriverBind
   GobiNetDriverUnbind
   GobiUSBNetURBCallback
   GobiUSBNetTXTimeout
   GobiUSBNetAutoPMThread
   GobiUSBNetStartXmit
   GobiUSBNetOpen
   GobiUSBNetStop
   GobiUSBNetProbe
   GobiUSBNetModInit
   GobiUSBNetModExit

Copyright (c) 2011,2015 The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------

#include "Structs.h"
#include "QMIDevice.h"
#include "QMI.h"
#include "qmap.h"
#include <linux/device.h>
#include <linux/if_arp.h>
#include <linux/platform_device.h>
#include "gsw_mb_data_conn.h"
#include <linux/wait.h>
#include <linux/delay.h>
//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

// Version Information
#define DRIVER_VERSION "2016-10-21"
#define DRIVER_AUTHOR "GOSUNCN"
#define DRIVER_DESC "GobiNet"
#define DRIVER_VERSION_NO "1.0.3" //Please add this value before update gobinet driver.

static struct kobject *kobj;
// Debug flag
int debug  = 1;
int Wdebug = 1;
int Edebug = 1;
int Ddebug = 0;

// Allow user interrupts
int interruptible = 1;

// Number of IP packets which may be queued up for transmit
int txQueueLength = 100;

//mux support
int mux = 0;

//Enable DHCP
int enableDhcp = 1;

// Class should be created during module init, so needs to be global
static struct class *gpClass;
//static int gobi_sys;
//static int check_perf = 0;
sGobiUSBNet *gpGobiDev;

//mobis gobinet global struct start
mb_data_conn_state_s_type gmb_data_conn0_conf;
mb_data_conn_state_s_type gmb_data_conn1_conf;
mb_data_conn_state_s_type gmb_data_conn2_conf;


//ind
//static int call_status = 0;
mb_data_conn_rsp_msg_s_type gpind;
/* fixed read_wait init by liyonggang at 20200617 start */
#if 0
wait_queue_head_t read_wait;
static int wait_flag = 0;
#else
static DECLARE_WAIT_QUEUE_HEAD(read_wait);
static volatile int wait_flag = 0;
#endif
/* fixed read_wait init by liyonggang at 20200617 end */
static void GobiUSBNet_Disconnect (struct usb_interface *intf);
void qmi_report_conn_status(u32 clientid, int pdp_status, u32 add_family);
//mobis gobinet ind end

/*===========================================================================
METHOD:
   GatherEndpoints (Public Method)

DESCRIPTION:
   Enumerate endpoints

PARAMETERS
   pIntf          [ I ] - Pointer to usb interface

RETURN VALUE:
   sEndpoints structure
              NULL for failure
===========================================================================*/
sEndpoints *GatherEndpoints(struct usb_interface *pIntf)
{
    int numEndpoints;
    int endpointIndex;
    sEndpoints *pOut;
    struct usb_host_endpoint *pEndpoint = NULL;

    pOut = kzalloc(sizeof(sEndpoints), GFP_ATOMIC);
    if (pOut == NULL)
    {
        EDBG("unable to allocate memory\n");
        return NULL;
    }

    pOut->mIntfNum = pIntf->cur_altsetting->desc.bInterfaceNumber;

    // Scan endpoints
    numEndpoints = pIntf->cur_altsetting->desc.bNumEndpoints;
    for (endpointIndex = 0; endpointIndex < numEndpoints; endpointIndex++)
    {
        pEndpoint = pIntf->cur_altsetting->endpoint + endpointIndex;
        if (pEndpoint == NULL)
        {
            EDBG("invalid endpoint %u\n", endpointIndex);
            kfree(pOut);
            return NULL;
        }

        if (usb_endpoint_dir_in(&pEndpoint->desc) == true
                &&  usb_endpoint_xfer_int(&pEndpoint->desc) == true)
        {
            pOut->mIntInEndp = pEndpoint->desc.bEndpointAddress;
        }
        else if (usb_endpoint_dir_in(&pEndpoint->desc) == true
                 &&  usb_endpoint_xfer_int(&pEndpoint->desc) == false)
        {
            pOut->mBlkInEndp = pEndpoint->desc.bEndpointAddress;
        }
        else if (usb_endpoint_dir_in(&pEndpoint->desc) == false
                 &&  usb_endpoint_xfer_int(&pEndpoint->desc) == false)
        {
            pOut->mBlkOutEndp = pEndpoint->desc.bEndpointAddress;
        }
    }

    if (pOut->mIntInEndp == 0
            ||  pOut->mBlkInEndp == 0
            ||  pOut->mBlkOutEndp == 0)
    {
        DBG("One or more endpoints missing\n");
        kfree(pOut);
        return NULL;
    }

    WDBG("intf %u\n", pOut->mIntfNum);
    WDBG("   int in  0x%02x\n", pOut->mIntInEndp);
    WDBG("   blk in  0x%02x\n", pOut->mBlkInEndp);
    WDBG("   blk out 0x%02x\n", pOut->mBlkOutEndp);

    return pOut;
}

/*===========================================================================
METHOD:
   GobiSuspend (Public Method)

DESCRIPTION:
   Stops QMI traffic while device is suspended

PARAMETERS
   pIntf          [ I ] - Pointer to interface
   powerEvent     [ I ] - Power management event

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int GobiSuspend(
    struct usb_interface      *pIntf,
    pm_message_t               powerEvent)
{
    struct usbnet *pDev;
    sGobiUSBNet *pGobiDev;

    if (pIntf == 0)
    {
        return -ENOMEM;
    }

#if (LINUX_VERSION_CODE > KERNEL_VERSION( 2,6,23 ))
    pDev = usb_get_intfdata(pIntf);
#else
    pDev = (struct usbnet *)pIntf->dev.platform_data;
#endif

    if (pDev == NULL || pDev->net == NULL)
    {
        EDBG("failed to get netdevice\n");
        return -ENXIO;
    }

    pGobiDev = (sGobiUSBNet *)pDev->data[0];
    if (pGobiDev == NULL)
    {
        EDBG("failed to get QMIDevice\n");
        return -ENXIO;
    }

    // Is this autosuspend or system suspend?
    //    do we allow remote wakeup?
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
    if (pDev->udev->auto_pm == 0)
#else
    if ((powerEvent.event & PM_EVENT_AUTO) == 0)
#endif
    {
        WDBG("device suspended to power level %d\n",
             powerEvent.event);
        GobiSetDownReason(pGobiDev, DRIVER_SUSPENDED);
    }
    else
    {
        WDBG("device autosuspend\n");
    }

    if (powerEvent.event & PM_EVENT_SUSPEND)
    {
        // Stop QMI read callbacks
        KillRead(pGobiDev);
        pDev->udev->reset_resume = 0;

        // Store power state to avoid duplicate resumes
        pIntf->dev.power.power_state.event = powerEvent.event;
    }
    else
    {
        // Other power modes cause QMI connection to be lost
        pDev->udev->reset_resume = 1;
    }

    // Run usbnet's suspend function
    return usbnet_suspend(pIntf, powerEvent);
}

/*===========================================================================
METHOD:
   GobiResume (Public Method)

DESCRIPTION:
   Resume QMI traffic or recreate QMI device

PARAMETERS
   pIntf          [ I ] - Pointer to interface

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int GobiResume(struct usb_interface *pIntf)
{
    struct usbnet *pDev;
    sGobiUSBNet *pGobiDev;
    int nRet;
    int oldPowerState;

    if (pIntf == 0)
    {
        return -ENOMEM;
    }

#if (LINUX_VERSION_CODE > KERNEL_VERSION( 2,6,23 ))
    pDev = usb_get_intfdata(pIntf);
#else
    pDev = (struct usbnet *)pIntf->dev.platform_data;
#endif

    if (pDev == NULL || pDev->net == NULL)
    {
        EDBG("failed to get netdevice\n");
        return -ENXIO;
    }

    pGobiDev = (sGobiUSBNet *)pDev->data[0];
    if (pGobiDev == NULL)
    {
        EDBG("failed to get QMIDevice\n");
        return -ENXIO;
    }

    oldPowerState = pIntf->dev.power.power_state.event;
    pIntf->dev.power.power_state.event = PM_EVENT_ON;
    WDBG("resuming from power mode %d\n", oldPowerState);

    if (oldPowerState & PM_EVENT_SUSPEND)
    {
        // It doesn't matter if this is autoresume or system resume
        GobiClearDownReason(pGobiDev, DRIVER_SUSPENDED);

        nRet = usbnet_resume(pIntf);
        if (nRet != 0)
        {
            EDBG("usbnet_resume error %d\n", nRet);
            return nRet;
        }

        // Restart QMI read callbacks
        nRet = StartRead(pGobiDev);
        if (nRet != 0)
        {
            EDBG("StartRead error %d\n", nRet);
            return nRet;
        }

        // Kick Auto PM thread to process any queued URBs
        complete(&pGobiDev->mAutoPM.mThreadDoWork);
    }
    else
    {
        WDBG("nothing to resume\n");
        return 0;
    }

    return nRet;
}

/*===========================================================================
METHOD:
   GobiNetDriverBind (Public Method)

DESCRIPTION:
   Setup in and out pipes

PARAMETERS
   pDev           [ I ] - Pointer to usbnet device
   pIntf          [ I ] - Pointer to interface

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
static int GobiNetDriverBind(
    struct usbnet          *pDev,
    struct usb_interface   *pIntf)
{
    int numEndpoints;
    int endpointIndex;
    struct usb_host_endpoint *pEndpoint = NULL;
    struct usb_host_endpoint *pIn = NULL;
    struct usb_host_endpoint *pOut = NULL;

    DBG("GobiNetDriverBind called!!!\n");
    // Verify one altsetting
    if (pIntf->num_altsetting != 1)
    {
        EDBG("invalid num_altsetting %u\n", pIntf->num_altsetting);
        return -ENODEV;
    }

    // Collect In and Out endpoints
    numEndpoints = pIntf->cur_altsetting->desc.bNumEndpoints;
    for (endpointIndex = 0; endpointIndex < numEndpoints; endpointIndex++)
    {
        pEndpoint = pIntf->cur_altsetting->endpoint + endpointIndex;
        if (pEndpoint == NULL)
        {
            EDBG("invalid endpoint %u\n", endpointIndex);
            return -ENODEV;
        }

        if (usb_endpoint_dir_in(&pEndpoint->desc) == true
                &&  usb_endpoint_xfer_int(&pEndpoint->desc) == false)
        {
            pIn = pEndpoint;
        }
        else if (usb_endpoint_dir_out(&pEndpoint->desc) == true)
        {
            pOut = pEndpoint;
        }
    }

    if (pIn == NULL || pOut == NULL)
    {
        EDBG("invalid endpoints\n");
        return -ENODEV;
    }

    if (usb_set_interface(pDev->udev,
                          pIntf->cur_altsetting->desc.bInterfaceNumber,
                          0) != 0)
    {
        EDBG("unable to set interface\n");
        return -ENODEV;
    }

    pDev->in = usb_rcvbulkpipe(pDev->udev,
                               pIn->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
    pDev->out = usb_sndbulkpipe(pDev->udev,
                                pOut->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);

    DBG("in %x, out %x\n",
        pIn->desc.bEndpointAddress,
        pOut->desc.bEndpointAddress);

    // In later versions of the kernel, usbnet helps with this
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))
    pIntf->dev.platform_data = (void *)pDev;
#endif

    return 0;
}

/* modify by liyonggang at 20200514 begin */
//static struct attribute_group dev_attr_grp;
/* modify by liyonggang at 20200514 end */
/*===========================================================================
METHOD:
   GobiNetDriverUnbind (Public Method)

DESCRIPTION:
   Deregisters QMI device (Registration happened in the probe function)

PARAMETERS
   pDev           [ I ] - Pointer to usbnet device
   pIntfUnused    [ I ] - Pointer to interface

RETURN VALUE:
   None
===========================================================================*/
static void GobiNetDriverUnbind(
    struct usbnet          *pDev,
    struct usb_interface   *pIntf)
{
    sGobiUSBNet *pGobiDev = (sGobiUSBNet *)pDev->data[0];

    // Should already be down, but just in case...
    //netif_carrier_off(pDev->net);

    DeregisterQMIDevice(pGobiDev);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,29 ))
    kfree(pDev->net->netdev_ops);
    pDev->net->netdev_ops = NULL;
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))
    pIntf->dev.platform_data = NULL;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,19 ))
    pIntf->needs_remote_wakeup = 0;
#endif
    if (pGobiDev)
    {
        /* modify by liyonggang at 20200514 begin */
#if 0
        if (pGobiDev->kobj_gobi)
        {
            sysfs_remove_group(pGobiDev->kobj_gobi, &dev_attr_grp);
            kobject_put(pGobiDev->kobj_gobi);
            pGobiDev->kobj_gobi = NULL;
        }
#endif
        /* modify by liyonggang at 20200514 end */
        kfree(pGobiDev->mpEndpoints);
        kfree(pGobiDev);
        pGobiDev = NULL;
    }
}

/*===========================================================================
METHOD:
   GobiUSBNetURBCallback (Public Method)

DESCRIPTION:
   Write is complete, cleanup and signal that we're ready for next packet

PARAMETERS
   pURB     [ I ] - Pointer to sAutoPM struct

RETURN VALUE:
   None
===========================================================================*/
void GobiUSBNetURBCallback(struct urb *pURB)
{
    unsigned long activeURBflags;
    sAutoPM *pAutoPM = (sAutoPM *)pURB->context;
    if (pAutoPM == NULL)
    {
        // Should never happen
        EDBG("bad context\n");
        return;
    }

    if (pURB->status != 0)
    {
        // Note that in case of an error, the behaviour is no different
        EDBG("urb finished with error %d\n", pURB->status);
    }

    // Remove activeURB (memory to be freed later)
    spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);

    // EAGAIN used to signify callback is done
    pAutoPM->mpActiveURB = ERR_PTR(-EAGAIN);

    spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);

    complete(&pAutoPM->mThreadDoWork);

    usb_free_urb(pURB);
}

/*===========================================================================
METHOD:
   GobiUSBNetTXTimeout (Public Method)

DESCRIPTION:
   Timeout declared by the net driver.  Stop all transfers

PARAMETERS
   pNet     [ I ] - Pointer to net device

RETURN VALUE:
   None
===========================================================================*/
void GobiUSBNetTXTimeout(struct net_device *pNet)
{
    struct sGobiUSBNet *pGobiDev;
    sAutoPM *pAutoPM;
    sURBList *pURBListEntry;
    unsigned long activeURBflags, URBListFlags;
    struct usbnet *pDev = netdev_priv(pNet);
    struct urb *pURB;

    if (pDev == NULL || pDev->net == NULL)
    {
        EDBG("failed to get usbnet device\n");
        return;
    }

    pGobiDev = (sGobiUSBNet *)pDev->data[0];
    if (pGobiDev == NULL)
    {
        EDBG("failed to get QMIDevice\n");
        return;
    }
    pAutoPM = &pGobiDev->mAutoPM;

    DBG("\n");

    // Grab a pointer to active URB
    spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);
    pURB = pAutoPM->mpActiveURB;
    spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);

    // Stop active URB
    if (pURB != NULL)
    {
        usb_kill_urb(pURB);
    }

    // Cleanup URB List
    spin_lock_irqsave(&pAutoPM->mURBListLock, URBListFlags);

    pURBListEntry = pAutoPM->mpURBList;
    while (pURBListEntry != NULL)
    {
        pAutoPM->mpURBList = pAutoPM->mpURBList->mpNext;
        atomic_dec(&pAutoPM->mURBListLen);
        usb_free_urb(pURBListEntry->mpURB);
        kfree(pURBListEntry);
        pURBListEntry = pAutoPM->mpURBList;
    }

    spin_unlock_irqrestore(&pAutoPM->mURBListLock, URBListFlags);

    complete(&pAutoPM->mThreadDoWork);

    return;
}

/*===========================================================================
METHOD:
   GobiUSBNetAutoPMThread (Public Method)

DESCRIPTION:
   Handle device Auto PM state asynchronously
   Handle network packet transmission asynchronously

PARAMETERS
   pData     [ I ] - Pointer to sAutoPM struct

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
static int GobiUSBNetAutoPMThread(void *pData)
{
    unsigned long activeURBflags, URBListFlags;
    sURBList *pURBListEntry;
    int status;
    struct usb_device *pUdev;
    sAutoPM *pAutoPM = (sAutoPM *)pData;
    struct urb *pURB;

    if (pAutoPM == NULL)
    {
        EDBG("passed null pointer\n");
        return -EINVAL;
    }

    pUdev = interface_to_usbdev(pAutoPM->mpIntf);

    DBG("traffic thread started\n");

    while (pAutoPM->mbExit == false)
    {
        // Wait for someone to poke us
        wait_for_completion_interruptible(&pAutoPM->mThreadDoWork);

        // Time to exit?
        if (pAutoPM->mbExit == true)
        {
            // Stop activeURB
            spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);
            pURB = pAutoPM->mpActiveURB;
            spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);

            if (pURB != NULL)
            {
                usb_kill_urb(pURB);
            }
            // Will be freed in callback function

            // Cleanup URB List
            spin_lock_irqsave(&pAutoPM->mURBListLock, URBListFlags);

            pURBListEntry = pAutoPM->mpURBList;
            while (pURBListEntry != NULL)
            {
                pAutoPM->mpURBList = pAutoPM->mpURBList->mpNext;
                atomic_dec(&pAutoPM->mURBListLen);
                usb_free_urb(pURBListEntry->mpURB);
                kfree(pURBListEntry);
                pURBListEntry = pAutoPM->mpURBList;
            }

            spin_unlock_irqrestore(&pAutoPM->mURBListLock, URBListFlags);

            break;
        }

        // Is our URB active?
        spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);

        // EAGAIN used to signify callback is done
        if (IS_ERR(pAutoPM->mpActiveURB)
                &&  PTR_ERR(pAutoPM->mpActiveURB) == -EAGAIN)
        {
            pAutoPM->mpActiveURB = NULL;

            // Restore IRQs so task can sleep
            spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);

            // URB is done, decrement the Auto PM usage count
            usb_autopm_put_interface(pAutoPM->mpIntf);

            // Lock ActiveURB again
            spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);
        }

        if (pAutoPM->mpActiveURB != NULL)
        {
            // There is already a URB active, go back to sleep
            spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);
            continue;
        }

        // Is there a URB waiting to be submitted?
        spin_lock_irqsave(&pAutoPM->mURBListLock, URBListFlags);
        if (pAutoPM->mpURBList == NULL)
        {
            // No more URBs to submit, go back to sleep
            spin_unlock_irqrestore(&pAutoPM->mURBListLock, URBListFlags);
            spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);
            continue;
        }

        // Pop an element
        pURBListEntry = pAutoPM->mpURBList;
        pAutoPM->mpURBList = pAutoPM->mpURBList->mpNext;
        atomic_dec(&pAutoPM->mURBListLen);
        spin_unlock_irqrestore(&pAutoPM->mURBListLock, URBListFlags);

        // Set ActiveURB
        pAutoPM->mpActiveURB = pURBListEntry->mpURB;
        spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);

        // Tell autopm core we need device woken up
        status = usb_autopm_get_interface(pAutoPM->mpIntf);
        if (status < 0)
        {
            DBG("unable to autoresume interface: %d\n", status);

            // likely caused by device going from autosuspend -> full suspend
            if (status == -EPERM)
            {
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
                pUdev->auto_pm = 0;
#endif
                GobiSuspend(pAutoPM->mpIntf, PMSG_SUSPEND);
            }

            // Add pURBListEntry back onto pAutoPM->mpURBList
            spin_lock_irqsave(&pAutoPM->mURBListLock, URBListFlags);
            pURBListEntry->mpNext = pAutoPM->mpURBList;
            pAutoPM->mpURBList = pURBListEntry;
            atomic_inc(&pAutoPM->mURBListLen);
            spin_unlock_irqrestore(&pAutoPM->mURBListLock, URBListFlags);

            spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);
            pAutoPM->mpActiveURB = NULL;
            spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);

            // Go back to sleep
            continue;
        }

        // Submit URB
        status = usb_submit_urb(pAutoPM->mpActiveURB, GFP_KERNEL);
        if (status < 0)
        {
            // Could happen for a number of reasons
            EDBG("Failed to submit URB: %d.  Packet dropped\n", status);
            spin_lock_irqsave(&pAutoPM->mActiveURBLock, activeURBflags);
            usb_free_urb(pAutoPM->mpActiveURB);
            pAutoPM->mpActiveURB = NULL;
            spin_unlock_irqrestore(&pAutoPM->mActiveURBLock, activeURBflags);
            usb_autopm_put_interface(pAutoPM->mpIntf);

            // Loop again
            complete(&pAutoPM->mThreadDoWork);
        }

        kfree(pURBListEntry);
    }

    DBG("traffic thread exiting\n");
    pAutoPM->mpThread = NULL;
    return 0;
}

/*===========================================================================
METHOD:
   GobiUSBNetStartXmit (Public Method)

DESCRIPTION:
   Convert sk_buff to usb URB and queue for transmit

PARAMETERS
   pNet     [ I ] - Pointer to net device

RETURN VALUE:
   NETDEV_TX_OK on success
   NETDEV_TX_BUSY on error
===========================================================================*/
int GobiUSBNetStartXmit(
    struct sk_buff      *pSKB,
    struct net_device   *pNet)
{
    unsigned long URBListFlags;
    struct sGobiUSBNet *pGobiDev;
    sAutoPM *pAutoPM;
    sURBList *pURBListEntry, ** ppURBListEnd;
    void *pURBData;
    struct usbnet *pDev = netdev_priv(pNet);

    DBG("\n");

    if (pDev == NULL || pDev->net == NULL)
    {
        EDBG("failed to get usbnet device\n");
        return NETDEV_TX_BUSY;
    }

    pGobiDev = (sGobiUSBNet *)pDev->data[0];
    if (pGobiDev == NULL)
    {
        EDBG("failed to get QMIDevice\n");
        return NETDEV_TX_BUSY;
    }
    pAutoPM = &pGobiDev->mAutoPM;

    if (GobiTestDownReason(pGobiDev, DRIVER_SUSPENDED) == true)
    {
        // Should not happen
        WDBG("device is suspended\n");
        dump_stack();
        return NETDEV_TX_BUSY;
    }

    if (enableDhcp == 1)
    {
        skb_pull(pSKB, ETH_HLEN);
    }
    // Add qmap hdr
    qmap_mux(pSKB, pGobiDev, 0);


    // Convert the sk_buff into a URB

    // Check if buffer is full
    if (atomic_read(&pAutoPM->mURBListLen) >= txQueueLength)
    {
        EDBG("not scheduling request, buffer is full\n");
        return NETDEV_TX_BUSY;
    }

    // Allocate URBListEntry
    pURBListEntry = kmalloc(sizeof(sURBList), GFP_ATOMIC);
    if (pURBListEntry == NULL)
    {
        EDBG("unable to allocate URBList memory\n");
        return NETDEV_TX_BUSY;
    }
    pURBListEntry->mpNext = NULL;

    // Allocate URB
    pURBListEntry->mpURB = usb_alloc_urb(0, GFP_ATOMIC);
    if (pURBListEntry->mpURB == NULL)
    {
        EDBG("unable to allocate URB\n");
        kfree(pURBListEntry);
        return NETDEV_TX_BUSY;
    }

    // Allocate URB transfer_buffer
    pURBData = kmalloc(pSKB->len, GFP_ATOMIC);
    if (pURBData == NULL)
    {
        EDBG("unable to allocate URB data\n");
        usb_free_urb(pURBListEntry->mpURB);
        kfree(pURBListEntry);
        return NETDEV_TX_BUSY;
    }
    // Fill will SKB's data
    memcpy(pURBData, pSKB->data, pSKB->len);

    usb_fill_bulk_urb(pURBListEntry->mpURB,
                      pGobiDev->mpNetDev->udev,
                      pGobiDev->mpNetDev->out,
                      pURBData,
                      pSKB->len,
                      GobiUSBNetURBCallback,
                      pAutoPM);

    DBG("Write %d bytes\n", pSKB->len);

    if (pSKB->len < 32)
        PrintHex(pURBData, pSKB->len);
    else
        PrintHex(pURBData, 32);

    // Free the transfer buffer on last reference dropped
    pURBListEntry->mpURB->transfer_flags |= URB_FREE_BUFFER;

    // Aquire lock on URBList
    spin_lock_irqsave(&pAutoPM->mURBListLock, URBListFlags);

    // Add URB to end of list
    ppURBListEnd = &pAutoPM->mpURBList;
    while ((*ppURBListEnd) != NULL)
    {
        ppURBListEnd = &(*ppURBListEnd)->mpNext;
    }
    *ppURBListEnd = pURBListEntry;
    atomic_inc(&pAutoPM->mURBListLen);

    spin_unlock_irqrestore(&pAutoPM->mURBListLock, URBListFlags);

    complete(&pAutoPM->mThreadDoWork);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
    // Start transfer timer
    pNet->trans_start = jiffies;
#else
    netif_trans_update(pNet);
#endif
    // Free SKB
    dev_kfree_skb_any(pSKB);

    return NETDEV_TX_OK;
}

/*===========================================================================
METHOD:
   GobiUSBNetOpen (Public Method)

DESCRIPTION:
   Wrapper to usbnet_open, correctly handling autosuspend
   Start AutoPM thread

PARAMETERS
   pNet     [ I ] - Pointer to net device

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int GobiUSBNetOpen(struct net_device *pNet)
{
    int status = 0;
    struct sGobiUSBNet *pGobiDev;
    struct usbnet *pDev = netdev_priv(pNet);

    if (pDev == NULL)
    {
        EDBG("failed to get usbnet device\n");
        return -ENXIO;
    }

    pGobiDev = (sGobiUSBNet *)pDev->data[0];
    if (pGobiDev == NULL)
    {
        EDBG("failed to get QMIDevice\n");
        return -ENXIO;
    }

    DBG("\n");

    // Start the AutoPM thread
    pGobiDev->mAutoPM.mpIntf = pGobiDev->mpIntf;
    pGobiDev->mAutoPM.mbExit = false;
    pGobiDev->mAutoPM.mpURBList = NULL;
    pGobiDev->mAutoPM.mpActiveURB = NULL;
    spin_lock_init(&pGobiDev->mAutoPM.mURBListLock);
    spin_lock_init(&pGobiDev->mAutoPM.mActiveURBLock);
    atomic_set(&pGobiDev->mAutoPM.mURBListLen, 0);
    init_completion(&pGobiDev->mAutoPM.mThreadDoWork);

    pGobiDev->mAutoPM.mpThread = kthread_run(GobiUSBNetAutoPMThread,
                                 &pGobiDev->mAutoPM,
                                 "GobiUSBNetAutoPMThread");
    if (IS_ERR(pGobiDev->mAutoPM.mpThread))
    {
        EDBG("AutoPM thread creation error\n");
        return PTR_ERR(pGobiDev->mAutoPM.mpThread);
    }

    // Allow traffic
    GobiClearDownReason(pGobiDev, NET_IFACE_STOPPED);

    // Pass to usbnet_open if defined
    if (pGobiDev->mpUSBNetOpen != NULL)
    {
        status = pGobiDev->mpUSBNetOpen(pNet);

        // If usbnet_open was successful enable Auto PM
        if (status == 0)
        {
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
            usb_autopm_enable(pGobiDev->mpIntf);
#else
            usb_autopm_put_interface(pGobiDev->mpIntf);
#endif
        }
    }
    else
    {
        WDBG("no USBNetOpen defined\n");
    }

    return status;
}

/*===========================================================================
METHOD:
   GobiUSBNetStop (Public Method)

DESCRIPTION:
   Wrapper to usbnet_stop, correctly handling autosuspend
   Stop AutoPM thread

PARAMETERS
   pNet     [ I ] - Pointer to net device
RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int GobiUSBNetStop(struct net_device *pNet)
{
    struct sGobiUSBNet *pGobiDev;
    struct usbnet *pDev = netdev_priv(pNet);

    if (pDev == NULL || pDev->net == NULL)
    {
        EDBG("failed to get netdevice\n");
        return -ENXIO;
    }

    pGobiDev = (sGobiUSBNet *)pDev->data[0];
    if (pGobiDev == NULL)
    {
        EDBG("failed to get QMIDevice\n");
        return -ENXIO;
    }

    // Stop traffic
    GobiSetDownReason(pGobiDev, NET_IFACE_STOPPED);

    // Tell traffic thread to exit
    pGobiDev->mAutoPM.mbExit = true;
    complete(&pGobiDev->mAutoPM.mThreadDoWork);

    // Wait for it to exit
    while (pGobiDev->mAutoPM.mpThread != NULL)
    {
        msleep(100);
    }
    DBG("thread stopped\n");

    // Pass to usbnet_stop, if defined
    if (pGobiDev->mpUSBNetStop != NULL)
    {
        return pGobiDev->mpUSBNetStop(pNet);
    }
    else
    {
        return 0;
    }
}

int PreparePacket(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
    char *dataPtr;
    PQC_ETH_HDR ethHdr;
    unsigned long ethLen;

    dataPtr = (char *)skb->data;
    ethLen = skb->len;
    ethHdr = (PQC_ETH_HDR)dataPtr;

    DBG("IPO: -->_PreparePacket: EtherType 0x%04X\n", ethHdr->EtherType);

    // examine Ethernet header
    switch (ntohs(ethHdr->EtherType))
    {
    case ETH_TYPE_ARP:   // && IPV4
    {
        // locally process ARP under IPV4
        ProcessARP(dev, skb);
        return 0;
    }
    case ETH_TYPE_IPV4:
    {
        DBG("IPO: _PreparePacket: IP 0x%04X\n", ntohs(ethHdr->EtherType));
        skb_pull(skb, ETH_HLEN);
        return 1;
    }

    case ETH_TYPE_IPV6:
    {
        DBG("IPO: _PreparePacket: IP 0x%04X\n", ntohs(ethHdr->EtherType));
        skb_pull(skb, ETH_HLEN);
        return 1;
    }
    default:
    {
        DBG("IPO: _PreparePacket: IP 0x%04X\n", ntohs(ethHdr->EtherType));
        return 0;
    }
    }
}  // MPUSB_PreparePacket

void ProcessARP(struct usbnet *dev, struct sk_buff *skb)
{
    PQC_ARP_HDR arpHdr;
    char *tempHA[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int sz;
    int bRespond = 0;
    unsigned long Length = skb->len;
    char *EthPkt;

    EthPkt = (char *)skb->data;
    DBG("IPO: -->ProcessARP: %dB\n", (int)Length);

    arpHdr = (PQC_ARP_HDR)(EthPkt + sizeof(QC_ETH_HDR));

    // Ignore non-Ethernet HW type and non-request Operation
    if ((ntohs(arpHdr->HardwareType) != 1) || (ntohs(arpHdr->Operation) != 1))
    {
        DBG("IPO: ProcessARP: ignore HW %d OP %d\n", arpHdr->HardwareType, arpHdr->Operation);
        return;
    }

    // Ignore non-IPV4 protocol type
    if (ntohs(arpHdr->ProtocolType) != ETH_TYPE_IPV4)
    {
        DBG("IPO: ProcessARP: ignore protocol %d\n", arpHdr->ProtocolType);
        return;
    }

    // Validate HLEN and PLEN
    if (arpHdr->HLEN != ETH_ALEN)
    {
        EDBG("IPO: ProcessARP: wrong HLEN %d\n", arpHdr->HLEN);
        return;
    }
    if (arpHdr->PLEN != 4)
    {
        EDBG("IPO: ProcessARP: wrong PLEN %d\n", arpHdr->PLEN);
        return;
    }

    // Ignore gratuitous ARP
    if (arpHdr->SenderIP == arpHdr->TargetIP)
    {
        DBG("IPO: ProcessARP: ignore gratuitous ARP (IP 0x%d)\n", (int)arpHdr->TargetIP);
        return;
    }

    // Request for HA
    sz = memcmp(arpHdr->TargetHA, tempHA, ETH_ALEN);

    if ((arpHdr->SenderIP != 0) && (ETH_ALEN == sz))
    {
        DBG(" IPO: ProcessARP: req for HA\n");
        bRespond = 1;
    }
    else
    {
        DBG("IPO: ProcessARP: Ignore\n");
    }

    if (bRespond == 1)
    {
        // respond with canned ARP
        ArpResponse(dev, skb);
    }

    DBG("IPO: <--ProcessARP: local rsp %d\n", bRespond);
}  // MPUSB_ProcessARP


void ArpResponse(struct usbnet *dev, struct sk_buff *skb)
{
    PQC_ARP_HDR        arpHdr;
    struct ethhdr *eth;
    struct sk_buff *skbn;
    sGobiUSBNet *pGobiDev;
    char *p;
    unsigned long Length = skb->len;

    pGobiDev = (sGobiUSBNet *)dev->data[0];

    DBG("IPO: -->MPUSB_ArpResponse: ETH_Len %dB\n", (int)Length);

    skbn = netdev_alloc_skb(dev->net, Length);
    skb_put(skbn, Length);




    eth = (struct ethhdr *)skb_push(skbn, ETH_HLEN + sizeof(QC_ARP_HDR));
    memcpy(eth->h_dest, dev->net->dev_addr, ETH_ALEN);
    memset(eth->h_source, 0, ETH_ALEN);
    eth->h_proto = __cpu_to_be16(ETH_P_IP);
    skbn->dev = dev->net;


    // 3. Formulate the response
    // Target: arpHdr->SenderHA & arpHdr->SenderIP
    // Sender: pAdapter->MacAddress2 & pAdapter->IPSettings.IPV4.Address
    p = (char *)eth;

    // ARP Header
    arpHdr = (PQC_ARP_HDR)(p + sizeof(QC_ETH_HDR));

    // target/requestor MAC and IP
    memcpy(arpHdr->TargetHA, arpHdr->SenderHA, ETH_ALEN);
    // arpHdr->SenderIP = arpHdr->TargetIP;

    // sender/remote MAC and IP
    memcpy(arpHdr->SenderHA, eth->h_source, ETH_ALEN);
    // arpHdr->TargetIP = pGobiDev->IPv4Addr;

    // Operation: reply
    arpHdr->Operation = ntohs(0x0002);

    usbnet_skb_return(dev, skbn);
    DBG("IPO: IPO: <--MPUSB_ArpResponse\n");
}  // MPUSB_ArpResponse


static struct sk_buff *GobiNetDriver_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
    sGobiUSBNet *pGobiDev = (sGobiUSBNet *)dev->data[0];

    if (enableDhcp == 1)
    {
        u16 tci = 0;
        skb_reset_mac_header(skb);
        if (skb->len > VLAN_ETH_HLEN && __vlan_get_tag(skb, &tci) == 0)
        {
            DDBG("VLAN tci: %04x", tci);
            skb_pull(skb, VLAN_ETH_HLEN);
        }
        else
        {
            skb_pull(skb, ETH_HLEN);
        }
        qmap_mux_vlan(skb, pGobiDev, tci & 0x0fff);
    }
    else
        qmap_mux(skb, pGobiDev, 0);


    DDBG("TxFixup %d bytes Protocol %d\n", skb->len, skb->protocol);
    if(Ddebug == 1)
    {
        if (skb->len < 32)
            PrintHex(skb->data, skb->len);
        else
            PrintHex(skb->data, 32);
    }
    return skb;
}
static int GobiNetDriver_rx_fixup(struct usbnet *dev, struct sk_buff *skb_in)
{
    struct sk_buff *skbn;
    struct sGobiUSBNet *pGobiDev;
    int offset = 0;
    int packet_len;
    int pad_len;
    qmap_t qhdr;
    u16 Protocol;
    void *tmp;

    pGobiDev = (sGobiUSBNet *)dev->data[0];

    while (offset < skb_in->len)
    {
        qhdr = (qmap_t)(skb_in->data + offset);
        pad_len = qhdr->cd_rsvd_pad & 0x3f;
        packet_len = (qhdr->pkt_len[0] << 8) | (qhdr->pkt_len[1]);

        if (skb_in->data[offset] & 0x80)
        {
            /* drop the packet, we do not know what to do */
            DDBG("Dropping command packet\n");
            goto _skip;
        }
        if (!packet_len)
        {
            DDBG("Dropping zero length packet\n");
            goto _skip;
        }

        switch (skb_in->data[offset + sizeof(struct qmap_hdr)] & 0xf0)
        {
        case 0x40:
            Protocol = htons(ETH_P_IP);
            break;
        case 0x60:
            Protocol = htons(ETH_P_IPV6);
            break;
        default:
            DBG("Dropping NON-IP packet\n");
            goto _skip;
        }

        if (enableDhcp == 1)
        {
            if (qhdr->mux_id == pGobiDev->mQMIDev.MuxId)
                skbn = netdev_alloc_skb_ip_align(dev->net, packet_len + ETH_HLEN);
            else
                skbn = netdev_alloc_skb_ip_align(dev->net, packet_len + VLAN_ETH_HLEN);
        }
        else
        {
            skbn = netdev_alloc_skb_ip_align(dev->net, packet_len);
        }
        if (!skbn)
            goto _error;

        if (enableDhcp == 1)
        {
            u16 vlan_id = 0;
            if (qhdr->mux_id != pGobiDev->mQMIDev.MuxId)
            {
                int i;
                for (i = 0; i < MAX_MUX_DEVICES; i++)
                {
                    if (pGobiDev->mQMIMUXDev[i].MuxId == qhdr->mux_id)
                    {
                        vlan_id = pGobiDev->mQMIMUXDev[i].vlan_id;
                        break;
                    }
                }
            }

            DDBG("RX: VLAN %d, MUX 0x%02x\n", vlan_id, qhdr->mux_id);
            if (vlan_id == 0)
            {
                struct ethhdr *eth;
                skb_reserve(skbn, ETH_HLEN);
                eth = (struct ethhdr *)skb_push(skbn, ETH_HLEN);
                skb_reset_mac_header(skbn);
                eth->h_proto = Protocol;
                memcpy(eth->h_dest, dev->net->dev_addr, ETH_ALEN);
                memcpy(eth->h_source, dev->net->dev_addr, ETH_ALEN);
            }
            else
            {
                struct vlan_ethhdr *vlan;
                skb_reserve(skbn, VLAN_ETH_HLEN);
                vlan = (struct vlan_ethhdr *)skb_push(skbn, VLAN_ETH_HLEN);
                skb_reset_mac_header(skbn);
                vlan->h_vlan_proto = htons(ETH_P_8021Q);
                vlan->h_vlan_TCI = htons(vlan_id);
                vlan->h_vlan_encapsulated_proto = Protocol;
                memcpy(vlan->h_dest, dev->net->dev_addr, ETH_ALEN);
                memcpy(vlan->h_source, dev->net->dev_addr, ETH_ALEN);
            }
        }
        else
        {
            skbn->protocol = Protocol;  //lxy cr 20200315
        }
        //skbn->protocol = Protocol;
        tmp = skb_put(skbn, packet_len);
        memcpy(tmp, skb_in->data + offset + sizeof(struct qmap_hdr), packet_len);

        DDBG("RxFixup %d bytes\n", skbn->len);
        if(Ddebug == 1)
        {
            if (skbn->len < 32)
                PrintHex(skbn->data, skbn->len);
            else
                PrintHex(skbn->data, 32);
        }
        usbnet_skb_return(dev, skbn);

_skip:
        offset += (packet_len + sizeof(struct qmap_hdr));

    }
    return 1;
_error:
    return 0;
}

/* modify by liyonggang at 20200514 begin */
#if 0
static ssize_t
gobi_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "Write gobi value %d\n", gobi_sys);
}

static ssize_t
gobi_perf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t
gobi_perf_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    check_perf = 1;
    return count;
}

static ssize_t
gobi_store(struct device *dev, struct device_attribute *attr,
           const char *buf, size_t count)
{
    int value;
    static int id0 = -1;
    static int ipv6id0 = -1;
    static int connectid0 = -1;
    static int connectipv6id0 = -1;
    u8 profileid;
    profileid = 0x01; /* set the correct profile ID here */

    sscanf(buf, "%d", &value);
    gobi_sys = value;

    if (value == 1)
    {
        // connect
        id0 = WDSConnect(gpGobiDev, &gpGobiDev->mQMIDev, profileid, &ipv6id0, &connectid0, &connectipv6id0);
        if (id0 < 0)
        {
            WDBG("Error Connecting\n");
        }
        else
        {
            WDBG("Client ID is %d\n", id0);
        }
    }
    else
    {
        if (id0 != -1)
            WDSDisConnect(id0, gpGobiDev, &gpGobiDev->mQMIDev, ipv6id0, connectid0, connectipv6id0);
        id0 = -1;
    }
    return count;
}

static ssize_t
gobi_show_1(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "Write gobi value %d\n", gobi_sys);
}


static ssize_t
gobi_store_1(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
    int value;
    static int id1 = -1;
    static int ipv6id1 = -1;
    static int connectid1 = -1;
    static int connectipv6id1 = -1;
    u8 profileid;
    profileid = 0x02; /* set the correct profile ID here */

    sscanf(buf, "%d", &value);
    gobi_sys = value;

    if (value == 1)
    {
        // connect
        id1 = WDSConnect(gpGobiDev, &gpGobiDev->mQMIMUXDev[0], profileid, &ipv6id1, &connectid1, &connectipv6id1);
        if (id1 < 0)
        {
            WDBG("Error Connecting\n");
        }
        else
        {
            WDBG("Client ID is %d\n", id1);
        }
    }
    else
    {
        if (id1 != -1)
            WDSDisConnect(id1, gpGobiDev, &gpGobiDev->mQMIMUXDev[0], ipv6id1, connectid1, connectipv6id1);
        id1 = -1;
    }
    return count;
}

static ssize_t
gobi_show_2(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "Write gobi value %d\n", gobi_sys);
}


static ssize_t
gobi_store_2(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
    int value;
    static int id2 = -1;
    static int ipv6id2 = -1;
    static int connectid2 = -1;
    static int connectipv6id2 = -1;
    u8 profileid;
    profileid = 0x03; /* set the correct profile ID here */

    sscanf(buf, "%d", &value);
    gobi_sys = value;

    if (value == 1)
    {
        // connect
        id2 = WDSConnect(gpGobiDev, &gpGobiDev->mQMIMUXDev[1], profileid, &ipv6id2, &connectid2, &connectipv6id2);
        if (id2 < 0)
        {
            WDBG("Error Connecting\n");
        }
        else
        {
            WDBG("Client ID is %d\n", id2);
        }
    }
    else
    {
        if (id2 != -1)
            WDSDisConnect(id2, gpGobiDev, &gpGobiDev->mQMIMUXDev[1], ipv6id2, connectid2, connectipv6id2);
        id2 = -1;
    }
    return count;
}

static ssize_t
ip_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sQMIDev *pDev = &gpGobiDev->mQMIDev;

    return snprintf(buf, PAGE_SIZE,
                    "IPV4 address: %pI4h\n"
                    "IPV4 netmask: %pI4h\n"
                    "IPV4 gateway: %pI4h\n"
                    "IPV4 DNS1: %pI4h\n"
                    "IPV4 DNS2: %pI4h\n"
                    "IPV6 address: %pI6/%d\n"
                    "IPV6 gateway: %pI6\n"
                    "IPV6 DNS1: %pI6\n"
                    "IPV6 DNS2: %pI6\n",
                    &pDev->IPv4Addr,
                    &pDev->IPv4SubnetMask,
                    &pDev->IPv4Gateway,
                    &pDev->IPv4PrimaryDNS,
                    &pDev->IPv4SecondaryDNS,
                    pDev->ipv6_address.ipv6addr, pDev->ipv6_address.prefix,
                    pDev->ipv6_gateway.ipv6addr,
                    pDev->ipv6_primaydns.ipv6addr,
                    pDev->ipv6_secondarydns.ipv6addr);
}
#endif
/* modify by liyonggang at 20200514 end */
//gobinet mobis start function
int  ip_show_switch( char *output_buf, mb_data_conn_state_s_type* mb_data_conn_conf )
{
    int ret = 0;

    mb_data_conn_rsp_msg_s_type     conn_correct_rsp;
    //mb_data_err_rsp_msg_s_type      conn_err_rsp ;
    mb_data_disconn_rsp_msg_s_type  disconn_rsp;
    mb_data_err_rsp_msg_s_type      err_rsp;


    WDBG("ip%d_show_switch\n",   mb_data_conn_conf->data_rsp.if_id);
    WDBG("show v4 end_type[%d]\n", mb_data_conn_conf->v4_conn_err_code.end_type);
    WDBG("show v6 end_type[%d]\n", mb_data_conn_conf->v6_conn_err_code.end_type);

    DDBG("v4 qmicli[0x%x],handle[0x%x]\n", mb_data_conn_conf->qmi_client_v4, mb_data_conn_conf->data_handle_v4);
    DDBG("v6 qmicli[0x%x],handle[0x%x]\n", mb_data_conn_conf->qmi_client_v6, mb_data_conn_conf->data_handle_v6);

    switch(mb_data_conn_conf->gobi_current_command)
    {
    case MB_DATACONN_REQ_MSG:
        if( (mb_data_conn_conf->data_rsp.pdp_type == MB_IPV4V6) &&
                (mb_data_conn_conf->v4_conn_err_code.end_type != 0) &&
                (mb_data_conn_conf->v6_conn_err_code.end_type != 0) )
        {
            //conn_err_rsp.msg = MB_DATACONN_ERR_MSG;
            memset(&err_rsp, 0 , sizeof(mb_data_err_rsp_msg_s_type) );
            err_rsp.msg = MB_DATACONN_ERR_MSG;
            err_rsp.v4_conn_err_code=mb_data_conn_conf->v4_conn_err_code;
            err_rsp.v6_conn_err_code=mb_data_conn_conf->v6_conn_err_code;
            memcpy(output_buf, &(err_rsp),	sizeof(mb_data_err_rsp_msg_s_type));
            ret = sizeof(mb_data_err_rsp_msg_s_type);

        }
        else if( (mb_data_conn_conf->data_rsp.pdp_type == MB_IPV4) &&
                 (mb_data_conn_conf->v4_conn_err_code.end_type != 0))
        {

            // conn_err_rsp.msg = MB_DATACONN_ERR_MSG;
            memset(&err_rsp, 0 , sizeof(mb_data_err_rsp_msg_s_type) );
            err_rsp.v4_conn_err_code=mb_data_conn_conf->v4_conn_err_code;
            //err_rsp.v6_conn_err_code=mb_data_conn_conf->v6_conn_err_code;
            memcpy(output_buf, &(err_rsp),	sizeof(mb_data_err_rsp_msg_s_type));
            ret = sizeof(mb_data_err_rsp_msg_s_type);


        }
        else if((mb_data_conn_conf->data_rsp.pdp_type == MB_IPV6) &&
                (mb_data_conn_conf->v6_conn_err_code.end_type != 0))
        {
            //conn_err_rsp.msg = MB_DATACONN_ERR_MSG;
            memset(&err_rsp, 0 , sizeof(mb_data_err_rsp_msg_s_type) );
            err_rsp.msg = MB_DATACONN_ERR_MSG;
            //err_rsp.v4_conn_err_code=mb_data_conn_conf->v4_conn_err_code;
            err_rsp.v6_conn_err_code=mb_data_conn_conf->v6_conn_err_code;
            memcpy(output_buf, &(err_rsp),	sizeof(mb_data_err_rsp_msg_s_type));
            ret = sizeof(mb_data_err_rsp_msg_s_type);
        }
        else
        {
            conn_correct_rsp.msg = MB_DATACONN_RSP_MSG;
            conn_correct_rsp.data_conn_rsp = mb_data_conn_conf->data_rsp;
            DBG("if[%d][%d]show ip info\n", conn_correct_rsp.data_conn_rsp.if_id, conn_correct_rsp.data_conn_rsp.pdp_type);

            memcpy(output_buf, &(conn_correct_rsp),  sizeof(mb_data_conn_rsp_msg_s_type));

            DBG("ipv4_info:[%d][%d][%s][%s][%s][%s]\n", conn_correct_rsp.data_conn_rsp.v4_pdp_status,
                conn_correct_rsp.data_conn_rsp.v4_mtu,
                conn_correct_rsp.data_conn_rsp.v4_addr.public_ip, conn_correct_rsp.data_conn_rsp.v4_addr.gateway_ip,
                conn_correct_rsp.data_conn_rsp.v4_addr.primary_dns, conn_correct_rsp.data_conn_rsp.v4_addr.secondary_dns);


            DBG("ipv6_info:[%d][%d][%s][%s][%s][%s]\n", conn_correct_rsp.data_conn_rsp.v6_pdp_status,
                conn_correct_rsp.data_conn_rsp.v6_mtu,
                conn_correct_rsp.data_conn_rsp.v6_addr.public_ipv6, conn_correct_rsp.data_conn_rsp.v6_addr.gateway_ipv6,
                conn_correct_rsp.data_conn_rsp.v6_addr.primary_dnsv6, conn_correct_rsp.data_conn_rsp.v6_addr.secondary_dnsv6);
            ret = sizeof(mb_data_conn_rsp_msg_s_type);
        }

        break;
    case MB_DATADISCONN_REQ_MSG:

        disconn_rsp.msg = MB_DATADISCONN_RSP_MSG;
        disconn_rsp.ret = 1; // success

        memcpy(output_buf, &(disconn_rsp),  sizeof(mb_data_disconn_rsp_msg_s_type));
        ret = sizeof(mb_data_disconn_rsp_msg_s_type);
        break;
    case MB_DATA_ERR_REQ_MSG:

        err_rsp.msg = MB_DATA_ERR_RSP_MSG;
        err_rsp.v4_conn_err_code=mb_data_conn_conf->v4_conn_err_code;
        WDBG("v4 type=%d,reason=%d", mb_data_conn_conf->v4_conn_err_code.end_type, mb_data_conn_conf->v4_conn_err_code.call_end_reason);

        err_rsp.v6_conn_err_code=mb_data_conn_conf->v6_conn_err_code;
        WDBG("v6 type=%d,reason=%d", mb_data_conn_conf->v6_conn_err_code.end_type, mb_data_conn_conf->v6_conn_err_code.call_end_reason);

        memcpy(output_buf, &err_rsp, sizeof(mb_data_err_rsp_msg_s_type));
        ret = sizeof(mb_data_err_rsp_msg_s_type);

        break;
    case MB_DATALIST_REQ_MSG:

        conn_correct_rsp.msg = MB_DATALIST_RSP_MSG;
        conn_correct_rsp.data_conn_rsp = mb_data_conn_conf->data_rsp;

        memcpy(output_buf, &(conn_correct_rsp),  sizeof(mb_data_conn_rsp_msg_s_type));
        ret = sizeof(mb_data_conn_rsp_msg_s_type);

        break;
    default:
        WDBG("has a other %d msg", mb_data_conn_conf->gobi_current_command);

    }
    return ret;
}

int  mb_data_disconn( const char *buf, size_t count,
                      mb_data_conn_state_s_type* mb_data_conn_conf)
{
    sQMIDev *QMIDev = NULL;
    int if_id = mb_data_conn_conf->data_rsp.if_id;
    if(if_id == MB_FIRST_INTERFACE)
    {
        QMIDev = &gpGobiDev->mQMIDev;
    }
    else if(if_id == MB_SEC_INTERFACE)
    {
        QMIDev = &gpGobiDev->mQMIMUXDev[0];
    }
    else
    {
        QMIDev = &gpGobiDev->mQMIMUXDev[1];
    }

    WDBG("if_id[%d]\n", if_id);

    if ( mb_data_conn_conf->data_handle_v4 != 0 ||
            mb_data_conn_conf->qmi_client_v4  != 0 )
    {
        DDBG("v4 qmiclientid[0x%x]datahandle[0x%x]\n", mb_data_conn_conf->qmi_client_v4, mb_data_conn_conf->data_handle_v4);

        WDSDisConnectv4( gpGobiDev, QMIDev, mb_data_conn_conf->qmi_client_v4, mb_data_conn_conf->data_handle_v4 );
        mb_data_conn_conf->data_handle_v4 = 0;
        mb_data_conn_conf->qmi_client_v4 = 0;

        mb_data_conn_conf->data_rsp.v4_pdp_status = MB_IPV4_DISCONNECTED;
    }


    if ( mb_data_conn_conf->data_handle_v6 != 0 ||
            mb_data_conn_conf->qmi_client_v6  != 0)
    {
        DDBG("v6 qmiclientid[0x%x]datahandle[0x%x]\n", mb_data_conn_conf->qmi_client_v6, mb_data_conn_conf->data_handle_v6 );

        WDSDisConnectv6( gpGobiDev, QMIDev, mb_data_conn_conf->qmi_client_v6, mb_data_conn_conf->data_handle_v6 );
        mb_data_conn_conf->data_handle_v6 = 0;
        mb_data_conn_conf->qmi_client_v6 = 0;

        mb_data_conn_conf->data_rsp.v6_pdp_status = MB_IPV6_DISCONNECTED;
    }
    return 0;

}


int u32toipstr(char* ipv4_str, u32 ipnum )
{
    if(ipnum == 0 )
    {
        EDBG("invalid ipv4 addr");
        return -1;
    }

    snprintf( ipv4_str, IPV4_ADDRSTRLEN,"%d.%d.%d.%d", (ipnum)>>24,
              (ipnum >> 16) & 0xff,
              (ipnum >> 8) & 0xff,
              (ipnum) & 0xff );
    return 0;
}

int  ipv6_addrtoipv6str(char*ipv6_str, ipv6_addr *addr)
{
    if(addr == 0 )
    {
        EDBG("invalid ipv6 addr");
        return -1;
    }

    sprintf(ipv6_str, "%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
            (int)addr->ipv6addr[0], (int)addr->ipv6addr[1],
            (int)addr->ipv6addr[2], (int)addr->ipv6addr[3],
            (int)addr->ipv6addr[4], (int)addr->ipv6addr[5],
            (int)addr->ipv6addr[6], (int)addr->ipv6addr[7],
            (int)addr->ipv6addr[8], (int)addr->ipv6addr[9],
            (int)addr->ipv6addr[10], (int)addr->ipv6addr[11],
            (int)addr->ipv6addr[12], (int)addr->ipv6addr[13],
            (int)addr->ipv6addr[14], (int)addr->ipv6addr[15]);
    return 0;

}

int mb_data_conn_v4( mb_data_conn_state_s_type* mb_data_conn_conf ,sQMIDev *QMIDev)
{

    int ret = 0;
    int if_id =0;
    if_id =  mb_data_conn_conf->data_rsp.if_id;

    if( mb_data_conn_conf->data_rsp.v4_pdp_status == MB_IPV4_CONNECTED )
    {
        EDBG("if_id(%d) v4 has already connected\n", if_id);
        return 0;

    }

    WDBG("qmiclientid[0x%x]datahandle[0x%x]\n", mb_data_conn_conf->qmi_client_v4, mb_data_conn_conf->data_handle_v4);

    if( mb_data_conn_conf->data_handle_v4 != 0 ||
            mb_data_conn_conf->qmi_client_v4  != 0)
    {
        WDBG("if_id(%d),release v4 handle\n", if_id);
        WDSDisConnectv4( gpGobiDev, QMIDev, mb_data_conn_conf->qmi_client_v4, mb_data_conn_conf->data_handle_v4 );
        mb_data_conn_conf->data_handle_v4 = 0;
        mb_data_conn_conf->qmi_client_v4 = 0;

    }

    // connect
    ret = WDSConnectv4(gpGobiDev, QMIDev, mb_data_conn_conf );
    if ( ret < 0)
    {
        EDBG("if_id(%d)v4 Connect Error[%d]\n", if_id, ret);
        mb_data_conn_conf->data_rsp.v4_pdp_status = MB_IPV4_DISCONNECTED;
        return ret;
    }
    else
    {
        mb_data_conn_conf->data_rsp.v4_pdp_status = MB_IPV4_CONNECTED;
        mb_data_conn_conf->v4_conn_err_code.end_type = 0;
        mb_data_conn_conf->v4_conn_err_code.call_end_reason = 0;

        u32toipstr(mb_data_conn_conf->data_rsp.v4_addr.public_ip, QMIDev->IPv4Addr);
        u32toipstr(mb_data_conn_conf->data_rsp.v4_addr.gateway_ip, QMIDev->IPv4Gateway);
        u32toipstr(mb_data_conn_conf->data_rsp.v4_addr.primary_dns, QMIDev->IPv4PrimaryDNS);
        u32toipstr(mb_data_conn_conf->data_rsp.v4_addr.secondary_dns, QMIDev->IPv4SecondaryDNS);
        mb_data_conn_conf->data_rsp.v4_mtu = QMIDev->mtu;
        qmi_report_conn_status(mb_data_conn_conf->qmi_client_v4, MB_IPV4_CONNECTED, 0x04); //0x04 is ipv4

    }

    return 0;
}

int mb_data_conn_v6( mb_data_conn_state_s_type* mb_data_conn_conf ,sQMIDev *QMIDev)
{

    int ret = 0;
    int if_id =0;
    if_id =  mb_data_conn_conf->data_rsp.if_id;

    if( mb_data_conn_conf->data_rsp.v6_pdp_status == MB_IPV6_CONNECTED)
    {
        EDBG("if_id(%d) v6 has already connected.", if_id);
        return ret;
    }

    WDBG("qmiclientid[0x%x]datahandle[0x%x]\n", mb_data_conn_conf->qmi_client_v6, mb_data_conn_conf->data_handle_v6);

    if(mb_data_conn_conf->data_handle_v6 != 0 ||
            mb_data_conn_conf->qmi_client_v6  != 0)
    {
        WDBG("if_id(%d),release v6 handle", if_id);
        WDSDisConnectv6( gpGobiDev, QMIDev, mb_data_conn_conf->qmi_client_v6, mb_data_conn_conf->data_handle_v6 );
        mb_data_conn_conf->data_handle_v6 = 0;
        mb_data_conn_conf->qmi_client_v6 = 0;

    }


    ret = WDSConnectv6(gpGobiDev, QMIDev, mb_data_conn_conf );
    if (ret < 0)
    {
        EDBG("if_id(%d)v6 Connect Error[%d]\n", if_id, ret);
        mb_data_conn_conf->data_rsp.v6_pdp_status = MB_IPV6_DISCONNECTED;
        return ret;
    }
    else
    {
        mb_data_conn_conf->data_rsp.v6_pdp_status = MB_IPV6_CONNECTED;
        mb_data_conn_conf->v6_conn_err_code.end_type = 0;
        mb_data_conn_conf->v6_conn_err_code.call_end_reason = 0;

        ipv6_addrtoipv6str(mb_data_conn_conf->data_rsp.v6_addr.public_ipv6, &QMIDev->ipv6_address);
        ipv6_addrtoipv6str(mb_data_conn_conf->data_rsp.v6_addr.gateway_ipv6, &QMIDev->ipv6_gateway);
        ipv6_addrtoipv6str(mb_data_conn_conf->data_rsp.v6_addr.primary_dnsv6, &QMIDev->ipv6_primaydns);
        ipv6_addrtoipv6str(mb_data_conn_conf->data_rsp.v6_addr.secondary_dnsv6, &QMIDev->ipv6_secondarydns);
        mb_data_conn_conf->data_rsp.v6_mtu = QMIDev->mtu;
        qmi_report_conn_status(mb_data_conn_conf->qmi_client_v6, MB_IPV6_CONNECTED, 0x06); //debug
    }

    return 0;
}


void mb_data_conn(const char *buf, size_t count,
                  mb_data_conn_state_s_type* mb_data_conn_conf)
{

    u8 profileid = 0x01; /* set the correct profile ID here */
    int ret = 0;
    int if_id = 0;
    int pdp_type = 0;
    char apn[APN_STRING_MAX]= {0};
    mb_data_conn_req_msg_s_type data_conn_req_msg;
    sQMIDev *QMIDev = NULL;

    if(sizeof(mb_data_conn_req_msg_s_type) != count)
    {
        EDBG("buf size is error\n");
        return;
    }

    memset( &data_conn_req_msg, 0 ,sizeof(mb_data_conn_req_msg_s_type));
    memcpy( &data_conn_req_msg, (mb_data_conn_req_msg_s_type*) buf, count );

    if_id = data_conn_req_msg.data_conn_req.if_id;
    profileid = data_conn_req_msg.data_conn_req.cid;
    pdp_type = data_conn_req_msg.data_conn_req.pdp_type;
    memcpy(apn, data_conn_req_msg.data_conn_req.apn, strlen(data_conn_req_msg.data_conn_req.apn));

    WDBG("if_id[%d],profileid[%d],pdp_type[%d],apn[%s]\n", if_id, profileid, pdp_type, apn);

    mb_data_conn_conf->data_rsp.if_id = if_id;
    mb_data_conn_conf->data_rsp.cid = profileid;
    mb_data_conn_conf->data_rsp.pdp_type = pdp_type;

    if(strlen(apn) > 0)
    {
        memcpy(mb_data_conn_conf->apn, apn, strlen(apn));
    }
    else
    {
        memset(mb_data_conn_conf->apn, 0, sizeof(APN_STRING_MAX));
    }

    if(if_id == MB_FIRST_INTERFACE)
    {
        QMIDev = &gpGobiDev->mQMIDev;
    }
    else if(if_id == MB_SEC_INTERFACE)
    {
        QMIDev = &gpGobiDev->mQMIMUXDev[0];
    }
    else
    {
        QMIDev = &gpGobiDev->mQMIMUXDev[1];
    }


    if( (pdp_type == MB_IPV4) || (pdp_type == MB_IPV4V6))
    {
        ret =  mb_data_conn_v4( mb_data_conn_conf , QMIDev);
        if(ret == -4)
        {
            // sometime writesync and readsync produce -4, try again will be ok.
            msleep(1 * 1000);
            ret =  mb_data_conn_v4( mb_data_conn_conf , QMIDev);
            EDBG("mb_data_conn_v4 ret[%d]\n", ret);
        }
    }

    if( (pdp_type== MB_IPV6) || (pdp_type== MB_IPV4V6))
    {
        ret =  mb_data_conn_v6( mb_data_conn_conf , QMIDev);
        if(ret == -4)
        {
            // sometime writesync and readsync produce -4, try again will be ok.
            msleep(1 * 1000);
            ret =  mb_data_conn_v6( mb_data_conn_conf , QMIDev);
            EDBG("mb_data_conn_v6 ret[%d]\n", ret);
        }
    }

    return;
}

void process_gobi_input_command(const char *buf, size_t count, mb_data_conn_state_s_type* mb_data_conn_conf)
{
    int input_msg_type = 0;
    input_msg_type = *(int *)buf;

    WDBG("recv RIL msg 0x%02x", input_msg_type);
    switch(input_msg_type)
    {
    case MB_DATACONN_REQ_MSG:
        DDBG("MB_DATACONN_REQ_MSG");
        mb_data_conn(buf, count, mb_data_conn_conf);
        mb_data_conn_conf->gobi_current_command = MB_DATACONN_REQ_MSG;
        break;
    case MB_DATADISCONN_REQ_MSG:
        DDBG("MB_DATADISCONN_REQ_MSG");
        mb_data_disconn(buf, count, mb_data_conn_conf);
        mb_data_conn_conf->gobi_current_command = MB_DATADISCONN_REQ_MSG;
        break;
    case MB_DATA_ERR_REQ_MSG:
        DDBG("MB_DATA_ERR_REQ_MSG");
        mb_data_conn_conf->gobi_current_command = MB_DATA_ERR_REQ_MSG;

        break;
    case MB_DATALIST_REQ_MSG:
        mb_data_conn_conf->gobi_current_command = MB_DATALIST_REQ_MSG;

        break;
    case MB_DATAIND_REQ_MSG:

        break;
    default:
        WDBG("recv RIL other msg %x", input_msg_type);
    }

    return ;
}


static ssize_t
data0_store(struct device *dev, struct device_attribute *attr,
            const char *buf, size_t count)
{
    if(count > 2)
    {
        gmb_data_conn0_conf.data_rsp.if_id = MB_FIRST_INTERFACE;
        process_gobi_input_command(buf, count, &gmb_data_conn0_conf);
    }
    else
    {
        WDBG("input invalid param\n");
    }

    return count;
}

static ssize_t
data0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "gobinet,data0_show");
}

static ssize_t
data0cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = ip_show_switch(buf, &gmb_data_conn0_conf);
    WDBG("return count = %d", ret);
    return ret ;
}

static ssize_t
data1_store(struct device *dev, struct device_attribute *attr,
            const char *buf, size_t count)
{
    if(count > 2)
    {
        gmb_data_conn1_conf.data_rsp.if_id = MB_SEC_INTERFACE;
        process_gobi_input_command(buf, count, &gmb_data_conn1_conf);
    }
    else
    {
        WDBG("input invalid param\n");
    }
    return count;

}

static ssize_t
data1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "gobinet,data1_show");
}

static ssize_t
data1cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = ip_show_switch(buf, &gmb_data_conn1_conf);
    WDBG("return count = %d", ret);
    return ret ;
}

static ssize_t
data2_store(struct device *dev, struct device_attribute *attr,
            const char *buf, size_t count)
{
    if(count > 2)
    {
        gmb_data_conn2_conf.data_rsp.if_id = MB_THIRD_INTERFACE;
        process_gobi_input_command(buf, count, &gmb_data_conn2_conf);
    }
    else
    {
        WDBG("input invalid param\n");
    }
    return count;

}

static ssize_t
data2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "gobinet,data2_show");
}

static ssize_t
data2cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = ip_show_switch(buf, &gmb_data_conn2_conf);
    WDBG("goginet,data2,return count = %d", ret);
    return ret ;
}

static ssize_t
gobiversion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "mobis gobinet version: %s\n", DRIVER_VERSION_NO);
}

static ssize_t
gobiversion_store(struct device *dev, struct device_attribute *attr,
                  const char *buf, size_t count)
{
    return count;
}

static ssize_t
gobi_call_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    ret = wait_event_interruptible(read_wait,wait_flag);
    if (ret)
    {
        return -EAGAIN;
    }

    WDBG("msg=0x%02x\n", gpind.msg);

    WDBG("if[%d][%d]\n", gpind.data_conn_rsp.if_id, gpind.data_conn_rsp.pdp_type);
    WDBG("ipv4_info:[%d][%s][%s][%s][%s]\n", gpind.data_conn_rsp.v4_pdp_status,
         gpind.data_conn_rsp.v4_addr.public_ip, gpind.data_conn_rsp.v4_addr.gateway_ip,
         gpind.data_conn_rsp.v4_addr.primary_dns, gpind.data_conn_rsp.v4_addr.secondary_dns);

    WDBG("ipv6_info:[%d][%s][%s][%s][%s]\n", gpind.data_conn_rsp.v6_pdp_status,
         gpind.data_conn_rsp.v6_addr.public_ipv6, gpind.data_conn_rsp.v6_addr.gateway_ipv6,
         gpind.data_conn_rsp.v6_addr.primary_dnsv6, gpind.data_conn_rsp.v6_addr.secondary_dnsv6);

    //ind
    if(buf)
    {
        memcpy(buf, &gpind, sizeof(mb_data_conn_rsp_msg_s_type));
    }
    wait_flag = 0;

    //return snprintf(buf, PAGE_SIZE, "gobi call status = %d\n", call_status);
    return sizeof(mb_data_conn_rsp_msg_s_type);
}
//this indicated function will tell someone pscall has a change

void qmi_report_conn_status(u32 clientid, int pdp_status, u32 add_family)
{

    //mb_data_conn_rsp_msg_s_type gpind;
    gpind.msg = MB_DATAIND_RSP_MSG;
    WDBG("qmiclientid 0x%x,pdp_status 0x%x, add_family 0x%x\n",clientid, pdp_status, add_family);

    if(clientid == 0x00)
    {
        WDBG("dormancy is changed\n");
        gpind.data_conn_rsp = gmb_data_conn0_conf.data_rsp;
        gpind.data_conn_rsp.v4_pdp_status = pdp_status;
        gpind.data_conn_rsp.v6_pdp_status = pdp_status;

        //ind
        wait_flag = 1;
        wake_up_interruptible(&read_wait);
        return;
    }


    if( clientid == gmb_data_conn0_conf.qmi_client_v4 ||
            clientid == gmb_data_conn0_conf.qmi_client_v6)
    {
        if(add_family == 0x04) //ipv4
        {
            gmb_data_conn0_conf.data_rsp.v4_pdp_status = pdp_status;
        }
        else if(add_family == 0x06) //ipv6
        {
            gmb_data_conn0_conf.data_rsp.v6_pdp_status = pdp_status;
        }
        else
        {
            WDBG("conn0 addr family error");
        }

        gpind.data_conn_rsp = gmb_data_conn0_conf.data_rsp;
        WDBG("if_id 0 v%d status become [0x%x]\n", add_family, pdp_status);
    }
    else if(clientid == gmb_data_conn1_conf.qmi_client_v4 ||
            clientid == gmb_data_conn1_conf.qmi_client_v6)
    {

        if(add_family == 0x04) //ipv4
        {
            gmb_data_conn1_conf.data_rsp.v4_pdp_status = pdp_status;
        }
        else if(add_family == 0x06) //ipv6
        {
            gmb_data_conn1_conf.data_rsp.v6_pdp_status = pdp_status;
        }
        else
        {
            WDBG("conn1 addr family error");
        }

        gpind.data_conn_rsp = gmb_data_conn1_conf.data_rsp;
        WDBG("if_id 1 v%d status become [0x%x]\n", add_family, pdp_status);
    }
    else if(clientid == gmb_data_conn2_conf.qmi_client_v4 ||
            clientid == gmb_data_conn2_conf.qmi_client_v6)
    {
        if(add_family == 0x04) //ipv4
        {
            gmb_data_conn2_conf.data_rsp.v4_pdp_status = pdp_status;
        }
        else if(add_family == 0x06) //ipv6
        {
            gmb_data_conn2_conf.data_rsp.v6_pdp_status = pdp_status;
        }
        else
        {
            WDBG("conn2 addr family error\n");
        }
        gpind.data_conn_rsp = gmb_data_conn2_conf.data_rsp;
        WDBG("if_id 2 v%d status become [0x%x]\n", add_family, pdp_status);
    }
    else
    {
        EDBG("invalid clientid");
        return;
    }


    WDBG("send RIL msg=0x%x\n",  gpind.msg);
    //WDBG("v4_pdp_status=0x%x\n", gpind.data_conn_rsp.v4_pdp_status);
    //WDBG("v6_pdp_status=0x%x\n", gpind.data_conn_rsp.v6_pdp_status);

    //ind
    wait_flag = 1;
    wake_up_interruptible(&read_wait);

}
//gobinet mobis end function

/* modify by liyonggang at 20200515 begin */
#if 0
static DEVICE_ATTR(gobi0, S_IRUGO | S_IWUSR,
                   gobi_show, gobi_store);

static DEVICE_ATTR(ip0, S_IRUGO | S_IWUSR,
                   ip_show, NULL);

static DEVICE_ATTR(gobi1, S_IRUGO | S_IWUSR,
                   gobi_show_1, gobi_store_1);

static DEVICE_ATTR(gobi2, S_IRUGO | S_IWUSR,
                   gobi_show_2, gobi_store_2);

static DEVICE_ATTR(gobi_perf, S_IRUGO | S_IWUSR,
                   gobi_perf_show, gobi_perf_store);

static struct attribute *dev_attrs[] =
{
    &dev_attr_gobi0.attr,
    &dev_attr_ip0.attr,
    &dev_attr_gobi1.attr,
    &dev_attr_gobi2.attr,
    &dev_attr_gobi_perf.attr,
#if 0 //modify by liyonggang at 20200509
    &dev_attr_data0.attr,
    &dev_attr_data0cfg.attr,
    &dev_attr_data1.attr,
    &dev_attr_data1cfg.attr,
    &dev_attr_data2.attr,
    &dev_attr_data2cfg.attr,
    &dev_attr_gobi_call_status.attr,
    &dev_attr_gobiversion.attr,
#endif
    NULL,
};
static struct attribute_group dev_attr_grp =
{
    .attrs = dev_attrs,
};
#else

//gobinet mobis start
static DEVICE_ATTR(data0, S_IRUGO | S_IWUSR,
                   data0_show, data0_store);

static DEVICE_ATTR(data0cfg, S_IRUGO | S_IWUSR,
                   data0cfg_show, NULL);

static DEVICE_ATTR(data1, S_IRUGO | S_IWUSR,
                   data1_show, data1_store);

static DEVICE_ATTR(data1cfg, S_IRUGO | S_IWUSR,
                   data1cfg_show, NULL);

static DEVICE_ATTR(data2, S_IRUGO | S_IWUSR,
                   data2_show, data2_store);

static DEVICE_ATTR(data2cfg, S_IRUGO | S_IWUSR,
                   data2cfg_show, NULL);

static DEVICE_ATTR(gobi_call_status, S_IRUGO | S_IWUSR,
                   gobi_call_status_show, NULL);

static DEVICE_ATTR(gobiversion, S_IRUGO | S_IWUSR,
                   gobiversion_show, gobiversion_store);

static struct attribute *dev_attrs[] =
{
    &dev_attr_data0.attr,
    &dev_attr_data0cfg.attr,
    &dev_attr_data1.attr,
    &dev_attr_data1cfg.attr,
    &dev_attr_data2.attr,
    &dev_attr_data2cfg.attr,
    &dev_attr_gobi_call_status.attr,
    &dev_attr_gobiversion.attr,
    NULL,
};
static struct attribute_group dev_attr_grp =
{
    .attrs = dev_attrs,
};
//gobinet mobis end
#endif
/* modify by liyonggang at 20200515 begin */
/*=========================================================================*/
// Struct driver_info
/*=========================================================================*/
static const struct driver_info GobiNetInfo =
{
    .description   = "GobiNet Ethernet Device",
    .flags         = FLAG_MULTI_PACKET | FLAG_NO_SETINT,
    .bind          = GobiNetDriverBind,
    .unbind        = GobiNetDriverUnbind,
    .data          = 0,
    .rx_fixup      = GobiNetDriver_rx_fixup,
    .tx_fixup      = GobiNetDriver_tx_fixup,
};

/*=========================================================================*/
// QTI VID/PIDs
/*=========================================================================*/
static const struct usb_device_id GobiVIDPIDTable [] =
{
    // Gobi 3000
    {
        USB_DEVICE_INTERFACE_NUMBER(0x305a, 0x1418, 4),
        .driver_info = (unsigned long) &GobiNetInfo
    },
    {
        USB_DEVICE_INTERFACE_NUMBER(0x05c6, 0x90A1, 2),
        .driver_info = (unsigned long) &GobiNetInfo
    },
    {
        USB_DEVICE_INTERFACE_NUMBER(0x05c6, 0x90A8, 4),
        .driver_info = (unsigned long) &GobiNetInfo
    },
    //Terminating entry
    { }
};

MODULE_DEVICE_TABLE(usb, GobiVIDPIDTable);

int gobi_class_sysfs_init(void)
{
    int retval;
    kobj = kobject_create_and_add("gswgobi", NULL);

    if (!kobj)
        return -ENOMEM;
    retval = sysfs_create_group(kobj, &dev_attr_grp);
    if (retval && kobj)
    {
        kobject_put(kobj);
        kobj = NULL;
    }
    return 0;
}

/*===========================================================================
METHOD:
   GobiUSBNetProbe (Public Method)

DESCRIPTION:
   Run usbnet_probe
   Setup QMI device

PARAMETERS
   pIntf        [ I ] - Pointer to interface
   pVIDPIDs     [ I ] - Pointer to VID/PID table

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int GobiUSBNetProbe(
    struct usb_interface         *pIntf,
    const struct usb_device_id   *pVIDPIDs)
{
    int status;
    //int retval;
    struct usbnet *pDev;
    sGobiUSBNet *pGobiDev;
    sEndpoints *pEndpoints;
    int pipe;
    int res;
    int i;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,29 ))
    struct net_device_ops *pNetDevOps;
#endif

    init_waitqueue_head(&read_wait);//maohong add
    pEndpoints = GatherEndpoints(pIntf);
    if (pEndpoints == NULL)
    {
        return -ENODEV;
    }

    status = usbnet_probe(pIntf, pVIDPIDs);
    if (status < 0)
    {
        EDBG("usbnet_probe failed %d\n", status);
        return status;
    }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,19 ))
    pIntf->needs_remote_wakeup = 1;
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION( 2,6,23 ))
    pDev = usb_get_intfdata(pIntf);
#else
    pDev = (struct usbnet *)pIntf->dev.platform_data;
#endif

    if (pDev == NULL || pDev->net == NULL)
    {
        EDBG("failed to get netdevice\n");
        //usbnet_disconnect(pIntf);
        GobiUSBNet_Disconnect(pIntf);
        kfree(pEndpoints);
        return -ENXIO;
    }

    pDev->msg_enable = 0xffff;

    pGobiDev = kmalloc(sizeof(sGobiUSBNet), GFP_KERNEL);
    if (pGobiDev == NULL)
    {
        EDBG("falied to allocate device buffers");
        //usbnet_disconnect(pIntf);
        GobiUSBNet_Disconnect(pIntf);
        kfree(pEndpoints);
        return -ENOMEM;
    }

    pDev->data[0] = (unsigned long)pGobiDev;

    pGobiDev->mpNetDev = pDev;
    pGobiDev->mpEndpoints = pEndpoints;

    // Clearing endpoint halt is a magic handshake that brings
    // the device out of low power (airplane) mode
    // NOTE: FCC verification should be done before this, if required
    pipe = usb_rcvintpipe(pGobiDev->mpNetDev->udev,
                          pGobiDev->mpEndpoints->mIntInEndp);
    usb_clear_halt(pGobiDev->mpNetDev->udev, pipe);


    pipe = usb_rcvbulkpipe(pGobiDev->mpNetDev->udev,
                           pGobiDev->mpEndpoints->mBlkInEndp);
    usb_clear_halt(pGobiDev->mpNetDev->udev, pipe);
    pipe = usb_sndbulkpipe(pGobiDev->mpNetDev->udev,
                           pGobiDev->mpEndpoints->mBlkOutEndp);
    usb_clear_halt(pGobiDev->mpNetDev->udev, pipe);

    pipe = usb_rcvbulkpipe(pGobiDev->mpNetDev->udev,
                           pGobiDev->mpEndpoints->mBlkInEndp);
    usb_clear_halt(pGobiDev->mpNetDev->udev, pipe);
    pipe = usb_sndbulkpipe(pGobiDev->mpNetDev->udev,
                           pGobiDev->mpEndpoints->mBlkOutEndp);
    usb_clear_halt(pGobiDev->mpNetDev->udev, pipe);

    /* Set Control Line State */
    res = usb_control_msg(pGobiDev->mpNetDev->udev,
                          usb_sndctrlpipe(pGobiDev->mpNetDev->udev, 0), 0x22,
                          USB_RECIP_INTERFACE | USB_TYPE_CLASS,
                          0, pGobiDev->mpEndpoints->mIntfNum, NULL, 0, USB_CTRL_SET_TIMEOUT);

    res = usb_control_msg(pGobiDev->mpNetDev->udev,
                          usb_sndctrlpipe(pGobiDev->mpNetDev->udev, 0), 0x22,
                          USB_RECIP_INTERFACE | USB_TYPE_CLASS,
                          1, pGobiDev->mpEndpoints->mIntfNum, NULL, 0, USB_CTRL_SET_TIMEOUT);

    // Overload PM related network functions
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,29 ))
    pGobiDev->mpUSBNetOpen = pDev->net->open;
    pDev->net->open = GobiUSBNetOpen;
    pGobiDev->mpUSBNetStop = pDev->net->stop;
    pDev->net->stop = GobiUSBNetStop;
    pDev->net->hard_start_xmit = GobiUSBNetStartXmit;
    pDev->net->tx_timeout = GobiUSBNetTXTimeout;
#else
    pNetDevOps = kmalloc(sizeof(struct net_device_ops), GFP_KERNEL);
    if (pNetDevOps == NULL)
    {
        EDBG("falied to allocate net device ops");
        //usbnet_disconnect(pIntf);
        GobiUSBNet_Disconnect(pIntf);
        return -ENOMEM;
    }
    memcpy(pNetDevOps, pDev->net->netdev_ops, sizeof(struct net_device_ops));

    pGobiDev->mpUSBNetOpen = pNetDevOps->ndo_open;
    pNetDevOps->ndo_open = GobiUSBNetOpen;
    pGobiDev->mpUSBNetStop = pNetDevOps->ndo_stop;
    pNetDevOps->ndo_stop = GobiUSBNetStop;

    pDev->net->netdev_ops = pNetDevOps;

    // set ETHER
    if (enableDhcp == 0)
    {
        pDev->net->header_ops      = NULL;  /* No header */
        pDev->net->type            = ARPHRD_NONE;
        pDev->net->hard_header_len = 0;
        pDev->net->addr_len        = 0;
        pDev->net->flags           = IFF_NOARP | IFF_MULTICAST;  //lxy cr 20200315
    }
    pDev->net->needed_headroom = sizeof(struct qmap_hdr);

    if (enableDhcp == 1)
    {
        ether_setup(pDev->net);
        pDev->net->flags |= IFF_NOARP;
    }
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,31 ))
    memset(&(pGobiDev->mpNetDev->stats), 0, sizeof(struct net_device_stats));
#else
    memset(&(pGobiDev->mpNetDev->net->stats), 0, sizeof(struct net_device_stats));
#endif

    pGobiDev->mpIntf = pIntf;
    memset(&(pGobiDev->mMEID), '0', 14);

    DBG("Mac Address:\n");
    PrintHex(&pGobiDev->mpNetDev->net->dev_addr[0], 6);

    pGobiDev->mbQMIValid = false;
    memset(&pGobiDev->mQMIDev, 0, sizeof(sQMIDev));
    pGobiDev->mQMIDev.mbCdevIsInitialized = false;

    pGobiDev->mQMIDev.mpDevClass = gpClass;

    init_completion(&pGobiDev->mAutoPM.mThreadDoWork);
    spin_lock_init(&pGobiDev->mQMIDev.mClientMemLock);


    for (i = 0; i < MAX_MUX_DEVICES; i++)
    {
        memset(&pGobiDev->mQMIMUXDev[i], 0, sizeof(sQMIDev));
        pGobiDev->mQMIMUXDev[i].mbCdevIsInitialized = false;

        pGobiDev->mQMIMUXDev[i].mpDevClass = gpClass;

        spin_lock_init(&pGobiDev->mQMIMUXDev[i].mClientMemLock);
    }
    // Default to device down
    pGobiDev->mDownReason = 0;
    GobiSetDownReason(pGobiDev, NO_NDIS_CONNECTION);
    GobiSetDownReason(pGobiDev, NET_IFACE_STOPPED);
    // Register QMI
    status = RegisterQMIDevice(pGobiDev);
    if (status != 0)
    {
        // usbnet_disconnect() will call GobiNetDriverUnbind() which will call
        // DeregisterQMIDevice() to clean up any partially created QMI device
        //usbnet_disconnect(pIntf);
        GobiUSBNet_Disconnect(pIntf);
        return status;
    }

    /* need to sub rx URBs for the aggregation size negotiated */
    pDev->rx_urb_size = pGobiDev->ULAggregationMaxSize;

    WDBG("The UL Datagram size is 0x%d\n", (int)pDev->rx_urb_size);
    /* modify by liyonggang at 20200515 begin */
#if 0
    pGobiDev->kobj_gobi = kobject_create_and_add("agswgobi", NULL);
    if (!pGobiDev->kobj_gobi)
        return -ENOMEM;
    retval = sysfs_create_group(pGobiDev->kobj_gobi, &dev_attr_grp);
    if (retval && pGobiDev->kobj_gobi)
    {
        kobject_put(pGobiDev->kobj_gobi);
        pGobiDev->kobj_gobi = NULL;
    }
#else
    pGobiDev->kobj_gobi = NULL;
#endif
    /* modify by liyonggang at 20200515 end */
    gpGobiDev = pGobiDev;
#if 1
    memset(&gmb_data_conn0_conf, 0 , sizeof(mb_data_conn_state_s_type));
    memset(&gmb_data_conn1_conf, 0 , sizeof(mb_data_conn_state_s_type));
    memset(&gmb_data_conn2_conf, 0 , sizeof(mb_data_conn_state_s_type));
#endif
    // Success
    return 0;
}

static void GobiUSBNet_Disconnect (struct usb_interface *intf)
{
    struct usbnet       *dev;
    struct net_device   *net;

    dev = usb_get_intfdata(intf);
    if (!dev)
        return;

    net = dev->net;

    netif_carrier_off(net);
    usbnet_disconnect(intf);
}

static struct usb_driver GobiNet =
{
    .name       = "GobiNet",
    .id_table   = GobiVIDPIDTable,
    .probe      = GobiUSBNetProbe,
    .disconnect = GobiUSBNet_Disconnect,
    .suspend    = GobiSuspend,
    .resume     = GobiResume,
    .supports_autosuspend = true,
};

#if 0
static const struct of_device_id gobinet_table[] =
{
    { .compatible = "qcom,gobi-9x35" },
    { },
};
MODULE_DEVICE_TABLE(of, gobinet_table);
struct platform_driver gobi_platform_driver;
#endif
/*===========================================================================
METHOD:
   GobiUSBNetModInit (Public Method)

DESCRIPTION:
   Initialize module
   Create device class
   Register out usb_driver struct

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
//static int GobiUSBNetModInit(struct platform_device *pdev)
static int GobiUSBNetModInit(void)
{
    gobi_class_sysfs_init();
    gpClass = class_create(THIS_MODULE, "GobiQMI");
    if (IS_ERR(gpClass) == true)
    {
        EDBG("error at class_create %ld\n",
             PTR_ERR(gpClass));
        return -ENOMEM;
    }

    // This will be shown whenever driver is loaded
    WDBG( "Author[%s] Version NO[%s]", DRIVER_AUTHOR, DRIVER_VERSION_NO);

    return usb_register(&GobiNet);
}

/*===========================================================================
METHOD:
   GobiUSBNetModExit (Public Method)

DESCRIPTION:
   Deregister module
   Destroy device class

RETURN VALUE:
   void
===========================================================================*/
// static int GobiUSBNetModExit(struct platform_device *pdev)
//static int GobiUSBNetModExit(void)
static void __exit GobiUSBNetModExit(void)
{
    /*add by liyonggang for release resource at 20200509 start*/
    if(kobj)
    {
        sysfs_remove_group(kobj,&dev_attr_grp);
        kobject_del(kobj);
    }
    /*add by liyonggang for release resource at 20200509 end*/
    usb_deregister(&GobiNet);
    class_destroy(gpClass);
    //return 0;
}

#if 0
struct platform_driver gobi_platform_driver =
{
    .probe = GobiUSBNetModInit,
    .remove = GobiUSBNetModExit,
    .driver = {
        .name = "gobinet_driver",
        .owner = THIS_MODULE,
        .of_match_table = gobinet_table,
    },
};

module_platform_driver(gobi_platform_driver);
#endif

module_init(GobiUSBNetModInit);
module_exit(GobiUSBNetModExit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual BSD/GPL");

#ifdef bool
#undef bool
#endif

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debuging enabled or not");

module_param(interruptible, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(interruptible, "Listen for and return on user interrupt");

module_param(txQueueLength, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(txQueueLength,
                 "Number of IP packets which may be queued up for transmit");

module_param(mux, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mux, "MUX support enabled or not");

module_param(enableDhcp, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enableDhcp,
                 "Enable DHCP over static IP address settings");

#if 0
module_param(RmnetIF, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(RmnetIF, "Rmnet Interface number");
#endif

