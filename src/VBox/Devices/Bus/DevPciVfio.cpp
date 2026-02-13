/* $Id: DevPciVfio.cpp 113018 2026-02-13 16:13:09Z alexander.eichner@oracle.com $ */
/** @file
 * PCI passthrough device emulation using VFIO/IOMMUFD.
 */

/*
 * Copyright (C) 2026 Oracle and/or its affiliates.
 *
 * This file is part of VirtualBox base platform packages, as
 * available from https://www.virtualbox.org.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, in version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses>.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */


/*********************************************************************************************************************************
*   Header Files                                                                                                                 *
*********************************************************************************************************************************/
#define LOG_GROUP LOG_GROUP_DEV_PCI_RAW
#define PDMPCIDEV_INCLUDE_PRIVATE  /* Hack to get pdmpcidevint.h included at the right point. */
#include <VBox/pci.h>
#include <VBox/log.h>
#include <VBox/msi.h>
#include <VBox/vmm/pdmdev.h>
#include <VBox/vmm/stam.h>
#include <VBox/vmm/pdmpci.h>
#include <VBox/vmm/pdmpcidev.h>
#include <iprt/assert.h>
#include <iprt/mem.h>
#include <iprt/string.h>
#include <iprt/errcore.h>

#include <linux/vfio.h>
#include <linux/iommufd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include "VBoxDD.h"


/*********************************************************************************************************************************
*   Defined Constants And Macros                                                                                                 *
*********************************************************************************************************************************/

/** eventfd2() syscall for the interrupt handling. */
#define LNX_SYSCALL_EVENTFD2          290

/** Invalid access. */
#define VFIO_PCI_CFG_SPACE_ACCESS_INVALID     0
/** Passthrough the access to VFIO. */
#define VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH 1
/** Just do the default action for the access. */
#define VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  2
/** Emulate the access using a special handler. */
#define VFIO_PCI_CFG_SPACE_ACCESS_EMULATE     3


/*********************************************************************************************************************************
*   Structures and Typedefs                                                                                                      *
*********************************************************************************************************************************/

/**
 * A single PCI BAR.
 */
typedef struct VFIOPCIBAR
{
    /** Region type, 0 - disabled, 1 - PIO, 2 - MMIO. */
    uint8_t             bType;
    /** Type dependent data. */
    union
    {
        /** Start offset of the PIO region. */
        uint64_t        offPio;
        /** Start of the MMIO mapping. */
        volatile void   *pvMmio;
    } u;
    /** Type dependent handle. */
    union
    {
        /** I/O port region. */
        IOMIOPORTHANDLE hIoPort;
        /** MMIO region. */
        IOMMMIOHANDLE   hMmio;
        /** MMIO2 region. */
        PGMMMIO2HANDLE  hMmio2;
    } hnd;
} VFIOPCIBAR;
typedef VFIOPCIBAR *PVFIOPCIBAR;
typedef const VFIOPCIBAR *PCVFIOPCIBAR;


/**
 * Passed through VFIO PCI device instance.
 */
typedef struct VFIOPCI
{
    /** Pointer to the device instance. */
    PPDMDEVINSR3         pDevIns;
    /** The device instance. */
    int                  iInstance;

    /** The IOMMU file descriptor. */
    int                  iFdIommu;
    /** The vfio cdev file descriptor .*/
    int                  iFdVfio;

    /** The IOMMU page table object id. */
    uint32_t             idIommuHwpt;

    /** The start offset of the PCI config space. */
    uint64_t             offPciCfg;
    /** Size of the PCI config space. */
    size_t               cbPciCfg;
    /** The access table indicating how to treat
     * config space accesses for each byte.. Each config space byte
     * requires 4bits (2bit each for read write) and the config space can be 4096 bytes large. */
    uint8_t              abPciCfgIntercept[(4096 * 4) / 8];
    /** The PCI BAR information. */
    VFIOPCIBAR           aBars[VBOX_PCI_NUM_REGIONS];

    /** Flag whether MMIO regions are intercepted and handled through
     * regular MMIO handlers or are mapped into the guest. */
    bool                 fInterceptMmio;
    /** Flag whether VGA capabilities are exposed. */
    bool                 fVga;
    /** The start offset of the VGA region. */
    uint64_t             offVga;
    /** The legacy I/O port range from 0x3b0 - 0x3bb. */
    IOMIOPORTHANDLE      hVgaIoPort1;
    /** The legacy I/O port range from 0x3c0 - 0x3df. */
    IOMIOPORTHANDLE      hVgaIoPort2;
    /** The legacy MMIO range from 0xa0000 - 0xbffff*/
    IOMMMIOHANDLE        hVgaMmio;

    /** ROM region start offset. */
    uint64_t             offRom;
    /** Size of the ROM region. */
    size_t               cbRom;
    /** Pointer to the ROM region memory. */
    void                 *pvRom;
    /** ROM region handle. */
    PGMMMIO2HANDLE       hRom;

    /** The eventfd to wakeup the IRQ poller. */
    int                  iFdWakeup;
    /** The poll structure for the interrupts. */
    struct pollfd        aIrqFds[2];
    /** The current IRQ mode.. */
    uint8_t              uIrqModeCur;
    /** The new confogured IRQ mode.. */
    volatile uint8_t     uIrqModeNew;

    /** The interrupt polling thread. */
    PPDMTHREAD           pThrdIrq;

    /** The MSI capability offset if enabled. */
    uint8_t              offMsiCtrl;

    /** Flag whether the guest RAM was mapped to the IOMMU. */
    bool                 fGuestRamMapped;
} VFIOPCI;
/** Pointer to the raw PCI instance data. */
typedef VFIOPCI *PVFIOPCI;


#ifndef VBOX_DEVICE_STRUCT_TESTCASE

/**
 * eventfd2() syscall wrapper.
 *
 * @returns IPRT status code.
 * @param   uValInit            The initial value of the maintained counter.
 * @param   fFlags              Flags controlling the eventfd behavior.
 * @param   piFdEvt             Where to store the file descriptor of the eventfd object on success.
 */
DECLINLINE(int) pciVfioLnxEventfd2(uint32_t uValInit, uint32_t fFlags, int *piFdEvt)
{
    int rcLnx = syscall(LNX_SYSCALL_EVENTFD2, uValInit, fFlags);
    if (RT_UNLIKELY(rcLnx == -1))
        return RTErrConvertFromErrno(errno);

    *piFdEvt = rcLnx;
    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceReadU8(PVFIOPCI pThis, uint32_t offReg, uint8_t *pb)
{
    ssize_t cb = pread(pThis->iFdVfio, pb, 1, pThis->offPciCfg + offReg);
    if (cb != 1)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceReadU16(PVFIOPCI pThis, uint32_t offReg, uint16_t *pu16)
{
    ssize_t cb = pread(pThis->iFdVfio, pu16, 2, pThis->offPciCfg + offReg);
    if (cb != 2)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceReadU32(PVFIOPCI pThis, uint32_t offReg, uint32_t *pu32)
{
    ssize_t cb = pread(pThis->iFdVfio, pu32, 4, pThis->offPciCfg + offReg);
    if (cb != 4)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceReadU64(PVFIOPCI pThis, uint32_t offReg, uint64_t *pu64)
{
    ssize_t cb = pread(pThis->iFdVfio, pu64, 8, pThis->offPciCfg + offReg);
    if (cb != 8)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceWriteU8(PVFIOPCI pThis, uint32_t offReg, uint8_t u8)
{
    ssize_t cb = pwrite(pThis->iFdVfio, &u8, 1, pThis->offPciCfg + offReg);
    if (cb != 1)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceWriteU16(PVFIOPCI pThis, uint32_t offReg, uint16_t u16)
{
    ssize_t cb = pwrite(pThis->iFdVfio, &u16, 2, pThis->offPciCfg + offReg);
    if (cb != 2)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}

DECLINLINE(int) pciVfioCfgSpaceWriteU32(PVFIOPCI pThis, uint32_t offReg, uint32_t u32)
{
    ssize_t cb = pwrite(pThis->iFdVfio, &u32, 4, pThis->offPciCfg + offReg);
    if (cb != 4)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioCfgSpaceWriteU64(PVFIOPCI pThis, uint32_t offReg, uint64_t u64)
{
    ssize_t cb = pwrite(pThis->iFdVfio, &u64, 8, pThis->offPciCfg + offReg);
    if (cb != 8)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMIOPORTNEWOUT}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioPioWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT offPort, uint32_t u32, unsigned cb)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);
    PCVFIOPCIBAR pBar = (PCVFIOPCIBAR)pvUser;
    ssize_t cbWritten = pwrite(pThis->iFdVfio, &u32, cb, pBar->u.offPio + offPort);
    if (cbWritten != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMIOPORTNEWIN}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioPioRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT offPort, uint32_t *pu32, unsigned cb)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);
    PCVFIOPCIBAR pBar = (PCVFIOPCIBAR)pvUser;
    ssize_t cbRead = pread(pThis->iFdVfio, pu32, cb, pBar->u.offPio + offPort);
    if (cbRead != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMMMIONEWREAD}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioMmioRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS off, void *pv, unsigned cb)
{
    RT_NOREF(pDevIns);
    PCVFIOPCIBAR pBar = (PCVFIOPCIBAR)pvUser;

    Assert(pBar->bType == 2);
    volatile uint8_t *pb = (volatile uint8_t *)pBar->u.pvMmio + off;
    switch (cb)
    {
        case 1: *(uint8_t *)pv = *pb; break;
        case 2: *(uint16_t *)pv = *(volatile uint16_t *)pb; break;
        case 4: *(uint32_t *)pv = *(volatile uint32_t *)pb; break;
        case 8: *(uint64_t *)pv = *(volatile uint64_t *)pb; break;
        default:
            memcpy(pv, (const void *)pb, cb); /** @todo Not correct as memcpy and volatile doesn't mix well */
            break;
    }

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMMMIONEWWRITE}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioMmioWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS off, void const *pv, unsigned cb)
{
    RT_NOREF(pDevIns);
    PCVFIOPCIBAR pBar = (PCVFIOPCIBAR)pvUser;

    Assert(pBar->bType == 2);
    volatile uint8_t *pb = (volatile uint8_t *)pBar->u.pvMmio + off;
    switch (cb)
    {
        case 1: *(volatile uint8_t *)pb  = *(uint8_t const *)pv;  break;
        case 2: *(volatile uint16_t *)pb = *(uint16_t const *)pv; break;
        case 4: *(volatile uint32_t *)pb = *(uint32_t const *)pv; break;
        case 8: *(volatile uint64_t *)pb = *(uint64_t const *)pv; break;
        default:
            memcpy((void *)pb, pv, cb); /** @todo Not correct as memcpy and volatile doesn't mix well */
            break;
    }

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioQueryRegionInfo(PVFIOPCI pThis, uint32_t uRegion, struct vfio_region_info *pRegionInfo)
{
    RT_ZERO(*pRegionInfo);
    pRegionInfo->argsz = sizeof(*pRegionInfo);
    pRegionInfo->index = uRegion;

    int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_GET_REGION_INFO, pRegionInfo);
    if (rcLnx == -1)
        return PDMDevHlpVMSetError(pThis->pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                N_("Getting information for region %u of opened VFIO device failed with %d"), uRegion, errno);

    const int iInstance = pThis->iInstance;
    LogRel(("VFIO#%d: Region %u:\n"
            "VFIO#%d:     flags:       %#RX32\n"
            "VFIO#%d:     index:       %#RU32\n"
            "VFIO#%d:     cap_offset:  %#RX32\n"
            "VFIO#%d:     size:        %#RX64\n"
            "VFIO#%d:     offset:      %#RX64\n",
            iInstance, uRegion,
            iInstance, pRegionInfo->flags,
            iInstance, pRegionInfo->index,
            iInstance, pRegionInfo->cap_offset,
            iInstance, pRegionInfo->size,
            iInstance, pRegionInfo->offset));

    return VINF_SUCCESS;
}


static int pciVfioSetupBar(PVFIOPCI pThis, PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t uRegion, uint32_t uVfioRegion)
{
    struct vfio_region_info RegionInfo; RT_ZERO(RegionInfo);
    int rc = pciVfioQueryRegionInfo(pThis, uVfioRegion, &RegionInfo);
    if (RT_FAILURE(rc))
        return rc;

    if (   RegionInfo.flags
        && RegionInfo.size)
    {
        uint32_t u32PciBar;
        rc = pciVfioCfgSpaceReadU32(pThis, VBOX_PCI_BASE_ADDRESS_0 + (uRegion * sizeof(uint32_t)), &u32PciBar);
        if (RT_FAILURE(rc))
            return rc;

        if (u32PciBar & RT_BIT_32(0))
        {
            /* PIO. */
            pThis->aBars[uRegion].bType    = 1;
            pThis->aBars[uRegion].u.offPio = RegionInfo.offset;

            rc = PDMDevHlpPCIIORegionCreateIo(pDevIns, uRegion, RegionInfo.size,
                                              pciVfioPioWrite, pciVfioPioRead, &pThis->aBars[uRegion],
                                              "PIO", NULL /*paExtDescs*/, &pThis->aBars[uRegion].hnd.hIoPort);
            AssertRCReturn(rc, PDMDEV_SET_ERROR(pDevIns, rc, N_("Cannot register PCI I/O region")));
        }
        else
        {
            Assert(RegionInfo.flags & VFIO_REGION_INFO_FLAG_MMAP);
            int fProt =   ((RegionInfo.flags & VFIO_REGION_INFO_FLAG_READ)  ? PROT_READ  : 0)
                        | ((RegionInfo.flags & VFIO_REGION_INFO_FLAG_WRITE) ? PROT_WRITE : 0);
            pThis->aBars[uRegion].bType    = 2;
            pThis->aBars[uRegion].u.pvMmio = mmap(NULL, RegionInfo.size, fProt, MAP_FILE | MAP_SHARED, pThis->iFdVfio, RegionInfo.offset);
            if (pThis->aBars[uRegion].u.pvMmio == MAP_FAILED)
                return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                           N_("Mapping BAR%u at offset %#RX64 with size %RX64 failed with %d"),
                                           uRegion, RegionInfo.offset, RegionInfo.size, errno);

            uint32_t enmAddrSpace = PCI_ADDRESS_SPACE_MEM;
            if ((u32PciBar & (RT_BIT_32(2) | RT_BIT_32(1))) == PCI_ADDRESS_SPACE_BAR64)
                enmAddrSpace |= PCI_ADDRESS_SPACE_BAR64;
            if (u32PciBar & PCI_ADDRESS_SPACE_MEM_PREFETCH)
                enmAddrSpace |= PCI_ADDRESS_SPACE_MEM_PREFETCH;

            if (pThis->fInterceptMmio)
            {
                rc = PDMDevHlpMmioCreate(pDevIns, RegionInfo.size, pPciDev, uRegion /*iPciRegion*/,
                                         pciVfioMmioWrite, pciVfioMmioRead, &pThis->aBars[uRegion],
                                         IOMMMIO_FLAGS_READ_PASSTHRU | IOMMMIO_FLAGS_WRITE_PASSTHRU, "MMIO",
                                         &pThis->aBars[uRegion].hnd.hMmio);
                AssertLogRelRCReturn(rc, rc);

                rc = PDMDevHlpPCIIORegionRegisterMmioEx(pDevIns, pPciDev, uRegion, RegionInfo.size, (PCIADDRESSSPACE)enmAddrSpace,
                                                        pThis->aBars[uRegion].hnd.hMmio, NULL);
            }
            else
                rc = PDMDevHlpPCIIORegionCreateMmio2FromExisting(pDevIns, uRegion, RegionInfo.size,
                                                                 (PCIADDRESSSPACE)enmAddrSpace,
                                                                 "MMIO", (void *)pThis->aBars[uRegion].u.pvMmio,
                                                                 &pThis->aBars[uRegion].hnd.hMmio2);

            AssertLogRelRCReturn(rc, rc);
        }
    }
    else /* Not available. */
        Assert(RegionInfo.size == 0);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMIOPORTNEWOUT}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioVgaPioWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT offPort, uint32_t u32, unsigned cb)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);
    PCVFIOPCIBAR pBar = (PCVFIOPCIBAR)pvUser;
    ssize_t cbWritten = pwrite(pThis->iFdVfio, &u32, cb, pBar->u.offPio + offPort);
    if (cbWritten != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMIOPORTNEWIN}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioVgaPioRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT offPort, uint32_t *pu32, unsigned cb)
{
    RT_NOREF(pvUser);
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    ssize_t cbRead = pread(pThis->iFdVfio, pu32, cb, pThis->offVga + offPort);
    if (cbRead != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMMMIONEWREAD}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioVgaMmioRead(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS off, void *pv, unsigned cb)
{
    RT_NOREF(pvUser);
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    ssize_t cbRead = pread(pThis->iFdVfio, pv, cb, pThis->offVga + off);
    if (cbRead != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMMMIONEWWRITE}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioVgaMmioWrite(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS off, void const *pv, unsigned cb)
{
    RT_NOREF(pvUser);
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    ssize_t cbWritten = pwrite(pThis->iFdVfio, pv, cb, pThis->offVga + off);
    if (cbWritten != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNIOMMMIONEWFILL}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioVgaMmioFill(PPDMDEVINS pDevIns, void *pvUser, RTGCPHYS off, uint32_t u32Item, unsigned cbItem, unsigned cItems)
{
    RT_NOREF(pvUser);
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    uint8_t abVal[4] = { 0 };
    for (uint8_t i = 0; i < RT_ELEMENTS(abVal); i++)
    {
        abVal[i] = u32Item & 0xff;
        u32Item >>= 8;
    }

    ssize_t const cb = (ssize_t)cbItem * cItems;
    uint8_t *pb = (uint8_t *)RTMemTmpAlloc(cb);
    if (!pb)
        return VERR_NO_MEMORY;

    uint8_t *pbCur = pb;

    switch (cbItem)
    {
        case 1:
            for (uint32_t i = 0; i < cItems; i++)
                *pbCur++ = abVal[0];
            break;
        case 2:
            for (uint32_t i = 0; i < cItems; i++)
            {
                pbCur[0] = abVal[0];
                pbCur[1] = abVal[1];
                pbCur += 2;
            }
            break;
        case 4:
            for (uint32_t i = 0; i < cItems; i++)
            {
                pbCur[0] = abVal[0];
                pbCur[1] = abVal[1];
                pbCur[2] = abVal[2];
                pbCur[3] = abVal[3];
                pbCur += 4;
            }
            break;
        default:
            AssertFailedReturn(VERR_NOT_SUPPORTED);
    }

    ssize_t cbWritten = pwrite(pThis->iFdVfio, pb, cb, pThis->offVga + off);
    RTMemTmpFree(pb);

    if (cbWritten != cb)
        return RTErrConvertFromErrno(errno);

    return VINF_SUCCESS;
}


static int pciVfioSetupVga(PVFIOPCI pThis, PPDMDEVINS pDevIns)
{
    struct vfio_region_info RegionInfo; RT_ZERO(RegionInfo);
    int rc = pciVfioQueryRegionInfo(pThis, VFIO_PCI_VGA_REGION_INDEX, &RegionInfo);
    if (RT_FAILURE(rc))
        return rc;

    AssertLogRelMsgReturn(   RegionInfo.flags
                          && RegionInfo.size,
                          ("VGA/GPU does not support VFIO_PCI_VGA_REGION_INDEX\n"),
                          VERR_NOT_SUPPORTED);

    pThis->offVga = RegionInfo.offset;

    /* Register the legacy VGA I/O port and MMIO ranges. */
    rc = PDMDevHlpIoPortCreateFlagsAndMap(pDevIns, 0x3b0, 0x3bb - 0x3b0 + 1, IOM_IOPORT_F_ABS,
                                          pciVfioVgaPioWrite, pciVfioVgaPioRead, "VFIO VGA #1",
                                          NULL /*paExtDescs*/, &pThis->hVgaIoPort1);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc, "Mapping legacy VGA ports 0x3b0 - 0x3bb failed");

    rc = PDMDevHlpIoPortCreateFlagsAndMap(pDevIns, 0x3c0, 0x3df - 0x3c0 + 1, IOM_IOPORT_F_ABS,
                                          pciVfioVgaPioWrite, pciVfioVgaPioRead, "VFIO VGA #2",
                                          NULL /*paExtDescs*/, &pThis->hVgaIoPort2);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc, "Mapping legacy VGA ports 0x3b0 - 0x3bb failed");

    /*
     * The MDA/CGA/EGA/VGA/whatever fixed MMIO area.
     */
    rc = PDMDevHlpMmioCreateExAndMap(pDevIns, 0x000a0000, 0x00020000,
                                     IOMMMIO_FLAGS_READ_PASSTHRU | IOMMMIO_FLAGS_WRITE_PASSTHRU | IOMMMIO_FLAGS_ABS,
                                     NULL /*pPciDev*/, UINT32_MAX /*iPciRegion*/,
                                     pciVfioVgaMmioWrite, pciVfioVgaMmioRead, pciVfioVgaMmioFill, NULL /*pvUser*/,
                                     "VFIO VGA - VGA Video Buffer", &pThis->hVgaMmio);
    AssertRCReturn(rc, rc);

    return VINF_SUCCESS;
}


static int pciVfioSetupRom(PVFIOPCI pThis, PPDMDEVINS pDevIns)
{
    struct vfio_region_info RegionInfo; RT_ZERO(RegionInfo);
    int rc = pciVfioQueryRegionInfo(pThis, VFIO_PCI_ROM_REGION_INDEX, &RegionInfo);
    if (RT_FAILURE(rc))
        return rc;

    /* No ROM, nothing to do. */
    if (   RegionInfo.flags == 0
        && RegionInfo.size == 0)
        return VINF_SUCCESS;

    /** @todo Currently we will map the ROM as MMIO2 region as we lack the necessary
     * infrastructure to register ROMs for PCI BARs. This is wrong because MMIO2 regions
     * are mapped read/write. OTOH the guest can only trash the virtual ROM and break itself. */

    pThis->offRom = RegionInfo.offset;
    pThis->cbRom  = RegionInfo.size;

    rc = PDMDevHlpPCIIORegionCreateMmio2(pDevIns, VBOX_PCI_ROM_SLOT, pThis->cbRom,
                                         PCI_ADDRESS_SPACE_MEM_PREFETCH, "ROM",
                                         &pThis->pvRom, &pThis->hRom);
    AssertLogRelRCReturn(rc, PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                                 N_("Failed to allocate %zu bytes of ROM"), pThis->cbRom));

    ssize_t cbRead = pread(pThis->iFdVfio, pThis->pvRom, pThis->cbRom, pThis->offRom);
    if (cbRead != (ssize_t)pThis->cbRom)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Failed to read %zu bytes of ROM from the device"), pThis->cbRom);

    return VINF_SUCCESS;
}


/**
 * The IRQ poller thread.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance.
 * @param   pThread     The command thread.
 */
static DECLCALLBACK(int) pciVfioIrqPoller(PPDMDEVINS pDevIns, PPDMTHREAD pThread)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    if (pThread->enmState == PDMTHREADSTATE_INITIALIZING)
    {
        pThis->aIrqFds[0].fd = pThis->iFdWakeup;
        pThis->aIrqFds[0].events = POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | POLLERR;
        return VINF_SUCCESS;
    }

    uint32_t cIrqs = 0;
    while (pThread->enmState == PDMTHREADSTATE_RUNNING)
    {
        uint8_t uIrqModeNew = ASMAtomicReadU8(&pThis->uIrqModeNew);
        if (pThis->uIrqModeCur != uIrqModeNew)
        {
            switch (uIrqModeNew)
            {
                case VFIO_PCI_MSI_IRQ_INDEX:
                {
                    cIrqs = 1;
                    break;
                }
                case UINT8_MAX:
                {
                    cIrqs = 0;
                    break;
                }
                default:
                    AssertReleaseFailed();
            }

            pThis->uIrqModeCur = uIrqModeNew;
        }

        int rcPsx = poll(&pThis->aIrqFds[0], 1 + cIrqs, -1);
        if (rcPsx > 0)
        {
            if (pThis->aIrqFds[0].revents)
            {
                /* We got woken up externally. */
                pThis->aIrqFds[0].revents = 0;
                uint64_t u64;
                ssize_t cb = read(pThis->aIrqFds[0].fd, &u64, sizeof(u64));
                Assert(cb == sizeof(u64)); RT_NOREF(cb);
                if (pThread->enmState != PDMTHREADSTATE_RUNNING)
                    break;
            }

            for (uint32_t i = 1; i < RT_ELEMENTS(pThis->aIrqFds); i++)
            {
                if (pThis->aIrqFds[i].revents)
                {
                    pThis->aIrqFds[i].revents = 0;
                    uint64_t u64;
                    ssize_t cb = read(pThis->aIrqFds[i].fd, &u64, sizeof(u64));
                    Assert(cb == sizeof(u64)); RT_NOREF(cb);

                    PDMDevHlpPCISetIrq(pDevIns, 0, 1);

                    /** @todo The interrupt seems to be masked and we would need a mechanism
                     * to get notified when the interrupt is de-asserted in the interrupt controller
                     * (through an EOI for example) so we can unmask them. With KVM this is supported
                     * when KVM_CAP_IRQFD_RESAMPLE is available.
                     */
#if 0
                    union
                    {
                        struct vfio_irq_set IrqSet;
                        uint32_t            au32[sizeof(struct vfio_irq_set) / sizeof(uint32_t) + 1];
                    } uBuf;

                    uBuf.IrqSet.argsz = sizeof(uBuf);
                    uBuf.IrqSet.flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_UNMASK;
                    uBuf.IrqSet.index = i;
                    uBuf.IrqSet.start = 0;
                    uBuf.IrqSet.count = 1;
                    uBuf.au32[sizeof(struct vfio_irq_set) / sizeof(uint32_t)] = pThis->aIrqFds[i].fd;

                    int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_SET_IRQS, &uBuf);
                    if (rcLnx == -1)
                        LogRel(("Unmasking one INTX interrupt failed with %d", errno));
#endif
                }
            }
        }
        else
            AssertFailed();
    }

    LogFlowFunc(("Poller thread terminating\n"));
    return VINF_SUCCESS;
}


/**
 * Wakes up the IRQ poller to respond to state changes.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance.
 * @param   pThread     The command thread.
 */
static DECLCALLBACK(int) pciVfioIrqPollerWakeup(PPDMDEVINS pDevIns, PPDMTHREAD pThread)
{
    RT_NOREF(pThread);
    Log4Func(("\n"));
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    uint64_t u64 = 1;
    ssize_t cb = write(pThis->iFdWakeup, &u64, sizeof(u64));
    Assert(cb == sizeof(u64)); RT_NOREF(cb);

    return VINF_SUCCESS;
}


DECLINLINE(int) pciVfioQueryIrqInfo(PVFIOPCI pThis, uint32_t uIrq, struct vfio_irq_info *pIrqInfo)
{
    RT_ZERO(*pIrqInfo);
    pIrqInfo->argsz = sizeof(*pIrqInfo);
    pIrqInfo->index = uIrq;

    int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_GET_IRQ_INFO, pIrqInfo);
    if (rcLnx == -1)
        return PDMDevHlpVMSetError(pThis->pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                N_("Getting information for irq %u of opened VFIO device failed with %d"), uIrq, errno);

    const int iInstance = pThis->iInstance;
    LogRel(("VFIO#%d: Irq %u:\n"
            "VFIO#%d:     flags:       %#RX32\n"
            "VFIO#%d:     index:       %#RU32\n"
            "VFIO#%d:     count:       %#RU32\n",
            iInstance, uIrq,
            iInstance, pIrqInfo->flags,
            iInstance, pIrqInfo->index,
            iInstance, pIrqInfo->count));

    return VINF_SUCCESS;
}


static int pciVfioSetupIrq(PVFIOPCI pThis, PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev, uint32_t uVfioIrq)
{
    RT_NOREF(pDevIns, pPciDev);
    struct vfio_irq_info IrqInfo; RT_ZERO(IrqInfo);
    int rc = pciVfioQueryIrqInfo(pThis, uVfioIrq, &IrqInfo);
    if (RT_FAILURE(rc))
        return rc;

    if (IrqInfo.count)
    {
        if (uVfioIrq == VFIO_PCI_INTX_IRQ_INDEX)
        {
            Assert(   IrqInfo.index == 0
                   && IrqInfo.count == 1
                   && (IrqInfo.flags & VFIO_IRQ_INFO_EVENTFD));

            /* Create an assign an eventfd. */
            int iFdEvt = 0;
            rc = pciVfioLnxEventfd2(0 /*uValInit*/, 0 /*fFlags*/, &iFdEvt);
            if (RT_FAILURE(rc))
                return rc;

            union
            {
                struct vfio_irq_set IrqSet;
                uint32_t            au32[sizeof(struct vfio_irq_set) / sizeof(uint32_t) + 1];
            } uBuf;

            uBuf.IrqSet.argsz = sizeof(uBuf);
            uBuf.IrqSet.flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
            uBuf.IrqSet.index = 0;
            uBuf.IrqSet.start = 0;
            uBuf.IrqSet.count = 1;
            uBuf.au32[sizeof(struct vfio_irq_set) / sizeof(uint32_t)] = iFdEvt;

            int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_SET_IRQS, &uBuf);
            if (rcLnx == -1)
                return PDMDevHlpVMSetError(pThis->pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                        N_("Assigning one INTX interrupt failed with %d (%u)"), errno, sizeof(uBuf));

            pThis->aIrqFds[0].fd     = iFdEvt;
            pThis->aIrqFds[0].events = POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | POLLERR;
        }
        else if (uVfioIrq == VFIO_PCI_MSI_IRQ_INDEX)
        {
            Assert(   IrqInfo.index == 1
                   && IrqInfo.count == 1
                   && (IrqInfo.flags & VFIO_IRQ_INFO_EVENTFD));

            /* Create an assign an eventfd. */
            int iFdEvt = 0;
            rc = pciVfioLnxEventfd2(0 /*uValInit*/, 0 /*fFlags*/, &iFdEvt);
            if (RT_FAILURE(rc))
                return rc;

            union
            {
                struct vfio_irq_set IrqSet;
                uint32_t            au32[sizeof(struct vfio_irq_set) / sizeof(uint32_t) + 1];
            } uBuf;

            uBuf.IrqSet.argsz = sizeof(uBuf);
            uBuf.IrqSet.flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
            uBuf.IrqSet.index = VFIO_PCI_MSI_IRQ_INDEX;
            uBuf.IrqSet.start = 0;
            uBuf.IrqSet.count = 1;
            uBuf.au32[sizeof(struct vfio_irq_set) / sizeof(uint32_t)] = iFdEvt;

            int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_SET_IRQS, &uBuf);
            if (rcLnx == -1)
                return PDMDevHlpVMSetError(pThis->pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                        N_("Assigning one INTX interrupt failed with %d (%u)"), errno, sizeof(uBuf));

            pThis->aIrqFds[1].fd     = iFdEvt;
            pThis->aIrqFds[1].events = POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | POLLERR;
        }
        /** @todo MSI-X */
    }
    else /* Not available. */
        Assert(IrqInfo.count == 0);

    /* Wakeup the IRQ poller to make up the new mode. */
    ASMAtomicWriteU8(&pThis->uIrqModeNew, uVfioIrq);
    pciVfioIrqPollerWakeup(pDevIns, pThis->pThrdIrq);
    return VINF_SUCCESS;
}


DECLINLINE(void) pciVfioCfgSpaceSetInterceptU8(PVFIOPCI pThis, uint32_t off, uint8_t fRd, uint8_t fWr)
{
    AssertReturnVoid(off < sizeof(pThis->abPciCfgIntercept));
    uint32_t offByte = off >> 1;
    uint8_t  cShift  = (off & 0x1) ? 4 : 0;

    pThis->abPciCfgIntercept[offByte] |= ((fWr << 2) | fRd) << cShift;
}


DECLINLINE(void) pciVfioCfgSpaceSetInterceptRoU8(PVFIOPCI pThis, uint32_t off, uint8_t fRd)
{
    pciVfioCfgSpaceSetInterceptU8(pThis, off,     fRd, VFIO_PCI_CFG_SPACE_ACCESS_INVALID);
}


DECLINLINE(void) pciVfioCfgSpaceSetInterceptU16(PVFIOPCI pThis, uint32_t off, uint8_t fRd, uint8_t fWr)
{
    AssertReturnVoid(off < sizeof(pThis->abPciCfgIntercept) - 1);
    pciVfioCfgSpaceSetInterceptU8(pThis, off,     fRd, fWr);
    pciVfioCfgSpaceSetInterceptU8(pThis, off + 1, fRd, fWr);
}


DECLINLINE(void) pciVfioCfgSpaceSetInterceptRoU16(PVFIOPCI pThis, uint32_t off, uint8_t fRd)
{
    AssertReturnVoid(off < sizeof(pThis->abPciCfgIntercept) - 1);
    pciVfioCfgSpaceSetInterceptU8(pThis, off,     fRd, VFIO_PCI_CFG_SPACE_ACCESS_INVALID);
    pciVfioCfgSpaceSetInterceptU8(pThis, off + 1, fRd, VFIO_PCI_CFG_SPACE_ACCESS_INVALID);
}


DECLINLINE(void) pciVfioCfgSpaceSetInterceptU32(PVFIOPCI pThis, uint32_t off, uint8_t fRd, uint8_t fWr)
{
    AssertReturnVoid(off < sizeof(pThis->abPciCfgIntercept) - 3);
    pciVfioCfgSpaceSetInterceptU16(pThis, off,     fRd, fWr);
    pciVfioCfgSpaceSetInterceptU16(pThis, off + 2, fRd, fWr);
}


DECLINLINE(uint8_t) pciVfioCfgSpaceGetInterceptRd(PVFIOPCI pThis, uint32_t off)
{
    AssertReturn(off < sizeof(pThis->abPciCfgIntercept), VFIO_PCI_CFG_SPACE_ACCESS_INVALID);

    uint32_t offByte = off >> 1;
    uint8_t  cShift  = (off & 0x1) ? 4 : 0;

    return (pThis->abPciCfgIntercept[offByte] >> cShift) & 0x3;
}


DECLINLINE(uint8_t) pciVfioCfgSpaceGetInterceptWr(PVFIOPCI pThis, uint32_t off)
{
    AssertReturn(off < sizeof(pThis->abPciCfgIntercept), VFIO_PCI_CFG_SPACE_ACCESS_INVALID);

    uint32_t offByte = off >> 1;
    uint8_t  cShift  = (off & 0x1) ? 4 + 2 : 2;

    return (pThis->abPciCfgIntercept[offByte] >> cShift) & 0x3;
}


static int pciVfioCfgSpaceParseCapabilities(PVFIOPCI pThis, PPDMPCIDEV pPciDev)
{
    PPDMDEVINS pDevIns = pThis->pDevIns;

    /*
     * Try building a 1:1 mapping of the capabilities, punching holes for capabilities
     * currently not being supported.
     */
    uint8_t offCap = 0;
    int rc = pciVfioCfgSpaceReadU8(pThis, VBOX_PCI_CAPABILITY_LIST, &offCap);
    if (RT_FAILURE(rc))
        return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                   N_("Failed to read capabilities list pointer with %Rrc"), rc);

    pciVfioCfgSpaceSetInterceptRoU8(pThis, VBOX_PCI_CAPABILITY_LIST, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT);
    if (!offCap)
    {
        /* No capabilities, return early. */
        PDMPciDevSetCapabilityList(pPciDev, 0x00);
        return VINF_SUCCESS;
    }

    /* Initialize with 0. */
    PDMPciDevSetCapabilityList(pPciDev, 0);

    /* This ASSUMES that the cpabilities are not going backwards when pointing to the next one. */
    uint8_t offCapNextPrev = VBOX_PCI_CAPABILITY_LIST;
    for (;;)
    {
        uint8_t bCapId = 0;
        rc = pciVfioCfgSpaceReadU8(pThis, offCap, &bCapId);
        if (RT_FAILURE(rc))
            return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                       N_("Failed to read capabilitiy ID at offset %#x with %Rrc"), offCap, rc);

        uint8_t offCapNext = 0;
        rc = pciVfioCfgSpaceReadU8(pThis, offCap + 1, &offCapNext);
        if (RT_FAILURE(rc))
            return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                       N_("Failed to read next capability pointer at offset %#x with %Rrc"), offCap, rc);

        uint8_t cbCap = 2;
        bool    fSupported = false;
        switch (bCapId)
        {
            case VBOX_PCI_CAP_ID_PM:
            {
                LogRel(("VFIO#%d: Cap[%#x]: PCI Power Management Interface -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_AGP:
            {
                LogRel(("VFIO#%d: Cap[%#x]: AGP -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_VPD:
            {
                LogRel(("VFIO#%d: Cap[%#x]: VPD -> passthrough\n", pThis->iInstance, offCap, bCapId));
                pciVfioCfgSpaceSetInterceptU16(pThis, offCap + 2, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH);
                pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 4, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH);
                cbCap      = 2 + 2 + 4;
                fSupported = true;
                break;
            }
            case VBOX_PCI_CAP_ID_SLOTID:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Slot Identification -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_MSI:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Message Signaled Interrupts -> emulate\n", pThis->iInstance, offCap));
                fSupported = true;

                /* Read the message control from the device. */
                uint16_t u16Mmc = 0;
                rc = pciVfioCfgSpaceReadU16(pThis, offCap + 2, &u16Mmc);
                if (RT_FAILURE(rc))
                    return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                               N_("Failed to read MSI message control register with %Rrc"), rc);

                bool f64Bit = RT_BOOL(u16Mmc & VBOX_PCI_MSI_FLAGS_64BIT);
                uint8_t cVectors = RT_BIT((u16Mmc & VBOX_PCI_MSI_FLAGS_QMASK) >> 1);

                /* Add capability. */
                PDMMSIREG MsiReg;
                RT_ZERO(MsiReg);
                MsiReg.cMsiVectors     = cVectors;
                MsiReg.iMsiCapOffset   = offCap;
                MsiReg.iMsiNextOffset  = 0; /* Gets updated later. */
                MsiReg.fMsi64bit       = f64Bit;
                rc = PDMDevHlpPCIRegisterMsi(pDevIns, &MsiReg);
                if (RT_FAILURE(rc))
                    AssertFailed();

                pciVfioCfgSpaceSetInterceptU16(pThis, offCap +  2, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_EMULATE);    /* Message Control */
                pciVfioCfgSpaceSetInterceptU32(pThis, offCap +  4, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Message Address [Low] */
                if (f64Bit)
                {
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap +  8, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Message Address High */
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 12, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Message Data */

                    /* We always implement MSI with per-vector masking support. */
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 16, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Mask */
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 20, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Pending */
                }
                else
                {
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 8, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Message Data */

                    /* We always implement MSI with per-vector masking support. */
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 12, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Mask */
                    pciVfioCfgSpaceSetInterceptU32(pThis, offCap + 16, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT); /* Pending */
                }

                pThis->offMsiCtrl = offCap + 2;
                break;
            }
            case VBOX_PCI_CAP_ID_CHSWP:
            {
                LogRel(("VFIO#%d: Cap[%#x]: CompactPCI Hot Swap -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_PCIX:
            {
                LogRel(("VFIO#%d: Cap[%#x]: PCI-X -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_HT:
            {
                LogRel(("VFIO#%d: Cap[%#x]: HyperTransport -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_VNDR:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Vendor Specific -> passthrough\n", pThis->iInstance, offCap));

                /* The next byte after the header is a length field. */
                uint8_t cbVendor = 0;
                rc = pciVfioCfgSpaceReadU8(pThis, offCap + 2, &cbVendor);
                if (RT_FAILURE(rc))
                    return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                               N_("Failed to read vendor length field at offset %#x with %Rrc"), offCap + 2, rc);
                if (cbVendor < 2 || cbVendor > 256 - offCap)
                    return PDMDevHlpVMSetError(pDevIns, VERR_BUFFER_OVERFLOW, RT_SRC_POS,
                                               N_("Invalid vendor length field %#x"), cbVendor);

                for (uint8_t i = 0; i < cbVendor - 2; i++)
                    pciVfioCfgSpaceSetInterceptU8(pThis, offCap + 2 + i, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH);

                cbCap      = cbVendor;
                fSupported = true;
                break;
            }
            case VBOX_PCI_CAP_ID_DBG:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Debug port -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_CCRC:
            {
                LogRel(("VFIO#%d: Cap[%#x]: CompactPCI central resource control -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_SHPC:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Standard PCI Hot-Plug Controller -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_SSVID:
            {
                LogRel(("VFIO#%d: Cap[%#x]: SPCI Bridge Subsystem Vendor ID -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_AGP3:
            {
                LogRel(("VFIO#%d: Cap[%#x]: AGP 8x -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_SECURE:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Secure Device -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_EXP:
            {
                LogRel(("VFIO#%d: Cap[%#x]: PCI Express -> emulate\n", pThis->iInstance, offCap));
                //fSupported = true;
                break;
            }
            case VBOX_PCI_CAP_ID_MSIX:
            {
                LogRel(("VFIO#%d: Cap[%#x]: MSI-X -> unsupported\n", pThis->iInstance, offCap));
                /** @todo */
                break;
            }
            case VBOX_PCI_CAP_ID_SATA:
            {
                LogRel(("VFIO#%d: Cap[%#x]: Serial ATA HBA -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            case VBOX_PCI_CAP_ID_AF:
            {
                LogRel(("VFIO#%d: Cap[%#x]: PCI Advanced Features -> unsupported\n", pThis->iInstance, offCap));
                break;
            }
            default:
                LogRel(("VFIO#%d: Cap[%#x]: Unknown capability ID %#x encountered -> unsupported\n", pThis->iInstance, offCap, bCapId));
                break;
        }

        if (fSupported)
        {
            /* Accesses to capability ID and pointer to the next capability are never passed through. */
            pciVfioCfgSpaceSetInterceptRoU16(pThis, offCap, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT);
            PDMPciDevSetByte(pPciDev, offCap, bCapId);
            PDMPciDevSetByte(pPciDev, offCapNextPrev, offCap);
            offCapNextPrev = offCap + 1;
        }

        if (!offCapNext)
            break;

        if (   offCapNext < offCap
            || offCapNext < offCap + cbCap)
            return PDMDevHlpVMSetError(pDevIns, VERR_INVALID_STATE, RT_SRC_POS,
                                       N_("Next capability pointer points backwards or inside the current capability (next offset %#x )"), offCapNext);

        offCap = offCapNext;
    }

    /* Mark the end of the list. */
    PDMPciDevSetByte(pPciDev, offCapNextPrev, 0);

    return VINF_SUCCESS;
}


/**
 * The descriptors for the standard PCI config space.
 */
static const struct
{
    uint8_t offReg;
    uint8_t cbReg;
    bool    fInitFromDev;
    uint8_t fRd;
    uint8_t fWr;
} s_aCfgSpaceDesc[] =
{
    { VBOX_PCI_VENDOR_ID,           sizeof(uint16_t), true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_DEVICE_ID,           sizeof(uint16_t), true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_COMMAND,             sizeof(uint16_t), true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_EMULATE     },
    { VBOX_PCI_STATUS,              sizeof(uint16_t), true,  VFIO_PCI_CFG_SPACE_ACCESS_EMULATE,     VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH },
    { VBOX_PCI_REVISION_ID,         sizeof(uint8_t),  true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_CLASS_PROG,          sizeof(uint8_t),  true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_CLASS_SUB,           sizeof(uint8_t),  true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_CLASS_BASE,          sizeof(uint8_t),  true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_CACHE_LINE_SIZE,     sizeof(uint8_t),  false, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH },
    { VBOX_PCI_LATENCY_TIMER,       sizeof(uint8_t),  false, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH },
    { VBOX_PCI_HEADER_TYPE,         sizeof(uint8_t),  false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_BIST,                sizeof(uint8_t),  false, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH, VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH },
    { VBOX_PCI_BASE_ADDRESS_0,      sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_BASE_ADDRESS_1,      sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_BASE_ADDRESS_2,      sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_BASE_ADDRESS_3,      sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_BASE_ADDRESS_4,      sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_BASE_ADDRESS_5,      sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_CARDBUS_CIS,         sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_SUBSYSTEM_VENDOR_ID, sizeof(uint16_t), true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_SUBSYSTEM_ID,        sizeof(uint16_t), true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_ROM_ADDRESS,         sizeof(uint32_t), false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_INTERRUPT_LINE,      sizeof(uint8_t),  false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT  },
    { VBOX_PCI_INTERRUPT_PIN,       sizeof(uint8_t),  false, VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_MIN_GNT,             sizeof(uint8_t),  true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
    { VBOX_PCI_MAX_LAT,             sizeof(uint8_t),  true,  VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT,  VFIO_PCI_CFG_SPACE_ACCESS_INVALID     },
};

static int pciVfioCfgSpaceSetup(PVFIOPCI pThis, PPDMPCIDEV pPciDev)
{
    for (uint32_t i = 0; i < RT_ELEMENTS(s_aCfgSpaceDesc); i++)
    {
        switch (s_aCfgSpaceDesc[i].cbReg)
        {
            case 1:
            {
                if (s_aCfgSpaceDesc[i].fInitFromDev)
                {
                    uint8_t u8;
                    int rc = pciVfioCfgSpaceReadU8(pThis, s_aCfgSpaceDesc[i].offReg, &u8);
                    if (RT_FAILURE(rc)) return rc;
                    PDMPciDevSetByte(pPciDev, s_aCfgSpaceDesc[i].offReg, u8);
                }
                pciVfioCfgSpaceSetInterceptU8(pThis, s_aCfgSpaceDesc[i].offReg, s_aCfgSpaceDesc[i].fRd, s_aCfgSpaceDesc[i].fWr);
                break;
            }
            case 2:
            {
                if (s_aCfgSpaceDesc[i].fInitFromDev)
                {
                    uint16_t u16;
                    int rc = pciVfioCfgSpaceReadU16(pThis, s_aCfgSpaceDesc[i].offReg, &u16);
                    if (RT_FAILURE(rc)) return rc;
                    PDMPciDevSetWord(pPciDev, s_aCfgSpaceDesc[i].offReg, u16);
                }
                pciVfioCfgSpaceSetInterceptU16(pThis, s_aCfgSpaceDesc[i].offReg, s_aCfgSpaceDesc[i].fRd, s_aCfgSpaceDesc[i].fWr);
                break;
            }
            case 4:
            {
                if (s_aCfgSpaceDesc[i].fInitFromDev)
                {
                    uint32_t u32;
                    int rc = pciVfioCfgSpaceReadU32(pThis, s_aCfgSpaceDesc[i].offReg, &u32);
                    if (RT_FAILURE(rc)) return rc;
                    PDMPciDevSetDWord(pPciDev, s_aCfgSpaceDesc[i].offReg, u32);
                }
                pciVfioCfgSpaceSetInterceptU32(pThis, s_aCfgSpaceDesc[i].offReg, s_aCfgSpaceDesc[i].fRd, s_aCfgSpaceDesc[i].fWr);
                break;
            }
            default:
                AssertReleaseFailed();
        }
    }

    return VINF_SUCCESS;
}


static int pciVfioMapRegion(PVFIOPCI pThis, RTGCPHYS GCPhysStart, uintptr_t uPtrMapping, size_t cbMapping)
{
    struct iommu_ioas_map Map;
    Map.size       = sizeof(Map);
    Map.flags      = IOMMU_IOAS_MAP_FIXED_IOVA | IOMMU_IOAS_MAP_WRITEABLE | IOMMU_IOAS_MAP_READABLE;
    Map.ioas_id    = pThis->idIommuHwpt;
    Map.__reserved = 0;
    Map.user_va    = uPtrMapping;
    Map.length     = cbMapping;
    Map.iova       = GCPhysStart;

    int rcLnx = ioctl(pThis->iFdIommu, IOMMU_IOAS_MAP, &Map);
    if (rcLnx == -1)
    {
        LogRel(("errno=%d\n", errno));
        return RTErrConvertFromErrno(errno);
    }

    return VINF_SUCCESS;
}


static int pciVfioIommuGuestRamMap(PVFIOPCI pThis, PPDMDEVINS pDevIns)
{
    if (!pThis->fGuestRamMapped)
    {
        /** @todo This is a really gross hack because we currently lack
         * a dedicated interface to get knowledge about guest RAM mappings.
         * This might also return mappings for stuff not being guest RAM.
         */
        RTGCPHYS  GCPhysStart = 0;
        uintptr_t uPtrMapping = 0;
        size_t    cbMapping   = 0;
        for (RTGCPHYS GCPhys = 0; GCPhys < 10 * _1G64; GCPhys += _4K)
        {
            void *pv = NULL;
            PGMPAGEMAPLOCK Lock;
            int rc = PDMDevHlpPhysGCPhys2CCPtr(pDevIns, GCPhys, 0 /*fFlags*/, &pv, &Lock);
            if (RT_SUCCESS(rc))
            {
                if (cbMapping)
                {
                    if (uPtrMapping + cbMapping == (uintptr_t)pv)
                        cbMapping += _4K;
                    else
                    {
                        rc = pciVfioMapRegion(pThis, GCPhysStart, uPtrMapping, cbMapping);
                        if (RT_FAILURE(rc))
                            LogRel(("Mapping %RGp/%zu failed with %Rrc\n", GCPhysStart, cbMapping, rc));

                        GCPhysStart = GCPhys;
                        uPtrMapping = (uintptr_t)pv;
                        cbMapping = _4K;
                    }
                }
                else
                {
                    Assert(!uPtrMapping);
                    GCPhysStart = GCPhys;
                    uPtrMapping = (uintptr_t)pv;
                    cbMapping = _4K;
                }
                PDMDevHlpPhysReleasePageMappingLock(pDevIns, &Lock);
            }
            else if (cbMapping)
            {
                LogRel(("PDMDevHlpPhysGCPhys2CCPtr(,%RGp) -> %Rrc\n", GCPhys, rc));
                /* Map what we currently have. */
                Assert(uPtrMapping);
                rc = pciVfioMapRegion(pThis, GCPhysStart, uPtrMapping, cbMapping);
                if (RT_FAILURE(rc))
                    LogRel(("Mapping %RGp/%zu failed with %Rrc\n", GCPhysStart, cbMapping, rc));
                cbMapping   = 0;
                uPtrMapping = 0;
                GCPhysStart = GCPhys;
            }
        }

        if (cbMapping)
        {
            /* Map what we currently have. */
            Assert(uPtrMapping);
            int rc = pciVfioMapRegion(pThis, GCPhysStart, uPtrMapping, cbMapping);
            if (RT_FAILURE(rc))
                LogRel(("Mapping %RGp/%zu failed with %Rrc\n", GCPhysStart, cbMapping, rc));
            cbMapping   = 0;
            uPtrMapping = 0;
        }

        pThis->fGuestRamMapped = true;
    }

    return VINF_SUCCESS;
}


static int pciVfioConfigPassthroughRead(PVFIOPCI pThis, uint32_t uAddress, unsigned cb, uint32_t *pu32Value)
{
    switch (cb)
    {
        case 1:
        {
            uint8_t u8;
            int rc = pciVfioCfgSpaceReadU8(pThis, uAddress, &u8);
            if (RT_FAILURE(rc)) return rc;
            *pu32Value = u8;
            break;
        }
        case 2:
        {
            uint16_t u16;
            int rc = pciVfioCfgSpaceReadU16(pThis, uAddress, &u16);
            if (RT_FAILURE(rc)) return rc;
            *pu32Value = u16;
            break;
        }
        case 4:
        {
            uint32_t u32;
            int rc = pciVfioCfgSpaceReadU32(pThis, uAddress, &u32);
            if (RT_FAILURE(rc)) return rc;
            *pu32Value = u32;
            break;
        }
        default:
            AssertFailedReturn(VERR_INVALID_PARAMETER);
    }

    return VINF_SUCCESS;
}


static int pciVfioConfigPassthroughWrite(PVFIOPCI pThis, uint32_t uAddress, unsigned cb, uint32_t u32Value)
{
    switch (cb)
    {
        case 1:
        {
            int rc = pciVfioCfgSpaceWriteU8(pThis, uAddress, (uint8_t)u32Value);
            if (RT_FAILURE(rc)) return rc;
            break;
        }
        case 2:
        {
            int rc = pciVfioCfgSpaceWriteU16(pThis, uAddress, (uint16_t)u32Value);
            if (RT_FAILURE(rc)) return rc;
            break;
        }
        case 4:
        {
            int rc = pciVfioCfgSpaceWriteU32(pThis, uAddress, (uint32_t)u32Value);
            if (RT_FAILURE(rc)) return rc;
            break;
        }
        default:
            AssertFailedReturn(VERR_INVALID_PARAMETER);
    }

    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNPCICONFIGREAD}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioConfigRead(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev,
                                                    uint32_t uAddress, unsigned cb, uint32_t *pu32Value)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    uint32_t u32 = 0;
    for (uint8_t i = 0; i < cb; i++)
    {
        uint32_t bRead = 0;
        uint8_t fRd = pciVfioCfgSpaceGetInterceptRd(pThis, uAddress + i);
        switch (fRd)
        {
            case VFIO_PCI_CFG_SPACE_ACCESS_INVALID:
            {
                Log(("VFIO#%d: Invalid PCI config read at offset %#x, returning 0\n", pThis->iInstance, uAddress + i));
                break;
            }
            case VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH:
            {
                int rc = pciVfioConfigPassthroughRead(pThis, uAddress + i, 1, &bRead);
                if (RT_FAILURE(rc))
                LogRel(("VFIO#%d: Failed to passthrough PCI config read at offset %#x with %Rrc, returning 0\n",
                        pThis->iInstance, uAddress + i, rc));
                break;
            }
            case VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT:
            {
                bRead = PDMPciDevGetByte(pPciDev, uAddress + i);
                break;
            }
            case VFIO_PCI_CFG_SPACE_ACCESS_EMULATE:
            {
                switch (uAddress + i)
                {
                    case VBOX_PCI_STATUS:
                    case VBOX_PCI_STATUS + 1:
                    {
                        uint16_t u16;
                        int rc = pciVfioCfgSpaceReadU16(pThis, VBOX_PCI_STATUS, &u16);
                        if (RT_SUCCESS(rc))
                        {
                            /* Reflect proper capabilities bit. */
                            uint8_t offCap = PDMPciDevGetCapabilityList(pPciDev);
                            if (offCap)
                                u16 |= RT_BIT(4);
                            else
                                u16 &= ~RT_BIT(4);
                            bRead = (u16 >> ((uAddress + i) == VBOX_PCI_STATUS ? 0 : 8)) & 0xff;
                        }
                        break;
                    }
                    default:
                        AssertReleaseFailed();
                }
                break;
            }
            default:
                AssertReleaseFailed();
        }

        u32 |= (uint32_t)bRead << (i * 8);
    }

    *pu32Value = u32;
    return VINF_SUCCESS;
}


/**
 * @callback_method_impl{FNPCICONFIGWRITE}
 */
static DECLCALLBACK(VBOXSTRICTRC) pciVfioConfigWrite(PPDMDEVINS pDevIns, PPDMPCIDEV pPciDev,
                                                     uint32_t uAddress, unsigned cb, uint32_t u32Value)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    /** @todo Only handles accesses with the same intercept config for now. */
    uint8_t fWr = pciVfioCfgSpaceGetInterceptWr(pThis, uAddress);
    for (uint8_t i = 1; i < cb; i++)
    {
        uint8_t fWrU8 = pciVfioCfgSpaceGetInterceptWr(pThis, uAddress + i);
        if (fWr != fWrU8)
        {
            LogRel(("VFIO#%d: Complicated PCI config space write at offset %#x (+ %u) intercept %u vs %u\n",
                    pThis->iInstance, uAddress, i, fWr, fWrU8));
            return VINF_SUCCESS;
        }
    }

    int rc = VINF_SUCCESS;
    switch (fWr)
    {
        case VFIO_PCI_CFG_SPACE_ACCESS_INVALID:
        {
            LogRel(("VFIO#%d: Invalid PCI config write at offset %#x, ignoring\n",
                    pThis->iInstance, uAddress));
            break;
        }
        case VFIO_PCI_CFG_SPACE_ACCESS_PASSTHROUGH:
        {
            rc = pciVfioConfigPassthroughWrite(pThis, uAddress, cb, u32Value);
            break;
        }
        case VFIO_PCI_CFG_SPACE_ACCESS_DO_DEFAULT:
            return VINF_PDM_PCI_DO_DEFAULT;
        case VFIO_PCI_CFG_SPACE_ACCESS_EMULATE:
        {
            /* Map all the guest memory into the IOMMU as soon as the BUSMASTER bit is enabled. */
            if (uAddress == VBOX_PCI_COMMAND)
            {
                if (u32Value & RT_BIT(2))
                {
                    rc = pciVfioIommuGuestRamMap(pThis, pDevIns);
                    if (RT_FAILURE(rc))
                        LogRel(("VFIO#%d: Failed to map guest RAM into IOMMU for device, expect broken device (%Rrc)\n",
                                pThis->iInstance, rc));
                }
                /* Now write to the device. */
                rc = pciVfioConfigPassthroughWrite(pThis, uAddress, cb, u32Value);
                if (RT_FAILURE(rc))
                    LogRel(("VFIO#%d: Failed to update command register in device (%Rrc)", pThis->iInstance, rc));
                rc = VINF_PDM_PCI_DO_DEFAULT; /* Need to update BAR mappings. */
            }
            else if (   pThis->offMsiCtrl == uAddress
                     && cb == sizeof(uint16_t))
            {
                if (u32Value & RT_BIT(0)) /* Configure MSI interrupts as soon as they get enabled. */
                {
                    rc = pciVfioSetupIrq(pThis, pDevIns, pPciDev, VFIO_PCI_MSI_IRQ_INDEX);
                    if (RT_FAILURE(rc))
                        return rc;
                }
                /** @todo Disable MSI */

                /* Now write to the device. */
                rc = pciVfioConfigPassthroughWrite(pThis, uAddress, cb, u32Value);
                if (RT_FAILURE(rc))
                    LogRel(("VFIO#%d: Failed to update MSI control register in device (%Rrc)", pThis->iInstance, rc));
                rc = VINF_PDM_PCI_DO_DEFAULT; /* Need to update our internal MSI state. */
            }
            else
                AssertFailed();
            break;
        }
        default:
            AssertReleaseFailed();
    }

    return rc;
}


/**
 * @interface_method_impl{PDMDEVREG,pfnReset}
 */
static DECLCALLBACK(void) pciVfioReset(PPDMDEVINS pDevIns)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_RESET, NULL);
    AssertLogRelMsg(!rcLnx, ("VFIO#%d: Failed to reset device %d\n", pThis->iInstance, errno));
}


/**
 * @interface_method_impl{PDMDEVREG,pfnDestruct}
 */
static DECLCALLBACK(int) pciVfioDestruct(PPDMDEVINS pDevIns)
{
    PVFIOPCI pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);

    /* Detach address space. */
    if (pThis->iFdIommu != -1 && pThis->iFdVfio != -1)
    {
        struct vfio_device_detach_iommufd_pt VfioDetach;
        VfioDetach.argsz   = sizeof(VfioDetach);
        VfioDetach.flags   = 0;
        int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_DETACH_IOMMUFD_PT, &VfioDetach);
        AssertLogRelMsg(!rcLnx, ("VFIO#%d: Failed to detach IOMMU page table with %d\n", pThis->iInstance, errno));

        struct iommu_destroy HwptDestroy;
        HwptDestroy.size = sizeof(HwptDestroy);
        HwptDestroy.id   = pThis->idIommuHwpt;
        rcLnx = ioctl(pThis->iFdIommu, IOMMU_DESTROY, &HwptDestroy);
        AssertLogRelMsg(!rcLnx, ("VFIO#%d: Failed to destroy I/O address space with %d\n", pThis->iInstance, errno));
    }

    if (pThis->iFdIommu != -1)
        close(pThis->iFdIommu);
    if (pThis->iFdVfio != -1)
        close(pThis->iFdIommu);
    if (pThis->iFdWakeup != -1)
        close(pThis->iFdWakeup);

    return VINF_SUCCESS;
}


/**
 * @interface_method_impl{PDMDEVREG,pfnConstruct}
 */
static DECLCALLBACK(int) pciVfioConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg)
{
    PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);

    PVFIOPCI      pThis = PDMDEVINS_2_DATA(pDevIns, PVFIOPCI);
    PCPDMDEVHLPR3 pHlp  = pDevIns->pHlpR3;

    PPDMPCIDEV pPciDev = pDevIns->apPciDevs[0];
    PDMPCIDEV_ASSERT_VALID(pDevIns, pPciDev);

    pThis->pDevIns        = pDevIns;
    pThis->iInstance      = iInstance;
    pThis->iFdIommu       = -1;
    pThis->iFdVfio        = -1;
    pThis->iFdWakeup      = -1;
    pThis->fVga           = false;
    pThis->fInterceptMmio = false;
    pThis->offMsiCtrl     = 0;
    pThis->uIrqModeCur    = UINT8_MAX;
    pThis->uIrqModeNew    = UINT8_MAX;

    int rc = PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Validate configuration.
     */
    if (!pHlp->pfnCFGMAreValuesValid(pCfg,
                                     "VfioPath\0"
                                     "IommuPath\0"
                                     "ExposeVga\0"
                                     "InterceptMmio\0"
                                    ))
        return VERR_PDM_DEVINS_UNKNOWN_CFG_VALUES;

    /* Query configuration. */
    rc = pHlp->pfnCFGMQueryBoolDef(pCfg, "InterceptMmio", &pThis->fInterceptMmio, false);
    if (RT_FAILURE(rc))
        return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                   N_("Configuration error: Querying \"InterceptMmio\" failed"));

    bool fVga = false;
    rc = pHlp->pfnCFGMQueryBoolDef(pCfg, "ExposeVga", &fVga, false);
    if (RT_FAILURE(rc))
        return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                   N_("Configuration error: Querying \"ExposeVga\" failed"));

    char szPath[RTPATH_MAX];
    rc = pHlp->pfnCFGMQueryString(pCfg, "VfioPath", &szPath[0], sizeof(szPath));
    if (RT_FAILURE(rc))
        return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                   N_("Configuration error: Querying \"VfioPath\" failed"));

    pThis->iFdVfio = open(szPath, O_RDWR);
    if (pThis->iFdVfio == -1)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Opening VFIO device \"%s\" failed with %d"), szPath, errno);

    rc = pHlp->pfnCFGMQueryString(pCfg, "IommuPath", &szPath[0], sizeof(szPath));
    if (RT_FAILURE(rc))
        return PDMDevHlpVMSetError(pDevIns, rc, RT_SRC_POS,
                                   N_("Configuration error: Querying \"IommuPath\" failed"));

    pThis->iFdIommu = open(szPath, O_RDWR);
    if (pThis->iFdIommu == -1)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Opening IOMMU path \"%s\" failed with %d"), szPath, errno);

    /* Bind the IOMMU to the device. */
    struct vfio_device_bind_iommufd VfioBind; RT_ZERO(VfioBind);
    VfioBind.argsz   = sizeof(VfioBind);
    VfioBind.flags   = 0;
    VfioBind.iommufd = pThis->iFdIommu;
    int rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_BIND_IOMMUFD, &VfioBind);
    if (rcLnx == -1)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Binding IOMMU device to opened VFIO device failed with %d"), errno);

    /* Allocate a new I/O address space on the IOMMU. */
    struct iommu_ioas_alloc IoasAlloc;
    IoasAlloc.size  = sizeof(IoasAlloc);
    IoasAlloc.flags = 0;
    rcLnx = ioctl(pThis->iFdIommu, IOMMU_IOAS_ALLOC, &IoasAlloc);
    if (rcLnx == -1)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Allocating I/O address space for VFIO device failed with %d"), errno);

    /* And bind it to the device. */
    struct vfio_device_attach_iommufd_pt VfioAttachIommu;
    VfioAttachIommu.argsz = sizeof(VfioAttachIommu);
    VfioAttachIommu.flags = 0;
    VfioAttachIommu.pt_id = IoasAlloc.out_ioas_id;
    pThis->idIommuHwpt = IoasAlloc.out_ioas_id;
    rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_ATTACH_IOMMUFD_PT, &VfioAttachIommu);
    if (rcLnx == -1)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Attaching IO address space to opened VFIO device failed with %d"), errno);

    struct vfio_device_info DevInfo;
    DevInfo.argsz = sizeof(DevInfo);
    rcLnx = ioctl(pThis->iFdVfio, VFIO_DEVICE_GET_INFO, &DevInfo);
    if (rcLnx == -1)
        return PDMDevHlpVMSetError(pDevIns, RTErrConvertFromErrno(errno), RT_SRC_POS,
                                   N_("Getting device information of opened VFIO device failed with %d"), errno);

    LogRel(("VFIO#%d: Info flags:       %#RX32\n"
            "VFIO#%d: Info num_regions: %#RU32\n"
            "VFIO#%d: Info num_irqs:    %#RU32\n"
            "VFIO#%d: Info cap_offset:  %#RX32\n",
            iInstance, DevInfo.flags,
            iInstance, DevInfo.num_regions,
            iInstance, DevInfo.num_irqs,
            iInstance, DevInfo.cap_offset));

    /** @todo Only support PCI devices. */

    /* Setup access to the PCI config space first. */
    struct vfio_region_info RegionInfo;
    rc = pciVfioQueryRegionInfo(pThis, VFIO_PCI_CONFIG_REGION_INDEX, &RegionInfo);
    if (RT_FAILURE(rc))
        return rc;

    if (   (RegionInfo.flags & (VFIO_REGION_INFO_FLAG_READ | VFIO_REGION_INFO_FLAG_WRITE))
        != (VFIO_REGION_INFO_FLAG_READ | VFIO_REGION_INFO_FLAG_WRITE))
        return PDMDevHlpVMSetError(pDevIns, VERR_INVALID_STATE, RT_SRC_POS,
                                   N_("The PCI config region is not marked as read/write as expected"));

    pThis->offPciCfg = RegionInfo.offset;
    pThis->cbPciCfg  = RegionInfo.size;

    /* Setup the PCI config space. */
    rc = pciVfioCfgSpaceSetup(pThis, pPciDev);
    if (RT_FAILURE(rc))
        return rc;

    LogRel(("VFIO#%d: Attached PCI device %04x:%04x\n", iInstance,
            PDMPciDevGetVendorId(pPciDev), PDMPciDevGetDeviceId(pPciDev)));

    /* Attach the device. */
    rc = PDMDevHlpPCIRegister(pDevIns, pPciDev);
    if (RT_FAILURE(rc))
        return rc;

    rc = PDMDevHlpPCIInterceptConfigAccesses(pDevIns, pPciDev, pciVfioConfigRead, pciVfioConfigWrite);
    if (RT_FAILURE(rc))
        return rc;

    /* Set up BAR0 through BAR5. */
    static uint32_t s_aVfioPciRegions[] =
    {
        VFIO_PCI_BAR0_REGION_INDEX,
        VFIO_PCI_BAR1_REGION_INDEX,
        VFIO_PCI_BAR2_REGION_INDEX,
        VFIO_PCI_BAR3_REGION_INDEX,
        VFIO_PCI_BAR4_REGION_INDEX,
        VFIO_PCI_BAR5_REGION_INDEX
    };

    for (uint32_t i = 0; i < RT_ELEMENTS(s_aVfioPciRegions); i++)
    {
        rc = pciVfioSetupBar(pThis, pDevIns, pPciDev, i, s_aVfioPciRegions[i]);
        if (RT_FAILURE(rc))
            return rc;
    }

    uint8_t bClassBase = PDMPciDevGetByte(pPciDev, VBOX_PCI_CLASS_BASE);
    uint8_t bClassSub  = PDMPciDevGetByte(pPciDev, VBOX_PCI_CLASS_SUB);
    if (   fVga
        && bClassBase == VBOX_PCI_CLASS_DISPLAY
        && bClassSub  == VBOX_PCI_SUB_DISPLAY_VGA)
    {
        rc = pciVfioSetupVga(pThis, pDevIns);
        if (RT_FAILURE(rc))
            return rc;
    }
    else
        pThis->fVga = false;

    rc = pciVfioSetupRom(pThis, pDevIns);
    if (RT_FAILURE(rc))
        return rc;

    /* Parse the capability lists now. */
    rc = pciVfioCfgSpaceParseCapabilities(pThis, pPciDev);
    if (RT_FAILURE(rc))
        return rc;

    /** @todo Parse the extended capability list */

    /* Create wakeup eventfd for IRQ poller. */
    rc = pciVfioLnxEventfd2(0 /*uValInit*/, 0 /*fFlags*/, &pThis->iFdWakeup);
    if (RT_FAILURE(rc))
        return rc;

    /* Spin up the interrupt poller. */
    char szDev[64];
    RT_ZERO(szDev);
    RTStrPrintf(szDev, sizeof(szDev), "VFIO-%u", iInstance);
    rc = PDMDevHlpThreadCreate(pDevIns, &pThis->pThrdIrq, pThis, pciVfioIrqPoller, pciVfioIrqPollerWakeup,
                               0 /* cbStack */, RTTHREADTYPE_IO, szDev);
    AssertLogRelRCReturn(rc, rc);

    return rc;
}


/**
 * The device registration structure.
 */
const PDMDEVREG g_DevicePciVfio =
{
    /* .u32Version = */             PDM_DEVREG_VERSION,
    /* .uReserved0 = */             0,
    /* .szName = */                 "pci-vfio",
    /* .fFlags = */                 PDM_DEVREG_FLAGS_DEFAULT_BITS | PDM_DEVREG_FLAGS_NEW_STYLE,
    /* .fClass = */                 PDM_DEVREG_CLASS_HOST_DEV,
    /* .cMaxInstances = */          ~0U,
    /* .uSharedVersion = */         42,
    /* .cbInstanceShared = */       sizeof(VFIOPCI),
    /* .cbInstanceCC = */           0,
    /* .cbInstanceRC = */           0,
    /* .cMaxPciDevices = */         1,
    /* .cMaxMsixVectors = */        VBOX_MSIX_MAX_ENTRIES,
    /* .pszDescription = */         "PCI passthrough through VFIO",
#if defined(IN_RING3)
    /* .pszRCMod = */               "",
    /* .pszR0Mod = */               "",
    /* .pfnConstruct = */           pciVfioConstruct,
    /* .pfnDestruct = */            pciVfioDestruct,
    /* .pfnRelocate = */            NULL,
    /* .pfnMemSetup = */            NULL,
    /* .pfnPowerOn = */             NULL,
    /* .pfnReset = */               pciVfioReset,
    /* .pfnSuspend = */             NULL,
    /* .pfnResume = */              NULL,
    /* .pfnAttach = */              NULL,
    /* .pfnDetach = */              NULL,
    /* .pfnQueryInterface = */      NULL,
    /* .pfnInitComplete = */        NULL,
    /* .pfnPowerOff = */            NULL,
    /* .pfnSoftReset = */           NULL,
    /* .pfnReserved0 = */           NULL,
    /* .pfnReserved1 = */           NULL,
    /* .pfnReserved2 = */           NULL,
    /* .pfnReserved3 = */           NULL,
    /* .pfnReserved4 = */           NULL,
    /* .pfnReserved5 = */           NULL,
    /* .pfnReserved6 = */           NULL,
    /* .pfnReserved7 = */           NULL,
#elif defined(IN_RING0)
    /* .pfnEarlyConstruct = */      NULL,
    /* .pfnConstruct = */           NULL,
    /* .pfnDestruct = */            NULL,
    /* .pfnFinalDestruct = */       NULL,
    /* .pfnRequest = */             NULL,
    /* .pfnReserved0 = */           NULL,
    /* .pfnReserved1 = */           NULL,
    /* .pfnReserved2 = */           NULL,
    /* .pfnReserved3 = */           NULL,
    /* .pfnReserved4 = */           NULL,
    /* .pfnReserved5 = */           NULL,
    /* .pfnReserved6 = */           NULL,
    /* .pfnReserved7 = */           NULL,
#elif defined(IN_RC)
    /* .pfnConstruct = */           NULL,
    /* .pfnReserved0 = */           NULL,
    /* .pfnReserved1 = */           NULL,
    /* .pfnReserved2 = */           NULL,
    /* .pfnReserved3 = */           NULL,
    /* .pfnReserved4 = */           NULL,
    /* .pfnReserved5 = */           NULL,
    /* .pfnReserved6 = */           NULL,
    /* .pfnReserved7 = */           NULL,
#else
# error "Not in IN_RING3, IN_RING0 or IN_RC!"
#endif
    /* .u32VersionEnd = */          PDM_DEVREG_VERSION
};

#endif /* !VBOX_DEVICE_STRUCT_TESTCASE */

