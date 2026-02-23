/* $Id: NEMR3NativeTemplate-linux.cpp.h 113132 2026-02-23 18:18:04Z alexander.eichner@oracle.com $ */
/** @file
 * NEM - Native execution manager, native ring-3 Linux backend, common bits for x86 and arm64.
 */

/*
 * Copyright (C) 2021-2026 Oracle and/or its affiliates.
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

/*
 * Supply stuff missing from the kvm.h on the build box.
 */

/*
 * Exit codes
 */
#ifndef KVM_EXIT_UNKNOWN
# define KVM_EXIT_UNKNOWN 0
#endif
#ifndef KVM_EXIT_EXCEPTION
# define KVM_EXIT_EXCEPTION 1
#endif
#ifndef KVM_EXIT_IO
# define KVM_EXIT_IO 2
#endif
#ifndef KVM_EXIT_HYPERCALL
# define KVM_EXIT_HYPERCALL 3
#endif
#ifndef KVM_EXIT_DEBUG
# define KVM_EXIT_DEBUG 4
#endif
#ifndef KVM_EXIT_HLT
# define KVM_EXIT_HLT 5
#endif
#ifndef KVM_EXIT_MMIO
# define KVM_EXIT_MMIO 6
#endif
#ifndef KVM_EXIT_IRQ_WINDOW_OPEN
# define KVM_EXIT_IRQ_WINDOW_OPEN 7
#endif
#ifndef KVM_EXIT_SHUTDOWN
# define KVM_EXIT_SHUTDOWN 8
#endif
#ifndef KVM_EXIT_FAIL_ENTRY
# define KVM_EXIT_FAIL_ENTRY 9
#endif
#ifndef KVM_EXIT_INTR
# define KVM_EXIT_INTR 10
#endif
#ifndef KVM_EXIT_SET_TPR
# define KVM_EXIT_SET_TPR 11
#endif
#ifndef KVM_EXIT_TPR_ACCESS
# define KVM_EXIT_TPR_ACCESS 12
#endif
#ifndef KVM_EXIT_S390_SIEIC
# define KVM_EXIT_S390_SIEIC 13
#endif
#ifndef KVM_EXIT_S390_RESET
# define KVM_EXIT_S390_RESET 14
#endif
#ifndef KVM_EXIT_DCR
# define KVM_EXIT_DCR 15
#endif
#ifndef KVM_EXIT_NMI
# define KVM_EXIT_NMI 16
#endif
#ifndef KVM_EXIT_INTERNAL_ERROR
# define KVM_EXIT_INTERNAL_ERROR 17
#endif
#ifndef KVM_EXIT_OSI
# define KVM_EXIT_OSI 18
#endif
#ifndef KVM_EXIT_PAPR_HCALL
# define KVM_EXIT_PAPR_HCALL 19
#endif
#ifndef KVM_EXIT_S390_UCONTROL
# define KVM_EXIT_S390_UCONTROL 20
#endif
#ifndef KVM_EXIT_WATCHDOG
# define KVM_EXIT_WATCHDOG 21
#endif
#ifndef KVM_EXIT_S390_TSCH
# define KVM_EXIT_S390_TSCH 22
#endif
#ifndef KVM_EXIT_EPR
# define KVM_EXIT_EPR 23
#endif
#ifndef KVM_EXIT_SYSTEM_EVENT
# define KVM_EXIT_SYSTEM_EVENT 24
#endif
#ifndef KVM_EXIT_S390_STSI
# define KVM_EXIT_S390_STSI 25
#endif
#ifndef KVM_EXIT_IOAPIC_EOI
# define KVM_EXIT_IOAPIC_EOI 26
#endif
#ifndef KVM_EXIT_HYPERV
# define KVM_EXIT_HYPERV 27
#endif
#ifndef KVM_EXIT_ARM_NISV
# define KVM_EXIT_ARM_NISV 28
#endif
#ifndef KVM_EXIT_X86_RDMSR
# define KVM_EXIT_X86_RDMSR 29
#endif
#ifndef KVM_EXIT_X86_WRMSR
# define KVM_EXIT_X86_WRMSR 30
#endif
#ifndef KVM_EXIT_DIRTY_RING_FULL
# define KVM_EXIT_DIRTY_RING_FULL 31
#endif
#ifndef KVM_EXIT_AP_RESET_HOLD
# define KVM_EXIT_AP_RESET_HOLD 32
#endif
#ifndef KVM_EXIT_X86_BUS_LOCK
# define KVM_EXIT_X86_BUS_LOCK 33
#endif
#ifndef KVM_EXIT_XEN
# define KVM_EXIT_XEN 34
#endif
#ifndef KVM_EXIT_RISCV_SBI
# define KVM_EXIT_RISCV_SBI 35
#endif
#ifndef KVM_EXIT_RISCV_CSR
# define KVM_EXIT_RISCV_CSR 36
#endif
#ifndef KVM_EXIT_NOTIFY
# define KVM_EXIT_NOTIFY 37
#endif
#ifndef KVM_EXIT_LOONGARCH_IOCSR
# define KVM_EXIT_LOONGARCH_IOCSR 38
#endif
#ifndef KVM_EXIT_MEMORY_FAULT
# define KVM_EXIT_MEMORY_FAULT 39
#endif
#ifndef KVM_EXIT_TDX
# define KVM_EXIT_TDX 40
#endif


/*
 * Flags for certain exits.
 */
#ifndef KVM_INTERNAL_ERROR_UNEXPECTED_EXIT_REASON /* since 5.4 */
# define KVM_INTERNAL_ERROR_UNEXPECTED_EXIT_REASON 4
#endif
#ifndef KVM_GUESTDBG_BLOCKIRQ
# define KVM_GUESTDBG_BLOCKIRQ 0x00100000
#endif


/*
 * Capabilities
 */
#ifndef KVM_CAP_PPC_NESTED_HV
# define KVM_CAP_PPC_NESTED_HV 160
#endif
#ifndef KVM_CAP_HYPERV_SEND_IPI
# define KVM_CAP_HYPERV_SEND_IPI 161
#endif
#ifndef KVM_CAP_COALESCED_PIO
# define KVM_CAP_COALESCED_PIO 162
#endif
#ifndef KVM_CAP_HYPERV_ENLIGHTENED_VMCS
# define KVM_CAP_HYPERV_ENLIGHTENED_VMCS 163
#endif
#ifndef KVM_CAP_EXCEPTION_PAYLOAD
# define KVM_CAP_EXCEPTION_PAYLOAD 164
#endif
#ifndef KVM_CAP_ARM_VM_IPA_SIZE
# define KVM_CAP_ARM_VM_IPA_SIZE 165
#endif
#ifndef KVM_CAP_MANUAL_DIRTY_LOG_PROTECT
# define KVM_CAP_MANUAL_DIRTY_LOG_PROTECT 166
#endif
#ifndef KVM_CAP_HYPERV_CPUID
# define KVM_CAP_HYPERV_CPUID 167
#endif
#ifndef KVM_CAP_MANUAL_DIRTY_LOG_PROTECT2
# define KVM_CAP_MANUAL_DIRTY_LOG_PROTECT2 168
#endif
#ifndef KVM_CAP_PPC_IRQ_XIVE
# define KVM_CAP_PPC_IRQ_XIVE 169
#endif
#ifndef KVM_CAP_ARM_SVE
# define KVM_CAP_ARM_SVE 170
#endif
#ifndef KVM_CAP_ARM_PTRAUTH_ADDRESS
# define KVM_CAP_ARM_PTRAUTH_ADDRESS 171
#endif
#ifndef KVM_CAP_ARM_PTRAUTH_GENERIC
# define KVM_CAP_ARM_PTRAUTH_GENERIC 172
#endif
#ifndef KVM_CAP_PMU_EVENT_FILTER
# define KVM_CAP_PMU_EVENT_FILTER 173
#endif
#ifndef KVM_CAP_ARM_IRQ_LINE_LAYOUT_2
# define KVM_CAP_ARM_IRQ_LINE_LAYOUT_2 174
#endif
#ifndef KVM_CAP_HYPERV_DIRECT_TLBFLUSH
# define KVM_CAP_HYPERV_DIRECT_TLBFLUSH 175
#endif
#ifndef KVM_CAP_PPC_GUEST_DEBUG_SSTEP
# define KVM_CAP_PPC_GUEST_DEBUG_SSTEP 176
#endif
#ifndef KVM_CAP_ARM_NISV_TO_USER
# define KVM_CAP_ARM_NISV_TO_USER 177
#endif
#ifndef KVM_CAP_ARM_INJECT_EXT_DABT
# define KVM_CAP_ARM_INJECT_EXT_DABT 178
#endif
#ifndef KVM_CAP_S390_VCPU_RESETS
# define KVM_CAP_S390_VCPU_RESETS 179
#endif
#ifndef KVM_CAP_S390_PROTECTED
# define KVM_CAP_S390_PROTECTED 180
#endif
#ifndef KVM_CAP_PPC_SECURE_GUEST
# define KVM_CAP_PPC_SECURE_GUEST 181
#endif
#ifndef KVM_CAP_HALT_POLL
# define KVM_CAP_HALT_POLL 181
#endif
#ifndef KVM_CAP_ASYNC_PF_INT
# define KVM_CAP_ASYNC_PF_INT 182
#endif
#ifndef KVM_CAP_LAST_CPU
# define KVM_CAP_LAST_CPU 183
#endif
#ifndef KVM_CAP_SMALLER_MAXPHYADDR
# define KVM_CAP_SMALLER_MAXPHYADDR 184
#endif
#ifndef KVM_CAP_S390_DIAG318
# define KVM_CAP_S390_DIAG318 185
#endif
#ifndef KVM_CAP_STEAL_TIME
# define KVM_CAP_STEAL_TIME 186
#endif
#ifndef KVM_CAP_X86_USER_SPACE_MSR
# define KVM_CAP_X86_USER_SPACE_MSR 187
#endif
#ifndef KVM_CAP_X86_MSR_FILTER
# define KVM_CAP_X86_MSR_FILTER 188
# define MY_KVM_DEFINE_MSR_FILTER
#endif
#ifndef KVM_CAP_ENFORCE_PV_FEATURE_CPUID
# define KVM_CAP_ENFORCE_PV_FEATURE_CPUID 189
#endif
#ifndef KVM_CAP_SYS_HYPERV_CPUID
# define KVM_CAP_SYS_HYPERV_CPUID 191
#endif
#ifndef KVM_CAP_DIRTY_LOG_RING
# define KVM_CAP_DIRTY_LOG_RING 192
#endif
#ifndef KVM_CAP_X86_BUS_LOCK_EXIT
# define KVM_CAP_X86_BUS_LOCK_EXIT 193
#endif
#ifndef KVM_CAP_PPC_DAWR1
# define KVM_CAP_PPC_DAWR1 194
#endif
#ifndef KVM_CAP_SET_GUEST_DEBUG2
# define KVM_CAP_SET_GUEST_DEBUG2 195
#endif
#ifndef KVM_CAP_SGX_ATTRIBUTE
# define KVM_CAP_SGX_ATTRIBUTE 196
#endif
#ifndef KVM_CAP_VM_COPY_ENC_CONTEXT_FROM
# define KVM_CAP_VM_COPY_ENC_CONTEXT_FROM 197
#endif
#ifndef KVM_CAP_PTP_KVM
# define KVM_CAP_PTP_KVM 198
#endif
#ifndef KVM_CAP_HYPERV_ENFORCE_CPUID
# define KVM_CAP_HYPERV_ENFORCE_CPUID 199
#endif
#ifndef KVM_CAP_SREGS2
# define KVM_CAP_SREGS2 200
#endif
#ifndef KVM_CAP_EXIT_HYPERCALL
# define KVM_CAP_EXIT_HYPERCALL 201
#endif
#ifndef KVM_CAP_PPC_RPT_INVALIDATE
# define KVM_CAP_PPC_RPT_INVALIDATE 202
#endif
#ifndef KVM_CAP_BINARY_STATS_FD
# define KVM_CAP_BINARY_STATS_FD 203
#endif
#ifndef KVM_CAP_EXIT_ON_EMULATION_FAILURE
# define KVM_CAP_EXIT_ON_EMULATION_FAILURE 204
#endif
#ifndef KVM_CAP_ARM_MTE
# define KVM_CAP_ARM_MTE 205
#endif
#ifndef KVM_CAP_VM_MOVE_ENC_CONTEXT_FROM
# define KVM_CAP_VM_MOVE_ENC_CONTEXT_FROM 206
#endif
#ifndef KVM_CAP_VM_GPA_BITS
# define KVM_CAP_VM_GPA_BITS 207
#endif
#ifndef KVM_CAP_XSAVE2
# define KVM_CAP_XSAVE2 208
#endif
#ifndef KVM_CAP_SYS_ATTRIBUTES
# define KVM_CAP_SYS_ATTRIBUTES 209
#endif
#ifndef KVM_CAP_PPC_AIL_MODE_3
# define KVM_CAP_PPC_AIL_MODE_3 210
#endif
#ifndef KVM_CAP_S390_MEM_OP_EXTENSION
# define KVM_CAP_S390_MEM_OP_EXTENSION 211
#endif
#ifndef KVM_CAP_PMU_CAPABILITY
# define KVM_CAP_PMU_CAPABILITY 212
#endif
#ifndef KVM_CAP_DISABLE_QUIRKS2
# define KVM_CAP_DISABLE_QUIRKS2 213
#endif
#ifndef KVM_CAP_VM_TSC_CONTROL
# define KVM_CAP_VM_TSC_CONTROL 214
#endif
#ifndef KVM_CAP_SYSTEM_EVENT_DATA
# define KVM_CAP_SYSTEM_EVENT_DATA 215
#endif
#ifndef KVM_CAP_ARM_SYSTEM_SUSPEND
# define KVM_CAP_ARM_SYSTEM_SUSPEND 216
#endif
#ifndef KVM_CAP_S390_PROTECTED_DUMP
# define KVM_CAP_S390_PROTECTED_DUMP 217
#endif
#ifndef KVM_CAP_X86_TRIPLE_FAULT_EVENT
# define KVM_CAP_X86_TRIPLE_FAULT_EVENT 218
#endif
#ifndef KVM_CAP_X86_NOTIFY_VMEXIT
# define KVM_CAP_X86_NOTIFY_VMEXIT 219
#endif
#ifndef KVM_CAP_VM_DISABLE_NX_HUGE_PAGES
# define KVM_CAP_VM_DISABLE_NX_HUGE_PAGES 220
#endif
#ifndef KVM_CAP_S390_ZPCI_OP
# define KVM_CAP_S390_ZPCI_OP 221
#endif
#ifndef KVM_CAP_S390_CPU_TOPOLOGY
# define KVM_CAP_S390_CPU_TOPOLOGY 222
#endif
#ifndef KVM_CAP_DIRTY_LOG_RING_ACQ_REL
# define KVM_CAP_DIRTY_LOG_RING_ACQ_REL 223
#endif
#ifndef KVM_CAP_S390_PROTECTED_ASYNC_DISABLE
# define KVM_CAP_S390_PROTECTED_ASYNC_DISABLE 224
#endif
#ifndef KVM_CAP_DIRTY_LOG_RING_WITH_BITMAP
# define KVM_CAP_DIRTY_LOG_RING_WITH_BITMAP 225
#endif
#ifndef KVM_CAP_PMU_EVENT_MASKED_EVENTS
# define KVM_CAP_PMU_EVENT_MASKED_EVENTS 226
#endif
#ifndef KVM_CAP_COUNTER_OFFSET
# define KVM_CAP_COUNTER_OFFSET 227
#endif
#ifndef KVM_CAP_ARM_EAGER_SPLIT_CHUNK_SIZE
# define KVM_CAP_ARM_EAGER_SPLIT_CHUNK_SIZE 228
#endif
#ifndef KVM_CAP_ARM_SUPPORTED_BLOCK_SIZES
# define KVM_CAP_ARM_SUPPORTED_BLOCK_SIZES 229
#endif
#ifndef KVM_CAP_ARM_SUPPORTED_REG_MASK_RANGES
# define KVM_CAP_ARM_SUPPORTED_REG_MASK_RANGES 230
#endif
#ifndef KVM_CAP_USER_MEMORY2
# define KVM_CAP_USER_MEMORY2 231
#endif
#ifndef KVM_CAP_MEMORY_FAULT_INFO
# define KVM_CAP_MEMORY_FAULT_INFO 232
#endif
#ifndef KVM_CAP_MEMORY_ATTRIBUTES
# define KVM_CAP_MEMORY_ATTRIBUTES 233
#endif
#ifndef KVM_CAP_GUEST_MEMFD
# define KVM_CAP_GUEST_MEMFD 234
#endif
#ifndef KVM_CAP_VM_TYPES
# define KVM_CAP_VM_TYPES 235
#endif
#ifndef KVM_CAP_PRE_FAULT_MEMORY
# define KVM_CAP_PRE_FAULT_MEMORY 236
#endif
#ifndef KVM_CAP_X86_APIC_BUS_CYCLES_NS
# define KVM_CAP_X86_APIC_BUS_CYCLES_NS 237
#endif
#ifndef KVM_CAP_X86_GUEST_MODE
# define KVM_CAP_X86_GUEST_MODE 238
#endif
#ifndef KVM_CAP_ARM_WRITABLE_IMP_ID_REGS
# define KVM_CAP_ARM_WRITABLE_IMP_ID_REGS 239
#endif
#ifndef KVM_CAP_ARM_EL2
# define KVM_CAP_ARM_EL2 240
#endif
#ifndef KVM_CAP_ARM_EL2_E2H0
# define KVM_CAP_ARM_EL2_E2H0 241
#endif
#ifndef KVM_CAP_RISCV_MP_STATE_RESET
# define KVM_CAP_RISCV_MP_STATE_RESET 242
#endif
#ifndef KVM_CAP_ARM_CACHEABLE_PFNMAP_SUPPORTED
# define KVM_CAP_ARM_CACHEABLE_PFNMAP_SUPPORTED 243
#endif
#ifndef KVM_CAP_GUEST_MEMFD_FLAGS
# define KVM_CAP_GUEST_MEMFD_FLAGS 244
#endif


#ifdef MY_KVM_DEFINE_MSR_FILTER
# define KVM_MSR_FILTER_MAX_RANGES 16
# define KVM_MSR_FILTER_MAX_BITMAP_SIZE 0x600

struct kvm_msr_filter_range
{
    uint32_t flags;
    uint32_t nmsrs;
    uint32_t base;
    uint8_t  *bitmap;
};

#define KVM_MSR_FILTER_READ  (1 << 0)
#define KVM_MSR_FILTER_WRITE (1 << 1)

struct kvm_msr_filter
{
    uint32_t flags;
    struct kvm_msr_filter_range ranges[KVM_MSR_FILTER_MAX_RANGES];
};

#define KVM_MSR_FILTER_DEFAULT_ALLOW 0
#define KVM_MSR_FILTER_DEFAULT_DENY  1

# define KVM_X86_SET_MSR_FILTER _IOW(KVMIO,  0xc6, struct kvm_msr_filter)

# define KVM_MSR_EXIT_REASON_INVAL   (1 << 0)
# define KVM_MSR_EXIT_REASON_UNKNOWN (1 << 1)
# define KVM_MSR_EXIT_REASON_FILTER  (1 << 2)
#endif

/* This needs to be defined always. */
typedef struct
{
    uint8_t error;
    uint8_t pad[7];
    uint32_t reason;
    uint32_t index;
    uint64_t data;
} kvm_run_exit_msr;



/**
 * Worker for nemR3NativeInit that gets the hypervisor capabilities.
 *
 * @returns VBox status code.
 * @param   pVM                 The cross context VM structure.
 * @param   pErrInfo            Where to always return error info.
 */
static int nemR3LnxInitCheckCapabilities(PVM pVM, PRTERRINFO pErrInfo)
{
    AssertReturn(pVM->nem.s.fdKvm != -1, RTErrInfoSet(pErrInfo, VERR_WRONG_ORDER, "Wrong initalization order"));

    /*
     * Capabilities.
     */
    static const struct
    {
        const char *pszName;
        int         iCap;
        uint32_t    offNem      : 24;
        uint32_t    cbNem       : 3;
        uint32_t    fReqNonZero : 1;
        uint32_t    uReserved   : 4;
    } s_aCaps[] =
    {
#define CAP_ENTRY__L(a_Define)           { #a_Define, a_Define,            UINT32_C(0x00ffffff), 0, 0, 0 }
#define CAP_ENTRY__S(a_Define, a_Member) { #a_Define, a_Define, RT_UOFFSETOF(NEM, a_Member), RT_SIZEOFMEMB(NEM, a_Member), 0, 0 }
#define CAP_ENTRY_MS(a_Define, a_Member) { #a_Define, a_Define, RT_UOFFSETOF(NEM, a_Member), RT_SIZEOFMEMB(NEM, a_Member), 1, 0 }
#define CAP_ENTRY__U(a_Number)           { "KVM_CAP_" #a_Number, a_Number, UINT32_C(0x00ffffff), 0, 0, 0 }
#define CAP_ENTRY_ML(a_Number)           { "KVM_CAP_" #a_Number, a_Number, UINT32_C(0x00ffffff), 0, 1, 0 }

        CAP_ENTRY__L(KVM_CAP_IRQCHIP),                       /* 0 */
#ifdef VBOX_VMM_TARGET_X86
        CAP_ENTRY_ML(KVM_CAP_HLT),
#else
        CAP_ENTRY__L(KVM_CAP_HLT),
#endif
        CAP_ENTRY__L(KVM_CAP_MMU_SHADOW_CACHE_CONTROL),
        CAP_ENTRY_ML(KVM_CAP_USER_MEMORY),
        CAP_ENTRY__L(KVM_CAP_SET_TSS_ADDR),
        CAP_ENTRY__U(5),
        CAP_ENTRY__L(KVM_CAP_VAPIC),
        CAP_ENTRY__L(KVM_CAP_EXT_CPUID),
        CAP_ENTRY__L(KVM_CAP_CLOCKSOURCE),
        CAP_ENTRY__L(KVM_CAP_NR_VCPUS),
        CAP_ENTRY_MS(KVM_CAP_NR_MEMSLOTS, cMaxMemSlots),     /* 10 */
        CAP_ENTRY__L(KVM_CAP_PIT),
        CAP_ENTRY__L(KVM_CAP_NOP_IO_DELAY),
        CAP_ENTRY__L(KVM_CAP_PV_MMU),
        CAP_ENTRY__L(KVM_CAP_MP_STATE),
        CAP_ENTRY__L(KVM_CAP_COALESCED_MMIO),
        CAP_ENTRY__L(KVM_CAP_SYNC_MMU),
        CAP_ENTRY__U(17),
        CAP_ENTRY__L(KVM_CAP_IOMMU),
        CAP_ENTRY__U(19), /* Buggy KVM_CAP_JOIN_MEMORY_REGIONS? */
        CAP_ENTRY__U(20), /* Non-working KVM_CAP_DESTROY_MEMORY_REGION? */
        CAP_ENTRY__L(KVM_CAP_DESTROY_MEMORY_REGION_WORKS),   /* 21 */
        CAP_ENTRY__L(KVM_CAP_USER_NMI),
#ifdef __KVM_HAVE_GUEST_DEBUG
        CAP_ENTRY__L(KVM_CAP_SET_GUEST_DEBUG),
#endif
#ifdef __KVM_HAVE_PIT
        CAP_ENTRY__L(KVM_CAP_REINJECT_CONTROL),
#endif
        CAP_ENTRY__L(KVM_CAP_IRQ_ROUTING),
        CAP_ENTRY__L(KVM_CAP_IRQ_INJECT_STATUS),
        CAP_ENTRY__U(27),
        CAP_ENTRY__U(28),
        CAP_ENTRY__L(KVM_CAP_ASSIGN_DEV_IRQ),
        CAP_ENTRY__L(KVM_CAP_JOIN_MEMORY_REGIONS_WORKS),     /* 30 */
#ifdef __KVM_HAVE_MCE
        CAP_ENTRY__L(KVM_CAP_MCE),
#endif
        CAP_ENTRY__L(KVM_CAP_IRQFD),
#ifdef __KVM_HAVE_PIT
        CAP_ENTRY__L(KVM_CAP_PIT2),
#endif
        CAP_ENTRY__L(KVM_CAP_SET_BOOT_CPU_ID),
#ifdef __KVM_HAVE_PIT_STATE2
        CAP_ENTRY__L(KVM_CAP_PIT_STATE2),
#endif
        CAP_ENTRY__L(KVM_CAP_IOEVENTFD),
        CAP_ENTRY__L(KVM_CAP_SET_IDENTITY_MAP_ADDR),
#ifdef __KVM_HAVE_XEN_HVM
        CAP_ENTRY__L(KVM_CAP_XEN_HVM),
#endif
#ifdef VBOX_VMM_TARGET_X86
        CAP_ENTRY_ML(KVM_CAP_ADJUST_CLOCK),
#else
        CAP_ENTRY__L(KVM_CAP_ADJUST_CLOCK),
#endif
        CAP_ENTRY__L(KVM_CAP_INTERNAL_ERROR_DATA),           /* 40 */
#ifdef __KVM_HAVE_VCPU_EVENTS
        CAP_ENTRY_ML(KVM_CAP_VCPU_EVENTS),
#else
        CAP_ENTRY__U(41),
#endif
        CAP_ENTRY__L(KVM_CAP_S390_PSW),
        CAP_ENTRY__L(KVM_CAP_PPC_SEGSTATE),
        CAP_ENTRY__L(KVM_CAP_HYPERV),
        CAP_ENTRY__L(KVM_CAP_HYPERV_VAPIC),
        CAP_ENTRY__L(KVM_CAP_HYPERV_SPIN),
        CAP_ENTRY__L(KVM_CAP_PCI_SEGMENT),
        CAP_ENTRY__L(KVM_CAP_PPC_PAIRED_SINGLES),
        CAP_ENTRY__L(KVM_CAP_INTR_SHADOW),
#ifdef __KVM_HAVE_DEBUGREGS
        CAP_ENTRY__L(KVM_CAP_DEBUGREGS),                     /* 50 */
#endif
#ifdef VBOX_VMM_TARGET_X86
        CAP_ENTRY__S(KVM_CAP_X86_ROBUST_SINGLESTEP, fRobustSingleStep),
#else
        CAP_ENTRY__L(KVM_CAP_X86_ROBUST_SINGLESTEP),
#endif
        CAP_ENTRY__L(KVM_CAP_PPC_OSI),
        CAP_ENTRY__L(KVM_CAP_PPC_UNSET_IRQ),
        CAP_ENTRY__L(KVM_CAP_ENABLE_CAP),
#ifdef __KVM_HAVE_XSAVE
        CAP_ENTRY_ML(KVM_CAP_XSAVE),
#else
        CAP_ENTRY__U(55),
#endif
#ifdef __KVM_HAVE_XCRS
        CAP_ENTRY_ML(KVM_CAP_XCRS),
#else
        CAP_ENTRY__U(56),
#endif
        CAP_ENTRY__L(KVM_CAP_PPC_GET_PVINFO),
        CAP_ENTRY__L(KVM_CAP_PPC_IRQ_LEVEL),
        CAP_ENTRY__L(KVM_CAP_ASYNC_PF),
        CAP_ENTRY__L(KVM_CAP_TSC_CONTROL),                   /* 60 */
        CAP_ENTRY__L(KVM_CAP_GET_TSC_KHZ),
        CAP_ENTRY__L(KVM_CAP_PPC_BOOKE_SREGS),
        CAP_ENTRY__L(KVM_CAP_SPAPR_TCE),
        CAP_ENTRY__L(KVM_CAP_PPC_SMT),
        CAP_ENTRY__L(KVM_CAP_PPC_RMA),
        CAP_ENTRY__L(KVM_CAP_MAX_VCPUS),
        CAP_ENTRY__L(KVM_CAP_PPC_HIOR),
        CAP_ENTRY__L(KVM_CAP_PPC_PAPR),
        CAP_ENTRY__L(KVM_CAP_SW_TLB),
        CAP_ENTRY__L(KVM_CAP_ONE_REG),                       /* 70 */
        CAP_ENTRY__L(KVM_CAP_S390_GMAP),
        CAP_ENTRY__L(KVM_CAP_TSC_DEADLINE_TIMER),
        CAP_ENTRY__L(KVM_CAP_S390_UCONTROL),
        CAP_ENTRY__L(KVM_CAP_SYNC_REGS),
        CAP_ENTRY__L(KVM_CAP_PCI_2_3),
        CAP_ENTRY__L(KVM_CAP_KVMCLOCK_CTRL),
        CAP_ENTRY__L(KVM_CAP_SIGNAL_MSI),
        CAP_ENTRY__L(KVM_CAP_PPC_GET_SMMU_INFO),
        CAP_ENTRY__L(KVM_CAP_S390_COW),
        CAP_ENTRY__L(KVM_CAP_PPC_ALLOC_HTAB),                /* 80 */
        CAP_ENTRY__L(KVM_CAP_READONLY_MEM),
        CAP_ENTRY__L(KVM_CAP_IRQFD_RESAMPLE),
        CAP_ENTRY__L(KVM_CAP_PPC_BOOKE_WATCHDOG),
        CAP_ENTRY__L(KVM_CAP_PPC_HTAB_FD),
        CAP_ENTRY__L(KVM_CAP_S390_CSS_SUPPORT),
        CAP_ENTRY__L(KVM_CAP_PPC_EPR),
#ifdef VBOX_VMM_TARGET_ARMV8
        CAP_ENTRY_ML(KVM_CAP_ARM_PSCI),
        CAP_ENTRY_ML(KVM_CAP_ARM_SET_DEVICE_ADDR),
        CAP_ENTRY_ML(KVM_CAP_DEVICE_CTRL),
#else
        CAP_ENTRY__L(KVM_CAP_ARM_PSCI),
        CAP_ENTRY__L(KVM_CAP_ARM_SET_DEVICE_ADDR),
        CAP_ENTRY__L(KVM_CAP_DEVICE_CTRL),
#endif
        CAP_ENTRY__L(KVM_CAP_IRQ_MPIC),                      /* 90 */
        CAP_ENTRY__L(KVM_CAP_PPC_RTAS),
        CAP_ENTRY__L(KVM_CAP_IRQ_XICS),
        CAP_ENTRY__L(KVM_CAP_ARM_EL1_32BIT),
        CAP_ENTRY__L(KVM_CAP_SPAPR_MULTITCE),
        CAP_ENTRY__L(KVM_CAP_EXT_EMUL_CPUID),
        CAP_ENTRY__L(KVM_CAP_HYPERV_TIME),
        CAP_ENTRY__L(KVM_CAP_IOAPIC_POLARITY_IGNORED),
        CAP_ENTRY__L(KVM_CAP_ENABLE_CAP_VM),
        CAP_ENTRY__L(KVM_CAP_S390_IRQCHIP),
        CAP_ENTRY__L(KVM_CAP_IOEVENTFD_NO_LENGTH),           /* 100 */
        CAP_ENTRY__L(KVM_CAP_VM_ATTRIBUTES),
#ifdef VBOX_VMM_TARGET_ARMV8
        CAP_ENTRY_ML(KVM_CAP_ARM_PSCI_0_2),
#else
        CAP_ENTRY__L(KVM_CAP_ARM_PSCI_0_2),
#endif
        CAP_ENTRY__L(KVM_CAP_PPC_FIXUP_HCALL),
        CAP_ENTRY__L(KVM_CAP_PPC_ENABLE_HCALL),
        CAP_ENTRY__L(KVM_CAP_CHECK_EXTENSION_VM),
        CAP_ENTRY__L(KVM_CAP_S390_USER_SIGP),
        CAP_ENTRY__L(KVM_CAP_S390_VECTOR_REGISTERS),
        CAP_ENTRY__L(KVM_CAP_S390_MEM_OP),
        CAP_ENTRY__L(KVM_CAP_S390_USER_STSI),
        CAP_ENTRY__L(KVM_CAP_S390_SKEYS),                    /* 110 */
        CAP_ENTRY__L(KVM_CAP_MIPS_FPU),
        CAP_ENTRY__L(KVM_CAP_MIPS_MSA),
        CAP_ENTRY__L(KVM_CAP_S390_INJECT_IRQ),
        CAP_ENTRY__L(KVM_CAP_S390_IRQ_STATE),
        CAP_ENTRY__L(KVM_CAP_PPC_HWRNG),
        CAP_ENTRY__L(KVM_CAP_DISABLE_QUIRKS),
        CAP_ENTRY__L(KVM_CAP_X86_SMM),
        CAP_ENTRY__L(KVM_CAP_MULTI_ADDRESS_SPACE),
        CAP_ENTRY__L(KVM_CAP_GUEST_DEBUG_HW_BPS),
        CAP_ENTRY__L(KVM_CAP_GUEST_DEBUG_HW_WPS),            /* 120 */
#ifdef VBOX_VMM_TARGET_X86
        CAP_ENTRY__S(KVM_CAP_SPLIT_IRQCHIP, fKvmApic),
#else
        CAP_ENTRY__L(KVM_CAP_SPLIT_IRQCHIP),
#endif
        CAP_ENTRY__L(KVM_CAP_IOEVENTFD_ANY_LENGTH),
        CAP_ENTRY__L(KVM_CAP_HYPERV_SYNIC),
        CAP_ENTRY__L(KVM_CAP_S390_RI),
        CAP_ENTRY__L(KVM_CAP_SPAPR_TCE_64),
        CAP_ENTRY__L(KVM_CAP_ARM_PMU_V3),
        CAP_ENTRY__L(KVM_CAP_VCPU_ATTRIBUTES),
        CAP_ENTRY__L(KVM_CAP_MAX_VCPU_ID),
        CAP_ENTRY__L(KVM_CAP_X2APIC_API),
        CAP_ENTRY__L(KVM_CAP_S390_USER_INSTR0),              /* 130 */
        CAP_ENTRY__L(KVM_CAP_MSI_DEVID),
        CAP_ENTRY__L(KVM_CAP_PPC_HTM),
        CAP_ENTRY__L(KVM_CAP_SPAPR_RESIZE_HPT),
        CAP_ENTRY__L(KVM_CAP_PPC_MMU_RADIX),
        CAP_ENTRY__L(KVM_CAP_PPC_MMU_HASH_V3),
        CAP_ENTRY_ML(KVM_CAP_IMMEDIATE_EXIT),
        CAP_ENTRY__L(KVM_CAP_MIPS_VZ),
        CAP_ENTRY__L(KVM_CAP_MIPS_TE),
        CAP_ENTRY__L(KVM_CAP_MIPS_64BIT),
        CAP_ENTRY__L(KVM_CAP_S390_GS),                       /* 140 */
        CAP_ENTRY__L(KVM_CAP_S390_AIS),
        CAP_ENTRY__L(KVM_CAP_SPAPR_TCE_VFIO),
        CAP_ENTRY__L(KVM_CAP_X86_DISABLE_EXITS),
        CAP_ENTRY__L(KVM_CAP_ARM_USER_IRQ),
        CAP_ENTRY__L(KVM_CAP_S390_CMMA_MIGRATION),
        CAP_ENTRY__L(KVM_CAP_PPC_FWNMI),
        CAP_ENTRY__L(KVM_CAP_PPC_SMT_POSSIBLE),
        CAP_ENTRY__L(KVM_CAP_HYPERV_SYNIC2),
        CAP_ENTRY__L(KVM_CAP_HYPERV_VP_INDEX),
        CAP_ENTRY__L(KVM_CAP_S390_AIS_MIGRATION),            /* 150 */
        CAP_ENTRY__L(KVM_CAP_PPC_GET_CPU_CHAR),
        CAP_ENTRY__L(KVM_CAP_S390_BPB),
        CAP_ENTRY__L(KVM_CAP_GET_MSR_FEATURES),
        CAP_ENTRY__L(KVM_CAP_HYPERV_EVENTFD),
        CAP_ENTRY__L(KVM_CAP_HYPERV_TLBFLUSH),
        CAP_ENTRY__L(KVM_CAP_S390_HPAGE_1M),
        CAP_ENTRY__S(KVM_CAP_NESTED_STATE, cbNestedState),
        CAP_ENTRY__L(KVM_CAP_ARM_INJECT_SERROR_ESR),
        CAP_ENTRY__L(KVM_CAP_MSR_PLATFORM_INFO),
        CAP_ENTRY__L(KVM_CAP_PPC_NESTED_HV),                 /* 160 */
        CAP_ENTRY__L(KVM_CAP_HYPERV_SEND_IPI),
        CAP_ENTRY__L(KVM_CAP_COALESCED_PIO),
        CAP_ENTRY__L(KVM_CAP_HYPERV_ENLIGHTENED_VMCS),
        CAP_ENTRY__L(KVM_CAP_EXCEPTION_PAYLOAD),
#ifdef VBOX_VMM_TARGET_ARMV8
        CAP_ENTRY_MS(KVM_CAP_ARM_VM_IPA_SIZE, cIpaBits),
#else
        CAP_ENTRY__L(KVM_CAP_ARM_VM_IPA_SIZE),
#endif
        CAP_ENTRY__L(KVM_CAP_MANUAL_DIRTY_LOG_PROTECT),
        CAP_ENTRY__L(KVM_CAP_HYPERV_CPUID),
        CAP_ENTRY__L(KVM_CAP_MANUAL_DIRTY_LOG_PROTECT2),
        CAP_ENTRY__L(KVM_CAP_PPC_IRQ_XIVE),
        CAP_ENTRY__L(KVM_CAP_ARM_SVE),                       /* 170 */
        CAP_ENTRY__L(KVM_CAP_ARM_PTRAUTH_ADDRESS),
        CAP_ENTRY__L(KVM_CAP_ARM_PTRAUTH_GENERIC),
        CAP_ENTRY__L(KVM_CAP_PMU_EVENT_FILTER),
        CAP_ENTRY__L(KVM_CAP_ARM_IRQ_LINE_LAYOUT_2),
        CAP_ENTRY__L(KVM_CAP_HYPERV_DIRECT_TLBFLUSH),
        CAP_ENTRY__L(KVM_CAP_PPC_GUEST_DEBUG_SSTEP),
        CAP_ENTRY__L(KVM_CAP_ARM_NISV_TO_USER),
        CAP_ENTRY__L(KVM_CAP_ARM_INJECT_EXT_DABT),
        CAP_ENTRY__L(KVM_CAP_S390_VCPU_RESETS),
        CAP_ENTRY__L(KVM_CAP_S390_PROTECTED),                /* 180 */
        CAP_ENTRY__L(KVM_CAP_PPC_SECURE_GUEST),
        CAP_ENTRY__L(KVM_CAP_HALT_POLL),
        CAP_ENTRY__L(KVM_CAP_ASYNC_PF_INT),
        CAP_ENTRY__L(KVM_CAP_LAST_CPU),
        CAP_ENTRY__L(KVM_CAP_SMALLER_MAXPHYADDR),
        CAP_ENTRY__L(KVM_CAP_S390_DIAG318),
        CAP_ENTRY__L(KVM_CAP_STEAL_TIME),
#ifdef VBOX_VMM_TARGET_X86
        CAP_ENTRY_ML(KVM_CAP_X86_USER_SPACE_MSR),            /* (since 5.10) */
        CAP_ENTRY_ML(KVM_CAP_X86_MSR_FILTER),
#else
        CAP_ENTRY__L(KVM_CAP_X86_USER_SPACE_MSR),            /* (since 5.10) */
        CAP_ENTRY__L(KVM_CAP_X86_MSR_FILTER),
#endif
        CAP_ENTRY__L(KVM_CAP_ENFORCE_PV_FEATURE_CPUID),      /* 190 */
        CAP_ENTRY__L(KVM_CAP_SYS_HYPERV_CPUID),
        CAP_ENTRY__L(KVM_CAP_DIRTY_LOG_RING),
        CAP_ENTRY__L(KVM_CAP_X86_BUS_LOCK_EXIT),
        CAP_ENTRY__L(KVM_CAP_PPC_DAWR1),
        CAP_ENTRY__L(KVM_CAP_SET_GUEST_DEBUG2),
        CAP_ENTRY__L(KVM_CAP_SGX_ATTRIBUTE),
        CAP_ENTRY__L(KVM_CAP_VM_COPY_ENC_CONTEXT_FROM),
        CAP_ENTRY__L(KVM_CAP_PTP_KVM),
        CAP_ENTRY__L(KVM_CAP_HYPERV_ENFORCE_CPUID),
        CAP_ENTRY__L(KVM_CAP_SREGS2),
        CAP_ENTRY__L(KVM_CAP_EXIT_HYPERCALL),
        CAP_ENTRY__L(KVM_CAP_PPC_RPT_INVALIDATE),
        CAP_ENTRY__L(KVM_CAP_BINARY_STATS_FD),
        CAP_ENTRY__L(KVM_CAP_EXIT_ON_EMULATION_FAILURE),
        CAP_ENTRY__L(KVM_CAP_ARM_MTE),
        CAP_ENTRY__L(KVM_CAP_VM_MOVE_ENC_CONTEXT_FROM),
        CAP_ENTRY__L(KVM_CAP_VM_GPA_BITS),
        CAP_ENTRY__L(KVM_CAP_XSAVE2),
        CAP_ENTRY__L(KVM_CAP_SYS_ATTRIBUTES),
        CAP_ENTRY__L(KVM_CAP_PPC_AIL_MODE_3),
        CAP_ENTRY__L(KVM_CAP_S390_MEM_OP_EXTENSION),
        CAP_ENTRY__L(KVM_CAP_PMU_CAPABILITY),
        CAP_ENTRY__L(KVM_CAP_DISABLE_QUIRKS2),
        CAP_ENTRY__L(KVM_CAP_VM_TSC_CONTROL),
        CAP_ENTRY__L(KVM_CAP_SYSTEM_EVENT_DATA),
        CAP_ENTRY__L(KVM_CAP_ARM_SYSTEM_SUSPEND),
        CAP_ENTRY__L(KVM_CAP_S390_PROTECTED_DUMP),
        CAP_ENTRY__L(KVM_CAP_X86_TRIPLE_FAULT_EVENT),
        CAP_ENTRY__L(KVM_CAP_X86_NOTIFY_VMEXIT),
        CAP_ENTRY__L(KVM_CAP_VM_DISABLE_NX_HUGE_PAGES),
        CAP_ENTRY__L(KVM_CAP_S390_ZPCI_OP),
        CAP_ENTRY__L(KVM_CAP_S390_CPU_TOPOLOGY),
        CAP_ENTRY__L(KVM_CAP_DIRTY_LOG_RING_ACQ_REL),
        CAP_ENTRY__L(KVM_CAP_S390_PROTECTED_ASYNC_DISABLE),
        CAP_ENTRY__L(KVM_CAP_DIRTY_LOG_RING_WITH_BITMAP),
        CAP_ENTRY__L(KVM_CAP_PMU_EVENT_MASKED_EVENTS),
        CAP_ENTRY__L(KVM_CAP_COUNTER_OFFSET),
        CAP_ENTRY__L(KVM_CAP_ARM_EAGER_SPLIT_CHUNK_SIZE),
        CAP_ENTRY__L(KVM_CAP_ARM_SUPPORTED_BLOCK_SIZES),
        CAP_ENTRY__L(KVM_CAP_ARM_SUPPORTED_REG_MASK_RANGES),
        CAP_ENTRY__L(KVM_CAP_USER_MEMORY2),
        CAP_ENTRY__L(KVM_CAP_MEMORY_FAULT_INFO),
        CAP_ENTRY__L(KVM_CAP_MEMORY_ATTRIBUTES),
        CAP_ENTRY__L(KVM_CAP_GUEST_MEMFD),
        CAP_ENTRY__L(KVM_CAP_VM_TYPES),
        CAP_ENTRY__L(KVM_CAP_PRE_FAULT_MEMORY),
        CAP_ENTRY__L(KVM_CAP_X86_APIC_BUS_CYCLES_NS),
        CAP_ENTRY__L(KVM_CAP_X86_GUEST_MODE),
        CAP_ENTRY__L(KVM_CAP_ARM_WRITABLE_IMP_ID_REGS),
        CAP_ENTRY__L(KVM_CAP_ARM_EL2),
        CAP_ENTRY__L(KVM_CAP_ARM_EL2_E2H0),
        CAP_ENTRY__L(KVM_CAP_RISCV_MP_STATE_RESET),
        CAP_ENTRY__L(KVM_CAP_ARM_CACHEABLE_PFNMAP_SUPPORTED),
        CAP_ENTRY__L(KVM_CAP_GUEST_MEMFD_FLAGS),
    };

    LogRel(("NEM: KVM capabilities (system):\n"));
    int rcRet = VINF_SUCCESS;
    for (unsigned i = 0; i < RT_ELEMENTS(s_aCaps); i++)
    {
        int rc = ioctl(pVM->nem.s.fdKvm, KVM_CHECK_EXTENSION, s_aCaps[i].iCap);
        if (rc >= 10)
            LogRel(("NEM:   %36s: %#x (%d)\n", s_aCaps[i].pszName, rc, rc));
        else if (rc >= 0)
            LogRel(("NEM:   %36s: %d\n", s_aCaps[i].pszName, rc));
        else
            LogRel(("NEM:   %s failed: %d/%d\n", s_aCaps[i].pszName, rc, errno));
        switch (s_aCaps[i].cbNem)
        {
            case 0:
                break;
            case 1:
            {
                uint8_t *puValue = (uint8_t *)&pVM->nem.padding[s_aCaps[i].offNem];
                AssertReturn(s_aCaps[i].offNem <= sizeof(NEM) - sizeof(*puValue), VERR_NEM_IPE_0);
                *puValue = (uint8_t)rc;
                AssertLogRelMsg((int)*puValue == rc, ("%s: %#x\n", s_aCaps[i].pszName, rc));
                break;
            }
            case 2:
            {
                uint16_t *puValue = (uint16_t *)&pVM->nem.padding[s_aCaps[i].offNem];
                AssertReturn(s_aCaps[i].offNem <= sizeof(NEM) - sizeof(*puValue), VERR_NEM_IPE_0);
                *puValue = (uint16_t)rc;
                AssertLogRelMsg((int)*puValue == rc, ("%s: %#x\n", s_aCaps[i].pszName, rc));
                break;
            }
            case 4:
            {
                uint32_t *puValue = (uint32_t *)&pVM->nem.padding[s_aCaps[i].offNem];
                AssertReturn(s_aCaps[i].offNem <= sizeof(NEM) - sizeof(*puValue), VERR_NEM_IPE_0);
                *puValue = (uint32_t)rc;
                AssertLogRelMsg((int)*puValue == rc, ("%s: %#x\n", s_aCaps[i].pszName, rc));
                break;
            }
            default:
                rcRet = RTErrInfoSetF(pErrInfo, VERR_NEM_IPE_0, "s_aCaps[%u] is bad: cbNem=%#x - %s",
                                      i, s_aCaps[i].pszName, s_aCaps[i].cbNem);
                AssertFailedReturn(rcRet);
        }

        /*
         * Is a require non-zero entry zero or failing?
         */
        if (s_aCaps[i].fReqNonZero && rc <= 0)
            rcRet = RTERRINFO_LOG_REL_ADD_F(pErrInfo, VERR_NEM_MISSING_FEATURE,
                                            "Required capability '%s' is missing!", s_aCaps[i].pszName);
    }

    /*
     * Get per VCpu KVM_RUN MMAP area size.
     */
    int rc = ioctl(pVM->nem.s.fdKvm, KVM_GET_VCPU_MMAP_SIZE, 0UL);
    if ((unsigned)rc < _64M)
    {
        pVM->nem.s.cbVCpuMmap = (uint32_t)rc;
        LogRel(("NEM:   %36s: %#x (%d)\n", "KVM_GET_VCPU_MMAP_SIZE", rc, rc));
    }
    else if (rc < 0)
        rcRet = RTERRINFO_LOG_REL_ADD_F(pErrInfo, VERR_NEM_MISSING_FEATURE, "KVM_GET_VCPU_MMAP_SIZE failed: %d", errno);
    else
        rcRet = RTERRINFO_LOG_REL_ADD_F(pErrInfo, VERR_NEM_INIT_FAILED, "Odd KVM_GET_VCPU_MMAP_SIZE value: %#x (%d)", rc, rc);

    /*
     * Init the slot ID bitmap.
     */
    ASMBitSet(&pVM->nem.s.bmSlotIds[0], 0);         /* don't use slot 0 */
    if (pVM->nem.s.cMaxMemSlots < _32K)
        ASMBitSetRange(&pVM->nem.s.bmSlotIds[0], pVM->nem.s.cMaxMemSlots, _32K);
    ASMBitSet(&pVM->nem.s.bmSlotIds[0], _32K - 1);  /* don't use the last slot */

    return rcRet;
}


/** @callback_method_impl{FNVMMEMTRENDEZVOUS}   */
static DECLCALLBACK(VBOXSTRICTRC) nemR3LnxFixThreadPoke(PVM pVM, PVMCPU pVCpu, void *pvUser)
{
    RT_NOREF(pVM, pvUser);
    int rc = RTThreadControlPokeSignal(pVCpu->hThread, true /*fEnable*/);
    AssertLogRelRC(rc);
    return VINF_SUCCESS;
}


/**
 * Makes sure GIM is not using HyperV mode as we don't support it with the KVM backend right now.
 *
 * This is rather ugly.
 *
 * @returns VBox status code
 * @param   pVM             The cross context VM structure.
 */
static int nemR3LnxDisableGimHyperV(PVM pVM)
{
    /*
     * Check the GIM config and rewrite it to Minimal if HyperV is currently configured.
     */
    PCFGMNODE pCfg = CFGMR3GetChild(CFGMR3GetRoot(pVM), "/GIM");
    if (pCfg)
    {
        char szProvider[64];
        int rc = CFGMR3QueryString(pCfg, "Provider", &szProvider[0], sizeof(szProvider));
        AssertLogRelMsgReturn(RT_SUCCESS(rc) || rc == VERR_CFGM_VALUE_NOT_FOUND, ("%Rrc\n", rc), rc);
        if (RT_SUCCESS(rc) && !strcmp(szProvider, "HyperV"))
        {
            LogRel(("NEM: Adjusting GIM configuration from HyperV to Minimal mode.  HyperV is currently not supported for GIM on NEM!\n"));
            LogRel(("NEM: Disable KVM if you need the HyperV GIM provider for your guests!\n"));
            rc = CFGMR3RemoveValue(pCfg, "Provider");
            rc = CFGMR3InsertString(pCfg, "Provider", "Minimal");
            AssertLogRelRCReturn(rc, rc);
        }
    }

    return VINF_SUCCESS;
}


DECLHIDDEN(int) nemR3NativeInit(PVM pVM, bool fFallback, bool fForced)
{
    RT_NOREF(pVM, fFallback, fForced);
    /*
     * Some state init.
     */
    pVM->nem.s.fdKvm = -1;
    pVM->nem.s.fdVm  = -1;
    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PNEMCPU pNemCpu = &pVM->apCpusR3[idCpu]->nem.s;
        pNemCpu->fdVCpu = -1;
    }

    /*
     * Error state.
     * The error message will be non-empty on failure and 'rc' will be set too.
     */
    RTERRINFOSTATIC ErrInfo;
    PRTERRINFO pErrInfo = RTErrInfoInitStatic(&ErrInfo);

    /*
     * Open kvm subsystem so we can issue system ioctls.
     */
    int rc;
    int fdKvm = open("/dev/kvm", O_RDWR | O_CLOEXEC);
    if (fdKvm >= 0)
    {
        pVM->nem.s.fdKvm = fdKvm;

        /*
         * Check capabilities.
         */
        rc = nemR3LnxInitCheckCapabilities(pVM, pErrInfo);
        if (RT_SUCCESS(rc))
        {
            /*
             * Create an empty VM since it is recommended we check capabilities on
             * the VM rather than the system descriptor.
             */
#ifdef VBOX_VMM_TARGET_ARMV8
            int fdVm = ioctl(fdKvm, KVM_CREATE_VM, pVM->nem.s.cIpaBits);
#else
            int fdVm = ioctl(fdKvm, KVM_CREATE_VM, 0UL /* Type must be zero on x86 */);
#endif
            if (fdVm >= 0)
            {
                pVM->nem.s.fdVm = fdVm;

                /*
                 * Set up the VM (more on this later).
                 */
                rc = nemR3LnxInitSetupVm(pVM, pErrInfo);
                if (RT_SUCCESS(rc))
                {
                    /*
                     * Set ourselves as the execution engine and make config adjustments.
                     */
                    VM_SET_MAIN_EXECUTION_ENGINE(pVM, VM_EXEC_ENGINE_NATIVE_API);
                    Log(("NEM: Marked active!\n"));
                    nemR3LnxDisableGimHyperV(pVM);
                    PGMR3EnableNemMode(pVM);

                    /*
                     * Register release statistics
                     */
                    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
                    {
                        PNEMCPU pNemCpu = &pVM->apCpusR3[idCpu]->nem.s;
                        STAMR3RegisterF(pVM, &pNemCpu->StatImportOnDemand,      STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of on-demand state imports",      "/NEM/CPU%u/ImportOnDemand", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatImportOnReturn,      STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of state imports on loop return", "/NEM/CPU%u/ImportOnReturn", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatImportOnReturnSkipped, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of skipped state imports on loop return", "/NEM/CPU%u/ImportOnReturnSkipped", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatImportPendingInterrupt, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times an interrupt was pending when importing from KVM", "/NEM/CPU%u/ImportPendingInterrupt", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExportPendingInterrupt, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times an interrupt was pending when exporting to KVM", "/NEM/CPU%u/ExportPendingInterrupt", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatFlushExitOnReturn,   STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times a KVM_EXIT_IO or KVM_EXIT_MMIO was flushed before returning to EM", "/NEM/CPU%u/FlushExitOnReturn", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatFlushExitOnReturn1Loop, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times a KVM_EXIT_IO or KVM_EXIT_MMIO was flushed before returning to EM", "/NEM/CPU%u/FlushExitOnReturn-01-loop", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatFlushExitOnReturn2Loops, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times a KVM_EXIT_IO or KVM_EXIT_MMIO was flushed before returning to EM", "/NEM/CPU%u/FlushExitOnReturn-02-loops", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatFlushExitOnReturn3Loops, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times a KVM_EXIT_IO or KVM_EXIT_MMIO was flushed before returning to EM", "/NEM/CPU%u/FlushExitOnReturn-03-loops", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatFlushExitOnReturn4PlusLoops, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of times a KVM_EXIT_IO or KVM_EXIT_MMIO was flushed before returning to EM", "/NEM/CPU%u/FlushExitOnReturn-04-to-7-loops", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatQueryCpuTick,        STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "Number of TSC queries",                  "/NEM/CPU%u/QueryCpuTick", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitTotal,           STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "All exits",                  "/NEM/CPU%u/Exit", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitIo,              STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_IO",                "/NEM/CPU%u/Exit/Io", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitMmio,            STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_MMIO",              "/NEM/CPU%u/Exit/Mmio", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitSetTpr,          STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_SET_TRP",           "/NEM/CPU%u/Exit/SetTpr", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitTprAccess,       STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_TPR_ACCESS",        "/NEM/CPU%u/Exit/TprAccess", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitRdMsr,           STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_RDMSR",             "/NEM/CPU%u/Exit/RdMsr", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitWrMsr,           STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_WRMSR",             "/NEM/CPU%u/Exit/WrMsr", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitIrqWindowOpen,   STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_IRQ_WINDOWS_OPEN",  "/NEM/CPU%u/Exit/IrqWindowOpen", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitHalt,            STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_HLT",               "/NEM/CPU%u/Exit/Hlt", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitIntr,            STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_INTR",              "/NEM/CPU%u/Exit/Intr", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitHypercall,       STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_HYPERCALL",         "/NEM/CPU%u/Exit/Hypercall", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitDebug,           STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_DEBUG",             "/NEM/CPU%u/Exit/Debug", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitBusLock,         STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_BUS_LOCK",          "/NEM/CPU%u/Exit/BusLock", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitInternalErrorEmulation, STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_INTERNAL_ERROR/EMULATION", "/NEM/CPU%u/Exit/InternalErrorEmulation", idCpu);
                        STAMR3RegisterF(pVM, &pNemCpu->StatExitInternalErrorFatal,     STAMTYPE_COUNTER, STAMVISIBILITY_ALWAYS, STAMUNIT_OCCURENCES, "KVM_EXIT_INTERNAL_ERROR/*", "/NEM/CPU%u/Exit/InternalErrorFatal", idCpu);
                    }

                    /*
                     * Success.
                     */
                    return VINF_SUCCESS;
                }
                close(fdVm);
                pVM->nem.s.fdVm = -1;

                /*
                 * Bail out.
                 */
            }
            else
                rc = RTErrInfoSetF(pErrInfo, VERR_NEM_VM_CREATE_FAILED, "KVM_CREATE_VM failed: %u", errno);
        }
        close(fdKvm);
        pVM->nem.s.fdKvm = -1;
    }
    else if (errno == EACCES)
        rc = RTErrInfoSet(pErrInfo, VERR_ACCESS_DENIED, "Do not have access to open /dev/kvm for reading & writing.");
    else if (errno == ENOENT)
        rc = RTErrInfoSet(pErrInfo, VERR_NOT_SUPPORTED, "KVM is not availble (/dev/kvm does not exist)");
    else
        rc = RTErrInfoSetF(pErrInfo, RTErrConvertFromErrno(errno), "Failed to open '/dev/kvm': %u", errno);

    /*
     * We only fail if in forced mode, otherwise just log the complaint and return.
     */
    Assert(RTErrInfoIsSet(pErrInfo));
    if (   (fForced || !fFallback)
        && pVM->bMainExecutionEngine != VM_EXEC_ENGINE_NATIVE_API)
        return VMSetError(pVM, RT_SUCCESS_NP(rc) ? VERR_NEM_NOT_AVAILABLE : rc, RT_SRC_POS, "%s", pErrInfo->pszMsg);
    LogRel(("NEM: Not available: %s\n", pErrInfo->pszMsg));
    return VINF_SUCCESS;
}


DECLHIDDEN(int) nemR3NativeInitAfterCPUM(PVM pVM)
{
    /*
     * Validate sanity.
     */
    AssertReturn(pVM->nem.s.fdKvm >= 0, VERR_WRONG_ORDER);
    AssertReturn(pVM->nem.s.fdVm >= 0, VERR_WRONG_ORDER);
    AssertReturn(pVM->bMainExecutionEngine == VM_EXEC_ENGINE_NATIVE_API, VERR_WRONG_ORDER);

    /** @todo */

    return VINF_SUCCESS;
}


DECLHIDDEN(int) nemR3NativeTerm(PVM pVM)
{
    /*
     * Per-cpu data
     */
    for (VMCPUID idCpu = 0; idCpu < pVM->cCpus; idCpu++)
    {
        PVMCPU pVCpu = pVM->apCpusR3[idCpu];

        if (pVCpu->nem.s.fdVCpu != -1)
        {
            close(pVCpu->nem.s.fdVCpu);
            pVCpu->nem.s.fdVCpu = -1;
        }
        if (pVCpu->nem.s.pRun)
        {
            munmap(pVCpu->nem.s.pRun, pVM->nem.s.cbVCpuMmap);
            pVCpu->nem.s.pRun = NULL;
        }
#ifdef VBOX_VMM_TARGET_X86
        if (pVCpu->nem.s.pNestedState)
            RTMemFree(pVCpu->nem.s.pNestedState);
#endif
    }

    /*
     * Global data.
     */
    if (pVM->nem.s.fdVm != -1)
    {
        close(pVM->nem.s.fdVm);
        pVM->nem.s.fdVm = -1;
    }

    if (pVM->nem.s.fdKvm != -1)
    {
        close(pVM->nem.s.fdKvm);
        pVM->nem.s.fdKvm = -1;
    }
    return VINF_SUCCESS;
}


DECLHIDDEN(void) nemR3NativeReset(PVM pVM)
{
    RT_NOREF(pVM);
}


DECLHIDDEN(void) nemR3NativeResetCpu(PVMCPU pVCpu, bool fInitIpi)
{
    RT_NOREF(pVCpu, fInitIpi);
}


/*********************************************************************************************************************************
*   Memory management                                                                                                            *
*********************************************************************************************************************************/


/**
 * Allocates a memory slot ID.
 *
 * @returns Slot ID on success, UINT16_MAX on failure.
 */
static uint16_t nemR3LnxMemSlotIdAlloc(PVM pVM)
{
    /* Use the hint first. */
    uint16_t idHint = pVM->nem.s.idPrevSlot;
    if (idHint < _32K - 1)
    {
        int32_t idx = ASMBitNextClear(&pVM->nem.s.bmSlotIds, _32K, idHint);
        Assert(idx < _32K);
        if (idx > 0 && !ASMAtomicBitTestAndSet(&pVM->nem.s.bmSlotIds, idx))
            return pVM->nem.s.idPrevSlot = (uint16_t)idx;
    }

    /*
     * Search the whole map from the start.
     */
    int32_t idx = ASMBitFirstClear(&pVM->nem.s.bmSlotIds, _32K);
    Assert(idx < _32K);
    if (idx > 0 && !ASMAtomicBitTestAndSet(&pVM->nem.s.bmSlotIds, idx))
        return pVM->nem.s.idPrevSlot = (uint16_t)idx;

    Assert(idx < 0 /*shouldn't trigger unless there is a race */);
    return UINT16_MAX; /* caller is expected to assert. */
}


/**
 * Frees a memory slot ID
 */
static void nemR3LnxMemSlotIdFree(PVM pVM, uint16_t idSlot)
{
    if (RT_LIKELY(idSlot < _32K && ASMAtomicBitTestAndClear(&pVM->nem.s.bmSlotIds, idSlot)))
    { /*likely*/ }
    else
        AssertMsgFailed(("idSlot=%u (%#x)\n", idSlot, idSlot));
}



VMMR3_INT_DECL(int) NEMR3NotifyPhysRamRegister(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, void *pvR3,
                                               uint8_t *pu2State, uint32_t *puNemRange)
{
    uint16_t idSlot = nemR3LnxMemSlotIdAlloc(pVM);
    AssertLogRelReturn(idSlot < _32K, VERR_NEM_MAP_PAGES_FAILED);

    Log5(("NEMR3NotifyPhysRamRegister: %RGp LB %RGp, pvR3=%p pu2State=%p (%d) puNemRange=%p (%d) - idSlot=%#x\n",
          GCPhys, cb, pvR3, pu2State, pu2State, puNemRange, *puNemRange, idSlot));

    struct kvm_userspace_memory_region Region;
    Region.slot             = idSlot;
    Region.flags            = 0;
    Region.guest_phys_addr  = GCPhys;
    Region.memory_size      = cb;
    Region.userspace_addr   = (uintptr_t)pvR3;

    int rc = ioctl(pVM->nem.s.fdVm, KVM_SET_USER_MEMORY_REGION, &Region);
    if (rc == 0)
    {
        *pu2State   = 0;
        *puNemRange = idSlot;
        return VINF_SUCCESS;
    }

    LogRel(("NEMR3NotifyPhysRamRegister: %RGp LB %RGp, pvR3=%p, idSlot=%#x failed: %u/%u\n", GCPhys, cb, pvR3, idSlot, rc, errno));
    nemR3LnxMemSlotIdFree(pVM, idSlot);
    return VERR_NEM_MAP_PAGES_FAILED;
}


VMMR3_INT_DECL(bool) NEMR3IsMmio2DirtyPageTrackingSupported(PVM pVM)
{
    RT_NOREF(pVM);
    return true;
}


VMMR3_INT_DECL(int) NEMR3NotifyPhysMmioExMapEarly(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, uint32_t fFlags,
                                                  void *pvRam, void *pvMmio2, uint8_t *pu2State, uint32_t *puNemRange)
{
    Log5(("NEMR3NotifyPhysMmioExMapEarly: %RGp LB %RGp fFlags=%#x pvRam=%p pvMmio2=%p pu2State=%p (%d) puNemRange=%p (%#x)\n",
          GCPhys, cb, fFlags, pvRam, pvMmio2, pu2State, *pu2State, puNemRange, puNemRange ? *puNemRange : UINT32_MAX));
    RT_NOREF(pvRam);

    if (fFlags & NEM_NOTIFY_PHYS_MMIO_EX_F_REPLACE)
    {
        /** @todo implement splitting and whatnot of ranges if we want to be 100%
         *        conforming (just modify RAM registrations in MM.cpp to test). */
        AssertLogRelMsgFailedReturn(("%RGp LB %RGp fFlags=%#x pvRam=%p pvMmio2=%p\n", GCPhys, cb, fFlags, pvRam, pvMmio2),
                                    VERR_NEM_MAP_PAGES_FAILED);
    }

    /*
     * Register MMIO2.
     */
    if (fFlags & NEM_NOTIFY_PHYS_MMIO_EX_F_MMIO2)
    {
        AssertReturn(pvMmio2, VERR_NEM_MAP_PAGES_FAILED);
        AssertReturn(puNemRange, VERR_NEM_MAP_PAGES_FAILED);

        uint16_t idSlot = nemR3LnxMemSlotIdAlloc(pVM);
        AssertLogRelReturn(idSlot < _32K, VERR_NEM_MAP_PAGES_FAILED);

        struct kvm_userspace_memory_region Region;
        Region.slot             = idSlot;
        Region.flags            = fFlags & NEM_NOTIFY_PHYS_MMIO_EX_F_TRACK_DIRTY_PAGES ? KVM_MEM_LOG_DIRTY_PAGES : 0;
        Region.guest_phys_addr  = GCPhys;
        Region.memory_size      = cb;
        Region.userspace_addr   = (uintptr_t)pvMmio2;

        int rc = ioctl(pVM->nem.s.fdVm, KVM_SET_USER_MEMORY_REGION, &Region);
        if (rc == 0)
        {
            *pu2State   = 0;
            *puNemRange = idSlot;
            Log5(("NEMR3NotifyPhysMmioExMapEarly: %RGp LB %RGp fFlags=%#x pvMmio2=%p - idSlot=%#x\n",
                  GCPhys, cb, fFlags, pvMmio2, idSlot));
            return VINF_SUCCESS;
        }

        nemR3LnxMemSlotIdFree(pVM, idSlot);
        AssertLogRelMsgFailedReturn(("%RGp LB %RGp fFlags=%#x, pvMmio2=%p, idSlot=%#x failed: %u/%u\n",
                                     GCPhys, cb, fFlags, pvMmio2, idSlot, errno, rc),
                                    VERR_NEM_MAP_PAGES_FAILED);
    }

    /* MMIO, don't care. */
    *pu2State   = 0;
    *puNemRange = UINT32_MAX;
    return VINF_SUCCESS;
}


VMMR3_INT_DECL(int) NEMR3NotifyPhysMmioExMapLate(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, uint32_t fFlags,
                                                 void *pvRam, void *pvMmio2, uint32_t *puNemRange)
{
    RT_NOREF(pVM, GCPhys, cb, fFlags, pvRam, pvMmio2, puNemRange);
    return VINF_SUCCESS;
}


VMMR3_INT_DECL(int) NEMR3NotifyPhysMmioExUnmap(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, uint32_t fFlags, void *pvRam,
                                               void *pvMmio2, uint8_t *pu2State, uint32_t *puNemRange)
{
    Log5(("NEMR3NotifyPhysMmioExUnmap: %RGp LB %RGp fFlags=%#x pvRam=%p pvMmio2=%p pu2State=%p puNemRange=%p (%#x)\n",
          GCPhys, cb, fFlags, pvRam, pvMmio2, pu2State, puNemRange, *puNemRange));
    RT_NOREF(pVM, GCPhys, cb, fFlags, pvRam, pvMmio2, pu2State);

    if (fFlags & NEM_NOTIFY_PHYS_MMIO_EX_F_REPLACE)
    {
        /** @todo implement splitting and whatnot of ranges if we want to be 100%
         *        conforming (just modify RAM registrations in MM.cpp to test). */
        AssertLogRelMsgFailedReturn(("%RGp LB %RGp fFlags=%#x pvRam=%p pvMmio2=%p\n", GCPhys, cb, fFlags, pvRam, pvMmio2),
                                    VERR_NEM_UNMAP_PAGES_FAILED);
    }

    if (fFlags & NEM_NOTIFY_PHYS_MMIO_EX_F_MMIO2)
    {
        uint32_t const idSlot = *puNemRange;
        AssertReturn(idSlot > 0 && idSlot < _32K, VERR_NEM_IPE_4);
        AssertReturn(ASMBitTest(pVM->nem.s.bmSlotIds, idSlot), VERR_NEM_IPE_4);

        struct kvm_userspace_memory_region Region;
        Region.slot             = idSlot;
        Region.flags            = 0;
        Region.guest_phys_addr  = GCPhys;
        Region.memory_size      = 0;    /* this deregisters it. */
        Region.userspace_addr   = (uintptr_t)pvMmio2;

        int rc = ioctl(pVM->nem.s.fdVm, KVM_SET_USER_MEMORY_REGION, &Region);
        if (rc == 0)
        {
            if (pu2State)
                *pu2State = 0;
            *puNemRange = UINT32_MAX;
            nemR3LnxMemSlotIdFree(pVM, idSlot);
            return VINF_SUCCESS;
        }

        AssertLogRelMsgFailedReturn(("%RGp LB %RGp fFlags=%#x, pvMmio2=%p, idSlot=%#x failed: %u/%u\n",
                                     GCPhys, cb, fFlags, pvMmio2, idSlot, errno, rc),
                                    VERR_NEM_UNMAP_PAGES_FAILED);
    }

    if (pu2State)
        *pu2State = UINT8_MAX;
    return VINF_SUCCESS;
}


VMMR3_INT_DECL(int) NEMR3PhysMmio2QueryAndResetDirtyBitmap(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, uint32_t uNemRange,
                                                           void *pvBitmap, size_t cbBitmap)
{
    AssertReturn(uNemRange > 0 && uNemRange < _32K, VERR_NEM_IPE_4);
    AssertReturn(ASMBitTest(pVM->nem.s.bmSlotIds, uNemRange), VERR_NEM_IPE_4);

    RT_NOREF(GCPhys, cbBitmap);

    struct kvm_dirty_log DirtyLog;
    DirtyLog.slot         = uNemRange;
    DirtyLog.padding1     = 0;
    DirtyLog.dirty_bitmap = pvBitmap;

    int rc = ioctl(pVM->nem.s.fdVm, KVM_GET_DIRTY_LOG, &DirtyLog);
    AssertLogRelMsgReturn(rc == 0, ("%RGp LB %RGp idSlot=%#x failed: %u/%u\n", GCPhys, cb, uNemRange, errno, rc),
                          VERR_NEM_QUERY_DIRTY_BITMAP_FAILED);

    return VINF_SUCCESS;
}


VMMR3_INT_DECL(int)  NEMR3NotifyPhysRomRegisterEarly(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, void *pvPages, uint32_t fFlags,
                                                     uint8_t *pu2State, uint32_t *puNemRange)
{
    Log5(("NEMR3NotifyPhysRomRegisterEarly: %RGp LB %RGp pvPages=%p fFlags=%#x\n", GCPhys, cb, pvPages, fFlags));
    *pu2State = UINT8_MAX;

    /* Don't support puttint ROM where there is already RAM.  For
       now just shuffle the registrations till it works... */
    AssertLogRelMsgReturn(!(fFlags & NEM_NOTIFY_PHYS_ROM_F_REPLACE), ("%RGp LB %RGp fFlags=%#x\n", GCPhys, cb, fFlags),
                          VERR_NEM_MAP_PAGES_FAILED);

    /** @todo figure out how to do shadow ROMs.   */

    /*
     * We only allocate a slot number here in case we need to use it to
     * fend of physical handler fun.
     */
    uint16_t idSlot = nemR3LnxMemSlotIdAlloc(pVM);
    AssertLogRelReturn(idSlot < _32K, VERR_NEM_MAP_PAGES_FAILED);

    *pu2State   = 0;
    *puNemRange = idSlot;
    Log5(("NEMR3NotifyPhysRomRegisterEarly: %RGp LB %RGp fFlags=%#x pvPages=%p - idSlot=%#x\n",
          GCPhys, cb, fFlags, pvPages, idSlot));
    RT_NOREF(GCPhys, cb, fFlags, pvPages);
    return VINF_SUCCESS;
}


VMMR3_INT_DECL(int)  NEMR3NotifyPhysRomRegisterLate(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cb, void *pvPages,
                                                    uint32_t fFlags, uint8_t *pu2State, uint32_t *puNemRange)
{
    Log5(("NEMR3NotifyPhysRomRegisterLate: %RGp LB %RGp pvPages=%p fFlags=%#x pu2State=%p (%d) puNemRange=%p (%#x)\n",
          GCPhys, cb, pvPages, fFlags, pu2State, *pu2State, puNemRange, *puNemRange));

    AssertPtrReturn(pvPages, VERR_NEM_IPE_5);

    uint32_t const idSlot = *puNemRange;
    AssertReturn(idSlot > 0 && idSlot < _32K, VERR_NEM_IPE_4);
    AssertReturn(ASMBitTest(pVM->nem.s.bmSlotIds, idSlot), VERR_NEM_IPE_4);

    *pu2State = UINT8_MAX;

    /*
     * Do the actual setting of the user pages here now that we've
     * got a valid pvPages (typically isn't available during the early
     * notification, unless we're replacing RAM).
     */
    /** @todo r=bird: if it's overlapping RAM, we shouldn't need an additional
     *        registration, should we? */
    struct kvm_userspace_memory_region Region;
    Region.slot             = idSlot;
    Region.flags            = 0;
    Region.guest_phys_addr  = GCPhys;
    Region.memory_size      = cb;
    Region.userspace_addr   = (uintptr_t)pvPages;

    int rc = ioctl(pVM->nem.s.fdVm, KVM_SET_USER_MEMORY_REGION, &Region);
    if (rc == 0)
    {
        *pu2State   = 0;
        Log5(("NEMR3NotifyPhysRomRegisterEarly: %RGp LB %RGp fFlags=%#x pvPages=%p - idSlot=%#x\n",
              GCPhys, cb, fFlags, pvPages, idSlot));
        return VINF_SUCCESS;
    }
    AssertLogRelMsgFailedReturn(("%RGp LB %RGp fFlags=%#x, pvPages=%p, idSlot=%#x failed: %u/%u\n",
                                 GCPhys, cb, fFlags, pvPages, idSlot, errno, rc),
                                VERR_NEM_MAP_PAGES_FAILED);
}


VMMR3_INT_DECL(void) NEMR3NotifySetA20(PVMCPU pVCpu, bool fEnabled)
{
    Log(("nemR3NativeNotifySetA20: fEnabled=%RTbool\n", fEnabled));
    Assert(VM_IS_NEM_ENABLED(pVCpu->CTX_SUFF(pVM)));
    RT_NOREF(pVCpu, fEnabled);
}


VMM_INT_DECL(void) NEMHCNotifyHandlerPhysicalDeregister(PVMCC pVM, PGMPHYSHANDLERKIND enmKind, RTGCPHYS GCPhys, RTGCPHYS cb,
                                                        RTR3PTR pvMemR3, uint8_t *pu2State)
{
    Log5(("NEMHCNotifyHandlerPhysicalDeregister: %RGp LB %RGp enmKind=%d pvMemR3=%p pu2State=%p (%d)\n",
          GCPhys, cb, enmKind, pvMemR3, pu2State, *pu2State));

    *pu2State = UINT8_MAX;
    RT_NOREF(pVM, enmKind, GCPhys, cb, pvMemR3);
}


DECLHIDDEN(void) nemHCNativeNotifyHandlerPhysicalRegister(PVMCC pVM, PGMPHYSHANDLERKIND enmKind, RTGCPHYS GCPhys, RTGCPHYS cb)
{
    Log5(("nemHCNativeNotifyHandlerPhysicalRegister: %RGp LB %RGp enmKind=%d\n", GCPhys, cb, enmKind));
    RT_NOREF(pVM, enmKind, GCPhys, cb);
}


DECLHIDDEN(void) nemHCNativeNotifyHandlerPhysicalModify(PVMCC pVM, PGMPHYSHANDLERKIND enmKind, RTGCPHYS GCPhysOld,
                                                        RTGCPHYS GCPhysNew, RTGCPHYS cb, bool fRestoreAsRAM)
{
    Log5(("nemHCNativeNotifyHandlerPhysicalModify: %RGp LB %RGp -> %RGp enmKind=%d fRestoreAsRAM=%d\n",
          GCPhysOld, cb, GCPhysNew, enmKind, fRestoreAsRAM));
    RT_NOREF(pVM, enmKind, GCPhysOld, GCPhysNew, cb, fRestoreAsRAM);
}


DECLHIDDEN(int) nemHCNativeNotifyPhysPageAllocated(PVMCC pVM, RTGCPHYS GCPhys, RTHCPHYS HCPhys, uint32_t fPageProt,
                                                   PGMPAGETYPE enmType, uint8_t *pu2State)
{
    Log5(("nemHCNativeNotifyPhysPageAllocated: %RGp HCPhys=%RHp fPageProt=%#x enmType=%d *pu2State=%d\n",
          GCPhys, HCPhys, fPageProt, enmType, *pu2State));
    RT_NOREF(pVM, GCPhys, HCPhys, fPageProt, enmType, pu2State);
    return VINF_SUCCESS;
}


VMM_INT_DECL(void) NEMHCNotifyPhysPageProtChanged(PVMCC pVM, RTGCPHYS GCPhys, RTHCPHYS HCPhys, RTR3PTR pvR3, uint32_t fPageProt,
                                                  PGMPAGETYPE enmType, uint8_t *pu2State)
{
    Log5(("NEMHCNotifyPhysPageProtChanged: %RGp HCPhys=%RHp fPageProt=%#x enmType=%d *pu2State=%d\n",
          GCPhys, HCPhys, fPageProt, enmType, *pu2State));
    Assert(VM_IS_NEM_ENABLED(pVM));
    RT_NOREF(pVM, GCPhys, HCPhys, pvR3, fPageProt, enmType, pu2State);

}


VMM_INT_DECL(void) NEMHCNotifyPhysPageChanged(PVMCC pVM, RTGCPHYS GCPhys, RTHCPHYS HCPhysPrev, RTHCPHYS HCPhysNew,
                                              RTR3PTR pvNewR3, uint32_t fPageProt, PGMPAGETYPE enmType, uint8_t *pu2State)
{
    Log5(("nemHCNativeNotifyPhysPageChanged: %RGp HCPhys=%RHp->%RHp pvNewR3=%p fPageProt=%#x enmType=%d *pu2State=%d\n",
          GCPhys, HCPhysPrev, HCPhysNew, pvNewR3, fPageProt, enmType, *pu2State));
    Assert(VM_IS_NEM_ENABLED(pVM));
    RT_NOREF(pVM, GCPhys, HCPhysPrev, HCPhysNew, pvNewR3, fPageProt, enmType, pu2State);
}
