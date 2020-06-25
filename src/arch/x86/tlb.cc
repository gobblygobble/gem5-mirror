/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "arch/x86/tlb.hh"

#include <cstring>
#include <memory>

#include "arch/x86/faults.hh"
#include "arch/x86/insts/microldstop.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/msr.hh"
#include "arch/x86/x86_traits.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/TLB.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

namespace X86ISA {

TLB::TLB(const Params *p)
    : BaseTLB(p), configAddress(0), size(p->size),
      associativity(p->associativity),
      lruSeq(0), m5opRange(p->system->m5opRange())
{
    // 0 means fully associative
    if (associativity == 0)
        associativity = size;
    if (size == 0)
        associativity = 1;
    
    index_bits = 0;
    int counter = size / associativity;
    while (counter > 1) {
        index_bits++;
        counter /= 2;
    }
    // trieVector
    trieVector = std::vector<TlbEntryTrie>(size / associativity);
    // tlbVector
    tlbVector = std::vector<std::vector <TlbEntry> >();
    for (int ind = 0; ind < size / associativity; ind++) {
        tlbVector.push_back(std::vector<TlbEntry>(associativity));
    }
    // freeListVector
    freeListVector = std::vector<EntryList>();
    for (int ind = 0; ind < size / associativity; ind++) {
        freeListVector.push_back(EntryList());
    }

    for (int ind = 0; ind < size / associativity; ind++) {
        for (int x = 0; x < associativity; x++) {
            tlbVector[ind][x].trieHandle = NULL;
            freeListVector[ind].push_back(&tlbVector[ind][x]);
        }
    }
    
    walker = p->walker;
    walker->setTLB(this);
}

uint32_t
TLB::getIndex(Addr vaddr)
{
    // For given address vaddr, we get the corresponding index
    // If the bits are numbered 63~0 were bits 11~0 would be 0
    // given that the page size is fixed to 4KB (which it is),
    // then we check # (index_bits) starting from bit 12
    uint32_t ind = ((1 << index_bits) - 1) & (vaddr >> 12);
    assert (ind < (1 << index_bits));
    return ind;
}

std::list<TlbEntry *> *
TLB::getFreeList(Addr vaddr)
{
    uint32_t ind = getIndex(vaddr);
    return &freeListVector[ind];
}

std::vector<TlbEntry> *
TLB::getTlb(Addr vaddr)
{
    uint32_t ind = getIndex(vaddr);
    return &tlbVector[ind];
}

TlbEntryTrie *
TLB::getTrie(Addr vaddr)
{
    uint32_t ind = getIndex(vaddr);
    return &trieVector[ind];
}

void
TLB::sendEntryToHigherLevel(Addr vpn, TlbEntry &entry, bool fromMemory)
{
    // boolMemory shows if this entry was brought in from memory (true)
    // or if it is just being sent to higher level from DTB2 (false)
    // object calling this function should be DTB2
    X86ISA::TLB *dtb = myUpper();
    std::list<TlbEntry *> *dtb_freelist = dtb->getFreeList(vpn);
    if (fromMemory) {
        if (dtb_freelist->empty()) {
            // if upper TLB (L1) is full,
            // we evict its LRU victim and bring it here (L2)
            TlbEntry victim = dtb->evictLRU(vpn);
            // 1. add victim from L1 here (L2)
            insert(victim.vaddr, victim);
            // 2. add entry to L1
            dtb->insert(vpn, entry);
        }
        else {
            // if upper TLB (L1) is not full,
            // just insert it and finish
            dtb->insert(vpn, entry);
        }
        return;
    }
    // reaching here means fromMemory == false
    assert(!fromMemory);
    if (dtb_freelist->empty()) {
        // if upper TLB (L1) is full,
        // we evict its LRU victim and bring it here (L2)
        // we want to 'swap' entries of L1 and L2
        TlbEntry victim = dtb->evictLRU(vpn);
        // 1. insert entry to upper TLB (L1)
        dtb->insert(vpn, entry);
        // 2. find entry from L2 and remove it
        uint32_t ind = getIndex(vpn);
        for (unsigned x = 0; x < size / associativity; x++) {
            if (tlbVector[ind][x].vaddr == entry.vaddr && tlbVector[ind][x].trieHandle) {
                trieVector[ind].remove(tlbVector[ind][x].trieHandle);
                tlbVector[ind][x].trieHandle = NULL;
                freeListVector[ind].push_back(&tlbVector[ind][x]);
                break;
            }
        }
        // 3. insert victim entry to L2
        insert(victim.vaddr, victim);
    }
    else {
        // if upper TLB (L1) is not full,
        // 1. insert it to upper TLB (L1)
        dtb->insert(vpn, entry);
        // 2. find entry from L2 and remove it
        uint32_t ind = getIndex(vpn);
        for (unsigned x = 0; x < size / associativity; x++) {
            if (tlbVector[ind][x].vaddr == entry.vaddr && tlbVector[ind][x].trieHandle) {
                trieVector[ind].remove(tlbVector[ind][x].trieHandle);
                tlbVector[ind][x].trieHandle = NULL;
                freeListVector[ind].push_back(&tlbVector[ind][x]);
                break;
            }
        }
    }
}

TlbEntry
TLB::evictLRU(Addr vaddr)
{
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    uint32_t ind = getIndex(vaddr);

    unsigned lru = 0;
    for (unsigned i = 1; i < associativity; i++) {
        if (tlbVector[ind][i].lruSeq < tlbVector[ind][lru].lruSeq)
            lru = i;
    }

    assert(tlbVector[ind][lru].trieHandle);
    trieVector[ind].remove(tlbVector[ind][lru].trieHandle);
    tlbVector[ind][lru].trieHandle = NULL;
    freeListVector[ind].push_back(&tlbVector[ind][lru]);
    return tlbVector[ind][lru];
}

TlbEntry *
TLB::insert(Addr vpn, const TlbEntry &entry)
{
    uint32_t ind = getIndex(vpn);

    // If somebody beat us to it, just use that existing entry.
    TlbEntry *newEntry = trieVector[ind].lookup(vpn);
    if (newEntry) {
        assert(newEntry->vaddr == vpn);
        return newEntry;
    }

    if (freeListVector[ind].empty())
        evictLRU(vpn);

    newEntry = freeListVector[ind].front();
    freeListVector[ind].pop_front();

    *newEntry = entry;
    newEntry->lruSeq = nextSeq();
    newEntry->vaddr = vpn;
    newEntry->trieHandle =
    trieVector[ind].insert(vpn, TlbEntryTrie::MaxBits - entry.logBytes, newEntry);
    return newEntry;
}

TlbEntry *
TLB::lookup(Addr va, bool update_lru)
{
    uint32_t ind = getIndex(va);

    TlbEntry *entry = trieVector[ind].lookup(va);
    if (entry && update_lru)
        entry->lruSeq = nextSeq();
    return entry;
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "Invalidating all entries.\n");
    for (int ind = 0; ind < size / associativity; ind++){
        for (unsigned i = 0; i < associativity; i++) {
            if (tlbVector[ind][i].trieHandle) {
                trieVector[ind].remove(tlbVector[ind][i].trieHandle);
                tlbVector[ind][i].trieHandle = NULL;
                freeListVector[ind].push_back(&tlbVector[ind][i]);
            }
        }
    }
}

void
TLB::setConfigAddress(uint32_t addr)
{
    configAddress = addr;
}

void
TLB::flushNonGlobal()
{
    DPRINTF(TLB, "Invalidating all non global entries.\n");
    for (int ind = 0; ind < size / associativity; ind++){
        for (unsigned i = 0; i < associativity; i++) {
            if (tlbVector[ind][i].trieHandle && !tlbVector[ind][i].global) {
                trieVector[ind].remove(tlbVector[ind][i].trieHandle);
                tlbVector[ind][i].trieHandle = NULL;
                freeListVector[ind].push_back(&tlbVector[ind][i]);
            }
        }
    }
}

void
TLB::demapPage(Addr va, uint64_t asn)
{
    uint32_t ind = getIndex(va);

    TlbEntry *entry = trieVector[ind].lookup(va);
    if (entry) {
        trieVector[ind].remove(entry->trieHandle);
        entry->trieHandle = NULL;
        freeListVector[ind].push_back(entry);
    }
}

Fault
TLB::translateInt(const RequestPtr &req, ThreadContext *tc)
{
    DPRINTF(TLB, "Addresses references internal memory.\n");
    Addr vaddr = req->getVaddr();
    Addr prefix = (vaddr >> 3) & IntAddrPrefixMask;
    if (prefix == IntAddrPrefixCPUID) {
        panic("CPUID memory space not yet implemented!\n");
    } else if (prefix == IntAddrPrefixMSR) {
        vaddr = (vaddr >> 3) & ~IntAddrPrefixMask;
        req->setFlags(Request::MMAPPED_IPR);

        MiscRegIndex regNum;
        if (!msrAddrToIndex(regNum, vaddr))
            return std::make_shared<GeneralProtection>(0);

        //The index is multiplied by the size of a RegVal so that
        //any memory dependence calculations will not see these as
        //overlapping.
        req->setPaddr((Addr)regNum * sizeof(RegVal));
        return NoFault;
    } else if (prefix == IntAddrPrefixIO) {
        // TODO If CPL > IOPL or in virtual mode, check the I/O permission
        // bitmap in the TSS.

        Addr IOPort = vaddr & ~IntAddrPrefixMask;
        // Make sure the address fits in the expected 16 bit IO address
        // space.
        assert(!(IOPort & ~0xFFFF));
        if (IOPort == 0xCF8 && req->getSize() == 4) {
            req->setFlags(Request::MMAPPED_IPR);
            req->setPaddr(MISCREG_PCI_CONFIG_ADDRESS * sizeof(RegVal));
        } else if ((IOPort & ~mask(2)) == 0xCFC) {
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            Addr configAddress =
                tc->readMiscRegNoEffect(MISCREG_PCI_CONFIG_ADDRESS);
            if (bits(configAddress, 31, 31)) {
                req->setPaddr(PhysAddrPrefixPciConfig |
                        mbits(configAddress, 30, 2) |
                        (IOPort & mask(2)));
            } else {
                req->setPaddr(PhysAddrPrefixIO | IOPort);
            }
        } else {
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            req->setPaddr(PhysAddrPrefixIO | IOPort);
        }
        return NoFault;
    } else {
        panic("Access to unrecognized internal address space %#x.\n",
                prefix);
    }
}

Fault
TLB::finalizePhysical(const RequestPtr &req,
                      ThreadContext *tc, Mode mode) const
{
    Addr paddr = req->getPaddr();

    if (m5opRange.contains(paddr)) {
        req->setFlags(Request::MMAPPED_IPR | Request::STRICT_ORDER);
    } else if (FullSystem) {
        // Check for an access to the local APIC
        LocalApicBase localApicBase =
            tc->readMiscRegNoEffect(MISCREG_APIC_BASE);
        AddrRange apicRange(localApicBase.base * PageBytes,
                            (localApicBase.base + 1) * PageBytes);

        if (apicRange.contains(paddr)) {
            // The Intel developer's manuals say the below restrictions apply,
            // but the linux kernel, because of a compiler optimization, breaks
            // them.
            /*
            // Check alignment
            if (paddr & ((32/8) - 1))
                return new GeneralProtection(0);
            // Check access size
            if (req->getSize() != (32/8))
                return new GeneralProtection(0);
            */
            // Force the access to be uncacheable.
            req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
            req->setPaddr(x86LocalAPICAddress(tc->contextId(),
                                              paddr - apicRange.start()));
        }
    }

    return NoFault;
}

Fault
TLB::translate(const RequestPtr &req,
        ThreadContext *tc, Translation *translation,
        Mode mode, bool &delayedResponse, bool timing)
{
    Request::Flags flags = req->getFlags();
    int seg = flags & SegmentFlagMask;
    bool storeCheck = flags & (StoreCheck << FlagShift);

    delayedResponse = false;

    // If this is true, we're dealing with a request to a non-memory address
    // space.
    if (seg == SEGMENT_REG_MS) {
        return translateInt(req, tc);
    }

    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);

    HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);

    // If protected mode has been enabled...
    if (m5Reg.prot) {
        DPRINTF(TLB, "In protected mode.\n");
        // If we're not in 64-bit mode, do protection/limit checks
        if (m5Reg.mode != LongMode) {
            DPRINTF(TLB, "Not in long mode. Checking segment protection.\n");
            // Check for a NULL segment selector.
            if (!(seg == SEGMENT_REG_TSG || seg == SYS_SEGMENT_REG_IDTR ||
                        seg == SEGMENT_REG_HS || seg == SEGMENT_REG_LS)
                    && !tc->readMiscRegNoEffect(MISCREG_SEG_SEL(seg)))
                return std::make_shared<GeneralProtection>(0);
            bool expandDown = false;
            SegAttr attr = tc->readMiscRegNoEffect(MISCREG_SEG_ATTR(seg));
            if (seg >= SEGMENT_REG_ES && seg <= SEGMENT_REG_HS) {
                if (!attr.writable && (mode == Write || storeCheck))
                    return std::make_shared<GeneralProtection>(0);
                if (!attr.readable && mode == Read)
                    return std::make_shared<GeneralProtection>(0);
                expandDown = attr.expandDown;

            }
            Addr base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(seg));
            Addr limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(seg));
            bool sizeOverride = (flags & (AddrSizeFlagBit << FlagShift));
            unsigned logSize = sizeOverride ? (unsigned)m5Reg.altAddr
                                            : (unsigned)m5Reg.defAddr;
            int size = (1 << logSize) * 8;
            Addr offset = bits(vaddr - base, size - 1, 0);
            Addr endOffset = offset + req->getSize() - 1;
            if (expandDown) {
                DPRINTF(TLB, "Checking an expand down segment.\n");
                warn_once("Expand down segments are untested.\n");
                if (offset <= limit || endOffset <= limit)
                    return std::make_shared<GeneralProtection>(0);
            } else {
                if (offset > limit || endOffset > limit)
                    return std::make_shared<GeneralProtection>(0);
            }
        }
        if (m5Reg.submode != SixtyFourBitMode ||
                (flags & (AddrSizeFlagBit << FlagShift)))
            vaddr &= mask(32);
        // If paging is enabled, do the translation.
        if (m5Reg.paging) {
            DPRINTF(TLB, "Paging enabled.\n");
            // The vaddr already has the segment base applied.
            TlbEntry *entry = lookup(vaddr);
            if (mode == Read) {
                rdAccesses++;
            } else {
                wrAccesses++;
            }
            if (!entry) {
                DPRINTF(TLB, "Handling a TLB miss for "
                        "address %#x at pc %#x.\n",
                        vaddr, tc->instAddr());
                if (mode == Read) {
                    rdMisses++;
                } else {
                    wrMisses++;
                }
                // miss in TLB.
                // if DTB, request to DTB2
                // if ITB/DTB2, proceed as usual
                if (myLower() && myLower()->size) {
                    // DTB
                    return myLower()->translate(req, tc, translation, mode,
                                                delayedResponse, timing);
                }
                if (FullSystem) {
                    Fault fault = walker->start(tc, translation, req, mode);
                    if (timing || fault != NoFault) {
                        // This gets ignored in atomic mode.
                        delayedResponse = true;
                        return fault;
                    }
                    entry = lookup(vaddr);
                    assert(entry);
                } else {
                    Process *p = tc->getProcessPtr();
                    const EmulationPageTable::Entry *pte =
                        p->pTable->lookup(vaddr);
                    if (!pte && mode != Execute) {
                        // Check if we just need to grow the stack.
                        if (p->fixupStackFault(vaddr)) {
                            // If we did, lookup the entry for the new page.
                            pte = p->pTable->lookup(vaddr);
                        }
                    }
                    if (!pte) {
                        return std::make_shared<PageFault>(vaddr, true, mode,
                                                           true, false);
                    } else {
                        Addr alignedVaddr = p->pTable->pageAlign(vaddr);
                        DPRINTF(TLB, "Mapping %#x to %#x\n", alignedVaddr,
                                pte->paddr);
                        if (myUpper()) {
                            // DTB2 right now
                            // miss in DTB, misw in DTB2, 
                            // brought in from memory
                            entry = new TlbEntry(
                                    p->pTable->pid(), alignedVaddr, pte->paddr,
                                    pte->flags & EmulationPageTable::Uncacheable,
                                    pte->flags & EmulationPageTable::ReadOnly);
                            Addr vpn = (entry->vaddr) & ~mask(entry->logBytes);
                            sendEntryToHigherLevel(vpn, *entry, true);
                        }
                        else {
                            // ITB right now
                            entry = insert(alignedVaddr, TlbEntry(
                                    p->pTable->pid(), alignedVaddr, pte->paddr,
                                    pte->flags & EmulationPageTable::Uncacheable,
                                    pte->flags & EmulationPageTable::ReadOnly));
                        }
                    }
                    DPRINTF(TLB, "Miss was serviced.\n");
                }
            }
            // entry found in TLB -> no need to go to lower level
            DPRINTF(TLB, "Entry found with paddr %#x, "
                    "doing protection checks.\n", entry->paddr);
            // Do paging protection checks.
            bool inUser = (m5Reg.cpl == 3 &&
                    !(flags & (CPL0FlagBit << FlagShift)));
            CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
            bool badWrite = (!entry->writable && (inUser || cr0.wp));
            if ((inUser && !entry->user) || (mode == Write && badWrite)) {
                // The page must have been present to get into the TLB in
                // the first place. We'll assume the reserved bits are
                // fine even though we're not checking them.
                return std::make_shared<PageFault>(vaddr, true, mode, inUser,
                                                   false);
            }
            if (storeCheck && badWrite) {
                // This would fault if this were a write, so return a page
                // fault that reflects that happening.
                return std::make_shared<PageFault>(vaddr, true, Write, inUser,
                                                   false);
            }

            if (myUpper()) {
                // DTB2 right now
                // miss in DTB, hit in DTB2
                Addr vpn = (entry->vaddr) & ~mask(entry->logBytes);
                sendEntryToHigherLevel(vpn, *entry, false);
            }

            Addr paddr = entry->paddr | (vaddr & mask(entry->logBytes));
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
            req->setPaddr(paddr);
            if (entry->uncacheable)
                req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);
        } else {
            //Use the address which already has segmentation applied.
            DPRINTF(TLB, "Paging disabled.\n");
            DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
            req->setPaddr(vaddr);
        }
    } else {
        // Real mode
        DPRINTF(TLB, "In real mode.\n");
        DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, vaddr);
        req->setPaddr(vaddr);
    }

    return finalizePhysical(req, tc, mode);
}

Fault
TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, Mode mode)
{
    bool delayedResponse;
    return TLB::translate(req, tc, NULL, mode, delayedResponse, false);
}

void
TLB::translateTiming(const RequestPtr &req, ThreadContext *tc,
        Translation *translation, Mode mode)
{
    bool delayedResponse;
    assert(translation);
    Fault fault =
        TLB::translate(req, tc, translation, mode, delayedResponse, true);
    if (!delayedResponse)
        translation->finish(fault, req, tc, mode);
    else
        translation->markDelayed();
}

Walker *
TLB::getWalker()
{
    return walker;
}

void
TLB::regStats()
{
    using namespace Stats;
    BaseTLB::regStats();
    rdAccesses
        .name(name() + ".rdAccesses")
        .desc("TLB accesses on read requests");

    wrAccesses
        .name(name() + ".wrAccesses")
        .desc("TLB accesses on write requests");

    rdMisses
        .name(name() + ".rdMisses")
        .desc("TLB misses on read requests");

    wrMisses
        .name(name() + ".wrMisses")
        .desc("TLB misses on write requests");

}

void
TLB::serialize(CheckpointOut &cp) const
{
    uint32_t not_in_use = 0;
    for (uint32_t it = 0; it < size / associativity; it++)
        not_in_use += freeListVector[it].size();
    // Only store the entries in use.
    //uint32_t _size = size - freeListVector[ind].size();
    uint32_t _size = size - not_in_use;
    SERIALIZE_SCALAR(_size);
    SERIALIZE_SCALAR(lruSeq);

    uint32_t _count = 0;
    for (uint32_t ind = 0; ind < size / associativity; ind++) {
        for (uint32_t x = 0; x < associativity; x++) {
            if (tlbVector[ind][x].trieHandle != NULL)
                tlbVector[ind][x].serializeSection(cp, csprintf("Entry%d", _count++));
        }
    }
}

void
TLB::unserialize(CheckpointIn &cp)
{
    // Do not allow to restore with a smaller tlb.
    uint32_t _size;
    UNSERIALIZE_SCALAR(_size);
    if (_size > size) {
        fatal("TLB size less than the one in checkpoint!");
    }

    UNSERIALIZE_SCALAR(lruSeq);
    for (uint32_t ind = 0; ind < size / associativity; ind++) {
        for (uint32_t x = 0; x < associativity; x++) {
            TlbEntry *newEntry = freeListVector[ind].front();
            freeListVector[ind].pop_front();

            newEntry->unserializeSection(cp, csprintf("Entry%d", x));
            newEntry->trieHandle = trieVector[ind].insert(newEntry->vaddr,
                TlbEntryTrie::MaxBits - newEntry->logBytes, newEntry);
        }
    }
}

Port *
TLB::getTableWalkerPort()
{
    return &walker->getPort("port");
}

} // namespace X86ISA

X86ISA::TLB *
X86TLBParams::create()
{
    return new X86ISA::TLB(this);
}