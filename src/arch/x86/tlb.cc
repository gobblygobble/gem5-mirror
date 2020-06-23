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
#include <cstdlib> //calloc()

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
      associativity(p->associativity), tlb(size),
      lruSeq(0), m5opRange(p->system->m5opRange())
{
    /* Now DTB2 can accept 0 when single-level TLB is desired
    if (!size)
        fatal("TLBs must have a non-zero size.\n");
    */

    for (int x = 0; x < size; x++) {
        tlb[x].trieHandle = NULL;
        freeList.push_back(&tlb[x]);
    }

    walker = p->walker;
    walker->setTLB(this);

    old_params = *p;
}

void
TLB::evictLRU()
{
    // Find the entry with the lowest (and hence least recently updated)
    // sequence number.

    unsigned lru = 0;
    for (unsigned i = 1; i < size; i++) {
        if (tlb[i].lruSeq < tlb[lru].lruSeq)
            lru = i;
    }

    assert(tlb[lru].trieHandle);
    trie.remove(tlb[lru].trieHandle);
    tlb[lru].trieHandle = NULL;
    freeList.push_back(&tlb[lru]);
}

void
TLB::evictSecondLRU()
{
    // Find the entry with the SECOND lowest
    // sequence number.
    unsigned lru1, lru2;
    if (tlb[0].lruSeq < tlb[1].lruSeq) {
        lru1 = 0;
        lru2 = 1;
    }
    else {
        lru1 = 1;
        lru2 = 0;
    }
    for (unsigned i = 2; i < size; i++) {
        if (tlb[i].lruSeq < tlb[lru2].lruSeq) {
            if (tlb[i].lruSeq < tlb[lru1].lruSeq) {
                // i < lru1 < lru2
                lru2 = lru1;
                lru1 = i;
            }
            else {
                // lru1 < i < lru2
                lru2 = i;
            }
        }
    }
    assert(tlb[lru2].trieHandle);
    trie.remove(tlb[lru2].trieHandle);
    tlb[lru2].trieHandle = NULL;
    freeList.push_back(&tlb[lru2]);
}

TlbEntry
TLB::findLRU()
{
    unsigned lru = 0;
    for (unsigned i = 1; i < size; i++) {
        if (tlb[i].lruSeq < tlb[lru].lruSeq)
            lru = i;
    }

    assert(tlb[lru].trieHandle);
    // JUST FIND, NOT ERASE!
    //trie.remove(tlb[lru].trieHandle);
    //tlb[lru].trieHandle = NULL;
    //freeList.push_back(&tlb[lru]);
    return tlb[lru];
}

void
TLB::sendEntryToHigherLevel(Addr vpn, TlbEntry &entry, bool fromMemory)
{
    // boolMemory shows if this entry was brought in from memory (true)
    // or if it is just being sent to higher level from DTB2 (false)
    X86ISA::TLB *dtb2 = (X86ISA::TLB *)upper_tlb;
    X86ISA::TLB *dtb = (X86ISA::TLB *)dtb2->upper_tlb;
    std::list<TlbEntry *> dtb_freelist = dtb->getFreeList();
    if (fromMemory) {
        if (dtb_freelist.empty()) {
            // if upper level TLB (DTB) is full,
            // we evict its LRU victim and bring it to a lower level
            TlbEntry victim = dtb->findLRU();
            // 1. check which set this address belongs to
            unsigned int masknum = dtb2->associativity - 1;
            unsigned int num = (victim.vaddr & (masknum << 12)) >> 12;
            X86ISA::TLB *correct_set = (X86ISA::TLB *)((*dtb2->UnifiedTLB)[num]);
            // 2. check if correct set is full
            if (correct_set->getFreeList().empty()) {
                // if set full, evict LRU before adding
                correct_set->evictLRU();
            }
            // 3. add victim entry from upper level TLB (DTB) to DTB2
            Addr victim_vpn = (victim.vaddr) & ~mask(victim.logBytes);
            correct_set->insert(victim_vpn, victim);
            // 4. delete entry from DTB
            dtb->evictLRU();
            // 5. add entry to DTB
            dtb->insert(vpn, entry);
        }
        else {
            // if upper level TLB (DTB) is not full,
            // just send the newly brought in entry
            dtb->insert(vpn, entry);
        }
        return;
    }
    // reaching here means fromMemory == false
    assert(!fromMemory);
    //std::vector<TlbEntry> dtb_tlb = dtb->getTlb();
    //TlbEntryTrie dtb_trie = dtb->getTrie();
    if (dtb_freelist.empty()) {
        // if upper level TLB (DTB) is full,
        // we evict its LRU victim and bring it to a lower level
        TlbEntry victim = dtb->findLRU();
        // 1. check which set this address belongs to
        unsigned int masknum = dtb2->associativity - 1;
        unsigned int num = (victim.vaddr & (masknum << 12)) >> 12;
        X86ISA::TLB *correct_set = (X86ISA::TLB *)((*dtb2->UnifiedTLB)[num]);
        // 2. check if correct set is full
        if (correct_set->getFreeList().empty()) {
            // if set full, evict LRU before adding
            // check if the victim from L2 TLB is DTB2 LRU victim
            // if so, evict 'second' LRU
            // this makes the L2 TLB not completely LRU
            // but this should not make a big difference
            // further information will be provided in the project report
            if (correct_set->findLRU().vaddr == entry.vaddr)
                correct_set->evictSecondLRU();
            else
                correct_set->evictLRU();
        }
        // 3. add victim entry from upper level TLB (DTB) to DTB2
        Addr victim_vpn = (victim.vaddr) & ~mask(victim.logBytes);
        correct_set->insert(victim_vpn, victim);
        // 4. delete entry from DTB
        dtb->evictLRU();
        // 5. add entry to DTB
        dtb->insert(vpn, entry);
    }
    else {
        // if upper level TLB (DTB) is not full,
        // just send and evict from current TLB (DTB2)
        // not sure if this is possible in single-core execution...
        // 1. add to upper level TLB (DTB)
        dtb->insert(vpn, entry);
        // 2. remove from lower level TLB (DTB2, this one)
        for (unsigned x = 0; x < size; x++) {
            if (tlb[x].vaddr == entry.vaddr) {
                trie.remove(tlb[x].trieHandle);
                tlb[x].trieHandle = NULL;
                freeList.push_back(&tlb[x]);
                break;
            }
        }
    }
}

TlbEntry *
TLB::insert(Addr vpn, const TlbEntry &entry)
{
    // If somebody beat us to it, just use that existing entry.
    TlbEntry *newEntry = trie.lookup(vpn);
    if (newEntry) {
        assert(newEntry->vaddr == vpn);
        return newEntry;
    }

    if (freeList.empty())
        evictLRU();

    newEntry = freeList.front();
    freeList.pop_front();

    *newEntry = entry;
    newEntry->lruSeq = nextSeq();
    newEntry->vaddr = vpn;
    newEntry->trieHandle =
    trie.insert(vpn, TlbEntryTrie::MaxBits - entry.logBytes, newEntry);
    return newEntry;
}

TlbEntry *
TLB::lookup(Addr va, bool update_lru)
{
    TlbEntry *entry = trie.lookup(va);
    if (entry && update_lru)
        entry->lruSeq = nextSeq();
    return entry;
}

void
TLB::flushAll()
{
    DPRINTF(TLB, "Invalidating all entries.\n");
    for (unsigned i = 0; i < size; i++) {
        if (tlb[i].trieHandle) {
            trie.remove(tlb[i].trieHandle);
            tlb[i].trieHandle = NULL;
            freeList.push_back(&tlb[i]);
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
    for (unsigned i = 0; i < size; i++) {
        if (tlb[i].trieHandle && !tlb[i].global) {
            trie.remove(tlb[i].trieHandle);
            tlb[i].trieHandle = NULL;
            freeList.push_back(&tlb[i]);
        }
    }
}

void
TLB::demapPage(Addr va, uint64_t asn)
{
    TlbEntry *entry = trie.lookup(va);
    if (entry) {
        trie.remove(entry->trieHandle);
        entry->trieHandle = NULL;
        freeList.push_back(entry);
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
    /* Here, we set UnifiedTLB  of DTB2 if it is DTB's first time accessing */
    if (need_init && (((TLB *)controller)->size != 0)) {
        controller->UnifiedTLB = (std::vector<BaseTLB *> *)new std::vector<X86ISA::TLB *>();
        
        assert(((X86ISA::TLB *)controller)->associativity != 1);
        for (unsigned int x = 0; x < ((X86ISA::TLB *)controller)->associativity; x++) {
            Params new_params = ((TLB *)controller)->old_params;
            new_params.associativity = 1;
            new_params.size = ((TLB *)controller)->size / ((X86ISA::TLB *)controller)->associativity;
            new_params.controller = 0;
            new_params.need_init = false;
            //new_params.last_level = true;
            new_params.name = "system.cpu.dtb2." + std::to_string(x);
            //new_params.upper_tlb = (uint64_t)controller;
            TLB *newTLB = new TLB(&new_params);newTLB->last_level = true;
            newTLB->last_level = true;
            newTLB->upper_tlb = controller;
            controller->UnifiedTLB->push_back(newTLB);
        }
        need_init = false;
    }
    else if (need_init && (((TLB *)controller)->size == 0)) {
        // single-level TLB
        need_init = false;
        //controller = nullptr;
        last_level = true;
    }
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
            // entry NOT found in TLB
            if (!entry) {
                DPRINTF(TLB, "Handling a TLB miss for "
                        "address %#x at pc %#x.\n",
                        vaddr, tc->instAddr());
                if (mode == Read) {
                    rdMisses++;
                } else {
                    wrMisses++;
                }
                // multi-level TLB
                // if it is DTB, just return after letting lower level handle
                if (!last_level) {
                    // DTB2 should call its set-associative TLBs here
                    unsigned int masknum = ((X86ISA::TLB *)controller)->associativity - 1;
                    assert(masknum);
                    unsigned int num = (vaddr & (masknum << 12)) >> 12;
                    TLB *correct_tlb = (TLB *)((*controller->UnifiedTLB)[num]);
                    return correct_tlb->translateLowerLevel(req, tc, translation,
                                                mode, delayedResponse, timing);
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
                        entry = insert(alignedVaddr, TlbEntry(
                                p->pTable->pid(), alignedVaddr, pte->paddr,
                                pte->flags & EmulationPageTable::Uncacheable,
                                pte->flags & EmulationPageTable::ReadOnly));
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
    // Only store the entries in use.
    uint32_t _size = size - freeList.size();
    SERIALIZE_SCALAR(_size);
    SERIALIZE_SCALAR(lruSeq);

    uint32_t _count = 0;
    for (uint32_t x = 0; x < size; x++) {
        if (tlb[x].trieHandle != NULL)
            tlb[x].serializeSection(cp, csprintf("Entry%d", _count++));
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

    for (uint32_t x = 0; x < _size; x++) {
        TlbEntry *newEntry = freeList.front();
        freeList.pop_front();

        newEntry->unserializeSection(cp, csprintf("Entry%d", x));
        newEntry->trieHandle = trie.insert(newEntry->vaddr,
            TlbEntryTrie::MaxBits - newEntry->logBytes, newEntry);
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

// translate function called inside TLB::translate()
// it is expected that this function is called at
// L2 set-associative DTLBs' translations to handle
// L1 DTLB (DTB)'s TLB miss
Fault
X86ISA::TLB::translateLowerLevel(const RequestPtr &req,
        ThreadContext *tc, Translation *translation,
        Mode mode, bool &delayedResponse, bool timing)
{
    // check if this is DTB2's set-associative TLB
    assert(last_level);
    assert(associativity == 1);
    // upper_tlb == DTB2
    // upper_tlb->upper_tlb = DTB
    X86ISA::TLB *dtb2 = (X86ISA::TLB *)upper_tlb;
    X86ISA::TLB *dtb = (X86ISA::TLB *)dtb->upper_tlb;
    assert(upper_tlb);

    Request::Flags flags = req->getFlags();
    int seg = flags & SegmentFlagMask;
    bool storeCheck = flags & (StoreCheck << FlagShift);
    /* 1 - hit in DTB2? */
    delayedResponse = false;
    assert(seg != SEGMENT_REG_MS);

    Addr vaddr = req->getVaddr();
    DPRINTF(TLB, "Translating vaddr %#x.\n", vaddr);

    HandyM5Reg m5Reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
    // Protection mode must be enabled to reach this function
    assert(m5Reg.prot);
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
    // Paging must be enabled to reach this function
    assert(m5Reg.paging);
    DPRINTF(TLB, "Paging enabled.\n");
    // The vaddr already has the segment base applied.
    TlbEntry *entry = lookup(vaddr);
    if (mode == Read) {
        rdAccesses++;
        dtb2->incrementRdAccesses();
    } else {
        wrAccesses++;
        dtb2->incrementWrAccesses();
    }
    // entry NOT found in DTB2 -> bring in and move to DTB
    if (!entry) {
        DPRINTF(TLB, "Handling a TLB miss for "
                "address %#x at pc %#x.\n",
                vaddr, tc->instAddr());
        if (mode == Read) {
            rdMisses++;
            dtb2->incrementRdMisses();
        } else {
            wrMisses++;
            dtb2->incrementWrMisses();
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
                /*
                entry = insert(alignedVaddr, TlbEntry(
                        p->pTable->pid(), alignedVaddr, pte->paddr,
                        pte->flags & EmulationPageTable::Uncacheable,
                        pte->flags & EmulationPageTable::ReadOnly));
                */
                entry = new TlbEntry(p->pTable->pid(), alignedVaddr, pte->paddr,
                                pte->flags & EmulationPageTable::Uncacheable,
                                pte->flags & EmulationPageTable::ReadOnly);
                // check if entry is VALID
                bool inUser = (m5Reg.cpl == 3 &&
                        !(flags & (CPL0FlagBit << FlagShift)));
                CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
                bool badWrite = (!entry->writable && (inUser || cr0.wp));
                // INVALID entry found -> raise Page Fault
                if ((inUser && !entry->user) || (mode == Write && badWrite)) {
                    return std::make_shared<PageFault>(vaddr, true, mode, inUser, false);
                }
                if (storeCheck && badWrite) {
                    return std::make_shared<PageFault>(vaddr, true, Write, inUser, false);
                }
                // CORRECT entry found -> send to DTB
                sendEntryToHigherLevel(alignedVaddr, *entry, true);
                Addr paddr = entry->paddr | (vaddr & mask(entry->logBytes));
                DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
                req->setPaddr(paddr);
                if (entry->uncacheable)
                    req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);

                return finalizePhysical(req, tc, mode);
            }
            DPRINTF(TLB, "Miss was serviced.\n");
        }
    }
    // entry found in DTB2
    DPRINTF(TLB, "Entry found with paddr %#x, "
            "doing protection checks.\n", entry->paddr);
    // check if entry is VALID
    bool inUser = (m5Reg.cpl == 3 &&
            !(flags & (CPL0FlagBit << FlagShift)));
    CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
    bool badWrite = (!entry->writable && (inUser || cr0.wp));
    // INVALID entry found -> raise Page Fault
    if ((inUser && !entry->user) || (mode == Write && badWrite)) {
        return std::make_shared<PageFault>(vaddr, true, mode, inUser,
                                            false);
    }
    if (storeCheck && badWrite) {
        return std::make_shared<PageFault>(vaddr, true, Write, inUser,
                                            false);
    }
    // CORRECT entry found -> move to DTB and swap with DTB's LRU
    Addr vpn = (entry->vaddr) & ~mask(entry->logBytes);
    sendEntryToHigherLevel(vpn, *entry, false);
    Addr paddr = entry->paddr | (vaddr & mask(entry->logBytes));
    DPRINTF(TLB, "Translated %#x -> %#x.\n", vaddr, paddr);
    req->setPaddr(paddr);
    if (entry->uncacheable)
        req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);

    return finalizePhysical(req, tc, mode);
}