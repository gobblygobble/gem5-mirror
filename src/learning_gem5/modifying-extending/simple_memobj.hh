#ifndef __LEARNING_GEM5_SIMPLE_MEMOBJ_HH__
#define __LEARNING_GEM5_SIMPLE_MEMOBJ_HH__

#include <vector>

#include "mem/port.hh"
#include "params/SimpleMemobj.hh"
#include "sim/sim_object.hh"

class SimpleMemobj : public SimObject
{
    private:

        class CPUSidePort : public SlavePort
        {
            private:
                SimpleMemobj *owner;
                bool needRetry;
                PacketPtr blockedPacket;

            public:
                CPUSidePort(const std::string& name, SimpleMemobj *owner) :
                    SlavePort(name, owner), owner(owner), needRetry(false), blockedPacket(nullptr)
                { }

                void sendPacket(PacketPtr pkt);
                AddrRangeList getAddrRanges() const override;
                void trySendRetry();

            protected:
                Tick recvAtomic(PacketPtr pkt) override { panic("recvAtomic unimpl."); }
                void recvFunctional(PacketPtr pkt) override;
                bool recvTimingReq(PacketPtr pkt) override;
                void recvRespRetry() override;
        };

        class MemSidePort : public MasterPort
        {
            private:
                SimpleMemobj *owner;
                PacketPtr blockedPacket;

            public:
                MemSidePort(const std::string& name, SimpleMemobj *owner) :
                    MasterPort(name, owner), owner(owner), blockedPacket(nullptr)
                { }

                void sendPacket(PacketPtr pkt);

            protected:
                bool recvTimingResp(PacketPtr pkt) override;
                void recvReqRetry() override;
                void recvRangeChange() override;
        };

        bool handleRequest(PacketPtr pkt);
        bool handleResponse(PacketPtr pkt);
        void handleFunctional(PacketPtr pkt);
        AddrRangeList getAddrRanges() const;
        void sendRangeChange();

        CPUSidePort instPort;
        CPUSidePort dataPort;
        MemSidePort memPort;

        bool blocked;

    public:
        SimpleMemobj(SimpleMemobjParams *params);
        /*
        BaseMasterPort& getMasterPort(const std::string& if_name,
                                    PortID idx = InvalidPortID) override;

        BaseSlavePort& getSlavePort(const std::string& if_name,
                                    PortID idx = InvalidPortID) override;
        */
        Port &getPort(const std::string &if_name, PortID idx=InvalidPortID) override;
};

#endif // __LEARNING_GEM5_SIMPLE_MEMOBJ_HH__
