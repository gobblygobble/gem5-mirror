import m5
from m5.objects import *
from caches import *
from optparse import OptionParser

# adding parameters to script
parser = OptionParser()
parser.add_option('--l1i_size', help = "L1 instruction cache size")
parser.add_option('--l1d_size', help = "L1 data cache size")
parser.add_option('--l2_size', help = "Unified L2 cache size")
options, args = parser.parse_args()

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
# we don't care about system power -> use default options for voltage domain
system.clk_domain.voltage_domain = VoltageDomain()

# memory simulation will usually be in timing mode
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange('512MB')]
# 'most simple' timing=based CPU
system.cpu = TimingSimpleCPU()
system.membus = SystemXBar()

# create I & D caches after CPU creation
system.cpu.icache = L1ICache(options)
system.cpu.dcache = L1DCache(options)
# connect caches to CPU ports
system.cpu.icache.connectCPU(system.cpu)
system.cpu.dcache.connectCPU(system.cpu)

# connect cache ports on the CPU to the memory bus
# THIS EXAMPLE SYSTEM HAS CACHES -> DO NOT connect caches to membus directly
#system.cpu.icache_port = system.membus.slave
#system.cpu.dcache_port = system.membus.slave
# requests are sent from master to slave, like: memobject1.master = memobject2,slave

# create L2 bus & connect I & D caches to the L2 bus
system.l2bus = L2XBar()
system.cpu.icache.connectBus(system.l2bus)
system.cpu.dcache.connectBus(system.l2bus)
# create L2 cache and connect to L2 bus & memory bus
system.l2cache = L2Cache(options)
system.l2cache.connectCPUSideBus(system.l2bus)
system.l2cache.connectMemSideBus(system.membus)

# connect few other ports - create I/O controller on CPU and connect to memory bus
# connecting PIO and interrupt ports to memory bus is x86-specific requirement
system.cpu.createInterruptController()
system.cpu.interrupts[0].pio = system.membus.master
system.cpu.interrupts[0].int_master = system.membus.slave
system.cpu.interrupts[0].int_slave = system.membus.master
# connect a special port in the system up to the memory bus (functional-only, to allow system to read and write memory)
system.system_port = system.membus.slave

# create memory controller and connect it to the memory bus
system.mem_ctrl = DDR3_1600_8x8()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.master

# create process and set processes command to the command we want to run, the list can be used like argv
# set CPU to use the process as its workload, and create functional execution contexts in CPU
process = Process()
process.cmd = ['tests/test-progs/hello/bin/x86/linux/hello']
system.cpu.workload = process
system.cpu.createThreads()

# create Root object and instantiate simulation
root = Root(full_system = False, system = system)
m5.instantiate()

# actual simulation takes place here
print("Begin simulation")
exit_event = m5.simulate()
#print("L1I, L1D, L2: ", system.cpu.icache.size, system.cpu.dcache.size, system.l2cache.size)
print("Exiting @ tick {} because {}".format(m5.curTick(), exit_event.getCause()))

