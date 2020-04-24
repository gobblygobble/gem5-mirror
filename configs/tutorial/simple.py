import m5
from m5.objects import *

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

# connect cache ports on the CPU to the memory bus
# THIS EXAMPLE SYSTEM HAS NO CACHES -> connect I-cache and D-cache directly to memory bus
system.cpu.icache_port = system.membus.slave
system.cpu.dcache_port = system.membus.slave
# requests are sent from master to slave, like: memobject1.master = memobject2,slave

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
print("Exiting @ tick {} because {}".format(m5.curTick(), exit_event.getCause()))

