import m5
from m5.objects import *

# instantiate Root object, as required by all gem5 instances
root = Root(full_system = False)
# instantiate HelloObject created
# make sure it is a child of the root object!!
# only SimObjects that are children of the Root are instantiated in C++
root.hello = HelloObject(time_to_wait = '2us', number_of_fires = 5)
root.hello.goodbye_object = GoodbyeObject(buffer_size='100B')

m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print("Exiting @ tick {} because {}".format(m5.curTick(), exit_event.getCause()))

