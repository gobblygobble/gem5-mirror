import os

name1 = "soplex"
name2 = "hmmer"
check_enabled = True
for filename in os.listdir(os.getcwd()):
    with open("pair_data.csv", 'a+') as wf:
        with open(os.path.join(os.getcwd(), filename), 'r') as f:
            if not (name1 in filename and name2 in filename):
                continue
            for line in f:
                if line.startswith("system.l2.overall_hits::.cpu0.inst"):
                    cpu0_i_hits = int(line.split()[1])
                elif line.startswith("system.l2.overall_hits::.cpu0.data"):
                    cpu0_d_hits = int(line.split()[1])
                elif line.startswith("system.l2.overall_hits::.cpu1.inst"):
                    cpu1_i_hits = int(line.split()[1])
                elif line.startswith("system.l2.overall_hits::.cpu1.data"):
                    cpu1_d_hits = int(line.split()[1])
                elif line.startswith("system.l2.overall_hits::total"):
                    overall_hits = int(line.split()[1])
                elif line.startswith("system.l2.overall_misses::.cpu0.inst"):
                    cpu0_i_misses = int(line.split()[1])
                elif line.startswith("system.l2.overall_misses::.cpu0.data"):
                    cpu0_d_misses = int(line.split()[1])
                elif line.startswith("system.l2.overall_misses::.cpu1.inst"):
                    cpu1_i_misses = int(line.split()[1])
                elif line.startswith("system.l2.overall_misses::.cpu1.data"):
                    cpu1_d_misses = int(line.split()[1])
                elif line.startswith("system.l2.overall_misses::total"):
                    overall_misses = int(line.split()[1])
                
            if check_enabled:
                if overall_hits != (cpu0_i_hits + cpu0_d_hits + cpu1_i_hits + cpu1_d_hits):
                    print("something wrong with hit sum in {}".format(filename))
                if overall_misses != (cpu0_i_misses + cpu0_d_misses + cpu1_i_misses + cpu1_d_misses):
                    print("something wrong with miss sum in {}".format(filename))
            
            file_list = [filename, str(cpu0_i_hits), str(cpu0_d_hits), str(cpu1_i_hits), str(cpu1_d_hits), str(overall_hits)]
            file_list.append(str(cpu0_i_hits + cpu0_d_hits + cpu0_i_misses + cpu0_d_misses))
            file_list.append(str(cpu1_i_hits + cpu1_d_hits + cpu1_i_misses + cpu1_d_misses))
            file_list.append(str(overall_hits + overall_misses))
            file_list.append(str(overall_hits / (overall_hits + overall_misses)))
            file_string = ",".join(file_list) + "\n"
            wf.write(file_string)