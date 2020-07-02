import os

check_enabled = False
for filename in os.listdir(os.getcwd()):
    with open("parsed_data.csv", 'a+') as wf:
        with open(os.path.join(os.getcwd(), filename), 'r') as f:
            if not filename.endswith(".txt"):
                continue
            file_string = filename
            file_string += ","
            for line in f:
                if line.startswith("system.l2.overall_hits::.cpu0.inst"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_hits::.cpu0.data"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_hits::.cpu1.inst"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_hits::.cpu1.data"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_hits::total"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_misses::.cpu0.inst"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_misses::.cpu0.data"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_misses::.cpu1.inst"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_misses::.cpu1.data"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_misses::total"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_miss_rate::.cpu0.inst"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_miss_rate::.cpu0.data"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_miss_rate::.cpu1.inst"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_miss_rate::.cpu1.data"):
                    file_string += line.split()[1]
                    file_string += ","
                elif line.startswith("system.l2.overall_miss_rate::total"):
                    file_string += line.split()[1]
                    file_string += ","
                
            if check_enabled:
                print(file_string)
                file_list = file_string.split(",")
                if (int(file_list[5]) != int(file_list[1]) + int(file_list[2]) + int(file_list[3]) + int(file_list[4])):
                    print("something wrong with file hit sum in {}".format(filename))
                if (int(file_list[10]) != int(file_list[6]) + int(file_list[7]) + int(file_list[8]) + int(file_list[9])):
                    print("something wrong with file hit sum in {}".format(filename))
                file_string += '\n'
            wf.write(file_string)