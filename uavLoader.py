import os
import sys

if len(sys.argv) != 2:
    print "ARGUMENT COUNT ERORORORORO!"
    sys.exit()

const_uav_count = int(sys.argv[1])
prefix = " --tab -e 'python main.py "
final_os_command = "gnome-terminal"
for next_id in range(const_uav_count):
    final_os_command += prefix + str(next_id) + "'"
os.system(final_os_command)
#print final_os_command