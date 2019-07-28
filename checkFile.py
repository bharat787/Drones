import os
import re

path = '/home/bharat/Downloads'

entries = os.listdir(path)
cleanr = re.compile('<.*?>')

for files in entries:
    if files.endswith('bluetooth_content_share.html'):
        print(files)
        fname = '/home/bharat/Downloads/'+files
        f = open(fname, 'r')
        v = re.sub(cleanr, '', f.read())
        with open("values.txt", 'w') as fyl:
            fyl.write(v)
        val = (v.split(','))
        size = len(val)
        print(v)
        if(size>1):
            print("goto")
            os.system('python /home/bharat/Desktop/dk/Drones/sendTo.py')
        else:
            print('hover')
            os.system('python /home/bharat/Desktop/dk/Drones/hover.py')
        os.remove(fname)



