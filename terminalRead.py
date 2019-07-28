import os
import re

path = '/home/bharat/Downloads'

entries = os.listdir(path)
cleanr = re.compile('<.*?>')

for files in entries:
    if files.endswith('bluetooth_content_share.html'):
        print(files, end='\n\n')
        fname = '/home/bharat/Downloads/'+files
        f = open(fname, 'r')
        print(re.sub(cleanr, '', f.read()))
        print(type(re.sub(cleanr, '', f.read())))
        os.remove(fname)

