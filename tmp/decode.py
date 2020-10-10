import os

path = os.path.join('./inst_golden.txt')

arr = []
with open(path, 'r') as f:
    arr = f.readlines()


for s in arr:
    h = hex(int(s.rstrip('\n'), 2))
    print(h)
