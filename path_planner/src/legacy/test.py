
import math
import heapq
import sys

l1 = [x for x in range(10)]
l2= [x for x in range(3)]
check = all(item in l1 for item in l2)

if check:
    print('im in')