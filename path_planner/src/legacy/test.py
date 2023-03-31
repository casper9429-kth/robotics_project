
import math
import heapq
import sys

from A_star_testing import Node

"""heap = []
heapq.heapify(heap)
startnode = Node(0.0,0.0,(10.0,10.0))
heapq.heappush(heap, (startnode.f,startnode))
for i in range(1,10):
    node = Node(float(i),float(i),(10.0,10.0),startnode)
    heapq.heappush(heap, (node.f-i,node))

while heap:
    node = heapq.heappop(heap)
    print (node[0],node[1].x,node[1].y,node[1].f)"""

lst = [i for i in range(3)]
print(lst)
biglist = [i for i in range(10)]
print(biglist)
for i in lst:
    if i in biglist:
        biglist.pop(i)
        #biglist.remove(i)
print(biglist)