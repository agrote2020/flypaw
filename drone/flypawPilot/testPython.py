from __future__ import annotations
from dataclasses import dataclass, field

from ast import Str
from turtle import position
import json
import jsonpickle
import json
import copy
from geographiclib.geodesic import Geodesic
import importlib
import os
import sys
script_dir = os.path.dirname( 'flypawClasses.py' )
mymodule_dir = os.path.join( script_dir, '..','..','basestation', 'basestationAgent' )
#sys.path.append( 'C:\\Users\\Andrew\\Documents\\delmont\\flypaw\\basestation\\basestationAgent' )
sys.path.append( 'C:\\Users\\andgr\\OneDrive\\Documents\\delmont\\flypaw\\basestation\\basestationAgent' )

from flypawClasses import *


tq:TaskQueue = TaskQueue()
tq.Count = 1
# #stringJ = tq.to_json()
# jsonobj = jsonpickle.encode(tq)
# print(jsonobj)

# #print(json.dumps(tq.__dict__))


# #print(stringJ)
#with open('C:\\Users\\Andrew\\Documents\\delmont\\flypaw\\drone\\flypawPilot\\json_dump_q.txt','r') as f: #~~~~~~~~~~~~~for desktop~~~~~~~~~~~
with open("C:\\Users\\andgr\\OneDrive\\Documents\\delmont\\flypaw\\drone\\flypawPilot\\json_dump_q.txt",'r') as f: #for laptop
    tq = jsonpickle.decode(f.read())

with open("C:\\Users\\andgr\\OneDrive\\Documents\\delmont\\flypaw\\drone\\flypawPilot\\json_dump_wph.txt",'r') as f: #for laptop
    wph:WaypointHistory = jsonpickle.decode(f.read())

with open("C:\\Users\\andgr\\OneDrive\\Documents\\delmont\\flypaw\\drone\\flypawPilot\\json_dump_t.txt",'r') as f: #for laptop
    t = jsonpickle.decode(f.read())
with open("C:\\Users\\andgr\\OneDrive\\Documents\\delmont\\flypaw\\drone\\flypawPilot\\json_dump_id.txt",'r') as f: #for laptop
    id:TaskIDGenerator = jsonpickle.decode(f.read())

#print(tq)
print('count:' + str(tq.Count))
print('count:' + str(wph.Count))
print("CURRENT " + str(id.CurrentTaskID))
#tq.PrintQ()

emptyList = list()
root:Node =  Node(0,tq,t,0,0,wph,id,emptyList)
tree:PredictiveTree = PredictiveTree(root)
tree.HaltPoint(root)
tree.PrintNodes()
tree.BranchAnalyze()
tree.testBranchDistance(tree.BranchNodes[0])


#print("empty?"+str(bool(wy._empty())))
#if(not wy._empty()):
#    print("wrong")


