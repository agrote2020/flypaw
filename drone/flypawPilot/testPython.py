from __future__ import annotations
from dataclasses import dataclass, field
from dataclass_wizard import JSONWizard
from ast import Str
from turtle import position
import json




class RadioMap(object):
    def __init__(self):
        self.lats = []
        self.lons = []
        self.headings = []
        self.dataRate = []
        self.length = 0
    def add(self, lat, lon, heading, rate):
        self.lats.append(lat)
        self.lons.append(lon)
        self.headings.append(heading)
        self.dataRate.append(rate)
        self.length = self.length + 1

class Task(object):
    def __init__(self, pos,task,sensitive,prio):
        self.position = pos
        self.task = task
        self.TimeSensitive = sensitive
        self.priority = prio

@dataclass
class TaskQueue(JSONWizard):

    def __init__(self):
        self.queue = []
        self.Count = 0
    def PushTask(self, task:Task):
        self.queue.insert(0,task)
        self.Count =  self.Count + 1
    def PopTask(self):
        if(self.queue):
            self.Count = self.Count-1 #Adjust count
            return self.queue.pop(self.Count)#Pop item at end of queue
    def PrintQ(self):#change this to lower case please
        if len(self.queue) < 1:
            print("Empty!")
        else:
            for idx, task in enumerate(self.queue):
                print("Task#: "+str(idx)+" Lat:"+ str(task.position.lat)+ " Lon:"+ str(task.position.lon)+" Alt:"+ str(task.position.alt) )

class WaypointHistory(object):
    def __init__(self):
        self.TrueWaypointsAndConnection =[]
        self.WaypointsAndConnection = []#list of Tuples (waypoint,connection_status,id)
        self.Count = 0
        self.TrueCount = 0
    def _empty(self):
        if(self.Count<1):
            return 1
        else:
            return 0

    def AddPoint(self,Waypoint,Connected):#compresses into tuple
        self.WaypointsAndConnection.insert(0,(Waypoint,Connected,self.TrueCount))
        self.TrueWaypointsAndConnection.insert(0,(Waypoint,Connected,self.TrueCount))
        self.Count = self.Count + 1
        self.TrueCount = self.TrueCount+1
    def StackPop(self):#return tuple
        if(not self._empty()):
            self.Count = self.Count - 1
            return self.WaypointsAndConnection.pop(0)

        else:
            return None

    def Peek(self):


        if(not self._empty()):
            return self.WaypointsAndConnection(0)
        else:
            return None


    def PeekConnectivity(self):
        if(not self._empty()):
            tuple = self.WaypointsAndConnection(0)
            return tuple[1]
        else:
            return None


    def BackTrackPathForConnectivity(self):
        Connected = 0
        print("Count?: "+str(self.Count))
        StartingLocation = self.StackPop()
        print("Count2?: "+str(self.Count))
        StepsBack = []
        StepsForward = []
        tasks = []
        StepsForward.insert(0,StartingLocation)
        print ("Empty?: "+str(bool(self._empty)))
        while((not Connected)and (not self._empty)):
            if(self.PeekConnectivity()):
                Step = self.StackPop()
                print("Step Popped: "+ str(Step))
                StepsBack.append(0,Step)
                Connected = 1
            else:
                Step = self.StackPop()
                print("Step Popped: "+ str(Step))
                StepsBack.append(Step)
                StepsForward.insert(0,Step)
        if(self._empty and (not Connected)):
            print("BackTrackError1")
            return None
        else:
            StepsBack.extend(StepsForward)
            return StepsBack
    def PrintWorkingHistory(self):
        print("WorkingHistory:")
        for tuple in self.TrueWaypointsAndConnection:
            print("ID: "+ str(tuple[2]) + " Position: "+ str(tuple[0])+ " Connected: "+ str(bool(tuple[1])))


    def PrintListOfStepsGeneric(self,list):
        for tuple in list:
            print("ID: "+ str(tuple[2]) + " Position: "+ str(tuple[0])+ " Connected: "+ str(bool(tuple[1])))


    



class Position(object):
    """
    lon: float units degrees (-180..180)
    lat: float units degrees (-90..90)
    alt: float units M AGL
    time: str, iso8601 currently
    fix_type: int (0..4), 0-1 = no fix, 2 = 2D fix, 3 = 3D fix
    satellites_visible: int (0..?)
    """
    def __init__(self):
        self.lon = float
        self.lat = float
        self.alt = float
        self.time = str
        self.fix_type = int
        self.satellites_visible = int
    def InitParams(self,lon,lat,alt,time,fix,satellites):
        self.lon = float
        self.lat = float
        self.alt = float
        self.time = str
        self.fix_type = int
        self.satellites_visible = int

def runIperf():

        msg = {}
        msg['uuid'] = str(5)
        msg['type'] = "iperfResults"
        msg['iperfResults'] = {}










        msg['iperfResults']['protocol'] = "tcp"
        msg['iperfResults']['location4d'] = [ 1,2,3,4 ]
        msg['iperfResults']['heading'] = [ 33 ]


        return msg

a = RadioMap()
p = Position()
p.InitParams(1,2,3,4,5,6)
wy = WaypointHistory()
wy.AddPoint(p,1)
wy.AddPoint(p,1)
wy.AddPoint(p,1)
wy.AddPoint(p,1)
wy.StackPop()
tq = TaskQueue()

with open('json_dump_q.txt','w') as f:
    f.write("HERE!")

#print("empty?"+str(bool(wy._empty())))
#if(not wy._empty()):
#    print("wrong")


