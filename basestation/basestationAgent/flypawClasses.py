#!/usr/bin/env python3
from __future__ import annotations
from dataclasses import dataclass, field
from dataclass_wizard import JSONWizard
import json
import copy
from pickle import FALSE
from queue import Empty
from geographiclib.geodesic import Geodesic


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
        self.lon = lon
        self.lat = lat
        self.alt = alt
        self.time = time
        self.fix_type = fix
        self.satellites_visible = satellites
    def __str__(self):
        return "(Lat:" + str(self.lat) + " Lon:" + str(self.lon) + " Altitude:" + str(self.alt) + ")"
    def __repr__(self):
        return "(Lat:" + str(self.lat) + " Lon:" + str(self.lon) + " Altitude:" + str(self.alt) + ")"

class Battery(object):
    """
    voltage: float units V
    current: float units mA
    level: int unitless (0-100)
    m_kg: battery mass, units kg
    """
    def __init__(self):
        self.voltage = float
        self.current = float
        self.level = float
        self.m_kg = float
        
class iperfInfo(object):
    def __init__(self, ipaddr="172.16.0.1", port=5201, protocol="tcp", priority=0, mbps=0, meanrtt=0):
        self.ipaddr = ipaddr #string server ip address
        self.port = port #string server port address 
        self.protocol = protocol #tcp, udp
        self.priority = 1 #normalized float 0-1         
        self.mbps = mbps #float, units mbps, representing throughput
        self.meanrtt = meanrtt #float, units ms, representing latency
        self.location4d = [float, float, float, str]
        
class collectVideoInfo(object):
    def __init__(self, dataformat="jpgframes", duration=5, quality=100, priority = 1):
        self.dataformat = dataformat #jpgframes, ffmpeg, etc
        self.duration = duration #units seconds
        self.quality = quality #arbitrary unit
        self.priority = priority #normalized float 0-1

class sendFrameInfo(object):
    def __init__(self, dataformat="jpgframes", ipaddr="172.16.0.1", port="8096", priority=1):
        self.dataformat = dataformat #jpgframes, ffmpeg, etc
        self.ipaddr = ipaddr #string ip address
        self.port = port #int port number
        self.priority = priority #normalized float 0-1
        
class sendVideoInfo(object):
    def __init__(self, dataformat="jpgframes", ipaddr="172.16.0.1", port="23000", priority=1):
        self.dataformat = dataformat #jpgframes, ffmpeg, etc
        self.ipaddr = ipaddr #string ip address
        self.port = port #int port number 
        self.priority = priority #normalized float 0-1

class flightInfo(object):
    def __init__(self):
        """
        coords : [float,float]--> [lon, lat]
        altitude: float --> M AGL(?)
        airspeed: float --> 
        """
        self.coords = [] #[lon, lat]
        self.altitude = float #meters 
        self.airspeed = float #airspeed 
        self.groundspeed = float #groundspeed
        self.priority = float #normalized float 0-1

class missionInfo(object):
    #we'll have to think this through for different mission types
    def __init__(self):
        self.defaultWaypoints = [] #planfile
        self.tasks = []#tasks associated with each waypoint
        self.missionType = str #videography, delivery, air taxi, etc.
        self.missionLeader = str #basestation, drone, cloud, edge device(s)
        self.priority = float #normalized float from 0-1
        self.planfile = str #path to planfile optional 
        self.name = str #the name of the mission
        self.resources = bool #true-> outside resources/edge devices false-> just drone and basestation
        self.STATUS = str
        self.missionObjectives = []

class MissionObjective(object):
        def __init__(self,way,type,static):
            self.Waypoint = way
            self.Type = type
            self.Static = static

        
class resourceInfo(object):
    def __init__(self):
        self.name = str #identifier for resource
        self.location = str #edge, cloud x, cloud y
        self.purpose = str #mission related I guess
        self.interface = str #thinking something like direct vs kubectl
        self.resourceAddresses = [] #one or more ways to communicate with resource... possibly a pairing? eg ("management", "xxx.xxx.xxx.xxx")
        self.state = str #resource reservation state
        self.load = float #placeholder for now... maybe if we have info from prometheus or something
    
class VehicleCommands(object):#This is like a task?
    def __init__(self):
        self.commands = {}
        self.commands['iperf'] = {}
        self.commands['sendFrame'] = {}
        self.commands['sendVideo'] = {} 
        self.commands['collectVideo'] = {}
        self.commands['flight'] = {}
        
    def setIperfCommand(self, iperfObj):
        self.commands['iperf'] = { "command" : "iperf", "protocol": iperfObj.protocol, "ipaddr": iperfObj.ipaddr, "port": iperfObj.port, "priority": iperfObj.priority } 
    def setCollectVideoCommand(self, collectVideoObj):
        self.commands['collectVideo'] = { "command" : "collectVideo", "dataformat" : collectVideoObj.dataformat, "duration": collectVideoObj.duration, "quality": collectVideoObj.quality, "priority": collectVideoObj.priority }
    def setSendFrameCommand(self, sendFrameObj):
        self.commands['sendFrame'] = { "command" : "sendFrame", "dataformat" : sendFrameObj.dataformat, "ipaddr": sendFrameObj.ipaddr, "port": sendFrameObj.port, "priority": sendFrameObj.priority  }
    def setSendVideoCommand(self, sendVideoObj):
        self.commands['sendVideo'] = { "command" : "sendVideo", "dataformat" : sendVideoObj.dataformat, "ipaddr": sendVideoObj.ipaddr, "port": sendVideoObj.port, "priority": sendVideoObj.priority  }
    def setFlightCommand(self, flightObj):
        self.commands['flight'] = { "command" : "flight", "destination" : flightObj.destination, "speed": flightObj.speed, "priority": flightObj.priority }
    def setMissionCommand(self, missionObj):
        self.commands['mission'] = { "command": "mission", "defaultWaypoints": missionObj.defaultWaypoints, "missionType": missionObj.missionType, "missionControl": missionObj.missionControl, "priority": missionObj.priority }

class droneSim(object):
    def __init__(self):
        self.position = Position()
        self.nextWaypoint = []
        self.battery = Battery()
        self.heading = float
        self.home = []


class taskedWaypoint(object):
    def __init__(self):
        self.position = Position()
        self.task = str
        self.TimeSensitive = bool

class Task(object):
    def __init__(self, pos,task,sensitive,prio,uid):
        self.position = pos
        self.task = task
        self.TimeSensitive = sensitive
        self.priority = prio
        self.comms_required = False
        self.dynamicTask = False
        self.uniqueID = uid
        self.LocationLocked = False
        self.ConnectionConfirmed = False

    def Actionable(self):#Stub, needs to be implemented--dictates whether a task can be completed under the current circumstances
        x=0

    def ChangePosition(self,pos:Position):
        if(not self.LocationLocked):
            self.position = pos
            return True
        else:
            return False



    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result


#Ensures IDs for tasks are unique and assigned by a third party....maybe I should've just built it into TaskQ, something like, NewEmptyTask()
class TaskIDGenerator(object):
    def __init__(self):
        self.CurrentTaskID= 0

    def Get(self):
        id = self.CurrentTaskID
        self.CurrentTaskID = self.CurrentTaskID + 1
        return id


    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result

class IDGenerator(object):
    def __init__(self):
        self.CurrentID= 0

    def Get(self):
        id = self.CurrentID
        self.CurrentID = self.CurrentID + 1
        return id

    def Check(self):
        return self
    
@dataclass
class TaskQueue(JSONWizard):

    def __init__(self):
        self.queue = []
        self.Count = 0
        self.TaskLock = TaskHold()
    def PushTask(self, task:Task):
        self.queue.insert(0,task)
        self.Count =  self.Count + 1
    def AppendTask(self, task:Task):
        self.queue.append(task)
        self.Count =  self.Count + 1    
    def AppendTasks(self, tasks):
        for task in tasks:
            self.queue.append(task)
            self.Count =  self.Count + 1  

    def PopTask(self):
        if(self.queue):
            self.Count = self.Count-1 #Adjust count
            return self.queue.pop(self.Count)#Pop item at end of queue
    def EnvelopeNextTask(self,tasks,pos):
        nextTask = self.PopTask()
        tasks.insert(nextTask,pos)
        for task in tasks :
            self.AppendTask(task)

        
    def PrintQ(self):#change this to lower case please
        if self.Empty():
            print("Empty!")
        else:
            print("")
            print("============")
            print("====TOP\u2193 ===")
            for idx, task in enumerate(self.queue):
                if(task.dynamicTask):
                    print("|D"+str(task.task).rjust(10,"+")+"|")
                else:
                    print("|"+str(task.task).rjust(10," ")+"|")
                #print("Task#: "+str(idx)+" Lat:"+ str(task.position.lat)+ " Lon:"+ str(task.position.lon)+" Alt:"+ str(task.position.alt) )
            print("===BOTTOM===")
            print("============")
            print("Count: "+ str(self.Count))
            print("Next Task: " + str(task.task) + ": Postion-- Lat:"+ str(task.position.lat)+ " Lon:"+ str(task.position.lon)+" Alt:"+ str(task.position.alt))
            print("")
    def Empty(self):
        if(self.Count == 0):
            return True
        elif(self.Count>0): 
            return False

    def Peek(self):
        if(not self._empty()):
            return self.queue(self.Count)
        else:
            return None

    def NextTask(self):
        if not self.Empty():
            return self.queue[self.Count-1]
        else:
            return False
    def AppendTask(self,task):
        self.queue.append(task)
        self.Count =  self.Count + 1

    def HoldTopTask(self):
        self.TaskLock.HoldTask(self.PopTask())


    def Release(self):
        return self.TaskLock.ReleaseTask()



    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result

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


    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result

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
            return self.WaypointsAndConnection[0]
        else:
            return None


    def PeekConnectivity(self):
        if(not self._empty()):
            tuple = self.WaypointsAndConnection[0]
            return tuple[1]
        else:
            return None


    def BackTrackPathForConnectivity(self):
        Connected = 0
        StartingLocation = self.StackPop()
        StepsBack = []
        StepsForward = []
        tasks = []
        StepsForward.insert(0,StartingLocation)
        while((not Connected)and (not self._empty())):
            if(self.PeekConnectivity()):
                Step = self.StackPop()
                print("Step Popped: "+ str(Step))
                StepsBack.append(Step)
                Connected = 1
            else:
                Step = self.StackPop()
                print("Step Popped: "+ str(Step))
                StepsBack.append(Step)
                StepsForward.insert(0,Step)
        if(self._empty() and (not Connected)):
            print("BackTrackError1")
            return None
        else:
            StepsBack.extend(StepsForward)
            return StepsBack



            
    def PrintWorkingHistory(self):
        print("History:")
        print("TrueCount: "+ str(self.TrueCount))
        print("Working Count: "+ str(self.Count))
        for tuple in self.TrueWaypointsAndConnection:
            print("ID: "+ str(tuple[2]) + " Position: "+ str(tuple[0])+ " Connected: "+ str(bool(tuple[1])))


    def PrintListOfStepsGeneric(self,list):
        for tuple in list:
            print("ID: "+ str(tuple[2]) + " Position: "+ str(tuple[0])+ " Connected: "+ str(bool(tuple[1])))


    






class RadioMap(object):
    def __init__(self,RadioPosition):
        self.lats = []
        self.lons = []
        self.headings = []
        self.dataRate = []
        self.length = 0
        self.positions =[]
        self.maxDistance = 0
        self.radioPosition = RadioPosition
    def Add(self, lat, lon, heading, rate,alt):
        self.lats.append(lat)
        self.lons.append(lon)
        self.headings.append(heading)
        self.dataRate.append(rate)
        self.length = self.length + 1
        pos = Position()
        pos.InitParams(lon,lat,alt,0,0,0)
        self.positions.append(pos)

        if(rate>0):
            x =0
            geo = Geodesic.WGS84.Inverse(pos.lat, pos.lon, self.radioPosition.lat,self.radioPosition.lon)
            distance_to_radio = geo.get('s12')
            if distance_to_radio>self.maxDistance:
                self.maxDistance = distance_to_radio
            #print("The distance to radio is {:.3f} m.".format(geo['s12']))




    def ConnectionProbabilty(self,currentPosition):
        rMax = self.maxDistance
        geo = Geodesic.WGS84.Inverse(currentPosition.lat, currentPosition.lon, self.radioPosition.lat,self.radioPosition.lon)
        d = geo.get('s12')
        if(d<1.05*rMax):
            return 0.95
        elif(d<1.1*rMax):
            return 0.75
        elif(d<1.2*rMax):
            return 0.50
        elif(d<1.3*rMax):
            return 0.30
        elif(d<1.4*rMax):
            return 0.15
        elif(d<1.5*rMax):
            return 0.03
        else:
            return 0.01



    def FindClosestPointWithConnection(self,nextPoint,currentPosition,radioPosition):
        geo = Geodesic.WGS84.Inverse(currentPosition.lat, currentPosition.lon, radioPosition.lat, radioPosition.lon)
        distance_to_base = geo.get('s12')
        minFlightDistance = distance_to_base
        suggestedPositon = radioPosition

        for idx, position in enumerate(self.positions) :
            if(self.dataRate[idx]>0):

                geo = Geodesic.WGS84.Inverse(currentPosition.lat, currentPosition.lon, position.lat, position.lon)
                distance_to_drone = geo.get('s12')
                if minFlightDistance>distance_to_drone :
                    minFlightDistance = distance_to_drone
                    suggestedPositon = position
            

        return suggestedPositon


class Node(object):#Interdependent PredictiveTree Class, can exist without one, but its hopelessly lost  :(
    def __init__(self,ID_num,q:TaskQueue,task:Task,finish,connected,waypointHistory:WaypointHistory, id_gen:TaskIDGenerator):
        self.Q = q.__deepcopy__()
        self.ID = ID_num #identifies node
        self.Parent = -1 #  -1 represents orphan status
        self.Children = []
        self.Finish = finish
        self.LeadingTask = task
        self.TravelHistory = waypointHistory.__deepcopy__()
        self.ID_GEN = id_gen.__deepcopy__()
        self.Connected = connected




        
    def Accept(self,parent):#child node recognizes and catalogs parent
        self.Parent = parent.ID
        if(self.ID ==-1):
            x=0
            #children can't be adopted twice, throw an exception or warning or something


    def Adopt(self,child):
        self.Children.append(child.ID)
        child.accept(self)

class TreeStatusHolder(object):
    def __init__(self):
        self.Postion = -1
        self.Connected = -1
    
    def Update(self,position:Position,connected:bool):
        self.Postion = position
        self.Connected = connected

    def Reset(self):
        self.Postion = -1
        self.Postion = -1

class TaskHold(object):
    def __init__(self):
        self.Captives = list
        self.Reasons = list

    def HoldTask(self, t:Task):
        self.Captives.append(t)
        self.Reasons.append("CONNECTION")

    def ReleaseTask(self):
        self.Reasons.pop()
        return self.Captives.pop()

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result





# class StateContainer(object):
#     def __init__(self,tasks:TaskQueue,history:WaypointHistory,id_gen:TaskIDGenerator):
#         self.CurrentQueue = tasks.__deepcopy__()
#         self.TravelHistory = history.__deepcopy__()
#         self.ID_GEN = id_gen.__deepcopy__()

class PredictiveTree(object):
    def __init__(self,root:Node):
        self.Nodes = []
        self.Root = root
        self.ID_Gen = IDGenerator()
        self.TasksHeld = TaskHold()
        self.maxDistance = 150 #this should be passed in eventually i.e. Dynamic
        self.Status = TreeStatusHolder()
        

        

    def NewNode(self,taskQ:TaskQueue,t:Task,finish,connected,prevWaypointHistory:WaypointHistory):#deines a new node and adds it to the list of nodes
        id  =  self.ID_Gen.Get()
        waypointHistory:WaypointHistory = prevWaypointHistory.__deepcopy__()
        if(t.task=="FLIGHT"):
            waypointHistory.AddPoint(t.position,connected)
        newNode = Node(id,taskQ,t,finish,connected,waypointHistory)
        self.Nodes.append(newNode)
        return newNode

    def ConnectionProbabilty(self,currentPosition):
        rMax = self.maxDistance
        geo = Geodesic.WGS84.Inverse(currentPosition.lat, currentPosition.lon, self.radioPosition['lat'],self.radioPosition['lon'])
        d = geo.get('s12')
        if(d<1.05*rMax):
            return 0.95
        elif(d<1.1*rMax):
            return 0.75
        elif(d<1.2*rMax):
            return 0.50
        elif(d<1.3*rMax):
            return 0.30
        elif(d<1.4*rMax):
            return 0.15
        elif(d<1.5*rMax):
            return 0.03
        else:
            return 0.01

    def PrintTree():#stub for now
        x=0
    def AnalyzeOptions():#stub for now
        x=0
    def Root(self,n:Node):#returns the root of a given node
        if(n.Parent == -1):
            return n
        else:
            return self.Root(self.Nodes[self.Parent])


    def BackStep(self,node:Node):
        Q = node.Q.__deepcopy__()
        wph:WaypointHistory = node.TravelHistory().__deepcopy__()

        
        # Update Q here to return to connection complete task, then return on same path
        # we should return a new node here
        

        backSteps = wph.BackTrackPathForConnectivity()
        taskConversion = []
        nextTask = Q.NextTask()
        backSteps.reverse()
        insertPostion = 0
        for idx, waypoint in enumerate(backSteps):
 
            if(waypoint[1]):
                print("Appending Next Task!")
                taskConversion.append(nextTask)
            t = Task(waypoint[0],"FLIGHT",0,0,self.TaskIDGen.Get())
            t.dynamicTask = True
            print("TASK ID: "+str(waypoint[2])) 
            taskConversion.append(t)
        

        BackTrackTaskList = taskConversion
        ForwardSteps = self.FindFowardConnection()
        taskConversion = []
        #for now, lets just return a list of two tasks sets, but this should be an object in the future
        taskConversion.append(BackTrackTaskList)
        if(len(ForwardSteps)):
            taskConversion.append(ForwardSteps)

        t = self.taskQ.PopTask()
        self.taskQ.AppendTasks(taskConversion)


        n = self.NewNode(Q,t,False,False,wph)
        return n
        


    def CheckHold(self,Q:TaskQueue):
        if(self.Status.Connected):
            while(not Q.TaskHold.Captives):
                t_ref:Task = Q.Release()


    def ActionableTask(self,t:Task,connected):#maybe this can be expanded to check battery etc.
        if(t.task == "SEND_DATA"):
            return connected #this means if we have a send task, its actionable if we have a connection
        else:
            return True


    def ConnectionThreshold(self,position,threshold):
        probabilty = self.ConnectionProbabilty(position)
        if(probabilty>threshold):
            return True
        else:
            return False
    

    def Continue(self,nextNode:Node):
        Q = nextNode.Q.__deepcopy__()
        currentNode:Node = nextNode
        self.Status.Update(currentNode.LeadingTask.position,currentNode.Connected)
        


        while(not Q.Empty):
            self.CheckHold(Q)
            t:Task = Q.pop()

            if(self.ActionableTask(t,self.Status.Connected)):#for now let's only continue that have success of >80%, others halt.Hypothetically, we could adopt both a failure and success node for each continue...this would be ideal...
                nextLocationConnected = self.ConnectionThreshold(t.position,0.80) #for now, were only persuing options with a greater than 80% chance of connection when needed
                finish = Q.Empty()
                n = self.NewNode(Q,t,finish,nextLocationConnected,currentNode.TravelHistory)
                currentNode.Adopt(n)
                currentNode = n
            else:
                break
        return currentNode

    def HaltPoint(self,HaltNode:Node):
        if(HaltNode.Finish):
            return
        else:
            BlockTreeHalt = self.Block(HaltNode)
            HaltNode.Adopt(self.Root(BlockTreeHalt))
            self.HaltPoint(BlockTreeHalt)

            HoldTreeHalt = self.Hold(HaltNode)
            HaltNode.Adopt(self.Root(HoldTreeHalt))
            self.HaltPoint(HoldTreeHalt)



    def Block(self,HaltNode:Node):
        nextNode = self.BackStep(HaltNode)
        HaltNode.Adopt(nextNode)
        return self.Continue(nextNode)

    
    def Hold(self,HaltNode:Node):
        Q:TaskQueue = HaltNode.Q.__deepcopy__() #want to return a node with the same Q (just updated)
        while(not self.ActionableTask(Q.Peek(),self.Status.Connected)):
            Q.HoldTopTask()
        if(not Q.Empty()):
            t:Task = Q.PopTask()
            n = self.NewNode(Q,t,False)
            HaltNode.Adopt(n)
            return self.Continue(n)
        else:
            x=0
            #This shouldn't happen...throw a warning or something like that...
        

