#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random, time
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from transitions.extensions import GraphMachine

class DroneSM(GraphMachine):
    ####################### States Declaration #######################   
    states = ['Initial', 'Start', 'Wait', 'TurnOn', 'TakeOff', 'Mission', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Start', 'source': 'Initial', 'dest': 'Start'},
        {'trigger': 'Start_to_Start', 'source': 'Start', 'dest': 'Start', 'before': 'before_Start_Start'},
        {'trigger': 'Start_to_Wait', 'source': 'Start', 'dest': 'Wait'},
        {'trigger': 'Wait_to_TurnOn', 'source': 'Wait', 'dest': 'TurnOn', 'before': 'before_Wait_TurnOn'},
        {'trigger': 'TurnOn_to_TakeOff', 'source': 'TurnOn', 'dest': 'TakeOff', 'conditions': 'cond_TurnOn_TakeOff'},
        {'trigger': 'TakeOff_to_Mission', 'source': 'TakeOff', 'dest': 'Mission', 'conditions': 'cond_TakeOff_Mission'},
        {'trigger': 'Mission_to_Final', 'source': 'Mission', 'dest': 'Final', 'before': 'before_Mission_Final'}
        ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="DroneSM", states=DroneSM.states, transitions=DroneSM.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)

        #connection_string = "tcp:127.0.0.1:5760"
        self.connection_string = "127.0.0.1:14550"
        self.heightTg = 0
        self.sleep_time = 0
        self.start = False

        self.vehicle = connect(self.connection_string, wait_ready=False)

        ####################### Draw State Machine ######################
        self.get_graph().draw('Data/DroneSM.canon', prog='dot') 
        self.get_graph().draw('Data/DroneSM.png', prog='dot')  

####################### Transition Conditions ####################### 
    def cond_Start_Start(self):
        if (not self.vehicle.is_armable):
            file.write("\ncond_Start_Start = True")
            print("Waiting for vehicle to initialise...")
            return True
        file.write("\ncond_Start_Start = False")
        return False
    
    def cond_Start_Wait(self):
        if (self.vehicle.is_armable):
            print("...")
            file.write("\ncond_Start_Start = True")
            return True
        file.write("\ncond_Start_Start = False")
        return False
    
    def  cond_Wait_TurnOn(self):
        if self.start:
            file.write("\ncond_Wait_TurnOn = True")
            text = " -> DroneSM::start.in." + str(self.heightTg)
            trace.write(text) 
            return True
        file.write("\ncond_Wait_TurnOn = False")
        return False
    
    def cond_TurnOn_TakeOff(self):
        if (self.vehicle.armed):         
            file.write("\ncond_TurnOn_TakeOff = True")
            return True 
        file.write("\ncond_TurnOn_TakeOff = False")
        return False

    def cond_TakeOff_Mission(self):
        if (self.vehicle.location.global_relative_frame.alt >= self.heightTg * 0.95):
            file.write("\ncond_TakeOff_Mission = True")
            return  True
        file.write("\ncond_TakeOff_Mission = False")
        return False

####################### Before Transitions ####################### 
    def before_Mission_Final(self):
        file.write("\nbefore_Mission_Final")
        file.write("\nReturning to Launch")
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(self.sleep_time)

    def before_Start_Start(self):
        file.write("\nbefore_Start_Start")
        print('...')
        time.sleep(1)

    def before_Wait_TurnOn(self):
        file.write("\nbefore_Wait_TurnOn")
        ################## Reading values from txt config file ########
        configFile = open("Pytransitions/DroneSM/Data/config.txt", "r")
        line = configFile.readline()
        lineCont = 0
        while line != "":
            line = configFile.readline()
            if lineCont == 0:
                print("heightTg: ", line)
                self.heightTg = float(line)
            if lineCont == 1:
                print("sleep_time: ", line)
                self.sleep_time = float(line)
            lineCont += 1
        configFile.close() 

        self.vehicle = connect(self.connection_string, wait_ready=False)

####################### On_enter States #######################    
    def on_enter_Init(self):
        file.write("\non_enter_Init")
        #connection_string = "tcp:127.0.0.1:5760"
        connection_string = "127.0.0.1:14550"

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=False)
     
    def on_enter_Start(self):
        file.write("\non_enter_Start")
        print("Performing basic pre-arming checks...")
        # Don't try to arm until autopilot is ready

    def on_enter_Wait(self):
        file.write("\non_enter_Wait")
        print("Waiting")
        # Don't try to arm until autopilot is ready

    def on_enter_TurnOn(self):
        file.write("\non_enter_TurnOn")
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

    def on_enter_TakeOff(self):
        file.write("\non_enter_TakeOff")
        text = "\n -> DroneSM::takeoffCall." + str(self.heightTg)
        trace.write(text)
        print("Taking off!")
        self.vehicle.simple_takeoff(self.heightTg)  # Take off to target altitude
        time.sleep(3)
        trace.write("\n -> tock -> tock -> tock")


    def on_enter_Mission(self):
        file.write("\non_enter_Mission")
        print("Going towards first point for 30 seconds ...")
        point1 = LocationGlobalRelative(-35.361354, 149.165218, self.heightTg)
        text = "\nGoing to " + str(point1.lat) + "; "+ str(point1.lon) + "; " + str(point1.alt) 
        file.write(text)
        trace.write(" -> DroneSM::gotoCall.1.1.1 ")

        self.vehicle.simple_goto(point1, 0, 120)

        # sleep so we can see the change in map
        time.sleep(30)
        trace.write(" -> tock ")


        print("Going towards second point for 30 seconds ...")
        point2 = LocationGlobalRelative(-35.363244, 149.168801, self.heightTg)
        self.vehicle.simple_goto(point2, groundspeed=120, airspeed=0)
        text = "\nGoing to " + str(point2.lat) + "; "+ str(point2.lon) + "; " + str(point2.alt) 
        file.write(text)
        trace.write(" -> DroneSM::gotoCall.2.2.2 ")

        time.sleep(30)
        trace.write(" -> tock ")


    def on_enter_Final(self):
        file.write("\non_enter_Final")
        # Close vehicle object before exiting script
        print("Close vehicle object")
        trace.write(" -> DroneSM::terminate ")

        self.vehicle.close()


    def run(self):
        file.write("\n#########run#########")
        trace.write(" DroneSM = ")

        cont = 0
        while True:
            if(self.state == 'Initial'):
                file.write("\n-> self.state == 'Initial'")
                cont +=1
                self.Initial_to_Start()
                
            if(self.state == 'Start' and self.cond_Start_Start()):
                file.write("\n-> self.state == 'Start' and self.cond_Start_Start()")
                print(" Waiting for vehicle to initialise...")
                cont +=1
                self.Start_to_Start()

            if(self.state == 'Start' and self.cond_Start_Wait()):
                file.write("\n-> self.state == 'Start' and self.cond_Start_Wait()")
                print(" Starded...")
                cont +=1
                self.Start_to_Wait()

            if(self.state == 'Wait' and self.cond_Wait_TurnOn()):
                file.write("\n-> self.state == 'Wait' and self.cond_Wait_TurnOn()")
                print(" Waiting...")
                cont +=1
                self.Wait_to_TurnOn()

            if(self.state == 'TurnOn' and self.cond_TurnOn_TakeOff()):
                file.write("\n-> self.state == 'TurnOn' and self.cond_TurnOn_TakeOff()")
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                cont +=1
                self.TurnOn_to_TakeOff()

            if(self.state == 'TakeOff' and self.cond_TakeOff_Mission()):
                file.write("\n-> self.state == 'TakeOff' and self.cond_TakeOff_Mission()")
                print("Reached target altitude")
                cont +=1
                self.TakeOff_to_Mission()
            
            if(self.state == 'Mission'):
                file.write("\n-> self.state == 'Mission'")
                cont +=1
                self.Mission_to_Final()

            if(self.state == 'Final'):
                file.write("\n-> self.state == 'Final'")
                cont +=1
                break
            
            if(cont == 0):
                file.write("\n-> tock")
                trace.write(" -> tock")
                time.sleep(5)
            else:
                cont = 0           

            self.start = random.choice([True, False])

######################## Instantiating and Running the State Machine #######################  
file = open("Pytransitions/DroneSM/Data/log.txt", "w")
trace = open("Pytransitions/DroneSM/Data/trace.txt", "w")
machine = DroneSM()
machine.run()
file.close()