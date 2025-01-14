#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
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
        self.aTargetAltitude = 0
        self.sleep_time = 0
        self.start = False

        self.vehicle = connect(self.connection_string, wait_ready=False)

        ####################### Draw State Machine ######################
        self.get_graph().draw('Data/DroneSM.canon', prog='dot') 
        self.get_graph().draw('Data/DroneSM.png', prog='dot')  

####################### Transition Conditions ####################### 
    def cond_Start_Start(self):
        if (not self.vehicle.is_armable):
            print("Waiting for vehicle to initialise...")
            return True
        return False
    
    def cond_Start_Wait(self):
        if (self.vehicle.is_armable):
            print("...")
            return True
        return False
    
    def  cond_Wait_TurnOn(self):
        if self.start:
            return True
        return False
    
    def cond_TurnOn_TakeOff(self):
        if (self.vehicle.armed):         
            return True 
        return False

    def cond_TakeOff_Mission(self):
        if (self.vehicle.location.global_relative_frame.alt >= self.aTargetAltitude * 0.95):
            return  True
        return False

####################### Before Transitions ####################### 
    def before_Mission_Final(self):
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(self.sleep_time)

    def before_Start_Start(self):
        print('...')
        time.sleep(1)

    def before_Wait_TurnOn(self):
        ################## Reading values from txt config file ########
        file = open("Pytransitions/DroneSM/Data/config.txt", "r")
        line = file.readline()
        lineCont = 0
        while line != "":
            line = file.readline()
            if lineCont == 0:
                print("aTargetAltitude: ", line)
                self.aTargetAltitude = float(line)
            if lineCont == 1:
                print("sleep_time: ", line)
                self.sleep_time = float(line)
            lineCont += 1
        file.close() 

        self.vehicle = connect(self.connection_string, wait_ready=False)

####################### On_enter States #######################    
    def on_enter_Init(self):
        #connection_string = "tcp:127.0.0.1:5760"
        connection_string = "127.0.0.1:14550"

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=False)
     
    def on_enter_Start(self):
        print("Performing basic pre-arming checks...")
        # Don't try to arm until autopilot is ready

    def on_enter_Wait(self):
        print("Waiting")
        self.start = True
        # Don't try to arm until autopilot is ready

    def on_enter_TurnOn(self):
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

    def on_enter_TakeOff(self):
        print("Taking off!")
        self.vehicle.simple_takeoff(self.aTargetAltitude)  # Take off to target altitude
        time.sleep(3)

    def on_enter_Mission(self):
        print("Going towards first point for 30 seconds ...")
        point1 = LocationGlobalRelative(-35.361354, 149.165218, self.aTargetAltitude)
        self.vehicle.simple_goto(point1, 0, 120)

        # sleep so we can see the change in map
        time.sleep(30)

        print("Going towards second point for 30 seconds ...")
        point2 = LocationGlobalRelative(-35.363244, 149.168801, self.aTargetAltitude)
        self.vehicle.simple_goto(point2, groundspeed=120, airspeed=0)

        time.sleep(30)

    def on_enter_Final(self):
        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()


    def run(self):
        while True:
            if(self.state == 'Initial'):
                self.Initial_to_Start()
                
            if(self.state == 'Start' and self.cond_Start_Start()):
                print(" Waiting for vehicle to initialise...")
                self.Start_to_Start()

            if(self.state == 'Start' and self.cond_Start_Wait()):
                print(" Starded...")
                self.Start_to_Wait()

            if(self.state == 'Wait' and self.cond_Wait_TurnOn()):
                print(" Waiting...")
                self.Wait_to_TurnOn()

            if(self.state == 'TurnOn' and self.cond_TurnOn_TakeOff()):
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                self.TurnOn_to_TakeOff()

            if(self.state == 'TakeOff' and self.cond_TakeOff_Mission()):
                print("Reached target altitude")
                self.TakeOff_to_Mission()
            
            if(self.state == 'Mission'):
                self.Mission_to_Final()

            if(self.state == 'Final'):
                break


######################## Instantiating and Running the State Machine #######################  
machine = DroneSM()

machine.run()