#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from transitions.extensions import GraphMachine

class DroneSM(GraphMachine):
    ####################### States Declaration #######################   
    states = ['Initial','TurnOn', 'TakeOff', 'Mission', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_TurnOn', 'source': 'Initial', 'dest': 'TurnOn'},
        {'trigger': 'TurnOn_to_TakeOff', 'source': 'TurnOn', 'dest': 'TakeOff', 'conditions': 'cond_TurnOn_TakeOff'},
        {'trigger': 'TakeOff_to_Mission', 'source': 'TakeOff', 'dest': 'Mission', 'conditions': 'cond_TakeOff_Mission'},
        {'trigger': 'Mission_to_Final', 'source': 'Mission', 'dest': 'Final', 'before': 'before_Mission_Final'}
        ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="DroneSM", states=DroneSM.states, transitions=DroneSM.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)

        self.aTargetAltitude = 10
        self.sleep_time = 45

        ####################### Draw State Machine ######################
        self.get_graph().draw('Data/DroneSM.canon', prog='dot') 
        self.get_graph().draw('Data/DroneSM.png', prog='dot')  

####################### Transition Conditions ####################### 
    def cond_TurnOn_TakeOff(self):
        if (self.aTargetAltitude > 0):
            # Confirm vehicle armed before attempting to take off
            if (not vehicle.armed):
                print(" Waiting for arming...")
                time.sleep(1)
                return False
            else:            
                return True 
        return False

    def cond_TakeOff_Mission(self):
        Finished = False
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= self.aTargetAltitude * 0.95:
            print("Reached target altitude")
            Finished =  True
        time.sleep(1)
        return Finished

####################### Before Transitions ####################### 
    def before_Mission_Final(self):
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        time.sleep(self.sleep_time)

####################### On_enter States #######################        
    def on_enter_Initial(self):
        #connection_string = "tcp:127.0.0.1:5760"
        connection_string = "127.0.0.1:14550"

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        vehicle = connect(connection_string, wait_ready=False)

        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    def on_enter_TurnOn(self):
        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

    def on_enter_TakeOff(self):
        print("Taking off!")
        vehicle.simple_takeoff(self.aTargetAltitude)  # Take off to target altitude
        time.sleep(3)

    def on_enter_Mission(self):
        print("Set default/target airspeed to 3")
        vehicle.airspeed = 120

        print("Going towards first point for 30 seconds ...")
        point1 = LocationGlobalRelative(-35.361354, 149.165218, self.aTargetAltitude)
        vehicle.simple_goto(point1)

        # sleep so we can see the change in map
        time.sleep(30)

        print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
        point2 = LocationGlobalRelative(-35.363244, 149.168801, self.aTargetAltitude)
        vehicle.simple_goto(point2, groundspeed=120)

        time.sleep(30)

    def on_enter_Final(self):
        # Close vehicle object before exiting script
        print("Close vehicle object")
        vehicle.close()


    def run(self):
        while True:
            if(self.state == 'Initial'):
                self.Initial_to_TurnOn()

            if(self.state == 'TurnOn'):
                print(" Altitude: ", vehicle.location.global_relative_frame.alt)
                if (self.cond_TurnOn_TakeOff()):
                    self.TurnOn_to_TakeOff()

            if(self.state == 'TakeOff'):
                if (self.cond_TakeOff_Mission()):
                    self.TakeOff_to_Mission()
            
            if(self.state == 'Mission'):
                self.Mission_to_Final()

            if(self.state == 'Final'):
                break


######################## Instantiating and Running the State Machine #######################  

#connection_string = "tcp:127.0.0.1:5760"
connection_string = "127.0.0.1:14550"

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=False)

machine = DroneSM()

machine.run()