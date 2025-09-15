#!/usr/bin/env python
# -*- coding: utf-8 -*-
import auxFuncs
from random import choice, randint
import time
from transitions.extensions import GraphMachine

class PMSM(GraphMachine):
####################### States Declaration #######################   
    def __init__(self):
        states = ['Initial', 'Connect', 'Wait', 'TurnOn', 'TakeOff', 'CheckHeight', 'Mission', 'Final']

####################### Transitions Statement  #######################  
        transitions = [
            {'trigger': 'Initial_to_Connect', 'source': 'Initial', 'dest': 'Connect'},
            {'trigger': 'Connect_to_Connect', 'source': 'Connect', 'dest': 'Connect', 'before': 'before_Connect_Connect'},
            {'trigger': 'Connect_to_Wait', 'source': 'Connect', 'dest': 'Wait'},
            {'trigger': 'Wait_to_TurnOn', 'source': 'Wait', 'dest': 'TurnOn', 'before': 'before_Wait_TurnOn'},
            {'trigger': 'TurnOn_to_TakeOff', 'source': 'TurnOn', 'dest': 'TakeOff', 'conditions': 'cond_TurnOn_TakeOff'},
            {'trigger': 'TakeOff_to_CheckHeight', 'source': 'TakeOff', 'dest': 'CheckHeight'},
            {'trigger': 'CheckHeight_to_CheckHeight', 'source': 'CheckHeight', 'dest': 'CheckHeight', 'conditions': 'cond_CheckHeight_CheckHeight', 'before': 'before_CheckHeight_CheckHeight'},
            {'trigger': 'CheckHeight_to_Mission', 'source': 'CheckHeight', 'dest': 'Mission', 'conditions': 'cond_CheckHeight_Mission'},
            {'trigger': 'Mission_to_Final', 'source': 'Mission', 'dest': 'Final', 'before': 'before_Mission_Final'}
            ]
    
####################### Init and util Functions ####################### 

        super().__init__(name="PMSM", states=states, transitions=transitions, initial='Initial', show_conditions=True, show_state_attributes=True)

        #connection_string = "tcp:127.0.0.1:5760"
        self.connection_string = "udp:127.0.0.1:14551"
        self.targetHeight = 0.0
        self.droneHeight = 0.0
        self.sleep_time = 25     
        self.drone = None
        self.start_time = time.time()
        self.configFile = "Pytransitions/Drone/PMSM/Data/config.txt"
        self.home_pos = [0, 0, 0, 0]

        ####################### Draw State Machine ######################
        self.get_graph().draw('Pytransitions/Drone/PMSM/Data/PMSM.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/Drone/PMSM/Data/PMSM.jpg', prog='dot')  


####################### Transition Conditions ####################### 
    def cond_Connect_Connect(self):
        if auxFuncs.wait_for_heartbeat(self.drone):
            file.write("\ncond_Connect_Connect = True")
            print("Heartbeat recebido do sistema")
            return False
        file.write("\ncond_Connect_Connect = False")
        return True
    
    def cond_Connect_Wait(self):
        if auxFuncs.wait_for_heartbeat(self.drone):
            file.write("\ncond_Connect_Wait = True")
            text = " -> PMSM::Connect.in." + str(self.targetHeight)
            trace.write(text) 
            return True
        file.write("\ncond_Connect_Wait = False")
        return False
    
    def cond_Wait_TurnOn(self):
        print("Start?", self.start)
        if self.start:
            self.targetHeight = randint(2, 5)
            file.write("\ncond_Wait_TurnOn = True")
            return True
        file.write("\ncond_Wait_TurnOn = False")
        return False
    
    def cond_TurnOn_TakeOff(self):
        #condition = auxFuncs.is_armed(self.drone)
        condition = True
        if (condition):         
            file.write("\ncond_TurnOn_TakeOff = True")
            return True 
        file.write("\ncond_TurnOn_TakeOff = False")
        return False

    def cond_CheckHeight_CheckHeight(self):
        if ((self.droneHeight < self.targetHeight * 0.90) and (self.droneHeight < abs(self.targetHeight) - 0.5)):
            file.write("\ncond_CheckHeight_CheckHeight = True")
            return True
        file.write("\ncond_CheckHeight_CheckHeight = False")
        return False

    def cond_CheckHeight_Mission(self):
        if (self.droneHeight >= self.targetHeight * 0.90 or (self.droneHeight >= abs(self.targetHeight) - 0.5)):
            file.write("\ncond_CheckHeight_Mission = True")
            return True
        file.write("\ncond_CheckHeight_Mission = False")
        return False

####################### Before Transitions ####################### 
    def before_Mission_Final(self):
        file.write("\nbefore_Mission_Final")
        file.write("\nReturning to Launch")
        print("Returning to Launch")
        auxFuncs.set_mode(self.drone, 'RTL')
        while True:
            pos = auxFuncs.get_global_position(self.drone)
            cond = auxFuncs.has_reached_position(self.drone, self.home_pos['lat'], self.home_pos['lon'], self.home_pos['alt'])
            time.sleep(1)
            if cond:
                print("Posição alvo alcançada!")
                break
        trace.write("\n -> tock")

    def before_Connect_Connect(self):
        file.write("\nbefore_Connect_Connect")
        time.sleep(1)
        trace.write("\n -> tock")

    def before_Wait_TurnOn(self):
        file.write("\nbefore_Wait_TurnOn")
        ################## Reading values from txt config file ########

    def before_CheckHeight_CheckHeight(self):
        file.write("\nbefore_CheckHeight_CheckHeight")
        time.sleep(1)
        trace.write("\n -> tock")

####################### On_enter States #######################    
    def on_enter_Initial(self):
        file.write("\non_enter_Init")
     
    def on_enter_Connect(self):
        file.write("\non_enter_Connect")
        # Connect to the drone
        self.drone = auxFuncs.connect_drone(self.connection_string)

    def on_enter_Wait(self):
        file.write("\non_enter_Wait")
        auxFuncs.set_home_to_current_position(self.drone)
        self.home_pos = auxFuncs.get_home_position(self.drone)
        print("Waiting for start...\n")
        

    def on_enter_TurnOn(self):
        file.write("\non_enter_TurnOn")
        auxFuncs.set_mode(self.drone, "GUIDED")
        auxFuncs.arm_drone(self.drone)
        #auxFuncs.wait_for_arming(self.drone)
        time.sleep(1)

    def on_enter_TakeOff(self):
        file.write("\non_enter_TakeOff")
        text = "\n -> PMSM::takeoffCall." + str(int(self.targetHeight))
        trace.write(text)
        auxFuncs.takeoff_relative(self.drone, self.targetHeight, self.home_pos['alt'])
        trace.write("\n -> tock -> tock -> tock")

    def on_enter_CheckHeight(self):
        file.write("\non_enter_CheckHeight")
        pos = auxFuncs.get_global_position(self.drone)
        if pos:
            self.droneHeight = pos['alt'] - self.home_pos['alt']
            print(f"Relative Altitude: {self.droneHeight:.2f} m")

    def on_enter_Mission(self):
        file.write("\non_enter_Mission")
        print("Going towards first point for 25 seconds ...")
        auxFuncs.move_to_relative(self.drone, self.start_time, -5, 5, 0, 0)
        text = "\nGoing to " + str(-35.361354) + "; "+ str(149.165218) + "; " + str(self.targetHeight) 
        file.write(text)
        trace.write(" -> PMSM::gotoCall.1.1.1 ")
        time.sleep(25)
        trace.write(" -> tock ")

        print("Going towards second point for 25 seconds ...")
        auxFuncs.move_to_relative(self.drone, self.start_time, 5, -5, 0, 0)
        text = "\nGoing to " + str(35.361354) + "; "+ str(149.165218) + "; " + str(self.targetHeight) 
        file.write(text)
        trace.write(" -> PMSM::gotoCall.1.1.1 ")
        time.sleep(25)
        trace.write(" -> tock ")

    def on_enter_Final(self):
        file.write("\non_enter_Final")
        trace.write(" -> PMSM::terminate -> SKIP")
        auxFuncs.close_connection(self.drone)

    def run(self):
        file.write("\n#########run#########")
        trace.write(" PMSM = ")

        while True:
            match (self.state):
                case 'Initial':
                    file.write("\n-> self.state == 'Initial'")
                    self.Initial_to_Connect()
                    
                case 'Connect' if self.cond_Connect_Connect():
                    file.write("\n-> self.state == 'Connect' and self.cond_Connect_Connect()")
                    self.Connect_to_Connect()

                case 'Connect' if self.cond_Connect_Wait():
                    file.write("\n-> self.state == 'Connect' and self.cond_Connect_Wait()")
                    self.Connect_to_Wait()

                case 'Wait' if self.cond_Wait_TurnOn():
                    file.write("\n-> self.state == 'Wait' and self.cond_Wait_TurnOn()")
                    self.Wait_to_TurnOn()

                case 'TurnOn' if self.cond_TurnOn_TakeOff():
                    file.write("\n-> self.state == 'TurnOn' and self.cond_TurnOn_TakeOff()")
                    print(" Altitude: ", self.droneHeight)
                    self.TurnOn_to_TakeOff()

                case 'TakeOff':
                    file.write("\n-> self.state == 'TakeOff' and self.cond_TakeOff_Mission()")
                    self.TakeOff_to_CheckHeight()

                case "CheckHeight" if self.cond_CheckHeight_CheckHeight():
                    print("\n-> self.state == 'CheckHeight' and self.cond_CheckHeight_CheckHeight()")
                    file.write("\n-> self.state == 'TakeOff' and self.cond_CheckHeight_CheckHeight()")
                    self.CheckHeight_to_CheckHeight()

                case "CheckHeight" if self.cond_CheckHeight_Mission():
                    print("Reached target altitude")

                    file.write("\n-> self.state == 'CheckHeight' and self.cond_CheckHeight_Mission()")
                    print("\n-> self.state == 'TakeOff' and self.cond_CheckHeight_Mission()")
                    self.CheckHeight_to_Mission()

                case 'Mission':
                    file.write("\n-> self.state == 'Mission'")
                    self.Mission_to_Final()

                case 'Final':
                    file.write("\n-> self.state == 'Final'")
                    time.sleep(5)
                    break
                
                case _:
                    file.write("\n-> tock")
                    trace.write(" -> tock")
                    time.sleep(5)

            self.start = choice([True, True])

######################## Instantiating and Running the State Machine #######################  
file = open("Pytransitions/Drone/PMSM/Data/log_cases.txt", "w")
trace = open("Pytransitions/Drone/PMSM/Data/trace_cases.txt", "w")
machine = PMSM()
machine.run()
file.close()