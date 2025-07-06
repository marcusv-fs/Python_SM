#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random, time
from pymavlink import mavutil
from transitions.extensions import GraphMachine

class PMSM(GraphMachine):
    ####################### States Declaration #######################   
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
    def __init__(self):
        super().__init__(name="PMSM", states=PMSM.states, transitions=PMSM.transitions, initial='Initial')

        #connection_string = "tcp:127.0.0.1:5760"
        self.connection_string = "udp:127.0.0.1:14551"
        self.heightTg = 0.0
        self.HeightD = 0.0
        self.sleep_time = 25     
        self.drone = None
        self.start_time = time.time()

        ####################### Draw State Machine ######################
        self.get_graph().draw('Pytransitions/PMSM/Data/PMSM.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/PMSM/Data/PMSM.jpg', prog='dot')  

    def update_value(self, configFile, key):
        try:
            with open(configFile, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if '=' in line:
                        current_key, valor = line.split('=', 1)
                        if current_key.strip() == key:
                            return valor.strip()
            return None  # Retorna None se a chave não for encontrada
        except FileNotFoundError:
            print("Value not found!")
            return None
        
    def is_armed(self):
        hb = self.drone.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb is None:
            return False
        # Check armed bit in base_mode
        return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    
    def send_position(self, x, y, z, yaw_deg):
        """Send local position (NED)"""
        yaw_rad = yaw_deg * 3.14159 / 180.0
        self.drone.mav.set_position_target_local_ned_send(
            int((time.time() - self.start_time) * 1000),
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # Bitmask: enable position only
            x, y, -z,  # z is negative in NED
            0, 0, 0,    # Velocity
            0, 0, 0,    # Acceleration
            yaw_rad, 0) # Yaw, yaw_rate


####################### Transition Conditions ####################### 
    def cond_Connect_Connect(self):
        try:
            self.drone.wait_heartbeat(timeout=5)
            print("Heartbeat from system (system %u component %u)" % (self.drone.target_system, self.drone.target_component))
            return False
        except Exception as e:
            print(f"❌ Failed to connect or receive heartbeat: {e}")
            return True
    
    def cond_Connect_Wait(self):
        try:
            self.drone.wait_heartbeat(timeout=5)
            print("Heartbeat from system (system %u component %u)" % (self.drone.target_system, self.drone.target_component))
            return True
        except Exception as e:
            print(f"❌ Failed to connect or receive heartbeat: {e}")
            return False
    
    def cond_Wait_TurnOn(self):
        if self.start:
            configFile = "Pytransitions/PMSM/Data/config.txt"
            self.heightTg = float(self.update_value(configFile,'heightTg'))
            self.heightTg = random.randint(0, 2)

            print(self.heightTg)
            file.write("\ncond_Wait_TurnOn = True")

            text = " -> PMSM::Connect.in." + str(self.heightTg)
            trace.write(text) 
            return True
        file.write("\ncond_Wait_TurnOn = False")
        return False
    
    def cond_TurnOn_TakeOff(self):
        condition = self.is_armed()
        if (condition):         
            file.write("\ncond_TurnOn_TakeOff = True")
            return True 
        file.write("\ncond_TurnOn_TakeOff = False")
        return False

    def cond_CheckHeight_CheckHeight(self):
        if (self.HeightD < self.heightTg * 0.95):
            file.write("\ncond_CheckHeight_CheckHeight = True")
            return  True
        file.write("\ncond_CheckHeight_CheckHeight = False")
        return False

    def cond_CheckHeight_Mission(self):
        if (self.HeightD >= self.heightTg * 0.95):
            file.write("\ncond_CheckHeight_Mission = True")
            return  True
        file.write("\ncond_CheckHeight_Mission = False")
        return False

####################### Before Transitions ####################### 
    def before_Mission_Final(self):
        file.write("\nbefore_Mission_Final")
        file.write("\nReturning to Launch")
        print("Returning to Launch")
        self.drone.set_mode('RTL')
        time.sleep(self.sleep_time)
        trace.write("\n -> tock")


    def before_Connect_Connect(self):
        file.write("\nbefore_Connect_Connect")
        print('...')
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
        print("Connecting to the drone")
        # Connect to the drone
        try:
            self.drone = mavutil.mavlink_connection(self.connection_string)
            return self.cond_Connect_Connect()
        except Exception as e:
            print(f"Connection failed: {e}")
            return True

    def on_enter_Wait(self):
        file.write("\non_enter_Wait")
        print("Waiting")
        # Don't try to arm until autopilot is ready

    def on_enter_TurnOn(self):
        file.write("\non_enter_TurnOn")
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.drone.set_mode("GUIDED")
        # Arm the drone
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        self.drone.motors_armed_wait()
        print("Armed!")
        time.sleep(1)


    def on_enter_TakeOff(self):
        file.write("\non_enter_TakeOff")
        text = "\n -> PMSM::takeoffCall." + str(int(self.heightTg))
        trace.write(text)
        print("Taking off!")
        self.drone.mav.command_long_send(
    self.drone.target_system,
    self.drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10)
        time.sleep(3)
        trace.write("\n -> tock -> tock -> tock")

    def on_enter_CheckHeight(self):
        file.write("\non_enter_CheckHeight")

        # Wait for GLOBAL_POSITION_INT message
        msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)

        if msg:
            # Relative altitude is in millimeters
            self.HeightD = msg.relative_alt / 1000.0
            print(f"Relative Altitude: {self.HeightD} m")
        else:
            print("No GLOBAL_POSITION_INT message received")



    def on_enter_Mission(self):
        file.write("\non_enter_Mission")
        print("Going towards first point for 25 seconds ...")
        self.send_position(-35.361354, 149.165218, self.heightTg, 0)
        text = "\nGoing to " + str(-35.361354) + "; "+ str(149.165218) + "; " + str(self.heightTg) 
        file.write(text)
        trace.write(" -> PMSM::gotoCall.1.1.1 ")
        time.sleep(25)
        trace.write(" -> tock ")

        print("Going towards second point for 25 seconds ...")
        self.send_position(35.361354, 149.165218, self.heightTg, 0)
        text = "\nGoing to " + str(35.361354) + "; "+ str(149.165218) + "; " + str(self.heightTg) 
        file.write(text)
        trace.write(" -> PMSM::gotoCall.1.1.1 ")
        time.sleep(25)
        trace.write(" -> tock ")


    def on_enter_Final(self):
        file.write("\non_enter_Final")
        # Close vehicle object before exiting script
        print("Close vehicle object")
        trace.write(" -> PMSM::terminate -> SKIP")
        self.drone.close()


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
                    print(" Waiting for vehicle to initialise...")
                    self.Connect_to_Connect()

                case 'Connect' if self.cond_Connect_Wait():
                    file.write("\n-> self.state == 'Connect' and self.cond_Connect_Wait()")
                    print(" Connected...")
                    self.Connect_to_Wait()

                case 'Wait' if self.cond_Wait_TurnOn():
                    file.write("\n-> self.state == 'Wait' and self.cond_Wait_TurnOn()")
                    print(" Waiting...")
                    self.Wait_to_TurnOn()

                case 'TurnOn' if self.cond_TurnOn_TakeOff():
                    file.write("\n-> self.state == 'TurnOn' and self.cond_TurnOn_TakeOff()")
                    print(" Altitude: ", self.HeightD)
                    self.TurnOn_to_TakeOff()

                case 'TakeOff':
                    file.write("\n-> self.state == 'TakeOff' and self.cond_TakeOff_Mission()")
                    print("Reached target altitude")
                    self.TakeOff_to_CheckHeight()

                case "CheckHeight" if self.cond_CheckHeight_CheckHeight():
                    print("\n-> self.state == 'TakeOff' and self.cond_CheckHeight_Mission()")
                    file.write("\n-> self.state == 'TakeOff' and self.cond_CheckHeight_CheckHeight()")
                    self.CheckHeight_to_CheckHeight()

                case "CheckHeight" if self.cond_CheckHeight_Mission():
                    file.write("\n-> self.state == 'TakeOff' and self.cond_CheckHeight_Mission()")
                    print("\n-> self.state == 'TakeOff' and self.cond_CheckHeight_Mission()")
                    self.CheckHeight_to_Mission()

                case 'Mission':
                    file.write("\n-> self.state == 'Mission'")
                    self.Mission_to_Final()

                case 'Final':
                    file.write("\n-> self.state == 'Final'")
                    break
                
                case _:
                    file.write("\n-> tock")
                    trace.write(" -> tock")
                    time.sleep(5)

            self.start = random.choice([True, False])

######################## Instantiating and Running the State Machine #######################  
file = open("Pytransitions/PMSM/Data/log_cases.txt", "w")
trace = open("Pytransitions/PMSM/Data/trace_cases.txt", "w")
machine = PMSM()
machine.run()
file.close()