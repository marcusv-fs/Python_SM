# pip install transitions
# pip install graphviz

from datetime import datetime
from math import floor
import random, time
from shutil import move
from transitions.extensions import GraphMachine


class MovementAndAvoidance(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Move', 'Avoid']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Move', 'source': 'Initial', 'dest': 'Move'},
        {'trigger': 'Move_to_Avoid', 'source': 'Move', 'dest': 'Avoid', 'conditions': 'Cond_Move_to_Avoid'},
        {'trigger': 'Avoid_to_Move', 'source': 'Avoid', 'dest': 'Move'}
        ]
    
####################### Init and util Functions ####################### 
    def __init__(self, MBC_Reseted, MBC_Time, av, lv, Position, MB, const = 355):
        super().__init__(name="MovementAndAvoidance", states=MovementAndAvoidance.states, transitions=MovementAndAvoidance.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        
        self.lv = lv
        self.PI = 3.14159265359
        self.Obstacle = False
        self.MBC_Reseted = MBC_Reseted
        self.MBC_Time = MBC_Time
        self.av = av
        self.p = Position
        self.MB = MB
        self.const = const
        
        ####################### Draw State Machine ######################
        self.file = open("Pytransitions/AlphaAlgorithm/Classic/MovementAndAvoidance/Data/MovementAndAvoidance_trace.txt", "w")
        self.get_graph().draw('Pytransitions/AlphaAlgorithm/Classic/MovementAndAvoidance/Data/MovementAndAvoidance.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/AlphaAlgorithm/Classic/MovementAndAvoidance/Data/MovementAndAvoidance.png', prog='dot') 

        ##############################Util Functions##############################
        
    def move(self, lv, av):
        print("Move(" + str(lv) + ", " + str(av) + ")")
        self.file.write("moveCall."+ str(lv) + "."+ str(av)+" -> ") 

    def stop(self):
        print("Stopping the robot.")
        self.file.write("stopCall -> ")
            

####################### Transition Conditions ####################### 
    def Cond_Move_to_Avoid(self):
        self.Obstacle = False
        if (self.MBC_Reseted == True):
            if(self.MBC_Time - time.time() < (self.MB - 360)/self.av):
                return True
        return self.Obstacle

####################### Before Transitions ####################### 


####################### On_enter States #######################        
    def on_enter_Move(self):
        print("I am in the Move state")
        self.move(self.lv, 0)
        time.sleep(0.1)
        self.file.write("tock -> ")

    def on_enter_Avoid(self):
        print("I am in the Avoid state.")
        if (self.p == "Positon::left"):
            self.move(0, self.av)
        else:
            self.move(0, - self.av)
        tempo = floor((random.randint(0, 1)*(360/self.av)))
        time.sleep(tempo)
        


    def run(self):
        self.cycle = 0
        print (time.time() - self.MBC_Time)
        while (True and (time.time() - self.MBC_Time + self.const) < self.MB):
            print("\n///////////////// M&A Cycle: " + str(self.cycle) + " /////////////////\n")
            print ("curr_time: " + str(time.time() - self.MBC_Time + self.const)) # Adding time just to finish the program early
            cond = 0
            self.cycle += 1
            
            print("Now my current state is " + self.state)

            if(self.state == 'Initial'):
                self.Initial_to_Move()
                
            print("Now Obstacle is " + str(self.Obstacle))
            if(self.state == 'Move' and self.Obstacle):
                print("Moving to avoid...")
                self.file.write("obstacle.in -> ")
                self.Move_to_Avoid()
                cond = 1

            print("MBC (" + str(time.time() - self.MBC_Time) + ") >= PI/av (" + str((self.MB - 360)/self.av) + ")? Answer: " + str(self.Cond_Move_to_Avoid()))
            self.Obstacle = self.Cond_Move_to_Avoid()
            if(self.state == 'Avoid'):
                self.Avoid_to_Move()
                cond = 1
                
            if cond == 0:
                self.file.write("tock -> ")
            
            ####################### Test Sector #######################
            self.Obstacle = self.Cond_Move_to_Avoid()
            time.sleep(0.3)
            #self.MB = random.randint(360, 7200)
            # if self.cycle == 15:
            #     print("\n Finishing the program. \n")
            #     break

        self.file.close()
            

MBC_reseted = True
MBC_time = time.time()
av = 360
lv = 0.5
Position = "Position::left"
MB = 360
# machine = MovementAndAvoidance(MBC_reseted, MBC_time, av, lv, Position, MB)
# machine.run()
