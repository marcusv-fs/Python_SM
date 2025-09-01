# pip install transitions
# pip install graphviz

import random, time
from transitions.extensions import GraphMachine


class SMovement(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Moving', 'MovementAndAvoidance']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'tp_Start', 'source': 'Initial', 'dest': 'Moving'},
        {'trigger': 'tp_Moving', 'source': 'MovementAndAvoidance', 'dest': 'Moving', 'conditions': 'c_Time'},
        {'trigger': 'tp_MovementAndAvoidance', 'source': 'Moving', 'dest': 'MovementAndAvoidance', 'conditions': 'c_Obs', 'before': 'before_tp_MovementAndAvoidance'},
        ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="SMovement", states=SMovement.states, transitions=SMovement.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        self.lv = 1 # Linear Velocity. I have seted a random number.
        self.PI = 3.14159265359
        self.av = 1 # Angular Velocity. I have seted a random number.
        self.MBC = time.time()
        self.MBC_Reseted = False
        self.Obstacle = False
        ####################### Draw State Machine ######################
        self.get_graph().draw('Pytransitions/Other_Machines/SMovement/Data/SMovement.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/Other_Machines/SMovement/Data/SMovement.png', prog='dot') 

        ##############################Util Functions##############################
    def move(self, lv, av):
        print("Move(" + str(lv) + ", " + str(av) + ")")
        file.write("moveCall."+ str(lv) + "."+ str(av)+" -> ") 

    def stop(self):
        print("Stopping the robot.")
        file.write("stopCall -> ")
            

####################### Transition Conditions ####################### 
    def c_Obs(self):
        return self.Obstacle
    
    def c_Time_Reseted(self):
        return self.MBC_Reseted
    
    def c_Time(self):
        return time.time() - self.MBC >= self.PI/self.av

####################### Before Transitions ####################### 
    def before_tp_MovementAndAvoidance(self):
        ##### Reset MBC #####
        print("Reseting the time.")
        self.MBC = time.time()
        self.MBC_Reseted = True
        ##### Stop the robot #####
        self.stop()


####################### On_enter States #######################        
    def on_enter_Moving(self):
        print("Moved to the Moving state.")
        self.move(self.lv, 0)
        time.sleep(0.1)
        file.write("tock -> ")

    def on_exit_Moving(self):
        print("Exit Moving state.")
        time.sleep(0.1)

    def on_enter_MovementAndAvoidance(self):
        print("Moved to the MovementAndAvoidance state.")
        self.move(0, self.av)


    def run(self):
        self.cycle = 0
        while True:
            print("\n/////////////////////// Cycle: " + str(self.cycle) + " ///////////////////////\n")
            cond = 0
            self.cycle += 1
            
            print("Now my current state is " + self.state)

            if(self.state == 'Initial'):
                self.tp_Start()
                
            print("Now Obstacle is " + str(self.Obstacle))
            if(self.state == 'Moving' and self.Obstacle):
                file.write("obstacle.in -> ")
                self.tp_MovementAndAvoidance()
                cond = 1

            print("MBC (" + str(time.time() - self.MBC) + ") >= PI/av (" + str(self.PI/self.av) + ")? Answer: " + str(self.c_Time()))
            if(self.state == 'MovementAndAvoidance' and self.c_Time()):
                self.tp_Moving()
                cond = 1
                
            if cond == 0:
                file.write("tock -> ")
            
            ####################### Test Sector #######################
            self.Obstacle = random.choice([True, False])
            time.sleep(0.3)
            #self.av = random.random() * random.randrange(1, 10) + 0.1 #Random value between 0.1 and 10
            #self.lv = random.random() * random.randrange(1, 10) + 0.1
            if self.cycle == 15:
                print("\n Finishing the program. \n")
                break

######################## Instantiating and Running the State Machine #######################  
machine = SMovement()

file = open("Pytransitions/Other_Machines/SMovement/Data/trace.txt", "w")
machine.run()
file.close()