# pip install transitions
# pip install graphviz

from datetime import datetime
from math import floor
import random, time
from shutil import move
from tkinter import N
from transitions.extensions import GraphMachine


class Turning(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial', 'Junction1', 'Turn180', 'RandomTurn', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Junction1', 'source': 'Initial', 'dest': 'Junction1'},
        {'trigger': 'Junction1_to_Turn180', 'source': 'Junction1', 'dest': 'Turn180', 'conditions': 'cond_Junction1_to_Turn180'},
        {'trigger': 'Junction1_to_RandomTurn', 'source': 'Junction1', 'dest': 'RandomTurn', 'conditions': 'cond_Junction1_to_RandomTurn'},
        {'trigger': 'Turn180_and_RandomTurn_to_Final', 'source': ['Turn180', 'RandomTurn'], 'dest': 'Final', 'before': 'before_Final'}
        ]
    
####################### Init and util Functions ####################### 
    def __init__(self,av, n, alpha):
        super().__init__(name="Turning", states=Turning.states, transitions=Turning.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        
        self.av = av
        self.n = n
        self.alpha = alpha
        self.turned = False
        
        ####################### Draw State Machine ######################
        self.file = open("Pytransitions/AlphaAlgorithm/Turning/Data/Turning_trace.txt", "w")
        self.get_graph().draw('Pytransitions/AlphaAlgorithm/Turning/Data/Turning.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/AlphaAlgorithm/Turning/Data/Turning.png', prog='dot') 

        ##############################Util Functions##############################
        
    def move(self, lv, av):
        print("Move(" + str(lv) + ", " + str(av) + ")")
        self.file.write("moveCall."+ str(lv) + "."+ str(av)+" -> ")             

####################### Transition Conditions ####################### 
    def cond_Junction1_to_Turn180(self):
        if(self.n < self.alpha):
            return True
        else:
            return False

    def cond_Junction1_to_RandomTurn(self):
        if(self.n >= self.alpha):
            return True
        else:
            return False

####################### Before Transitions ####################### 
    def before_Final(self):
        print("Transitioning to the final state.")
        self.file.write("Transitioning to the final state.")
        self.turned = True


####################### On_enter States #######################        
    def on_enter_Turn180(self):
        print("Turned to the Turn180 state")
        self.move(0, self.av)
        tempo = floor((random.randint(0, 1)*(360/self.av)))
        time.sleep(tempo)
        self.file.write("tock -> ")
        self.turned = True

    def on_enter_RandomTurn(self):
        print("Turned to RandomTurn state.")
        self.move(0, self.av)
        tempo = floor((random.randint(0, 1)*(360/self.av)))
        time.sleep(tempo)
        self.file.write("tock -> ")
        self.turned = True
        

    def run(self):
        while True:
            print("\n--------------- Turning Cycle: --------------\n")
            print("Now my current state is " + self.state)

            if(self.state == 'Initial'):
                self.Initial_to_Junction1()
                
            if(self.state == 'Junction1'):
                print("I am in the Junction1")

                self.file.write("I am in the Junction1")
                if (self.cond_Junction1_to_Turn180()):
                    self.Junction1_to_Turn180()
                elif (self.cond_Junction1_to_RandomTurn()):
                    self.Junction1_to_RandomTurn()

            if(self.state == 'RandomTurn'):
                print("I am in the RandomTurn State")
                self.file.write("I am in the RandomTurn State")
                self.to_Final()

            if(self.state == 'Turn180'):
                print("I am in the Turn180 State")
                self.file.write("I am in the Turn180 State")
                self.to_Final()

            if(self.state == 'Final'):
                print("I am in the Final State")
                self.file.write("I am in the Final State")
                self.file.close()
                break

        return self.turned




machine = Turning(360, 35, 15)
machine.run()
