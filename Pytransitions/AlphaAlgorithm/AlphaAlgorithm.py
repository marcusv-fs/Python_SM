import random, time
from transitions.extensions import GraphMachine
from MovementAndAvoidance import MovementAndAvoidance
from Turning import Turning


class AlphaAlgorithm(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','MovementAndAvoidance', 'Turning']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_MovementAndAvoidance', 'source': 'Initial', 'dest': 'MovementAndAvoidance'},
        {'trigger': 'MovementAndAvoidance_to_Turning', 'source': 'MovementAndAvoidance', 'dest': 'Turning', 'conditions': 'cond_MovementAndAvoidance_to_Turning', 'before': 'before_MovementAndAvoidance_to_Turning'},
        {'trigger': 'Turning_to_MovementAndAvoidance', 'source': 'Turning', 'dest': 'MovementAndAvoidance', 'conditions': 'cond_Turning_to_MovementAndAvoidance'}
        ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="AlphaAlgorithm", states=AlphaAlgorithm.states, transitions=AlphaAlgorithm.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        self.MBC_Time = time.time()
        self.MBC_Reseted = False
        self.turned = False
        self.MB = 6
        self.av = 360
        self.lv = 0.5 
        self.p = "Position::left" 
        self.MB = 360
        self.n = 35
        self.alpha = 15

        ####################### Draw State Machine ######################
        self.get_graph().draw('Pytransitions/AlphaAlgorithm/Data/AlphaAlgorithm.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/AlphaAlgorithm/Data/AlphaAlgorithm.png', prog='dot')  

####################### Transition Conditions ####################### 
    def cond_MovementAndAvoidance_to_Turning(self):
        return (self.MBC_Time>= self.MB)
    
    def cond_Turning_to_MovementAndAvoidance(self):
        return self.turned   

####################### Before Transitions ####################### 
    def before_MovementAndAvoidance_to_Turning(self):
        ##### Reset MBC #####
        print("Reseting the time.")
        self.MBC_Time= time.time()
        self.MBC_Reseted = True

####################### On_enter States #######################        
    def on_enter_MovementAndAvoidance(self):
        print("I am in the MovementAndAvoidance State")
        self.turned = False
        MovementAndAvoidance.MovementAndAvoidance(self.MBC_Reseted, self.MBC_Time, self.av, self.lv, self.p, self.MB).run()


    def on_enter_Turning(self):
        print("I am in the MovementAndAvoidance State")
        self.turned = Turning.Turning(self.av, self.n, self.alpha).run()


    def run(self):
        self.cycle = 0
        while True:
            print("\n******************** Alpha Cycle: " + str(self.cycle) + " *****************\n")
            cond = 0
            self.cycle += 1
            
            print("Now my current state is " + self.state)

            if(self.state == 'Initial'):
                self.Initial_to_MovementAndAvoidance()

            if(self.state == 'MovementAndAvoidance' and self.cond_MovementAndAvoidance_to_Turning()):
                cond = 1
                self.MovementAndAvoidance_to_Turning()
                
            if(self.state == 'Turning' and self.cond_Turning_to_MovementAndAvoidance):
                cond = 1
                self.Turning_to_MovementAndAvoidance()

            if cond == 0:
                print("tock -> state:" + str(self.state))
            
            ####################### Test Sector #######################
            self.Obstacle = random.choice([True, False])
            time.sleep(0.3)
            #self.av = random.random() * random.randrange(1, 10) + 0.1 #Random value between 0.1 and 10
            #self.lv = random.random() * random.randrange(1, 10) + 0.1
            if self.cycle == 15:
                print("\n Finishing the program. \n")
                break

######################## Instantiating and Running the State Machine #######################  
machine = AlphaAlgorithm()

machine.run()