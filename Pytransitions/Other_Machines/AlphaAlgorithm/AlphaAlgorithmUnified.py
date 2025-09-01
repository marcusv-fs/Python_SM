# pip install transitions
# pip install graphviz

from math import floor
import random, time
from transitions.extensions import GraphMachine

###########################Global Variables############################
alpha = 15
av = 360
const = 355
lv = 0.5 
MB = 370
MBC_Time = time.time()
MBC_Reseted = False
n = 35
p = "Position::left" 
PI = 3.14159265359
Obstacle = False
turned = False

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
    def __init__(self):
        super().__init__(name="MovementAndAvoidance", states=MovementAndAvoidance.states, transitions=MovementAndAvoidance.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        
        ####################### Draw State Machine ######################
        self.file = open("Pytransitions/Other_Machines/AlphaAlgorithm/Data/MovementAndAvoidance/MovementAndAvoidance_trace.txt", "w")
        self.get_graph().draw('Pytransitions/Other_Machines/AlphaAlgorithm/Data/MovementAndAvoidance/MovementAndAvoidance.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/Other_Machines/AlphaAlgorithm/Data/MovementAndAvoidance/MovementAndAvoidance.png', prog='dot') 

        ##############################Util Functions##############################
        
    def move(self, lv, av):
        print("Move(" + str(lv) + ", " + str(av) + ")")
        self.file.write("moveCall."+ str(lv) + "."+ str(av)+" -> ") 

    def stop(self):
        print("Stopping the robot.")
        self.file.write("stopCall -> ")
            

####################### Transition Conditions ####################### 
    def Cond_Move_to_Avoid(self):
        global Obstacle
        Obstacle = False

        if (MBC_Reseted == True):
            if(MBC_Time - time.time() < (MB - 360)/av):
                return True
        return Obstacle

####################### On_enter States #######################        
    def on_enter_Move(self):
        print("I am in the Move state")
        self.move(lv, 0)
        time.sleep(0.1)
        self.file.write("tock -> ")

    def on_enter_Avoid(self):
        print("I am in the Avoid state.")
        if (p == "Positon::left"):
            self.move(0, av)
        else:
            self.move(0, - av)
        tempo = floor((random.randint(0, 1)*(360/av)))
        time.sleep(tempo)
        


    def run(self):
        global Obstacle
        self.cycle = 0
        print (time.time() - MBC_Time)
        while (True and (time.time() - MBC_Time + const) < MB):
            print("\n///////////////// M&A Cycle: " + str(self.cycle) + " /////////////////\n")
            print ("curr_time: " + str(time.time() - MBC_Time + const)) # Adding time just to finish the program early
            cond = 0
            self.cycle += 1
            
            print("Now my current state is " + self.state)

            if(self.state == 'Initial'):
                self.Initial_to_Move()
                
            print("Now Obstacle is " + str(Obstacle))
            if(self.state == 'Move' and Obstacle == True):
                print("Moving to avoid...")
                self.file.write("obstacle.in -> ")
                self.Move_to_Avoid()
                cond = 1

            print("MBC (" + str(time.time() - MBC_Time) + ") >= PI/av (" + str((MB - 360)/av) + ")? Answer: " + str(self.Cond_Move_to_Avoid()))
            Obstacle = self.Cond_Move_to_Avoid()
            if(self.state == 'Avoid'):
                self.Avoid_to_Move()
                cond = 1
                
            if cond == 0:
                self.file.write("tock -> ")
            
            ####################### Test Sector #######################
            Obstacle = self.Cond_Move_to_Avoid()
            time.sleep(0.3)
            #MB = random.randint(360, 7200)
            # if self.cycle == 15:
            #     print("\n Finishing the program. \n")
            #     break

        self.file.close()
            

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
    def __init__(self):
        super().__init__(name="Turning", states=Turning.states, transitions=Turning.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        global turned
        neighbours = n

        turned = False
        ####################### Draw State Machine ######################
        self.file = open("Pytransitions/Other_Machines/AlphaAlgorithm/Data/Turning/Turning_trace.txt", "w")
        self.get_graph().draw('Pytransitions/Other_Machines/AlphaAlgorithm/Data/Turning/Turning.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/Other_Machines/AlphaAlgorithm/Data/Turning/Turning.png', prog='dot') 

        ##############################Util Functions##############################
        
    def move(self, lv, av):
        print("Move(" + str(lv) + ", " + str(av) + ")")
        self.file.write("moveCall."+ str(lv) + "."+ str(av)+" -> ")             

####################### Transition Conditions ####################### 
    def cond_Junction1_to_Turn180(self):
        if(n < alpha):
            return True
        else:
            return False

    def cond_Junction1_to_RandomTurn(self):
        if(n >= alpha):
            return True
        else:
            return False

####################### Before Transitions ####################### 
    def before_Final(self):
        global turned
        print("Transitioning to the final state.")
        self.file.write("Transitioning to the final state.")
        turned = True


####################### On_enter States #######################        
    def on_enter_Turn180(self):
        global turned 
        print("Turned to the Turn180 state")
        self.move(0, av)
        tempo = floor((random.randint(0, 1)*(360/av)))
        time.sleep(tempo)
        self.file.write("tock -> ")
        turned = True

    def on_enter_RandomTurn(self):
        global turned
        print("Turned to RandomTurn state.")
        self.move(0, av)
        tempo = floor((random.randint(0, 1)*(360/av)))
        time.sleep(tempo)
        self.file.write("tock -> ")
        turned = True
        

    def run(self):
        global turned
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

        return turned


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
        global MBC_Time 
        global MBC_Reseted 
        global turned 
        global MB 
        global av 
        global lv
        global p 
        global MB 
        global n
        global alpha

        ####################### Draw State Machine ######################
        self.get_graph().draw('Pytransitions/Other_Machines/AlphaAlgorithm/Data/AlphaAlgorithm/AlphaAlgorithm.canon', prog='dot') 
        self.get_graph().draw('Pytransitions/Other_Machines/AlphaAlgorithm/Data/AlphaAlgorithm/AlphaAlgorithm.png', prog='dot')  

####################### Transition Conditions ####################### 
    def cond_MovementAndAvoidance_to_Turning(self):
        return (MBC_Time >= MB)
    
    def cond_Turning_to_MovementAndAvoidance(self):
        return turned   

####################### Before Transitions ####################### 
    def before_MovementAndAvoidance_to_Turning(self):
        ##### Reset MBC #####
        print("Reseting the time.")
        global MBC_Time
        global MBC_Reseted
        MBC_Time = time.time()
        MBC_Reseted = True

####################### On_enter States #######################        
    def on_enter_MovementAndAvoidance(self):
        print("I am in the MovementAndAvoidance State")
        global turned
        turned = False
        MovementAndAvoidance().run()


    def on_enter_Turning(self):
        print("I am in the MovementAndAvoidance State")
        global turned
        turned = Turning().run()


    def run(self):
        global Obstacle
        cycle = 0
        while True:
            print("\n******************** Alpha Cycle: " + str(cycle) + " *****************\n")
            cond = 0
            cycle += 1
            
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
            Obstacle = random.choice([True, False])
            time.sleep(0.3)
            #av= random.random() * random.randrange(1, 10) + 0.1 #Random value between 0.1 and 10
            #lv = random.random() * random.randrange(1, 10) + 0.1
            if cycle == 15:
                print("\n Finishing the program. \n")
                break


######################## Instantiating and Running the State Machine #######################  
machine = AlphaAlgorithm()

machine.run()