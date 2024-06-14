from distutils.errors import UnknownFileError
import random, time
from transitions.extensions import GraphMachine


class Semaphore(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Red','Yellow', 'NoEnergy','Green', 'Junction1', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'tp_Start', 'source': 'Initial', 'dest': 'Red'},
        {'trigger': 'tp_Green', 'source': 'Red', 'dest': 'Green', 'conditions': 'c_EnoughEnergy'},
        {'trigger': 'tp_Yellow', 'source': 'Green', 'dest': 'Yellow', 'conditions': 'c_EnoughEnergy'},
        {'trigger': 'tp_Red', 'source': 'Yellow', 'dest': 'Red', 'before':['make_a_noise', 'before_tp_Red'], 'conditions': 'c_EnoughEnergy', 'unless': 'c_TooManyErrors'},
        
        {'trigger': 'tp_Junction1', 'source': 'Yellow', 'dest': 'Junction1', 'unless': 'c_EnoughEnergy'},
        {'trigger': 'tp_LowEnergy', 'source': 'Junction1', 'dest': 'NoEnergy', 'unless': 'c_TooManyErrors'},
        {'trigger': 'tp_End2', 'source': 'Junction1', 'dest': 'Final', 'conditions': 'c_TooManyErrors'},
        
        {'trigger': 'tp_Defeito', 'source': ['Green', 'Red', 'NoEnergy'], 'dest': 'NoEnergy', 'before':'before_tp_Defeito', 'unless': 'c_EnoughEnergy'},
        {'trigger': 'tp_Normalizou', 'source': 'NoEnergy', 'dest': 'Red', 'conditions': 'c_EnoughEnergy', 'unless': 'c_TooManyErrors'},
        {'trigger': 'tp_End', 'source': 'NoEnergy', 'dest': 'Final', 'conditions': 'c_TooManyErrors'}
     ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="Semaphore", states=Semaphore.states, transitions=Semaphore.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        self.Energy = 100
        self.Errors = 0
        self.MIN_ENERGY = 10
        self.MAX_ERRORS = 10
        self.TIME_SLEEP = 0.1

        ####################### Draw State Machine ######################
        
        try:
            self.get_graph().draw('Pytransitions/Semaphore/PytransSemaphore4.cgimage', prog='dot')
            #Generates the digraph textual description, ignore the error
        except:
            pass
        
        self.get_graph().draw('Pytransitions/Semaphore/PytransSemaphore.canon', prog='dot')
        
        self.get_graph().draw('Pytransitions/Semaphore/PytransSemaphore.plain', prog='dot')

        

        


####################### Transition Conditions ####################### 
    def c_EnoughEnergy(self):
        return self.Energy >= self.MIN_ENERGY
    
    def c_TooManyErrors(self):
        return self.Errors >= self.MAX_ERRORS

####################### Before Transitions ####################### 
    def before_tp_Defeito(self):
        print(" The Energy is very low. The traffic light is faulty. Current energy: " + str(self.Energy))

    def make_a_noise(self):
        print(" Beep beep!")

    def before_tp_Red(self):
        print(" I'm switching to Red!")
        self.make_a_noise()

####################### On_enter States #######################     
    def on_enter_Final(self):
        print(" The traffic light has many errors. Finishing traffic light.")
   
    def on_enter_Red(self):
        print(" I am in the Red state. Current Energy: " + str(self.Energy))
        self.Energy = self.Energy - 10
        time.sleep(self.TIME_SLEEP)

    def on_enter_NoEnergy(self):
        print(" No energy. Waiting for normalization.")
        print(" Current Energy : " + str(self.Energy))
        self.Errors = self.Errors + 1
        print(" Errors: " + str(self.Errors))
        if(self.Errors <= self.MAX_ERRORS):
            #recharge = random.randint(0,100)
            recharge = recharge = 35
            self.Energy = self.Energy + recharge
            print(" " + str(recharge) + " energy units have now been recharged.")
            print("///////////////////////////////////////////////////////////////////////\n")


    def on_enter_Yellow(self):               
        print(" I am in the Yellow state. Current Energy: " + str(self.Energy))
        self.Energy = self.Energy - 10
        time.sleep(self.TIME_SLEEP)

    def on_enter_Green(self):
        print(" I am in the Green state. Current Energy: " + str(self.Energy))
        self.Energy = self.Energy - 10
        time.sleep(self.TIME_SLEEP)

    def on_enter_Junction1(self):               
        print(" I entered the junction")

    def run(self):
        while True:
            if(self.state == 'Initial'):
                self.tp_Start()

            if(self.state == 'Red' and self.Energy >= self.MIN_ENERGY):
                self.tp_Green()
            if(self.state == 'Red' and self.Energy < self.MIN_ENERGY):
                self.tp_Defeito()

            if(self.state == 'Yellow' and self.Energy >= self.MIN_ENERGY):
                self.tp_Red()
            if(self.state == 'Yellow' and self.Energy < self.MIN_ENERGY):
                self.tp_Junction1()

            if(self.state == 'Junction1' and self.Errors >= self.MAX_ERRORS):
                self.tp_End2()
            if(self.state == 'Junction1' and self.Errors < self.MAX_ERRORS):
                self.tp_LowEnergy()

            if(self.state == 'Green' and self.Energy >= self.MIN_ENERGY):
                self.tp_Yellow()
            if(self.state == 'Green' and self.Energy < self.MIN_ENERGY):
                self.tp_Defeito()

            if(self.state == 'NoEnergy' and self.Energy >= self.MIN_ENERGY and self.Errors <= self.MAX_ERRORS):
                self.tp_Normalizou()
            if(self.state == 'NoEnergy' and self.Energy < self.MIN_ENERGY and self.Errors <= self.MAX_ERRORS):
                self.tp_Defeito()
            if(self.state == 'NoEnergy' and self.Errors >= self.MAX_ERRORS):
                self.tp_End()
            
            if(self.state == 'Final'):
                break

# ####################### Instantiating and Running the State Machine #######################  
machine = Semaphore()
machine.run()