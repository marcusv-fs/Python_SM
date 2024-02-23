import random, time
from transitions.extensions import GraphMachine

####################### Parameters #######################   
MIN_ENERGIA = 10
MAX_DEFEITOS = 1

class Semaforo(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Vermelho','Amarelo','Verde','Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'start', 'source': 'Initial', 'dest': 'Vermelho'},
        {'trigger': 'tp_Verde', 'source': 'Vermelho', 'dest': 'Verde', 'conditions': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Vermelho', 'source': 'Amarelo', 'dest': 'Vermelho', 'conditions': 'c_EnergiaSuficiente', 'unless': 'c_MuitosDefeitos'},
        {'trigger': 'tp_Defeito', 'source': 'Amarelo', 'dest': 'Amarelo', 'before': 'b_tp_Defeito', 'unless': ['c_EnergiaSuficiente', 'c_MuitosDefeitos']},
        {'trigger': 'tp_Amarelo', 'source': 'Verde', 'dest': 'Amarelo', 'conditions': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Emergencia', 'source': ['Verde', 'Vermelho'], 'dest': 'Amarelo', 'before':'b_tp_Emergencia', 'unless': 'c_EnergiaSuficiente'},
        {'trigger': 'end', 'source': 'Amarelo', 'dest': 'Final', 'conditions': 'c_MuitosDefeitos'}
     ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="Semaforo", states=Semaforo.states, transitions=Semaforo.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        self.Energia = 100
        self.Defeitos = 0
        ####################### Draw State Machine ######################
        self.get_graph().draw('Pytransitions/PytransSemaforo.png', prog='dot')

    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        self.Energia = self.Energia - 10
        while tempo_atual - tempo_inicial < segundos and self.Energia > MIN_ENERGIA:
            tempo_atual = time.time()

####################### Transition Conditions ####################### 
    def c_EnergiaSuficiente(self):
        return self.Energia >= MIN_ENERGIA
    
    def c_MuitosDefeitos(self):
        return self.Defeitos > MAX_DEFEITOS

####################### Before Transitions ####################### 
    def b_tp_Emergencia(self):
        print(' Emergência!!! Energia Baixa. Mudando para o Amarelo')
    
    def b_tp_Defeito(self):
        print(' A energia está muito baixa. O semáforo está com defeito. Energia atual: ' + str(self.Energia))
        recarga = random.randint(0,100)
        self.Energia = self.Energia + recarga
        print(" Foram recarregados " + str(recarga) + " de energia agora")

####################### On_enter States ####################### 
    def on_enter_Final(self):
        print(" Finalizando semáforo")
   
    def on_enter_Vermelho(self):
        print(' Estou no Vermelho. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def on_enter_Amarelo(self):
        if(self.Energia < MIN_ENERGIA):
            print(' Energia atual: ' + str(self.Energia))
            self.Defeitos = self.Defeitos + 1
            print(" Defeitos: " + str(self.Defeitos))
            if(self.Defeitos > MAX_DEFEITOS):
                self.end()
            else:
                self.tp_Defeito()          
        else:        
            print(' Estou no Amarelo. Energia atual: ' + str(self.Energia))
            self.temporizador(0.1)

    def on_enter_Verde(self):
        print(' Estou no Verde. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def run(self):
        self.start()
        while True:
            if(self.state == 'Final'):
                break
            else:
                if self.Energia < MIN_ENERGIA:
                    if self.state != 'Amarelo':
                        self.tp_Emergencia()
                    else:
                        self.tp_Defeito()
                else:
                    if(self.state == 'Vermelho'):
                        self.tp_Verde()  
                    elif(self.state == 'Amarelo'):
                        self.tp_Vermelho()
                    else:
                        self.tp_Amarelo()


# ####################### Instantiating all objects #######################  
# machine = Semaforo()

# ####################### Running the State Machine ####################### 
# machine.start()
# while True:
#     if(machine.state == 'Final'):
#         break
#     else:
#         if machine.Energia < MIN_ENERGIA:
#             if machine.state != 'Amarelo':
#                 machine.tp_Emergencia()
#             else:
#                 machine.tp_Defeito()
#         else:
#             if(machine.state == 'Vermelho'):
#                 machine.tp_Verde()  
#             elif(machine.state == 'Amarelo'):
#                 machine.tp_Vermelho()
#             else:
#                 machine.tp_Amarelo()
