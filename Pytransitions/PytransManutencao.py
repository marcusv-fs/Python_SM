import random, time
from transitions.extensions import GraphMachine
from PytransSemaforo import *

####################### Parameters #######################   
MAX_CICLOS = 3

class ManutencaoSemaforo(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Defeituoso','Funcional','Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Start', 'source': 'Initial', 'dest': 'Funcional'},
        {'trigger': 'tp_Funcional', 'source': 'Defeituoso', 'dest': 'Funcional', 'conditions': 'c_SemaforoConsertado'},
        {'trigger': 'tp_Defeituoso', 'source': 'Funcional', 'dest': 'Defeituoso', 'unless': 'c_SemaforoConsertado'},
        {'trigger': 'End', 'source': 'Defeituoso', 'dest': 'Final', 'conditions': 'c_End'}
    ]
    
####################### Init and util Functions ####################### 
    def __init__(self):
        super().__init__(name="Manutenção Semáforo", states=ManutencaoSemaforo.states, transitions=ManutencaoSemaforo.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        self.ciclos = 0
        self.semaforoFuncional = False
        ####################### Draw State Machine ####################### 
        self.get_graph().draw('Pytransitions/PytransManutencao.png', prog='dot')

    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        while tempo_atual - tempo_inicial < segundos:
            tempo_atual = time.time()

####################### Transition Conditions ####################### 
    def c_SemaforoConsertado(self):
        return self.semaforoFuncional
    
    def c_End(self):
        return self.ciclos >= MAX_CICLOS

####################### On_enter States ####################### 
    def on_enter_Final(self):
        print("\nEsse semáforo foi desativado devido aos muitos problemas.")
   
    def on_enter_Defeituoso(self):
        print('\nO Semáforo está apresentando muitos defeitos. Uma equipe foi chamada para manutenção.')
        self.ciclos = self.ciclos + 1
        self.temporizador(0.1)
        self.semaforoFuncional = True

    def on_enter_Funcional(self):
        print('\nO semáforo está operacional. Iniciando ciclo: ' + str(self.ciclos) + "\n")
        semaforo1 = Semaforo()
        while True:
            if(semaforo1.state == 'Final'):
                self.semaforoFuncional = False
                self.temporizador(0.1)
                break
            else:
                semaforo1.run()
                self.semaforoFuncional = False

####################### Instantiating the Machine #######################  
manutencaoSemaforo = ManutencaoSemaforo()
semaforo1 = Semaforo()

####################### Running the State Machine ####################### 
manutencaoSemaforo.Start()
while manutencaoSemaforo.state != 'Final':
    if manutencaoSemaforo.ciclos >= MAX_CICLOS:
        manutencaoSemaforo.End()
    else:
        if(manutencaoSemaforo.semaforoFuncional == False):
            manutencaoSemaforo.tp_Defeituoso()
        else:
            manutencaoSemaforo.tp_Funcional()
            