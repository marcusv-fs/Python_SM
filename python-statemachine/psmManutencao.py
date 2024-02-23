import random, time, sys
from statemachine import State, StateMachine
from psmSemaforo import *
####################### Parameters #######################   
MAX_CICLOS = 300

class ManutencaoSemaforo(StateMachine):
####################### States Declaration #######################   
    Initial = State(initial=True)
    Defeituoso = State()
    Funcional = State()
    Final = State()

####################### Transitions Statement  #######################  
    Start = Initial.to(Funcional)
    tp_Funcional = Defeituoso.to(Funcional, cond="c_SemaforoConsertado")
    tp_Defeituoso = Funcional.to(Defeituoso, unless="c_SemaforoConsertado")
    End = Defeituoso.to(Final, cond="c_End")

####################### Init and util Functions ####################### 
    ciclos = 0
    semaforoFuncional = False

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
            if(semaforo1.current_state.id == 'Final'):
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
while manutencaoSemaforo.current_state.id != 'Final':
    if manutencaoSemaforo.ciclos >= MAX_CICLOS:
        manutencaoSemaforo.End()
    else:
        if(manutencaoSemaforo.semaforoFuncional == False):
            manutencaoSemaforo.tp_Defeituoso()
        else:
            manutencaoSemaforo.tp_Funcional()
            