# Criar um semáforo com 3 estados
# 1 - Vermelho, 2 - Amarelo, 3 - Verde
# Verde -> Amarelo -> Vermelho -> Verde
# Ele deve demorar 10s no vermelho, 2s no amarelo e 5s no verde.
# Se receber um sinal de um botão enquanto está no verde, 
# ele deve mudar de estado para o amarelo imediatamente

import random
import sys
import time
from transitions.extensions import GraphMachine

class Semaforo(GraphMachine):
    states = ['Initial','Vermelho','Amarelo','Verde','Final']

    transitions = [
        {'trigger': 'start', 'source': 'Initial', 'dest': 'Vermelho'},
        {'trigger': 'tp_Verde', 'source': 'Vermelho', 'dest': 'Verde', 'conditions': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Vermelho', 'source': 'Amarelo', 'dest': 'Vermelho', 'conditions': 'c_EnergiaSuficiente', 'unless': 'c_MuitosDefeitos'},
        {'trigger': 'tp_Defeito', 'source': 'Amarelo', 'dest': 'Amarelo', 'before': 'b_Erro', 'unless': ['c_EnergiaSuficiente', 'c_MuitosDefeitos']},
        {'trigger': 'tp_Amarelo', 'source': 'Verde', 'dest': 'Amarelo', 'conditions': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Emergencia', 'source': ['Verde', 'Vermelho'], 'dest': 'Amarelo', 'before':'b_Aviso', 'unless': 'c_EnergiaSuficiente'},
        {'trigger': 'end', 'source': 'Amarelo', 'dest': 'Final', 'conditions': 'c_MuitosDefeitos'}
     ]
    
####################### Transition Conditions ####################### 
    def c_EnergiaSuficiente(self):
        return self.Energia >= 10
    
    def c_MuitosDefeitos(self):
        return self.Defeitos >= 3

####################### Init and assist functions ####################### 
    def __init__(self):
        super().__init__(name="Semaforo", states=Semaforo.states, transitions=Semaforo.transitions, initial='Initial', show_conditions=True, show_state_attributes=True)
        self.Energia = 100
        self.Defeitos = 0

    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        self.Energia = self.Energia - 10
        while tempo_atual - tempo_inicial < segundos and self.Energia > 10:
            tempo_atual = time.time()

####################### Before Transitions ####################### 
    def b_Aviso(self):
        print('Emergência!!! Energia Baixa. Mudando para o Amarelo')
    
    def b_Erro(self):
        print('A energia está muito baixa. O semáforo está com defeito. Energia atual: ' + str(self.Energia))
        
####################### On_enter States ####################### 
    def on_enter_Final(self):
        print("Fim de execução")
        sys.exit()
   
    def on_enter_Vermelho(self):
        print('Estou no Vermelho. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def on_enter_Amarelo(self): #####Falta arrumar a energia aqui do amarelo
        if(self.Energia < 10):
            print('Energia atual: ' + str(self.Energia))
            self.Defeitos = self.Defeitos + 1
            print("Defeitos: " + str(self.Defeitos))
            if(self.Defeitos >= 3):
                self.end()
            self.Energia = self.Energia + random.randint(0,100)
            self.tp_Defeito()        
        print('Estou no Amarelo. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def on_enter_Verde(self):
        print('Estou no Verde. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)


####################### Instantiating all objects #######################  
machine = Semaforo()

####################### Draw State Machine ####################### 
from transitions.extensions import GraphMachine
m = Semaforo()
graph = GraphMachine(model=Semaforo)
m.get_graph().draw('Pytransitions/Pytransitions.png', prog='dot')

####################### Running the State Machine ####################### 
machine.start()
while True:
    if machine.Energia < 10:
        if machine.state != 'Amarelo':
            machine.tp_Emergencia()
        else:
            machine.tp_Defeito()
    else:
        if(machine.state == 'Vermelho'):
            machine.tp_Verde()  
        elif(machine.state == 'Amarelo'):
            machine.tp_Vermelho()
        else:
            machine.tp_Amarelo()
