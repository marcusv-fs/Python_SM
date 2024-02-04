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
    states = ['Initial','Vermelho','Amarelo','Verde', 'Final']

    transitions = [
        {'trigger': 'start', 'source': 'Initial', 'dest': 'Vermelho'},
        {'trigger': 'tp_Verde', 'source': 'Vermelho', 'dest': 'Verde'},
        {'trigger': 'tp_Vermelho', 'source': 'Amarelo', 'dest': 'Vermelho'},
        {'trigger': 'tp_Defeito', 'source': 'Amarelo', 'dest': 'Amarelo', 'before': 'Erro'},
        {'trigger': 'tp_Amarelo', 'source': 'Verde', 'dest': 'Amarelo'},
        {'trigger': 'tp_Emergencia', 'source': ['Verde', 'Vermelho'], 'dest': 'Amarelo', 'before':'Aviso'},
        {'trigger': 'end', 'source': 'Amarelo', 'dest': 'Final'}
     ]

    def __init__(self):
        super().__init__(name="Semaforo", states=Semaforo.states, transitions=Semaforo.transitions, initial='Initial')
        self.Energia = 100
        self.Defeitos = 0

    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        self.Energia = self.Energia - 10
        while tempo_atual - tempo_inicial < segundos and self.Energia > 10:
            tempo_atual = time.time()
        if self.Energia < 10:
            if self.state != 'Amarelo':
                self.tp_Emergencia()
            else:
                self.tp_Defeito()

    def Aviso(self):
        print('Emergência!!! Energia Baixa. Mudando para o Amarelo')
    
    def Erro(self):
        print('A energia está muito baixa. O semáforo está com defeito. Energia atual: ' + str(self.Energia))
        

    def on_enter_Final(self):
        print("Fim de execução")
        sys.exit()
    
    def on_enter_Vermelho(self):
        print('Estou no Vermelho. Energia atual: ' + str(self.Energia))
        self.temporizador(1)
        self.tp_Verde()

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
        self.temporizador(1)
        self.tp_Vermelho()

    def on_enter_Verde(self):
        print('Estou no Verde. Energia atual: ' + str(self.Energia))
        self.temporizador(1)
        self.tp_Amarelo()



# instantiating all objects
machine = Semaforo()

# Desenho automático da máquina de estados
from transitions.extensions import GraphMachine

m = Semaforo()
graph = GraphMachine(model=Semaforo)
m.get_graph().draw('PytransitionsSemaphore.png', prog='dot')


#Starting
machine.start()