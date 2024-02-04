import time
from transitions import Machine

class Semaforo(Machine):

    states = ['Initial','Vermelho','Amarelo','Verde']

    transitions = [
        {'trigger': 'start', 'source': 'Initial', 'dest': 'Vermelho'},
        {'trigger': 'tp_Verde', 'source': 'Vermelho', 'dest': 'Verde'},
        {'trigger': 'tp_Vermelho', 'source': 'Amarelo', 'dest': 'Vermelho'},
        {'trigger': 'tp_Defeito', 'source': 'Amarelo', 'dest': None},
        {'trigger': 'tp_Amarelo', 'source': 'Verde', 'dest': 'Amarelo'},
        {'trigger': 'tp_Emergencia', 'source': ['Verde', 'Vermelho'], 'dest': 'Amarelo'}
     ]

    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        while tempo_atual - tempo_inicial < segundos:
            tempo_atual = time.time()

    def __init__(self):
        super().__init__(name="Semaforo", states=Semaforo.states, transitions=Semaforo.transitions, initial='Initial')

    def on_enter_Vermelho(self):
        print('Estou no Vermelho')
        self.temporizador(1)
        self.tp_Verde()

    def on_enter_Amarelo(self):
        print('Estou no Amarelo')
        self.temporizador(1)
        self.tp_Vermelho()

    def on_enter_Verde(self):
        print('Estou no Verde')
        self.temporizador(1)
        self.tp_Amarelo()

# instantiating all objects
machine = Semaforo()

#Starting
machine.start()