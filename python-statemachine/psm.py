import random, time
import sys
from statemachine import State, StateMachine

class TrafficLightIsolatedTransitions(StateMachine):
    Energia = 100
    Defeitos = 0
    "A traffic light machine"
    Initial = State(initial=True)
    Verde = State()
    Amarelo = State()
    Vermelho= State()
    Final = State()
    
    tp_Amarelo = Verde.to(Amarelo)
    tp_Vermelho = Amarelo.to(Vermelho)
    tp_Verde = Vermelho.to(Verde)
    start = Initial.to(Vermelho)
    end = Amarelo.to(Final)

    tp_Emergencia = Verde.to(Amarelo) | Vermelho.to(Amarelo)
    tp_Defeito = Amarelo.to(Amarelo)

    def before_tp_Emergência(self):
        print('Emergência!!! Energia Baixa. Mudando para o Amarelo')
    
    def before_tp_Defeito(self):
        print('A energia está muito baixa. O semáforo está com defeito. Energia atual: ' + str(self.Energia))

    def on_enter_Vermelho(self):
        print('Estou no Vermelho. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def on_enter_Final(self):
        print("Fim de execução")
        sys.exit()

    def on_enter_Amarelo(self):
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

    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        self.Energia = self.Energia - 10
        while tempo_atual - tempo_inicial < segundos and self.Energia > 10:
            tempo_atual = time.time()
        

machine = TrafficLightIsolatedTransitions()

img_path = "python-statemachine/psm.png"
machine._graph().write_png(img_path)

machine.start()

while True:
    if machine.Energia < 10:
        if machine.current_state.id != 'Amarelo':
            machine.tp_Emergencia()
        else:
            machine.tp_Defeito()
    else:
        if(machine.current_state.id == 'Vermelho'):
            machine.tp_Verde()  
        elif(machine.current_state.id == 'Amarelo'):
            machine.tp_Vermelho()
        else:
            machine.tp_Amarelo()
