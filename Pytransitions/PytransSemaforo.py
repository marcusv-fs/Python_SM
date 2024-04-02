import random, time
from transitions.extensions import GraphMachine

####################### Parameters #######################   
MIN_ENERGIA = 10
MAX_DEFEITOS = 10

class Semaforo(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Vermelho','Amarelo', 'SemEnergia','Verde','Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'tp_Start', 'source': 'Initial', 'dest': 'Vermelho'},
        {'trigger': 'tp_Verde', 'source': 'Vermelho', 'dest': 'Verde', 'conditions': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Vermelho', 'source': 'Amarelo', 'dest': 'Vermelho', 'conditions': 'c_EnergiaSuficiente', 'unless': 'c_MuitosDefeitos'},
        {'trigger': 'tp_Amarelo', 'source': 'Verde', 'dest': 'Amarelo', 'conditions': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Defeito', 'source': ['Verde', 'Vermelho', 'Amarelo', 'SemEnergia'], 'dest': 'SemEnergia', 'before':'before_tp_Defeito', 'unless': 'c_EnergiaSuficiente'},
        {'trigger': 'tp_Normalizou', 'source': 'SemEnergia', 'dest': 'Vermelho', 'conditions': 'c_EnergiaSuficiente', 'unless': 'c_MuitosDefeitos'},
        {'trigger': 'tp_End', 'source': 'SemEnergia', 'dest': 'Final', 'conditions': 'c_MuitosDefeitos'}
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
    def before_tp_Defeito(self):
        print(' A energia está muito baixa. O semáforo está com defeito. Energia atual: ' + str(self.Energia))

####################### On_enter States #######################     
    def on_enter_Final(self):
        print(" O semáforo apresenta muitos defeitos. Finalizando semáforo")
   
    def on_enter_Vermelho(self):
        print(' Estou no Vermelho. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def on_enter_SemEnergia(self):
        print(" Sem energia. Aguardando normalização.")
        print(' Energia atual: ' + str(self.Energia))
        self.Defeitos = self.Defeitos + 1
        print(" Defeitos: " + str(self.Defeitos))
        if(self.Defeitos <= MAX_DEFEITOS):
            recarga = random.randint(0,100)
            self.Energia = self.Energia + recarga
            print(" Foram recarregados " + str(recarga) + " de energia agora")


    def on_enter_Amarelo(self):               
        print(' Estou no Amarelo. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def on_enter_Verde(self):
        print(' Estou no Verde. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)

    def run(self):
        while True:
            if(self.state == 'Initial'):
                self.tp_Start()

            if(self.state == 'Vermelho' and self.Energia >= MIN_ENERGIA):
                self.tp_Verde()
            if(self.state == 'Vermelho' and self.Energia < MIN_ENERGIA):
                self.tp_Defeito()

            if(self.state == 'Amarelo' and self.Energia >= MIN_ENERGIA):
                self.tp_Vermelho()
            if(self.state == 'Amarelo' and self.Energia < MIN_ENERGIA):
                self.tp_Defeito()

            if(self.state == 'Verde' and self.Energia >= MIN_ENERGIA):
                self.tp_Amarelo()
            if(self.state == 'Verde' and self.Energia < MIN_ENERGIA):
                self.tp_Defeito()

            if(self.state == 'SemEnergia' and self.Energia >= MIN_ENERGIA and self.Defeitos <= MAX_DEFEITOS):
                self.tp_Normalizou()
            if(self.state == 'SemEnergia' and self.Energia < MIN_ENERGIA and self.Defeitos <= MAX_DEFEITOS):
                self.tp_Defeito()
            if(self.state == 'SemEnergia' and self.Defeitos > MAX_DEFEITOS):
                self.tp_End()
            
            if(self.state == 'Final'):
                break

# ####################### Instantiating and Running the State Machine #######################  
machine = Semaforo()
machine.run()