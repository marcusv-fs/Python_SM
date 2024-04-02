import random, time, sys
from statemachine import State, StateMachine

####################### Parameters #######################   
MIN_ENERGIA = 10
MAX_DEFEITOS = 10

class Semaforo(StateMachine):
####################### States Declaration #######################   
    Initial = State(initial=True)
    Verde = State()
    Amarelo = State()
    Vermelho= State()
    SemEnergia = State()
    Final = State()

####################### Transitions Statement  #######################  
    tp_Start = Initial.to(Vermelho)
    tp_Verde = Vermelho.to(Verde, cond="c_EnergiaSuficiente")
    tp_Vermelho = Amarelo.to(Vermelho, cond="c_EnergiaSuficiente", unless="c_MuitosDefeitos")   
    tp_Amarelo = Verde.to(Amarelo, cond="c_EnergiaSuficiente")
    tp_Defeito = Amarelo.to(SemEnergia, unless="c_EnergiaSuficiente") | Verde.to(SemEnergia, unless="c_EnergiaSuficiente") | SemEnergia.to(SemEnergia, unless="c_EnergiaSuficiente") |Vermelho.to(SemEnergia, unless=["c_MuitosDefeitos", "c_EnergiaSuficiente"])
    tp_Normalizou = SemEnergia.to(Vermelho, cond="c_EnergiaSuficiente", unless="c_MuitosDefeitos")
    tp_End = SemEnergia.to(Final, cond="c_MuitosDefeitos")

    
####################### Init and util Functions ####################### 
    Energia = 100
    Defeitos = 0
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
        ####################### Draw State Machine ####################### 
        self._graph().write_png("python-statemachine/psm.png")
        
    
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
            if(self.current_state.id == 'Initial'):
                self.tp_Start()

            if(self.current_state.id == 'Vermelho' and self.Energia >= MIN_ENERGIA):
                self.tp_Verde()
            if(self.current_state.id == 'Vermelho' and self.Energia <= MIN_ENERGIA):
                self.tp_Defeito()

            if(self.current_state.id == 'Amarelo' and self.Energia >= MIN_ENERGIA):
                self.tp_Vermelho()
            if(self.current_state.id == 'Amarelo' and self.Energia <= MIN_ENERGIA):
                self.tp_Defeito()

            if(self.current_state.id == 'Verde' and self.Energia >= MIN_ENERGIA):
                self.tp_Amarelo()
            if(self.current_state.id == 'Verde' and self.Energia <= MIN_ENERGIA):
                self.tp_Defeito()

            if(self.current_state.id == 'SemEnergia' and self.Energia >= MIN_ENERGIA and self.Defeitos <= MAX_DEFEITOS):
                self.tp_Normalizou()
            if(self.current_state.id == 'SemEnergia' and self.Energia <= MIN_ENERGIA and self.Defeitos <= MAX_DEFEITOS):
                self.tp_Defeito()
            if(self.current_state.id == 'SemEnergia' and self.Defeitos > MAX_DEFEITOS):
                self.tp_End()
            
            if(self.current_state.id == 'Final'):
                break

        
####################### Instantiating all objects #######################  
machine = Semaforo()
machine.run()
