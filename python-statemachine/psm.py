import random, time, sys
from statemachine import State, StateMachine

class Python_statemachine(StateMachine):
####################### States Declaration #######################   
    Initial = State(initial=True)
    Verde = State()
    Amarelo = State()
    Vermelho= State()
    Final = State()

####################### Transitions Statement  #######################  
    tp_Amarelo = Verde.to(Amarelo, cond="c_EnergiaSuficiente")
    tp_Vermelho = Amarelo.to(Vermelho, cond="c_EnergiaSuficiente", unless="c_MuitosDefeitos")
    tp_Verde = Vermelho.to(Verde, cond="c_EnergiaSuficiente")
    start = Initial.to(Vermelho)
    end = Amarelo.to(Final, cond="c_MuitosDefeitos")

    tp_Emergencia = Verde.to(Amarelo, unless="c_EnergiaSuficiente") | Vermelho.to(Amarelo, unless="c_EnergiaSuficiente")
    # tp_Normal = tp_Verde | tp_Vermelho | tp_Amarelo # Descomente esse e a condição no while true para usar o modo alternativo

    tp_Defeito = Amarelo.to(Amarelo, unless=["c_MuitosDefeitos", "c_EnergiaSuficiente"])

####################### Init and util Functions ####################### 
    Energia = 100
    Defeitos = 0
    def temporizador(self, segundos):
        tempo_inicial = time.time()
        tempo_atual = tempo_inicial
        self.Energia = self.Energia - 10
        while tempo_atual - tempo_inicial < segundos and self.Energia > 10:
            tempo_atual = time.time()
            
####################### Transition Conditions ####################### 
    def c_EnergiaSuficiente(self):
        return self.Energia >= 10
    
    def c_MuitosDefeitos(self):
        return self.Defeitos > 3

####################### Before Transitions ####################### 
    def before_tp_Emergencia(self):
        print('Emergência!!! Energia Baixa. Mudando para o Amarelo')
    
    def before_tp_Defeito(self):
        print('A energia está muito baixa. O semáforo está com defeito. Energia atual: ' + str(self.Energia))
        recarga = random.randint(0,100)
        self.Energia = self.Energia + recarga
        print("Foram recarregados " + str(recarga) + " de energia agora")


####################### On_enter States ####################### 
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
            if(self.Defeitos > 3):
                self.end()
            self.tp_Defeito()   
        else:        
            print('Estou no Amarelo. Energia atual: ' + str(self.Energia))
            self.temporizador(0.1)
 
    def on_enter_Verde(self):
        print('Estou no Verde. Energia atual: ' + str(self.Energia))
        self.temporizador(0.1)
        
####################### Instantiating all objects #######################  
machine = Python_statemachine()

####################### Draw State Machine ####################### 
img_path = "python-statemachine/psm.png"
machine._graph().write_png(img_path)

####################### Running the State Machine ####################### 
machine.start()
while True:
    if machine.Energia < 10:
        if machine.current_state.id != 'Amarelo':
            machine.tp_Emergencia()
        else:
            machine.tp_Defeito()
    else:
        # machine.tp_Normal() # Desse jeito, podemos deletar tudo pra baixo e deixar só essa linha
        if(machine.current_state.id == 'Vermelho'):
            machine.tp_Verde()  
        elif(machine.current_state.id == 'Amarelo'):
            machine.tp_Vermelho()
        else:
            machine.tp_Amarelo()
