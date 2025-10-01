transitions = [
        {'trigger': 'Initial_to_Phase1', 'source': 'Initial', 'dest': 'Phase1'},
        {'trigger': 'Phase1_to_Phase2', 'source': 'Phase1', 'dest': 'Phase2', 'conditions': 'cond_Phase1_Phase2'},
        {'trigger': 'Phase2_to_Phase1', 'source': 'Phase2', 'dest': 'Phase1', 'conditions': 'cond_Phase2_Phase1'},
        {'trigger': 'Phase2_to_Phase3', 'source': 'Phase2', 'dest': 'Phase3', 'conditions': 'cond_Phase2_Phase3'},
        {'trigger': 'Phase3_to_Final', 'source': 'Phase3', 'dest': 'Final', 'conditions': 'cond_Phase3_Final'},
    ]


for transition in transitions:
    print(transition.get("source"))