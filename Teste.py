status = 61651

def teste(status):
    if status == 3:
        return True
    else:
        return False


match (status):
    case 1:
        print(1);   
    case 2 if False:
        print(2, "false")
    case 2 if True:
        print(2, "true")
    case 3 if teste(status):
        print(3, "true")
    case 3 if not teste(status):
        print(3, "false")
    case _:
        print("tock")

