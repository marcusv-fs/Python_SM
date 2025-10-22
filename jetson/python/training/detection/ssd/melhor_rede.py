import os
import re

def encontrar_melhor_rede(pasta, apagar_restantes=False):
    # Expressão regular para capturar o número após "Loss-"
    padrao_loss = re.compile(r"Loss-([0-9.]+)\.pth$")
    
    melhor_arquivo = None
    menor_loss = float("inf")

    # Passo 1: encontrar o melhor arquivo
    for nome_arquivo in os.listdir(pasta):
        if nome_arquivo.endswith(".pth"):
            caminho = os.path.join(pasta, nome_arquivo)
            match = padrao_loss.search(nome_arquivo)
            if match:
                loss = float(match.group(1))
                if loss < menor_loss:
                    menor_loss = loss
                    melhor_arquivo = caminho

    if melhor_arquivo:
        print(f"✅ Melhor rede: {os.path.basename(melhor_arquivo)}")
        print(f"📉 Loss: {menor_loss}")

        # Passo 2: apagar os demais, se desejado
        if apagar_restantes:
            for nome_arquivo in os.listdir(pasta):
                caminho = os.path.join(pasta, nome_arquivo)
                if (
                    nome_arquivo.endswith(".pth")
                    and os.path.abspath(caminho) != os.path.abspath(melhor_arquivo)
                ):
                    os.remove(caminho)
                    print(f"🗑️ Apagado: {nome_arquivo}")

        return melhor_arquivo, menor_loss
    else:
        print("⚠️ Nenhum arquivo compatível encontrado.")
        return None, None


# Exemplo de uso:
if __name__ == "__main__":
    pasta = "./models/FRTL"  # coloque o caminho da sua pasta aqui
    encontrar_melhor_rede(pasta, apagar_restantes=True)
