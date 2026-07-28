# Oscillating Platform 🚁🌊

Este pacote ROS 2 implementa um sistema de controle e estimação preditiva para operações de pouso e "surfe" (hovering compensado) de um nano-drone em uma plataforma com movimento periódico (ex: embarcações no mar).

O algoritmo utiliza um Modelo Autorregressivo (AR) acoplado a Mínimos Quadrados Recursivos (RLS), junto a uma ação de controle Feedforward de velocidade e posição em tempo real.

## ⚠️ Dependências e Requisitos
* Este pacote foi inteiramente construído para operar em conjunto com o ecossistema **[Crazyswarm2](https://imrclab.github.io/crazyswarm2/)**.
* **[TODO]**: O correto funcionamento exige configurações específicas de tópicos e *bitmasks* nos arquivos `crazyflie.yaml` e `teleop.yaml` do Crazyswarm2 (a documentação detalhada desses parâmetros será adicionada no futuro).

---

## 📁 Estrutura do Pacote

```text
oscillating_platform/
├── launch/
│   └── launch.py                    
├── oscillating_platform/            
│   ├── __init__.py                  
│   ├── ar_platform_estimator.py     
│   └── joystick.py                  
├── rviz/
│   └── config.rviz                  
├── package.xml                      
├── setup.py                         
└── README.md

```

### O que cada arquivo faz?

* **`launch/launch.py`**: Ponto de entrada do projeto. Ele inicializa o servidor do Crazyswarm2, o RViz com a nossa interface customizada e os dois nós locais do projeto simultaneamente.
* **`ar_platform_estimator.py`**: O cérebro matemático. Ele funde os dados do TF2 (MoCap) e do sensor a laser (RangeFinder), aplica um filtro passa-baixa (EMA), rejeita anomalias e roda o Filtro Adaptativo (AR + RLS) para prever a posição futura ($Z$) e a velocidade ($\dot{Z}$) da plataforma.
* **`joystick.py`**: O coordenador de voo. Uma máquina de estados que lê os botões do controle e orquestra transições seguras. Ele consome a previsão do estimador e usa o tópico `/crazyflie/cmd_full_state` para injetar o Feedforward no firmware do Crazyflie.
* **`rviz/config.rviz`**: Configuração visual para monitorar a leitura atual (marcador verde) e o horizonte de previsões da onda (marcadores amarelos com fade-out) no espaço 3D.

---

## 🚀 Como Compilar e Executar

1. Navegue até a raiz do seu *workspace* (ex: `ros2_ws`):
```bash
cd ~/ros2_ws

```


2. Compile apenas este pacote utilizando o link simbólico (para que alterações no código Python não exijam recompilação):
```bash
colcon build --symlink-install --packages-select oscillating_platform --cmake-args -DCMAKE_BUILD_TYPE=Release

```


3. Atualize o ambiente do terminal:
```bash
source install/setup.bash

```


4. Dispare o sistema completo:
```bash
ros2 launch oscillating_platform launch.py

```



### 🎮 Controles Padrão (Joystick)

* **Botão 0 (A/X)**: Inicia voo em Figura-8 travado na altitude atual.
* **Botão 1 (B/Círculo)**: Aborta ação e retorna o drone para a Origem (0.0, 0.0, 0.3m).
* **Botão 2 (X/Quadrado)**: Rotina de aproximação (Sobe para 1m e vai para o X/Y da plataforma).
* **Botão 3 (Y/Triângulo)**: Ativa/Desativa o "Surfe" (Compensação Feedforward baseada na previsão).

```
