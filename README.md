# Projeto Carrinho Controle de Direção (ESP32)

Este repositório contém três versões de um projeto de controle de direção para um carrinho robô baseado em **ESP32**, com implementação de **controle PID** para ajuste fino da direção. Cada pasta corresponde a uma abordagem diferente de leitura de sensores e controle:

## Estrutura de Pastas

```
└── CarrinhoControleDirecaoInterrupcao.ino
└── CarrinhoControleDirecaoLoop.ino
└── CarrinhoControleDirecaoLoopBluetooth.ino
```

## Descrição dos Exemplos

### 1. CarrinhoControleDirecaoInterrupcao

* **Arquivo**: `CarrinhoControleDirecaoInterrupcao.ino`
* **Abordagem**: Utiliza **interrupções por timer** (TimerInterrupt) para disparos periódicos e execução da lógica de controle.
* **Controle**: Algoritmo PID dentro da rotina de interrupção para estabilizar a direção.
* **Fluxo**: O ESP32 gera interrupções de tempo para ler sensores, calcular o PID e ajustar a direção e velocidade dos motores.

### 2. CarrinhoControleDirecaoLoop

* **Arquivo**: `CarrinhoControleDirecaoLoop.ino`
* **Abordagem**: Monitora sensores e variáveis dentro do loop principal do ESP32, fazendo leituras periódicas.
* **Controle**: Implementação de controle PID no `loop()`, ajustando a saída PWM conforme o erro de direção.
* **Fluxo**: A função `loop()` avalia continuamente os sensores, executa o PID e ajusta motores sem uso de interrupções específicas.

### 3. CarrinhoControleDirecaoLoopBluetooth

* **Arquivo**: `CarrinhoControleDirecaoLoopBluetooth.ino`
* **Abordagem**: Extende o exemplo de loop, adicionando comunicação via Bluetooth Low Energy (BLE) ou UART clássico.
* **Controle**: Controle PID ajustado por parâmetros recebidos por Bluetooth, permitindo tuning dinâmico.
* **Fluxo**: Recebe comandos por Bluetooth (via app no smartphone ou terminal serial).

## Componentes de Hardware

* **Placa**: ESP32 Dev Kit
* Driver de motor (L298N)
* Dois motores DC com rodas
* Fonte de alimentação (bateria ou regulador 5V)
* Sensor de linha
* Módulo Bluetooth BLE integrado no ESP32 (apenas para o exemplo Bluetooth)

> **Observação**: Ajuste os pinos no código (`#define`) conforme seu circuito e modelo de ESP32.
