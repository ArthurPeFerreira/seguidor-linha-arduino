# Projeto Carrinho Controle de Direção (ESP32)

Este repositório contém três versões de um projeto de controle de direção para um carrinho segue linha baseado em **ESP32**, com implementação de **controle PID** para ajuste fino da direção. 

## Estrutura de Pastas

```
└── CarrinhoControleDirecaoLoop.ino
```

* **Arquivo**: `CarrinhoControleDirecaoLoop.ino`
* **Abordagem**: Monitora sensores e variáveis dentro do loop principal do ESP32, fazendo leituras periódicas.
* **Controle**: Implementação de controle PID no `loop()`, ajustando a saída PWM conforme o erro de direção.
* **Fluxo**: A função `loop()` avalia continuamente os sensores, executa o PID e ajusta motores sem uso de interrupções específicas.

## Componentes de Hardware

* **Placa**: ESP32 Dev Kit
* Driver de motor (L298N)
* Dois motores DC com rodas
* Fonte de alimentação (bateria ou regulador 5V)
* Sensor de linha
* Módulo Bluetooth BLE integrado no ESP32 (apenas para o exemplo Bluetooth)

> **Observação**: Ajuste os pinos no código (`#define`) conforme seu circuito e modelo de ESP32.
