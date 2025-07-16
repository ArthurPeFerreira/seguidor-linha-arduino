// Bibliotecas
#include <math.h>
#include <QTRSensors.h>
#include <L298N.h>

//  ________________________
// |                        |
// | Esquerda       Direita |
// |_______          _______|
//         |        |
//         |        |
//         |        |
//         |        |
//         |        |
//         |        |
//         |        |
//         |        |
//  _______|        |_______
// |                        |
// | Parte de Tras(Motores) |
// |________________________|

//Motor 1 - Direita, Motor 2 - Esquerda

// Pinos
#define DO_PWM_M1 19  // Pino do ligado ao PWM para controle de Velocidade do Motor 1 - Direita
#define DO_DIR_M1 18  // Pino do ligado ao IN1 para controle de Direção do Motor 1 - Direita
#define DO_PWM_M2 22  // Pino do ligado ao PWM para controle de Velocidade do Motor 2 - Esquerda
#define DO_DIR_M2 23  // Pino do ligado ao IN3 para controle de Direção do Motor 2 - Esquerda

#define DI_SENSOR_ESQUERDA 5  // Pino do ligado ao Sensor a Esquerda do Carrinho
#define DI_SENSOR_DIREITA 33   // Pino do ligado ao Sensor a Direita do Carrinho

#define QUANTIDADE_SENSORES 8  // Quantidade de Sensores do QTRSensor

//Sensores QTR
#define SENSOR_01 27 //Pino Ligado ao sensor mais a Direita do Arduino
#define SENSOR_02 26 
#define SENSOR_03 25 
#define SENSOR_04 32 
#define SENSOR_05 35 
#define SENSOR_06 34 
#define SENSOR_07 39 
#define SENSOR_08 36 //Pino Ligado ao sensor mais a Esquerda do Arduino
#define EMITER_PIN 14

// PID PWM
#define PID_SETPOINT 0    // Setpoint do PID
#define PID_MAX 255       // Máximo da Saída do PID
#define PID_MIN -255         // Mínimo da Saída do PID
#define PID_KP 0.055      // KP do PID
#define PID_KI 0.0        // KI do PID
#define PID_KD 0.55       // KD do PID
#define PID_AMOSTRA 1800  // Tempo em Microsegundos de Amostragem

#define PWM_MEDIO 120    // Velocidade Média do Carrinho
#define GANHO_LINHA 1.0  // Múltiplo do Sensor da Linha

#define VALOR_DETECTA_LINHA 3200            // Valor em que é identificado linha branca
#define QUANTIDADE_SENSORES_CRUZAMENTO 5  // Quantidade de sensores que precisam estar para identificar um cruzamento

// Declaração Motores
L298N M1(DO_PWM_M1, DO_DIR_M1);
L298N M2(DO_PWM_M2, DO_DIR_M2);

// Declaração da parte dos sensores
QTRSensors qtr;
uint16_t ValorSensores[QUANTIDADE_SENSORES];

// Declaração Variáveis Globais
float ErroProporcional, ErroIntegrativo, ErroDerivativo, ErroUltimo, SaidaPID, PwmM1, PwmM2 = 0;
int16_t ErroAtual;
int QuantidadeSensoresLinha;
uint16_t Posicao;
bool SensorEsquerda, SensorDireita, Parar, Cruzamento, InicioResetCruzamento, Inicia = false;
int pwm_medio_atual = 0;
char RecebimentoBluetooth;

// Estados da máquina de estado para parada automática
enum : int { INICIO_ST,
             INICIO_SAIDA_LINHA_ST,
             CORRENDO_ST,
             CHEGADA_ST,
             AVANCAR_ST,
             PARADA_ST } MaquinaEstadoParar = INICIO_ST;

// Função para calcular o PID
void calcula_pid() {
  SensorEsquerda = !digitalRead(DI_SENSOR_ESQUERDA);  // Lê o sensor da esquerda

  Posicao = qtr.readLineWhite(ValorSensores);  // Lê a posição da linha usando os sensores QTR

  // Calcula o erro atual
  ErroAtual = PID_SETPOINT - ((3500 - (int16_t)Posicao) * GANHO_LINHA);

  ErroProporcional = (float)ErroAtual;

  // Acumula o erro integrativo se o sensor da esquerda não estiver detectando linha
  if (!SensorEsquerda) {
    ErroIntegrativo = ErroIntegrativo + (float)ErroAtual;
  } else {
    ErroIntegrativo = 0.0;
  }

  ErroDerivativo = (float)ErroAtual - ErroUltimo;  // Calcula o erro derivativo

  ErroUltimo = (float)ErroAtual;  // Atualiza o erro anterior

  // Calcula a saída do PID
  SaidaPID = ((PID_KP * ErroProporcional) + (PID_KI * ErroIntegrativo) + (PID_KD * ErroDerivativo));

  if (pwm_medio_atual < PWM_MEDIO){
    // pwm_medio_atual = pwm_medio_atual + 1;
    pwm_medio_atual = PWM_MEDIO;
  }

  // Calcula os PWMs dos motores
  PwmM1 = ((float)pwm_medio_atual + SaidaPID);
  PwmM2 = ((float)pwm_medio_atual - SaidaPID);

  // Limita os PWMs dentro dos valores mínimos e máximos
  if (PwmM1 < ((float)PID_MIN)) {
    PwmM1 = ((float)PID_MIN);
  } else if ((PwmM1 > (float)PID_MAX)) {
    PwmM1 = ((float)PID_MAX);
  }

  if (PwmM2 < ((float)PID_MIN)) {
    PwmM2 = ((float)PID_MIN);
  } else if ((PwmM2 > (float)PID_MAX)) {
    PwmM2 = ((float)PID_MAX);
  }
}

// Função para associar a velocidade dos motores com os PWMs calculados
void associa_velocidade_motores() {
  if (PwmM1 > 0) {
    M1.setSpeed((int)PwmM1);
    M1.forward();
  } else if (PwmM1 < 0) {
    PwmM1 = PwmM1 * (-1);
    M1.setSpeed((int)PwmM1);
    M1.backward();
  } else{
    M1.stop();
  }

  if (PwmM2 > 0) {
    M2.setSpeed((int)PwmM2);
    M2.forward();
  } else if (PwmM2 < 0) {
    PwmM2 = PwmM2 * (-1);
    M2.setSpeed((int)PwmM2);
    M2.backward();
  } else{
    M2.stop();
  }
}

// Função para parar automaticamente o robô
void parada_automatica() {
  SensorDireita = (analogRead(DI_SENSOR_DIREITA) < VALOR_DETECTA_LINHA);
  QuantidadeSensoresLinha = 0;

  uint16_t LeituraSensoresIndividual[QUANTIDADE_SENSORES];

  // Lê os valores de refletância de cada sensor
  qtr.read(LeituraSensoresIndividual);

// Conta quantos sensores estão detectando a linha
  for (int i = 0; i < QUANTIDADE_SENSORES; i++) {
    if (LeituraSensoresIndividual[i] < VALOR_DETECTA_LINHA) {
      QuantidadeSensoresLinha = QuantidadeSensoresLinha + 1;
    }
  }

// Detecta cruzamento se muitos sensores estão detectando linha
  if ((QuantidadeSensoresLinha >= QUANTIDADE_SENSORES_CRUZAMENTO) && !Cruzamento) {
    Cruzamento = true;
  }

// Verifica se saiu do cruzamento
  if (Cruzamento && SensorDireita && !InicioResetCruzamento) {
    InicioResetCruzamento = true;
  }

  if (InicioResetCruzamento && !SensorDireita) {
    InicioResetCruzamento = false;
    Cruzamento = false;
  }

// Máquina de estado para controle de parada automática
  //INICIO_ST, INICIO_SAIDA_LINHA_ST, CORRENDO_ST, CHEGADA_ST, AVANCAR_ST, PARADA_ST
  switch (MaquinaEstadoParar) {
    case INICIO_ST:
      if (!Cruzamento && SensorDireita) {
        MaquinaEstadoParar = INICIO_SAIDA_LINHA_ST;
      }
      break;

    case INICIO_SAIDA_LINHA_ST:
      if (!SensorDireita) {
        MaquinaEstadoParar = CORRENDO_ST;
        digitalWrite(LED_BUILTIN, LOW); // Desliga o LED
      }
      break;

    case CORRENDO_ST:
      if (!Cruzamento && SensorDireita) {
        MaquinaEstadoParar = CHEGADA_ST;
      }
      break;

    case CHEGADA_ST:
      if (!SensorDireita) {
        MaquinaEstadoParar = AVANCAR_ST;
      }
      break;

    case AVANCAR_ST:
      Parar = true;

      // Define a velocidade média para os motores
      M1.setSpeed(PWM_MEDIO);
      M1.forward();
      M2.setSpeed(PWM_MEDIO);
      M2.forward();
      MaquinaEstadoParar = PARADA_ST;
      delay(500);
      break;

    case PARADA_ST:
    // Para os motores
      M1.setSpeed(0);
      M1.stop();
      M2.setSpeed(0);
      M2.stop();
      Inicia = false;
      break;
  }
}
// Função que calcula o controle dos motores
void calcula_controle_motores() {
if (!Parar) {
calcula_pid(); // Calcula o PID
associa_velocidade_motores(); // Associa a velocidade dos motores
}
parada_automatica(); // Verifica a parada automática
}

// Início do Programa
void setup() {
// Inicia Serial
Serial.begin(115200);

pinMode(DI_SENSOR_ESQUERDA, INPUT); // Configuração como Input do Pino do ligado ao Sensor a Esquerda do Carrinho
pinMode(DI_SENSOR_DIREITA, INPUT); // Configuração como Input do Pino do ligado ao Sensor a Direita do Carrinho

pinMode(LED_BUILTIN, OUTPUT); // Configuração como Output do Led interno do

qtr.setTypeAnalog();
qtr.setSensorPins((const uint8_t[]){ SENSOR_01, SENSOR_02, SENSOR_03, SENSOR_04, SENSOR_05, SENSOR_06, SENSOR_07, SENSOR_08 }, QUANTIDADE_SENSORES);
qtr.setEmitterPin(EMITER_PIN);

// Lógica de Calibração dos Sensores
digitalWrite(LED_BUILTIN, HIGH); // Liga o LED para mostrar que está calibrando

// Calibra o Sensor
for (uint16_t i = 0; i < 500; i++) {
qtr.calibrate();
}

// Lógica para Avisar que vai começar
digitalWrite(LED_BUILTIN, LOW);
delay(500);
digitalWrite(LED_BUILTIN, HIGH);
delay(500);
digitalWrite(LED_BUILTIN, LOW);
delay(500);
digitalWrite(LED_BUILTIN, HIGH);
delay(500);
digitalWrite(LED_BUILTIN, LOW);
delay(500);
digitalWrite(LED_BUILTIN, HIGH);
delay(500);

Parar = false; // Inicializa a variável Parar como falsa
}

void loop() {
  calcula_controle_motores();
}