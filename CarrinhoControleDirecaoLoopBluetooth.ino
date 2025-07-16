// Sketch: Leitura Analógica em A0 e impressão na Serial
// Autor: ChatGPT (com humor rápido e sagaz)
// Data: 2025-06-12

const int pinoAnalogico = A0;  // pino analógico 0

void setup() {
  Serial.begin(9600);           // inicia a Serial a 9600 bps
  while (!Serial) {             // espera a conexão (só em placas com USB nativo)
    ; 
  }
  Serial.println("🛰️ Iniciando leitura analógica em A0...");
}

void loop() {
  int valorBruto = analogRead(pinoAnalogico);         // lê valor de 0 a 1023
  float tensao = valorBruto * (5.0 / 1023.0);         // converte pra tensão (0–5V)
  
  // Imprime de forma descolada
  Serial.print("Raw: ");
  Serial.print(valorBruto);
  Serial.print("  |  Volts: ");
  Serial.print(tensao, 3);
  Serial.println(" V 🚀");
  
  delay(500);  // meio segundo de suspense antes da próxima leitura
}
