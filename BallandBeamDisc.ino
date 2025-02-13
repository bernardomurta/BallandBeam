#include <Servo.h>
#include <HCSR04.h>

// Definição dos pinos
#define TRIGGER  11
#define ECHO     12
#define SERVO_PIN 9
#define START_PIN 6   // Pino de ativação com pull-up interno

// Configuração do sensor ultrassônico
UltraSonicDistanceSensor distanceSensor(TRIGGER, ECHO);
Servo myServo;

// Parâmetros do sistema
const float Ts = 0.01;  // Tempo de amostragem (10ms)
const float Kp = 3;   // Ganho proporcional
const float Ki = 0;   // Ganho integral
const float Kd = 1;   // Ganho derivativo

// Limites do sistema
const int setpoint = 15;   // Setpoint da posição da bola (cm)
const int neg = -30;       // Novo limite negativo (corrigido para ângulo realista)
const int pos = 50;        // Novo limite positivo
const int base = 95;       // Posição neutra do servo ajustada para MG995

// Variáveis do PID
float erro = 0, erro_anterior = 0;
float integral = 0;
float derivativo = 0;
float controle = 0;
bool sistema_ativado = false;

// Função para medir a posição da bola (distância em cm)
float readPosition() {
    int distancia = distanceSensor.measureDistanceCm();
    if (distancia < 2 || distancia > 30) return 15;  // Ignora leituras inválidas, assume valor médio
    return distancia;
}

// Função de controle PID discretizado (Backward Euler)
float controlePID(float erro) {
    integral += Ki * Ts * erro;
    if (integral > pos) integral = pos; // Impede que a integral cresça sem controle, o que poderia levar a uma saída exagerada.
    if (integral < neg) integral = neg;
    
    derivativo = (erro - erro_anterior) / Ts;
    float saida = -(Kp * erro + integral + Kd * derivativo);
    
    // Limitação da saída
    if (saida > pos) saida = pos;
    if (saida < neg) saida = neg;

    erro_anterior = erro;
    return saida;
}

void setup() {
    Serial.begin(9600);
    myServo.attach(SERVO_PIN);
    pinMode(START_PIN, INPUT_PULLUP);
    myServo.write(base); // Inicia servo na posição neutra
}

void loop() {
    static bool botao_pressionado = true;

    if (digitalRead(START_PIN) == LOW && !botao_pressionado) {
        sistema_ativado = !sistema_ativado;
        botao_pressionado = true;
        delay(200); // Debounce para evitar múltiplas leituras
    }

    if (digitalRead(START_PIN) == HIGH) {
        botao_pressionado = false;
    }

    if (!sistema_ativado) {
        Serial.println("Sistema aguardando ativação...");
        delay(100);
        return;
    }

    float posicao = readPosition();
    erro = setpoint - posicao;
    controle = controlePID(erro);

    // Ajusta ângulo do servo dentro de uma faixa segura
    int anguloServo = base + controle;
    if (anguloServo > 180) anguloServo = 180;
    if (anguloServo < 0) anguloServo = 0;

    myServo.write(anguloServo);

    Serial.print("Posição: "); Serial.print(posicao); Serial.println(" cm");
    
    delay(10);
}

