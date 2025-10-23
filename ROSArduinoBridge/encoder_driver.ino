#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER

volatile long left_enc_pos = 0;
volatile long right_enc_pos = 0;

// estados anteriores para decodificação de quadratura
volatile int left_last_A = 0;
volatile int right_last_A = 0;

// pinos dos encoders (mantendo os seus)
#define LEFT_ENC_PIN_A 3
#define LEFT_ENC_PIN_B 18
#define RIGHT_ENC_PIN_A 2
#define RIGHT_ENC_PIN_B 19

// --- Funções de interrupção ---

void leftEncoderA() {
  int A = digitalRead(LEFT_ENC_PIN_A);
  int B = digitalRead(LEFT_ENC_PIN_B);
  if (A == B)
    left_enc_pos++;
  else
    left_enc_pos--;
}

void rightEncoderA() {
  int A = digitalRead(RIGHT_ENC_PIN_A);
  int B = digitalRead(RIGHT_ENC_PIN_B);
  if (A == B)
    right_enc_pos++;
  else
    right_enc_pos--;
}

// --- Inicialização dos pinos e interrupções ---
void setupEncoders() {
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  // interrupções externas — Mega2560
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderA, CHANGE);
}

// --- Leitura e reset ---
long readEncoder(int i) {
  if (i == LEFT)
    return left_enc_pos;
  else
    return -right_enc_pos;
}

void resetEncoder(int i) {
  if (i == LEFT)
    left_enc_pos = 0;
  else
    right_enc_pos = 0;
}

void resetEncoders() {
  left_enc_pos = 0;
  right_enc_pos = 0;
}

#endif
#endif
