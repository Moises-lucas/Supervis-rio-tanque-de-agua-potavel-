/* =============================================================================
 *  FIRMWARE ESP32 — ETA (Estação de Tratamento de Água)
 *  -----------------------------------------------------------------------------
 *  Objetivo:
 *    Interface física entre o supervisório Python (rodando no PC) e o processo
 *    real da ETA. O ESP32 lê os sensores, aciona relés/válvulas e troca dados
 *    com o PC por comunicação SERIAL USB (115200 baud), usando JSON por linha.
 *
 *  Comunicação (Serial USB — cabo direto PC <-> ESP32):
 *    - Baud rate: 115200
 *    - Cada mensagem é uma linha JSON terminada em '\n'
 *
 *    PC -> ESP32 (comandos, um por linha):
 *      {"cmd":"MOTOR","val":1}     -> liga(1)/desliga(0) a bomba B-01
 *      {"cmd":"V01","val":1}       -> abre(1)/fecha(0) a válvula de entrada V-01
 *      {"cmd":"VDRAIN","val":1}    -> abre(1)/fecha(0) o dreno V-DRAIN
 *      {"cmd":"VCP","val":75}      -> abertura da válvula proporcional (0-100%)
 *      {"cmd":"SP","val":50}       -> setpoint de nível (0-100%) - informativo
 *      {"cmd":"PING","val":0}      -> teste de conexão (responde PONG)
 *
 *    ESP32 -> PC (enviado a cada 200 ms):
 *      {"type":"DATA","lit":45.2,"fit1":12.0,"fit2":4.0,"motor":1,"v01":1,
 *       "vdrain":0,"vcp":75,"alarm":0,"uptime":12345}
 *      {"type":"LOG","msg":"Bomba B-01 ligada","level":"ok"}
 *      {"type":"PONG","uptime":12345}
 *
 *  -----------------------------------------------------------------------------
 *  BIBLIOTECA NECESSÁRIA: ArduinoJson (por Benoit Blanchon)
 *    Instalação na Arduino IDE:
 *      Sketch -> Include Library -> Manage Libraries...
 *      Pesquise por "ArduinoJson" e instale a versão 6.x (ou 7.x).
 *    Este código usa a API da versão 6/7 (StaticJsonDocument / JsonDocument).
 *
 *  PLACA: selecione "ESP32 Dev Module" (ou equivalente) em Ferramentas -> Placa.
 * ===========================================================================*/

#include <ArduinoJson.h>

// ─── MAPEAMENTO DE PINOS ─────────────────────────────────────────────────────
// Saídas digitais (relés). Se seu módulo de relé for ATIVO EM NÍVEL BAIXO
// (a maioria dos módulos de 1/2/4 canais chineses é assim), troque
// RELAY_ACTIVE_HIGH para 'false' abaixo — a lógica é invertida automaticamente.
const int PIN_MOTOR  = 26;   // Relé da Bomba B-01 (motor de recalque)
const int PIN_V01    = 27;   // Relé da Válvula V-01 (entrada de água)
const int PIN_VDRAIN = 14;   // Relé do Dreno V-DRAIN

// Saída analógica (DAC nativo do ESP32) para a válvula proporcional VCP-01.
// O ESP32 tem 2 canais de DAC: GPIO25 (DAC1) e GPIO26 (DAC2).
// Aqui usamos GPIO25. Faixa do DAC: 0-255 (0 V a ~3.3 V).
// Se o atuador da válvula exigir 0-10 V ou 4-20 mA, use um conversor/driver
// externo (ex.: amplificador operacional, módulo 0-10V, ou saída 4-20mA).
const int PIN_VCP    = 25;   // DAC1 -> VCP-01 (0-100% mapeado para 0-255)

// Entrada analógica (ADC) para o sensor de nível LIT-01.
// >>> TIPO ESCOLHIDO PELO USUÁRIO: BOIA COM POTENCIÔMETRO (0-3.3 V) <<<
// O cursor do potenciômetro fornece uma tensão proporcional ao ângulo/altura
// da boia. Ligue: extremos do potenciômetro em 3.3V e GND, cursor no GPIO34.
// ADC do ESP32: 12 bits -> 0 a 4095 (0 V a ~3.3 V).
// IMPORTANTE: use pinos ADC1 (GPIO32..39) — o ADC2 não funciona com Wi-Fi ligado.
const int PIN_LIT    = 34;   // ADC1_CH6 -> LIT-01 (boia + potenciômetro)

// Entradas de pulso (sensores de vazão tipo turbina/hall) via interrupção.
const int PIN_FIT1   = 35;   // FIT-01 (vazão de entrada) — contador de pulsos
const int PIN_FIT2   = 32;   // FIT-02 (vazão de saída)   — contador de pulsos

// LED de status (LED onboard da maioria das placas ESP32 DevKit).
const int PIN_LED    = 2;    // Pisca 1 Hz quando em comunicação; fixo se offline

// ─── CONFIGURAÇÕES DE LÓGICA ─────────────────────────────────────────────────
const bool RELAY_ACTIVE_HIGH = true;   // true = relé liga com HIGH; false = com LOW

// Watchdog de comunicação: se o PC não enviar NENHUM comando por este tempo,
// o ESP32 entra em modo seguro (desliga bomba e fecha válvulas).
const unsigned long COMM_TIMEOUT_MS = 10000;   // 10 segundos

// Período de envio dos dados (telemetria) para o PC.
const unsigned long DATA_PERIOD_MS  = 200;     // 200 ms

// ─── CALIBRAÇÃO DOS SENSORES DE VAZÃO ────────────────────────────────────────
// Fator K (pulsos por litro) do sensor de vazão. Consulte o datasheet do seu
// sensor. Ex.: YF-S201 ~ 450 pulsos/litro. Ajuste conforme o modelo real.
const float FLOW_PULSES_PER_LITER = 450.0;

// ─── VARIÁVEIS DE ESTADO ─────────────────────────────────────────────────────
volatile unsigned long fit1_pulses = 0;   // contador de pulsos FIT-01 (ISR)
volatile unsigned long fit2_pulses = 0;   // contador de pulsos FIT-02 (ISR)

bool  motor_on  = false;    // estado atual da bomba B-01
bool  v01_open  = false;    // estado atual da válvula de entrada
bool  vdrain_open = false;  // estado atual do dreno
int   vcp_pct   = 0;        // abertura da VCP-01 (0-100%)
int   setpoint  = 50;       // setpoint recebido (informativo)
bool  alarmFlag = false;    // flag de alarme (nível crítico / modo seguro)

float lit_level = 0.0;      // nível filtrado do LIT-01 (%)
float flow1     = 0.0;      // vazão FIT-01 (m³/h)
float flow2     = 0.0;      // vazão FIT-02 (m³/h)

unsigned long last_data_ms   = 0;   // último envio de telemetria
unsigned long last_cmd_ms    = 0;   // último comando recebido do PC
unsigned long last_flow_ms   = 0;   // última janela de cálculo de vazão
unsigned long last_led_ms    = 0;   // controle de piscar do LED
bool  led_state  = false;
bool  comm_alive = false;   // true enquanto recebemos comandos dentro do timeout

// ─── ROTINAS DE INTERRUPÇÃO (ISR) — contagem de pulsos ───────────────────────
// IRAM_ATTR: a ISR fica na RAM interna (obrigatório no ESP32 para ISRs rápidas).
void IRAM_ATTR isr_fit1() { fit1_pulses++; }
void IRAM_ATTR isr_fit2() { fit2_pulses++; }

// ─── AUXILIARES DE RELÉ ──────────────────────────────────────────────────────
// Escreve no relé respeitando a lógica ativa em alto/baixo.
void writeRelay(int pin, bool on) {
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(pin, level ? HIGH : LOW);
}

// ─── ENVIO DE LOG PARA O PC ──────────────────────────────────────────────────
void sendLog(const char* msg, const char* level) {
  StaticJsonDocument<128> doc;
  doc["type"]  = "LOG";
  doc["msg"]   = msg;
  doc["level"] = level;
  serializeJson(doc, Serial);
  Serial.println();
}

// ─── LEITURA DO SENSOR DE NÍVEL LIT-01 ───────────────────────────────────────
// Boia com potenciômetro: lê o ADC e converte para 0-100%.
// Faz média de 10 amostras para filtrar ruído elétrico.
float readLevel() {
  const int N = 10;
  long acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(PIN_LIT);   // 0..4095 (12 bits)
    delayMicroseconds(200);       // pequeno intervalo entre amostras
  }
  float raw = acc / (float)N;     // média das amostras

  // Converte 0..4095 -> 0..100%.
  // Ajuste ADC_MIN/ADC_MAX após calibrar com a boia no fundo e no topo do tanque.
  const float ADC_MIN = 0.0;      // leitura com tanque vazio (calibrar)
  const float ADC_MAX = 4095.0;   // leitura com tanque cheio (calibrar)
  float pct = (raw - ADC_MIN) * 100.0 / (ADC_MAX - ADC_MIN);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;

  /* -----------------------------------------------------------------------
   * ALTERNATIVA 1 — SENSOR ULTRASSÔNICO (HC-SR04):
   *   Meça a distância até a lâmina d'água e converta para nível.
   *   Requer 2 pinos (TRIG e ECHO) em vez do ADC.
   *
   *   long duration; float distance_cm;
   *   digitalWrite(TRIG, LOW);  delayMicroseconds(2);
   *   digitalWrite(TRIG, HIGH); delayMicroseconds(10);
   *   digitalWrite(TRIG, LOW);
   *   duration = pulseIn(ECHO, HIGH, 30000);   // timeout 30 ms
   *   distance_cm = duration * 0.0343 / 2.0;   // velocidade do som
   *   // nível = altura_tanque - distância_medida
   *   float nivel_cm = ALTURA_TANQUE_CM - distance_cm;
   *   float pct = nivel_cm * 100.0 / ALTURA_TANQUE_CM;
   *
   * ALTERNATIVA 2 — SENSOR DE PRESSÃO / TRANSMISSOR 4-20 mA:
   *   Use um resistor shunt de 165 Ω para converter 4-20 mA em ~0.66-3.3 V,
   *   leia no ADC e mapeie:
   *     4 mA  (tanque vazio) -> ~0.66 V -> ADC ~820
   *     20 mA (tanque cheio) -> ~3.30 V -> ADC ~4095
   *   float pct = (raw - 820.0) * 100.0 / (4095.0 - 820.0);
   * --------------------------------------------------------------------- */
}

// ─── CÁLCULO DAS VAZÕES (a partir dos pulsos acumulados) ─────────────────────
void updateFlows(unsigned long dt_ms) {
  if (dt_ms == 0) return;

  // Captura e zera os contadores de forma atômica (desabilita interrupções).
  noInterrupts();
  unsigned long p1 = fit1_pulses; fit1_pulses = 0;
  unsigned long p2 = fit2_pulses; fit2_pulses = 0;
  interrupts();

  float dt_s = dt_ms / 1000.0;

  // litros no intervalo = pulsos / (pulsos por litro)
  float liters1 = p1 / FLOW_PULSES_PER_LITER;
  float liters2 = p2 / FLOW_PULSES_PER_LITER;

  // vazão (L/s) -> m³/h : (L/s) * 3.6
  flow1 = (liters1 / dt_s) * 3.6;
  flow2 = (liters2 / dt_s) * 3.6;
}

// ─── ENVIO DA TELEMETRIA (DATA) ──────────────────────────────────────────────
void sendData() {
  StaticJsonDocument<256> doc;
  doc["type"]   = "DATA";
  doc["lit"]    = round(lit_level * 10) / 10.0;   // 1 casa decimal
  doc["fit1"]   = round(flow1 * 10) / 10.0;
  doc["fit2"]   = round(flow2 * 10) / 10.0;
  doc["motor"]  = motor_on ? 1 : 0;
  doc["v01"]    = v01_open ? 1 : 0;
  doc["vdrain"] = vdrain_open ? 1 : 0;
  doc["vcp"]    = vcp_pct;
  doc["alarm"]  = alarmFlag ? 1 : 0;
  doc["uptime"] = millis();
  serializeJson(doc, Serial);
  Serial.println();
}

// ─── APLICA A ABERTURA DA VÁLVULA PROPORCIONAL VCP-01 (via DAC) ──────────────
void applyVCP(int pct) {
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  vcp_pct = pct;
  int dacValue = map(pct, 0, 100, 0, 255);   // 0-100% -> 0-255 (DAC)
  dacWrite(PIN_VCP, dacValue);
}

// ─── MODO SEGURO (acionado pelo watchdog) ────────────────────────────────────
// Desliga a bomba e fecha as válvulas para proteger o processo quando a
// comunicação com o PC é perdida.
void enterSafeMode() {
  motor_on    = false;
  v01_open    = false;
  vdrain_open = false;
  applyVCP(0);
  writeRelay(PIN_MOTOR,  false);
  writeRelay(PIN_V01,    false);
  writeRelay(PIN_VDRAIN, false);
  alarmFlag = true;
}

// ─── PROCESSA UM COMANDO JSON RECEBIDO DO PC ─────────────────────────────────
void handleCommand(const String& line) {
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    sendLog("Comando JSON invalido recebido", "warn");
    return;
  }

  const char* cmd = doc["cmd"] | "";
  int val = doc["val"] | 0;

  // Qualquer comando válido "alimenta" o watchdog (comunicação viva).
  last_cmd_ms = millis();

  if (strcmp(cmd, "MOTOR") == 0) {
    motor_on = (val != 0);
    writeRelay(PIN_MOTOR, motor_on);
    sendLog(motor_on ? "Bomba B-01 ligada" : "Bomba B-01 desligada",
            motor_on ? "ok" : "warn");
  }
  else if (strcmp(cmd, "V01") == 0) {
    v01_open = (val != 0);
    writeRelay(PIN_V01, v01_open);
    sendLog(v01_open ? "Valvula V-01 aberta" : "Valvula V-01 fechada", "info");
  }
  else if (strcmp(cmd, "VDRAIN") == 0) {
    vdrain_open = (val != 0);
    writeRelay(PIN_VDRAIN, vdrain_open);
    sendLog(vdrain_open ? "Dreno V-DRAIN aberto" : "Dreno V-DRAIN fechado", "warn");
  }
  else if (strcmp(cmd, "VCP") == 0) {
    applyVCP(val);
    sendLog("VCP-01 ajustada", "info");
  }
  else if (strcmp(cmd, "SP") == 0) {
    setpoint = val;   // informativo (o controle de nível fica no supervisório)
  }
  else if (strcmp(cmd, "PING") == 0) {
    StaticJsonDocument<64> pong;
    pong["type"]   = "PONG";
    pong["uptime"] = millis();
    serializeJson(pong, Serial);
    Serial.println();
  }
  else {
    sendLog("Comando desconhecido", "warn");
  }
}

// ─── LEITURA DA SERIAL (comandos linha a linha) ──────────────────────────────
String rxBuffer = "";
void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rxBuffer.trim();
      if (rxBuffer.length() > 0) handleCommand(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 200) rxBuffer = "";   // proteção contra lixo
    }
  }
}

// ─── ATUALIZA O LED DE STATUS ────────────────────────────────────────────────
// Pisca a 1 Hz quando a comunicação está ativa; fica aceso fixo quando offline.
void updateLed() {
  if (comm_alive) {
    if (millis() - last_led_ms >= 500) {   // 500 ms on / 500 ms off = 1 Hz
      last_led_ms = millis();
      led_state = !led_state;
      digitalWrite(PIN_LED, led_state ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_LED, HIGH);   // aceso fixo = sem comunicação
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Configura saídas digitais (relés) e garante estado seguro (desligado).
  pinMode(PIN_MOTOR,  OUTPUT);
  pinMode(PIN_V01,    OUTPUT);
  pinMode(PIN_VDRAIN, OUTPUT);
  writeRelay(PIN_MOTOR,  false);
  writeRelay(PIN_V01,    false);
  writeRelay(PIN_VDRAIN, false);

  // LED de status.
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);   // começa aceso (sem comunicação ainda)

  // Entrada analógica do nível (boia + potenciômetro).
  // Resolução padrão do ADC do ESP32 = 12 bits (0..4095).
  analogReadResolution(12);
  // Atenuação 11 dB permite ler a faixa completa ~0..3.3 V.
  analogSetPinAttenuation(PIN_LIT, ADC_11db);

  // Entradas de pulso das vazões, com interrupção na borda de subida.
  pinMode(PIN_FIT1, INPUT_PULLUP);
  pinMode(PIN_FIT2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FIT1), isr_fit1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FIT2), isr_fit2, RISING);

  // Válvula proporcional em 0% ao ligar.
  applyVCP(0);

  unsigned long now = millis();
  last_data_ms = now;
  last_cmd_ms  = now;
  last_flow_ms = now;
  last_led_ms  = now;

  sendLog("ESP32 ETA inicializado", "info");
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  // 1) Recebe e processa comandos do PC.
  readSerial();

  unsigned long now = millis();

  // 2) Watchdog de comunicação.
  //    Se passou muito tempo sem comando, entra em modo seguro.
  bool prev_alive = comm_alive;
  comm_alive = (now - last_cmd_ms) < COMM_TIMEOUT_MS;
  if (!comm_alive && prev_alive) {
    // Acabou de perder a comunicação -> modo seguro.
    enterSafeMode();
    sendLog("SEM COMUNICACAO com o PC - modo seguro acionado", "alarm");
  }
  if (comm_alive && !prev_alive) {
    alarmFlag = false;   // comunicação restabelecida
    sendLog("Comunicacao restabelecida", "ok");
  }

  // 3) Telemetria a cada DATA_PERIOD_MS.
  if (now - last_data_ms >= DATA_PERIOD_MS) {
    unsigned long dt = now - last_flow_ms;
    updateFlows(dt);          // calcula vazões desde a última janela
    last_flow_ms = now;

    lit_level = readLevel();  // lê o nível filtrado

    sendData();               // envia DATA ao PC
    last_data_ms = now;
  }

  // 4) LED de status.
  updateLed();
}
