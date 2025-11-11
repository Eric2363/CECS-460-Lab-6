/*
  CECS 460 – Lab 6: Packet Verification + LUT-Based BPSK Receiver
  Student B – Receiver (BPSK RX)

  Requirements Added:
    • Detect SYNC (0xAA)
    • Read LENGTH
    • Store DATA bytes
    • Compute XOR checksum
    • If valid → Green LED + UART "OK"
    • If invalid → Red LED + UART "ERR"
*/

#include <Arduino.h>

// ------------------------------------------------------
// Pin Configuration
// ------------------------------------------------------
#define RX_PIN          26      // D26 (your Lab 5 setting)
#define LED_OK           2      // Green LED  (change if needed)
#define LED_ERR          4      // Red LED    (change if needed)

// ------------------------------------------------------
// Packet Format
// ------------------------------------------------------
#define SYNC_BYTE     0xAA
#define MAX_PAYLOAD      8      // Lab limit
uint8_t frameBuf[16];
uint8_t frameCnt = 0;

// ------------------------------------------------------
// BPSK Parameters (same as lab 5)
// ------------------------------------------------------
#define BIT_RATE_HZ     1000
#define SAMPLES_PER_BIT 64
#define SAMPLE_RATE_HZ  (BIT_RATE_HZ * SAMPLES_PER_BIT)
#define UART_BAUD       115200

float ref_cos[SAMPLES_PER_BIT];
float ref_negcos[SAMPLES_PER_BIT];

// Sampling state
volatile int sampleBuffer[SAMPLES_PER_BIT];
volatile int sampleIndex = 0;
volatile bool bufferReady = false;

// Timer ISR
hw_timer_t *samplingTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// ------------------------------------------------------
// XOR checksum
// ------------------------------------------------------
uint8_t xor_checksum(const uint8_t *d, uint8_t n){
  uint8_t c = 0;
  for(uint8_t i = 0; i < n; i++){
    c ^= d[i];
  }
  return c;
}


// ------------------------------------------------------
// Setup LUT with cosine reference waveforms
// ------------------------------------------------------
void setupLUT() {
  for (int i = 0; i < SAMPLES_PER_BIT; i++) {
    float angle = 2.0 * PI * i / SAMPLES_PER_BIT;
    ref_cos[i] = cos(angle);
    ref_negcos[i] = -ref_cos[i];
  }
}


// ------------------------------------------------------
// Timer ISR: Sample the input pin at 64 kHz
// ------------------------------------------------------
void IRAM_ATTR onSampleTimer() {
  int sample = digitalRead(RX_PIN);
  sampleBuffer[sampleIndex] = (sample == HIGH) ? 1 : -1;
  sampleIndex++;

  if (sampleIndex >= SAMPLES_PER_BIT) {
    bufferReady = true;
    sampleIndex = 0;
  }
}


// ------------------------------------------------------
// Correlate sampled waveform with reference LUTs
// ------------------------------------------------------
int detectPhase(volatile int samples[], int len) {
  float corr_pos = 0.0;
  float corr_neg = 0.0;

  for (int i = 0; i < len; i++) {
    corr_pos += samples[i] * ref_cos[i];
    corr_neg += samples[i] * ref_negcos[i];
  }

  return (corr_pos > corr_neg) ? 1 : 0;
}


// ------------------------------------------------------
// SETUP
// ------------------------------------------------------
void setup() {
  Serial.begin(UART_BAUD);
  delay(500);

  pinMode(RX_PIN, INPUT);
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_ERR, OUTPUT);
  digitalWrite(LED_OK, LOW);
  digitalWrite(LED_ERR, LOW);

  setupLUT();

  Serial.println("\n=== BPSK Receiver Ready (Lab 6) ===");
  Serial.println("Waiting for packets...\n");

  samplingTimer = timerBegin(SAMPLE_RATE_HZ);
  timerAttachInterrupt(samplingTimer, &onSampleTimer);
  timerStart(samplingTimer);
}


// ------------------------------------------------------
// Add received byte into packet parser
// ------------------------------------------------------
void processReceivedByte(uint8_t b)
{
  frameBuf[frameCnt++] = b;

  // Too many bytes? Reset
  if(frameCnt > 15){
    frameCnt = 0;
  }

  // Not enough bytes to parse yet
  if(frameCnt < 3) return;

  // SYNC not found -> reset
  if(frameBuf[0] != SYNC_BYTE){
    frameCnt = 0;
    return;
  }

  // Read length
  uint8_t L = frameBuf[1];
  if(L > MAX_PAYLOAD){
    frameCnt = 0;
    return;
  }

  // Full frame received?
  if(frameCnt == (uint8_t)(3 + L)){
    uint8_t chk_rx  = frameBuf[2 + L];
    uint8_t chk_cal = xor_checksum(&frameBuf[2], L);

    if(chk_rx == chk_cal){
      // ---------------- VALID FRAME ----------------
      digitalWrite(LED_OK, HIGH);
      delay(10);
      digitalWrite(LED_OK, LOW);

      Serial.print("OK  LEN=");
      Serial.print(L);
      Serial.print("  DATA: ");
      for(uint8_t i=0;i<L;i++){
        Serial.print(frameBuf[2+i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    else {
      // ---------------- INVALID FRAME ----------------
      digitalWrite(LED_ERR, HIGH);
      delay(10);
      digitalWrite(LED_ERR, LOW);

      Serial.println("ERR (Checksum)");
    }

    // Reset for next packet
    frameCnt = 0;
  }
}


// ------------------------------------------------------
// LOOP: same as Lab 5 but now feeding bytes into parser
// ------------------------------------------------------
void loop() {
  static uint8_t bitIndex = 0;
  static uint8_t byteBuffer = 0;
  static int lastPhase = -1;

  if (bufferReady) {
    portENTER_CRITICAL(&timerMux);
    int localBuffer[SAMPLES_PER_BIT];
    for (int i = 0; i < SAMPLES_PER_BIT; i++) {
      localBuffer[i] = sampleBuffer[i];
    }
    bufferReady = false;
    portEXIT_CRITICAL(&timerMux);

    int currentPhase = detectPhase(localBuffer, SAMPLES_PER_BIT);

    int bit;
    if (lastPhase == -1) {
      bit = 0;
      lastPhase = currentPhase;
    } else {
      bit = (currentPhase != lastPhase) ? 1 : 0;
      lastPhase = currentPhase;
    }

    // Build byte (MSB first)
    byteBuffer = (byteBuffer << 1) | (bit & 0x01);
    bitIndex++;

    // Complete byte
    if (bitIndex == 8) {
      processReceivedByte(byteBuffer);    // <-- Lab 6 upgrade here

      bitIndex = 0;
      byteBuffer = 0;
    }
  }

  delayMicroseconds(100);
}
