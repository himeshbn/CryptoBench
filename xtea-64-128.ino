/*
  ESP32 XTEA-64-128 Benchmark (Final - Clean Version)
  -----------------------------------------------
  - Official XTEA implementation (32 rounds)
  - Measures key schedule, encryption, decryption times
  - Repeats multiple runs (mean ± SD)
  - Toggles TRIG_PIN for external Arduino UNO + INA219 logger
  - Disables WiFi/Bluetooth for stable power readings
  - Tested with ESP32 Arduino Core v3.x and later
  - Author: 
*/

#include <Arduino.h>
#include "esp_timer.h"
#include "WiFi.h"
#include "BluetoothSerial.h"   // exposes btStop()

// ======= USER CONFIGURATION =======
const uint32_t CPU_FREQ_HZ = 240000000UL; // 240 MHz default
const int TRIG_PIN = 4;                   // GPIO4 → UNO D2 (trigger)
const uint32_t N_RUNS = 20;               // number of repetitions
const uint32_t N_LOOPS = 200000;          // encryption iterations per run (~1–3s window)
const size_t BLOCK_BYTES = 8;             // 64-bit block = 8 bytes
const size_t KEY_BYTES = 16;              // 128-bit key = 16 bytes
// =================================

// ---------- XTEA implementation ----------
typedef struct { uint32_t k[4]; } xtea_key_t;
const uint32_t XTEA_DELTA = 0x9E3779B9;

static inline uint32_t pack_be(const uint8_t *b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}
static inline void unpack_be(uint32_t v, uint8_t *b) {
  b[0] = (v >> 24) & 0xFF;
  b[1] = (v >> 16) & 0xFF;
  b[2] = (v >> 8) & 0xFF;
  b[3] = v & 0xFF;
}

void xtea_expand_key(const uint8_t key[16], xtea_key_t *ek) {
  for (int i = 0; i < 4; i++) ek->k[i] = pack_be(key + 4 * i);
}

void xtea_encrypt_block(const xtea_key_t *ek, const uint8_t in[8], uint8_t out[8]) {
  uint32_t v0 = pack_be(in);
  uint32_t v1 = pack_be(in + 4);
  uint32_t sum = 0;
  for (int i = 0; i < 32; i++) {
    v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + ek->k[sum & 3]);
    sum += XTEA_DELTA;
    v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + ek->k[(sum >> 11) & 3]);
  }
  unpack_be(v0, out);
  unpack_be(v1, out + 4);
}

void xtea_decrypt_block(const xtea_key_t *ek, const uint8_t in[8], uint8_t out[8]) {
  uint32_t v0 = pack_be(in);
  uint32_t v1 = pack_be(in + 4);
  uint32_t sum = XTEA_DELTA * 32;
  for (int i = 0; i < 32; i++) {
    v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + ek->k[(sum >> 11) & 3]);
    sum -= XTEA_DELTA;
    v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + ek->k[sum & 3]);
  }
  unpack_be(v0, out);
  unpack_be(v1, out + 4);
}

// ---------- Test vectors ----------
uint8_t test_key[KEY_BYTES] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};
uint8_t test_plain[BLOCK_BYTES] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };

// ---------- Stats utilities ----------
double ks_times_us[20], enc_times_us[20], dec_times_us[20];

void compute_stats(const double *vals, uint32_t n, double &mean, double &sd) {
  if (n == 0) { mean = sd = 0; return; }
  double sum = 0; for (uint32_t i = 0; i < n; i++) sum += vals[i];
  mean = sum / n;
  if (n == 1) { sd = 0; return; }
  double s = 0; for (uint32_t i = 0; i < n; i++) s += pow(vals[i] - mean, 2);
  sd = sqrt(s / (n - 1));
}

// ---------- Benchmark helpers ----------
inline void trig_high() { digitalWrite(TRIG_PIN, HIGH); }
inline void trig_low()  { digitalWrite(TRIG_PIN, LOW); }

uint64_t bench_key_schedule(uint32_t loops, const uint8_t *key, xtea_key_t *ek) {
  uint64_t t0 = esp_timer_get_time();
  for (uint32_t i = 0; i < loops; i++) xtea_expand_key(key, ek);
  return esp_timer_get_time() - t0;
}
uint64_t bench_encrypt(uint32_t loops, const xtea_key_t *ek, const uint8_t *in, uint8_t *out) {
  uint64_t t0 = esp_timer_get_time();
  for (uint32_t i = 0; i < loops; i++) xtea_encrypt_block(ek, in, out);
  return esp_timer_get_time() - t0;
}
uint64_t bench_decrypt(uint32_t loops, const xtea_key_t *ek, const uint8_t *in, uint8_t *out) {
  uint64_t t0 = esp_timer_get_time();
  for (uint32_t i = 0; i < loops; i++) xtea_decrypt_block(ek, in, out);
  return esp_timer_get_time() - t0;
}

// ---------- Setup ----------
void print_csv_header() {
  Serial.println("ALG,key_bits,block_bits,run,N_loops,ks_us,enc_us,dec_us,throughput_Bps,cycles_per_block");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_OFF);  // disable Wi-Fi
  btStop();             // disable Bluetooth

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  delay(2000);  // warm-up
  Serial.println("# XTEA-64-128 ESP32 benchmark (final)");
  Serial.printf("# CPU: %.2f MHz, N_RUNS=%u, N_LOOPS=%u\n", CPU_FREQ_HZ / 1e6, N_RUNS, N_LOOPS);

  print_csv_header();
}

// ---------- Main loop ----------
void loop() {
  uint8_t cipher[BLOCK_BYTES], recovered[BLOCK_BYTES];
  xtea_key_t ek;

  // Verify algorithm correctness once
  xtea_expand_key(test_key, &ek);
  xtea_encrypt_block(&ek, test_plain, cipher);
  xtea_decrypt_block(&ek, cipher, recovered);
  if (memcmp(test_plain, recovered, BLOCK_BYTES) != 0) {
    Serial.println("# ERROR: XTEA self-test failed!");
    while (true) delay(1000);
  }

  // Run benchmark N_RUNS times
  for (uint32_t run = 0; run < N_RUNS; run++) {
    trig_high();  // Key schedule window
    uint64_t ks_us = bench_key_schedule(N_LOOPS, test_key, &ek);
    trig_low(); ks_times_us[run] = ks_us; delay(50);

    trig_high();  // Encryption window
    uint64_t enc_us = bench_encrypt(N_LOOPS, &ek, test_plain, cipher);
    trig_low(); enc_times_us[run] = enc_us; delay(50);

    trig_high();  // Decryption window
    uint64_t dec_us = bench_decrypt(N_LOOPS, &ek, cipher, recovered);
    trig_low(); dec_times_us[run] = dec_us; delay(200);

    // Per-run calculations
    double enc_s = enc_us / 1e6;
    double throughput = (BLOCK_BYTES * (double)N_LOOPS) / enc_s;
    double cycles_per_block = (CPU_FREQ_HZ * enc_s) / N_LOOPS;

    Serial.printf("XTEA-64-128,128,64,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                  run + 1, N_LOOPS,
                  (double)ks_us, (double)enc_us, (double)dec_us,
                  throughput, cycles_per_block);
    delay(200);
  }

  // Compute and print summary
  double ks_mean, ks_sd, enc_mean, enc_sd, dec_mean, dec_sd;
  compute_stats(ks_times_us, N_RUNS, ks_mean, ks_sd);
  compute_stats(enc_times_us, N_RUNS, enc_mean, enc_sd);
  compute_stats(dec_times_us, N_RUNS, dec_mean, dec_sd);

  double enc_s_mean = enc_mean / 1e6;
  double throughput_mean = (BLOCK_BYTES * N_LOOPS) / enc_s_mean;
  double cycles_mean = (CPU_FREQ_HZ * enc_s_mean) / N_LOOPS;

  Serial.println("# SUMMARY");
  Serial.printf("# KS mean=%.3fus ±%.3f\n", ks_mean, ks_sd);
  Serial.printf("# ENC mean=%.3fus ±%.3f\n", enc_mean, enc_sd);
  Serial.printf("# DEC mean=%.3fus ±%.3f\n", dec_mean, dec_sd);
  Serial.printf("# Throughput=%.3f B/s, Cycles/block=%.3f\n", throughput_mean, cycles_mean);

  Serial.println("ALG,key_bits,block_bits,N_runs,N_loops,enc_mean_us,enc_sd_us,throughput_Bps,cycles_per_block");
  Serial.printf("XTEA-64-128,128,64,%u,%u,%.3f,%.3f,%.3f,%.3f\n",
                N_RUNS, N_LOOPS, enc_mean, enc_sd, throughput_mean, cycles_mean);

  Serial.println("# BENCHMARK COMPLETE - Halting.");
  while (true) delay(1000);
}
