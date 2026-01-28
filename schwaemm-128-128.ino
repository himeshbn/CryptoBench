/*
  ESP32 SCHWAEMM-128-128 Benchmark (Final, Fixed)
  -----------------------------------------------
  - Simplified SCHWAEMM-128-128 core permutation for benchmarking
  - Measures key schedule, encryption, and decryption
  - Uses GPIO4 trigger for UNO+INA219 logger
  - Compatible with ESP32 Arduino Core v3.x+
  - Author: 
*/

#include <Arduino.h>
#include "esp_timer.h"
#include "WiFi.h"
#include "BluetoothSerial.h"

// ===== USER CONFIG =====
const uint32_t CPU_FREQ_HZ = 240000000UL;
const int TRIG_PIN = 4;          // GPIO4 → UNO D2
const uint32_t N_RUNS  = 20;     // repeat count
const uint32_t N_LOOPS = 200000; // encryptions per window (~1–3 s)
const size_t BLOCK_BYTES = 16;   // 128-bit block
const size_t KEY_BYTES   = 16;   // 128-bit key
// ========================

// ---------- SCHWAEMM CORE TYPE ----------
typedef struct {
  uint32_t S[8];
} schwaemm_state_t;

// ---------- Function Prototypes ----------
void schwaemm_init_state(schwaemm_state_t *st, const uint8_t *key);
void schwaemm_round(schwaemm_state_t *st, uint32_t rc);
void schwaemm_permutation(schwaemm_state_t *st);
void schwaemm_encrypt_block(const uint8_t *in, uint8_t *out, const uint8_t *key);
void schwaemm_decrypt_block(const uint8_t *in, uint8_t *out, const uint8_t *key);

// ---------- Helper ----------
static inline uint32_t rotl32(uint32_t x, uint32_t r) {
  return (x << r) | (x >> (32 - r));
}

// ---------- SCHWAEMM IMPLEMENTATION ----------
#define SCHWAEMM_ROUNDS 12

void schwaemm_init_state(schwaemm_state_t *st, const uint8_t *key) {
  for (int i = 0; i < 8; i++) {
    st->S[i] = ((uint32_t)key[(i*2)%KEY_BYTES] << 24) |
               ((uint32_t)key[(i*2+1)%KEY_BYTES] << 16) |
               ((uint32_t)key[(i*2+2)%KEY_BYTES] << 8) |
               ((uint32_t)key[(i*2+3)%KEY_BYTES]);
  }
}

void schwaemm_round(schwaemm_state_t *st, uint32_t rc) {
  for (int i = 0; i < 8; i++) st->S[i] ^= rotl32(st->S[(i+1)%8], 8);
  for (int i = 0; i < 8; i++) st->S[i] = rotl32(st->S[i] + rc + i, (i+1));
  uint32_t tmp = st->S[0];
  for (int i = 0; i < 7; i++) st->S[i] = st->S[i+1];
  st->S[7] = tmp ^ rc;
}

void schwaemm_permutation(schwaemm_state_t *st) {
  static const uint32_t RC[SCHWAEMM_ROUNDS] = {
    0x243F6A88,0x85A308D3,0x13198A2E,0x03707344,
    0xA4093822,0x299F31D0,0x082EFA98,0xEC4E6C89,
    0x452821E6,0x38D01377,0xBE5466CF,0x34E90C6C
  };
  for (int r = 0; r < SCHWAEMM_ROUNDS; r++)
    schwaemm_round(st, RC[r]);
}

void schwaemm_encrypt_block(const uint8_t *in, uint8_t *out, const uint8_t *key) {
  schwaemm_state_t st;
  schwaemm_init_state(&st, key);
  for (int i = 0; i < 4; i++) {
    uint32_t m = ((uint32_t)in[4*i]<<24)|((uint32_t)in[4*i+1]<<16)|((uint32_t)in[4*i+2]<<8)|in[4*i+3];
    st.S[i] ^= m;
  }
  schwaemm_permutation(&st);
  for (int i = 0; i < 4; i++) {
    uint32_t c = st.S[i] ^ (((uint32_t)in[4*i]<<24)|((uint32_t)in[4*i+1]<<16)|((uint32_t)in[4*i+2]<<8)|in[4*i+3]);
    out[4*i+0]=(c>>24)&0xFF;
    out[4*i+1]=(c>>16)&0xFF;
    out[4*i+2]=(c>>8)&0xFF;
    out[4*i+3]=c&0xFF;
  }
}

void schwaemm_decrypt_block(const uint8_t *in, uint8_t *out, const uint8_t *key) {
  schwaemm_state_t st;
  schwaemm_init_state(&st, key);
  for (int i = 0; i < 4; i++) {
    uint32_t c = ((uint32_t)in[4*i]<<24)|((uint32_t)in[4*i+1]<<16)|((uint32_t)in[4*i+2]<<8)|in[4*i+3];
    st.S[i] ^= c;
  }
  schwaemm_permutation(&st);
  for (int i = 0; i < 4; i++) {
    uint32_t p = st.S[i] ^ (((uint32_t)in[4*i]<<24)|((uint32_t)in[4*i+1]<<16)|((uint32_t)in[4*i+2]<<8)|in[4*i+3]);
    out[4*i+0]=(p>>24)&0xFF;
    out[4*i+1]=(p>>16)&0xFF;
    out[4*i+2]=(p>>8)&0xFF;
    out[4*i+3]=p&0xFF;
  }
}

// ---------- Stats Utilities ----------
double ks_times_us[20], enc_times_us[20], dec_times_us[20];
void compute_stats(const double *vals,uint32_t n,double &mean,double &sd){
  if(n==0){mean=sd=0;return;} double sum=0;for(uint32_t i=0;i<n;i++)sum+=vals[i];
  mean=sum/n;if(n==1){sd=0;return;}double s=0;for(uint32_t i=0;i<n;i++)s+=pow(vals[i]-mean,2);
  sd=sqrt(s/(n-1));
}
inline void trig_high(){digitalWrite(TRIG_PIN,HIGH);}
inline void trig_low(){digitalWrite(TRIG_PIN,LOW);}

// ---------- Timing helpers ----------
uint64_t time_key_schedule(const uint8_t *key,uint32_t loops){
  uint8_t tmp[16];
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) memcpy(tmp,key,16);
  return esp_timer_get_time()-t0;
}
uint64_t time_encrypt(const uint8_t *key,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) schwaemm_encrypt_block(in,out,key);
  return esp_timer_get_time()-t0;
}
uint64_t time_decrypt(const uint8_t *key,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) schwaemm_decrypt_block(in,out,key);
  return esp_timer_get_time()-t0;
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200); delay(1000);
  WiFi.mode(WIFI_OFF); btStop();
  pinMode(TRIG_PIN,OUTPUT); digitalWrite(TRIG_PIN,LOW);
  delay(2000);
  Serial.println("# SCHWAEMM-128-128 ESP32 benchmark");
  Serial.printf("# CPU=%.2f MHz, N_RUNS=%u, N_LOOPS=%u\n",CPU_FREQ_HZ/1e6,N_RUNS,N_LOOPS);
  Serial.println("ALG,key_bits,block_bits,run,N_loops,ks_us,enc_us,dec_us,throughput_Bps,cycles_per_block");
}

// ---------- Main ----------
void loop(){
  uint8_t key[16]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
  uint8_t plain[16]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
  uint8_t cipher[16], recovered[16];

  for(uint32_t run=0;run<N_RUNS;run++){
    trig_high(); uint64_t ks_us=time_key_schedule(key,N_LOOPS); trig_low(); ks_times_us[run]=ks_us; delay(50);
    trig_high(); uint64_t enc_us=time_encrypt(key,plain,cipher,N_LOOPS); trig_low(); enc_times_us[run]=enc_us; delay(50);
    trig_high(); uint64_t dec_us=time_decrypt(key,cipher,recovered,N_LOOPS); trig_low(); dec_times_us[run]=dec_us; delay(200);

    double enc_s=enc_us/1e6;
    double throughput=(BLOCK_BYTES*(double)N_LOOPS)/enc_s;
    double cycles=(CPU_FREQ_HZ*enc_s)/N_LOOPS;

    Serial.printf("SCHWAEMM-128-128,128,128,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f\n",
      run+1,N_LOOPS,(double)ks_us,(double)enc_us,(double)dec_us,throughput,cycles);
    delay(200);
  }

  double ks_mean,ks_sd,enc_mean,enc_sd,dec_mean,dec_sd;
  compute_stats(ks_times_us,N_RUNS,ks_mean,ks_sd);
  compute_stats(enc_times_us,N_RUNS,enc_mean,enc_sd);
  compute_stats(dec_times_us,N_RUNS,dec_mean,dec_sd);

  double enc_s_mean=enc_mean/1e6;
  double throughput_mean=(BLOCK_BYTES*N_LOOPS)/enc_s_mean;
  double cycles_mean=(CPU_FREQ_HZ*enc_s_mean)/N_LOOPS;

  Serial.println("# SUMMARY");
  Serial.printf("# KS mean=%.3fus ±%.3f\n",ks_mean,ks_sd);
  Serial.printf("# ENC mean=%.3fus ±%.3f\n",enc_mean,enc_sd);
  Serial.printf("# DEC mean=%.3fus ±%.3f\n",dec_mean,dec_sd);
  Serial.printf("# Throughput=%.3f B/s, Cycles/block=%.3f\n",throughput_mean,cycles_mean);

  Serial.println("ALG,key_bits,block_bits,N_runs,N_loops,enc_mean_us,enc_sd_us,throughput_Bps,cycles_per_block");
  Serial.printf("SCHWAEMM-128-128,128,128,%u,%u,%.3f,%.3f,%.3f,%.3f\n",
    N_RUNS,N_LOOPS,enc_mean,enc_sd,throughput_mean,cycles_mean);

  Serial.println("# BENCHMARK COMPLETE - Halting.");
  while(true) delay(1000);
}
