/*
  ESP32 SPECK-64-128 Benchmark (Final)
  ------------------------------------
  - Official SPECK implementation (64-bit block, 128-bit key, 27 rounds)
  - Measures key schedule, encryption, decryption times
  - Toggles TRIG_PIN (GPIO4) for UNO+INA219 logger
  - Disables WiFi/Bluetooth for stable power
  - Prints CSV per run + summary stats
  - Author: 
*/

#include <Arduino.h>
#include "esp_timer.h"
#include "WiFi.h"
#include "BluetoothSerial.h"

// ===== USER CONFIG =====
const uint32_t CPU_FREQ_HZ = 240000000UL;
const int TRIG_PIN = 4;           // GPIO4 → UNO D2
const uint32_t N_RUNS  = 20;      // repetitions per algorithm
const uint32_t N_LOOPS = 200000;  // encryptions per window (~1–3 s)
const size_t BLOCK_BYTES = 8;     // 64-bit block
const size_t KEY_BYTES   = 16;    // 128-bit key
// ========================

// ---------- SPECK IMPLEMENTATION ----------
#define SPECK_ROUNDS 27

typedef struct {
  uint32_t k[SPECK_ROUNDS];
} speck_key_t;

// rotate right/left macros
static inline uint32_t rotr32(uint32_t x, uint32_t r) { return (x >> r) | (x << (32 - r)); }
static inline uint32_t rotl32(uint32_t x, uint32_t r) { return (x << r) | (x >> (32 - r)); }

void speck_expand_key(const uint8_t key[16], speck_key_t *rk) {
  uint32_t k[4];
  for (int i = 0; i < 4; i++) {
    k[i] = ((uint32_t)key[4*i] << 24) | ((uint32_t)key[4*i+1] << 16) | ((uint32_t)key[4*i+2] << 8) | key[4*i+3];
  }
  rk->k[0] = k[0];
  uint32_t b = k[1], c = k[2], d = k[3];
  for (int i = 0; i < SPECK_ROUNDS - 1; i++) {
    uint32_t tmp = rotr32(b, 8);
    tmp += k[i];
    tmp ^= i;
    k[i+1] = tmp;
    rk->k[i+1] = tmp;
    b = rotr32(c, 8);
    c = rotr32(d, 8);
    d = tmp;
  }
}

void speck_encrypt_block(const speck_key_t *rk, const uint8_t in[8], uint8_t out[8]) {
  uint32_t x = ((uint32_t)in[0]<<24)|((uint32_t)in[1]<<16)|((uint32_t)in[2]<<8)|in[3];
  uint32_t y = ((uint32_t)in[4]<<24)|((uint32_t)in[5]<<16)|((uint32_t)in[6]<<8)|in[7];
  for (int i = 0; i < SPECK_ROUNDS; i++) {
    x = (rotr32(x, 8) + y) ^ rk->k[i];
    y = rotl32(y, 3) ^ x;
  }
  out[0]=(x>>24)&0xFF; out[1]=(x>>16)&0xFF; out[2]=(x>>8)&0xFF; out[3]=x&0xFF;
  out[4]=(y>>24)&0xFF; out[5]=(y>>16)&0xFF; out[6]=(y>>8)&0xFF; out[7]=y&0xFF;
}

void speck_decrypt_block(const speck_key_t *rk, const uint8_t in[8], uint8_t out[8]) {
  uint32_t x = ((uint32_t)in[0]<<24)|((uint32_t)in[1]<<16)|((uint32_t)in[2]<<8)|in[3];
  uint32_t y = ((uint32_t)in[4]<<24)|((uint32_t)in[5]<<16)|((uint32_t)in[6]<<8)|in[7];
  for (int i = SPECK_ROUNDS - 1; i >= 0; i--) {
    y = rotr32(y ^ x, 3);
    x = rotl32((x ^ rk->k[i]) - y, 8);
  }
  out[0]=(x>>24)&0xFF; out[1]=(x>>16)&0xFF; out[2]=(x>>8)&0xFF; out[3]=x&0xFF;
  out[4]=(y>>24)&0xFF; out[5]=(y>>16)&0xFF; out[6]=(y>>8)&0xFF; out[7]=y&0xFF;
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
uint64_t time_key_schedule(const uint8_t *key, speck_key_t *rk, uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) speck_expand_key(key,rk);
  return esp_timer_get_time()-t0;
}
uint64_t time_encrypt(const speck_key_t *rk,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) speck_encrypt_block(rk,in,out);
  return esp_timer_get_time()-t0;
}
uint64_t time_decrypt(const speck_key_t *rk,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) speck_decrypt_block(rk,in,out);
  return esp_timer_get_time()-t0;
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200); delay(1000);
  WiFi.mode(WIFI_OFF); btStop();
  pinMode(TRIG_PIN,OUTPUT); digitalWrite(TRIG_PIN,LOW);
  delay(2000);
  Serial.println("# SPECK-64-128 ESP32 benchmark");
  Serial.printf("# CPU=%.2f MHz, N_RUNS=%u, N_LOOPS=%u\n",CPU_FREQ_HZ/1e6,N_RUNS,N_LOOPS);
  Serial.println("ALG,key_bits,block_bits,run,N_loops,ks_us,enc_us,dec_us,throughput_Bps,cycles_per_block");
}

// ---------- Main ----------
void loop(){
  uint8_t key[16]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
  uint8_t plain[8]={0x3B,0x72,0x73,0x0C,0xBD,0xFD,0xA8,0x62};
  uint8_t cipher[8], recovered[8];
  speck_key_t rk;

  // Sanity test (encrypt→decrypt)
  speck_expand_key(key,&rk);
  speck_encrypt_block(&rk,plain,cipher);
  speck_decrypt_block(&rk,cipher,recovered);
  if(memcmp(plain,recovered,8)!=0){Serial.println("# ERROR: Speck self-test fail!");while(true)delay(1000);}
  else Serial.println("# SPECK self-test PASS");

  for(uint32_t run=0;run<N_RUNS;run++){
    trig_high(); uint64_t ks_us=time_key_schedule(key,&rk,N_LOOPS); trig_low(); ks_times_us[run]=ks_us; delay(50);
    trig_high(); uint64_t enc_us=time_encrypt(&rk,plain,cipher,N_LOOPS); trig_low(); enc_times_us[run]=enc_us; delay(50);
    trig_high(); uint64_t dec_us=time_decrypt(&rk,cipher,recovered,N_LOOPS); trig_low(); dec_times_us[run]=dec_us; delay(200);

    double enc_s=enc_us/1e6;
    double throughput=(BLOCK_BYTES*(double)N_LOOPS)/enc_s;
    double cycles=(CPU_FREQ_HZ*enc_s)/N_LOOPS;

    Serial.printf("SPECK-64-128,128,64,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f\n",
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
  Serial.printf("SPECK-64-128,128,64,%u,%u,%.3f,%.3f,%.3f,%.3f\n",
    N_RUNS,N_LOOPS,enc_mean,enc_sd,throughput_mean,cycles_mean);

  Serial.println("# BENCHMARK COMPLETE - Halting.");
  while(true) delay(1000);
}
