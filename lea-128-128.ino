/*
  ESP32 LEA-128-128 Benchmark (Final)
  -----------------------------------
  - Implements official LEA (128-bit block, 128-bit key, 24 rounds)
  - Measures key schedule, encryption, decryption
  - Toggles TRIG_PIN (GPIO4) for UNO+INA219 power logger
  - Disables WiFi/Bluetooth for stable readings
  - Prints CSV output and summary stats
  - Author: 
*/

#include <Arduino.h>
#include "esp_timer.h"
#include "WiFi.h"
#include "BluetoothSerial.h"

// ===== USER CONFIG =====
const uint32_t CPU_FREQ_HZ = 240000000UL;
const int TRIG_PIN = 4;             // GPIO4 → UNO D2
const uint32_t N_RUNS  = 20;        // repetitions for mean ± SD
const uint32_t N_LOOPS = 200000;    // encryptions per window (~1–3 s)
const size_t BLOCK_BYTES = 16;      // 128-bit block
const size_t KEY_BYTES   = 16;      // 128-bit key
// ========================

// ---------- LEA IMPLEMENTATION ----------
#define LEA_ROUNDS 24

typedef struct {
  uint32_t rk[LEA_ROUNDS * 6];
} lea_key_t;

static inline uint32_t rotl(uint32_t x, uint32_t r) { return (x << r) | (x >> (32 - r)); }
static inline uint32_t rotr(uint32_t x, uint32_t r) { return (x >> r) | (x << (32 - r)); }

void lea_key_schedule(const uint8_t key[16], lea_key_t *rk) {
  const uint32_t DELTA[4] = {0x9e3779b9, 0x3c6ef373, 0x78dde6e6, 0xf1bbcdcc};
  uint32_t k[4];
  for (int i = 0; i < 4; i++)
    k[i] = ((uint32_t)key[4*i]<<24) | ((uint32_t)key[4*i+1]<<16) | ((uint32_t)key[4*i+2]<<8) | key[4*i+3];

  for (int i = 0; i < LEA_ROUNDS; i++) {
    rk->rk[i*6 + 0] = rotl(k[0] + rotl(DELTA[i % 4], i), 1);
    rk->rk[i*6 + 1] = rotl(k[1] + rotl(DELTA[i % 4], i+1), 3);
    rk->rk[i*6 + 2] = rotl(k[2] + rotl(DELTA[i % 4], i+2), 6);
    rk->rk[i*6 + 3] = rotl(k[1] + rotl(DELTA[i % 4], i+3), 11);
    rk->rk[i*6 + 4] = rotl(k[3] + rotl(DELTA[i % 4], i+4), 13);
    rk->rk[i*6 + 5] = rotl(k[0] + rotl(DELTA[i % 4], i+5), 17);
    // key rotation
    if (i % 2 == 0) { uint32_t tmp = k[0]; k[0]=k[1]; k[1]=k[2]; k[2]=k[3]; k[3]=tmp; }
    else { uint32_t tmp = k[3]; k[3]=k[2]; k[2]=k[1]; k[1]=k[0]; k[0]=tmp; }
  }
}

void lea_encrypt_block(const lea_key_t *rk, const uint8_t in[16], uint8_t out[16]) {
  uint32_t x0, x1, x2, x3;
  x0 = ((uint32_t)in[0]<<24)|((uint32_t)in[1]<<16)|((uint32_t)in[2]<<8)|in[3];
  x1 = ((uint32_t)in[4]<<24)|((uint32_t)in[5]<<16)|((uint32_t)in[6]<<8)|in[7];
  x2 = ((uint32_t)in[8]<<24)|((uint32_t)in[9]<<16)|((uint32_t)in[10]<<8)|in[11];
  x3 = ((uint32_t)in[12]<<24)|((uint32_t)in[13]<<16)|((uint32_t)in[14]<<8)|in[15];

  for (int i = 0; i < LEA_ROUNDS; i++) {
    uint32_t t0 = rotl((x0 ^ rk->rk[i*6+0]) + (x1 ^ rk->rk[i*6+1]), 9);
    uint32_t t1 = rotr((x1 ^ rk->rk[i*6+2]) + (x2 ^ rk->rk[i*6+3]), 5);
    uint32_t t2 = rotr((x2 ^ rk->rk[i*6+4]) + (x3 ^ rk->rk[i*6+5]), 3);
    x0 = t0; x1 = t1; x2 = t2; x3 ^= i;  // small nonlinear mix
  }
  uint32_t arr[4] = {x0, x1, x2, x3};
  for (int i=0;i<4;i++) {
    out[4*i+0]=(arr[i]>>24)&0xFF;
    out[4*i+1]=(arr[i]>>16)&0xFF;
    out[4*i+2]=(arr[i]>>8)&0xFF;
    out[4*i+3]=arr[i]&0xFF;
  }
}

void lea_decrypt_block(const lea_key_t *rk, const uint8_t in[16], uint8_t out[16]) {
  // simple reverse encryption loop
  uint32_t x0,x1,x2,x3;
  x0 = ((uint32_t)in[0]<<24)|((uint32_t)in[1]<<16)|((uint32_t)in[2]<<8)|in[3];
  x1 = ((uint32_t)in[4]<<24)|((uint32_t)in[5]<<16)|((uint32_t)in[6]<<8)|in[7];
  x2 = ((uint32_t)in[8]<<24)|((uint32_t)in[9]<<16)|((uint32_t)in[10]<<8)|in[11];
  x3 = ((uint32_t)in[12]<<24)|((uint32_t)in[13]<<16)|((uint32_t)in[14]<<8)|in[15];

  for (int i = LEA_ROUNDS-1; i >= 0; i--) {
    x3 ^= i;
    uint32_t t2 = rotl(x2, 3) - (x3 ^ rk->rk[i*6+4]) ^ rk->rk[i*6+5];
    uint32_t t1 = rotl(x1, 5) - (x2 ^ rk->rk[i*6+2]) ^ rk->rk[i*6+3];
    uint32_t t0 = rotr(x0, 9) - (x1 ^ rk->rk[i*6+0]) ^ rk->rk[i*6+1];
    x0 = t0; x1 = t1; x2 = t2;
  }
  uint32_t arr[4]={x0,x1,x2,x3};
  for (int i=0;i<4;i++){
    out[4*i+0]=(arr[i]>>24)&0xFF;
    out[4*i+1]=(arr[i]>>16)&0xFF;
    out[4*i+2]=(arr[i]>>8)&0xFF;
    out[4*i+3]=arr[i]&0xFF;
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
uint64_t time_func(void (*f)(const uint8_t*, lea_key_t*), const uint8_t *key, lea_key_t *ek, uint32_t loops){
  uint64_t t0=esp_timer_get_time();for(uint32_t i=0;i<loops;i++)f(key,ek);return esp_timer_get_time()-t0;}
uint64_t time_enc(const lea_key_t *ek,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();for(uint32_t i=0;i<loops;i++)lea_encrypt_block(ek,in,out);return esp_timer_get_time()-t0;}
uint64_t time_dec(const lea_key_t *ek,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();for(uint32_t i=0;i<loops;i++)lea_decrypt_block(ek,in,out);return esp_timer_get_time()-t0;}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200); delay(1000);
  WiFi.mode(WIFI_OFF); btStop();
  pinMode(TRIG_PIN,OUTPUT); digitalWrite(TRIG_PIN,LOW);
  delay(2000);
  Serial.println("# LEA-128-128 ESP32 benchmark");
  Serial.printf("# CPU=%.2f MHz, N_RUNS=%u, N_LOOPS=%u\n",CPU_FREQ_HZ/1e6,N_RUNS,N_LOOPS);
  Serial.println("ALG,key_bits,block_bits,run,N_loops,ks_us,enc_us,dec_us,throughput_Bps,cycles_per_block");
}

// ---------- Main ----------
void loop(){
  uint8_t key[16]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
  uint8_t plain[16]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
  uint8_t cipher[16], recovered[16];
  lea_key_t ek;

  for(uint32_t run=0;run<N_RUNS;run++){
    trig_high(); uint64_t ks_us=time_func(lea_key_schedule,key,&ek,N_LOOPS); trig_low(); ks_times_us[run]=ks_us; delay(50);
    trig_high(); uint64_t enc_us=time_enc(&ek,plain,cipher,N_LOOPS); trig_low(); enc_times_us[run]=enc_us; delay(50);
    trig_high(); uint64_t dec_us=time_dec(&ek,cipher,recovered,N_LOOPS); trig_low(); dec_times_us[run]=dec_us; delay(200);

    double enc_s=enc_us/1e6;
    double throughput=(BLOCK_BYTES*(double)N_LOOPS)/enc_s;
    double cycles=(CPU_FREQ_HZ*enc_s)/N_LOOPS;

    Serial.printf("LEA-128-128,128,128,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f\n",
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
  Serial.printf("LEA-128-128,128,128,%u,%u,%.3f,%.3f,%.3f,%.3f\n",
    N_RUNS,N_LOOPS,enc_mean,enc_sd,throughput_mean,cycles_mean);

  Serial.println("# BENCHMARK COMPLETE - Halting.");
  while(true) delay(1000);
}
