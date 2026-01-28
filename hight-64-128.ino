/*
  ESP32 HIGHT-64-128 Benchmark (Final)
  ------------------------------------
  - Implements official HIGHT block cipher (64-bit block, 128-bit key, 32 rounds)
  - Measures key schedule, encryption, decryption
  - Toggles TRIG_PIN (GPIO4) for UNO+INA219 power logger
  - Disables WiFi/Bluetooth for stable readings
  - Prints CSV-compatible timing and summary results
  - Author: 
*/

#include <Arduino.h>
#include "esp_timer.h"
#include "WiFi.h"
#include "BluetoothSerial.h"

// ===== USER CONFIG =====
const uint32_t CPU_FREQ_HZ = 240000000UL;
const int TRIG_PIN = 4;           // GPIO4 → UNO D2
const uint32_t N_RUNS  = 20;      // repetitions
const uint32_t N_LOOPS = 200000;  // encryptions per window (~1–3s)
const size_t BLOCK_BYTES = 8;     // 64-bit block
const size_t KEY_BYTES   = 16;    // 128-bit key
// ========================

// ---------- HIGHT IMPLEMENTATION ----------
#define HIGHT_ROUNDS 32

// HIGHT constants (DELTA and key-dependent whitening)
static const uint8_t DELTA[128] = {
  0x5A,0x84,0xB9,0xC5,0xDE,0x9A,0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,
  0xC5,0xDE,0x9A,0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,0xC5,0xDE,0x9A,
  0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,0xC5,0xDE,0x9A,0x7A,0xB9,0xE3,
  0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,0xC5,0xDE,0x9A,0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,
  0xE0,0x5A,0x84,0xB9,0xC5,0xDE,0x9A,0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,
  0xB9,0xC5,0xDE,0x9A,0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,0xC5,0xDE,
  0x9A,0x7A,0xB9,0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,0xC5,0xDE,0x9A,0x7A,0xB9,
  0xE3,0xD1,0xC6,0xBF,0xE0,0x5A,0x84,0xB9,0xC5,0xDE,0x9A,0x7A,0xB9
};

typedef struct {
  uint8_t roundKey[136]; // 4 whitening + 128 round keys + 4 whitening
} hight_key_t;

static inline uint8_t f0(uint8_t x){ return (uint8_t)(((x<<1) | (x>>7)) ^ ((x<<2)|(x>>6)) ^ ((x<<7)|(x>>1))); }
static inline uint8_t f1(uint8_t x){ return (uint8_t)(((x<<3)|(x>>5)) ^ ((x<<4)|(x>>4)) ^ ((x<<6)|(x>>2))); }

void hight_key_schedule(const uint8_t key[16], hight_key_t *rk) {
  for (int i=0;i<4;i++) rk->roundKey[i]=key[i+12]; // whitening 0–3
  for (int i=0;i<128;i++)
    rk->roundKey[i+4]=(uint8_t)(key[i%8]+DELTA[i]);
  for (int i=0;i<4;i++) rk->roundKey[132+i]=key[i]; // whitening 132–135
}

void hight_encrypt_block(const hight_key_t *rk, const uint8_t in[8], uint8_t out[8]) {
  uint8_t X[8];
  for (int i=0;i<8;i++) X[i]=in[i];
  X[1]=(uint8_t)(X[1]+rk->roundKey[0]);
  X[3]=(uint8_t)(X[3]^rk->roundKey[1]);
  X[5]=(uint8_t)(X[5]+rk->roundKey[2]);
  X[7]=(uint8_t)(X[7]^rk->roundKey[3]);

  for (int r=0;r<32;r++){
    uint8_t t7=X[7],t6=X[6],t5=X[5],t4=X[4],t3=X[3],t2=X[2],t1=X[1],t0=X[0];
    X[7]=(uint8_t)((t6 + (f1(t7)^rk->roundKey[4+4*r+3])) & 0xFF);
    X[6]=(uint8_t)(t5 ^ (f0(X[7]) + rk->roundKey[4+4*r+2]));
    X[5]=(uint8_t)((t4 + (f1(X[6])^rk->roundKey[4+4*r+1])) & 0xFF);
    X[4]=(uint8_t)(t3 ^ (f0(X[5]) + rk->roundKey[4+4*r+0]));
    X[3]=t2; X[2]=t1; X[1]=t0; X[0]=t7;
  }

  out[0]=(uint8_t)(X[1]+rk->roundKey[132]);
  out[1]=X[2];
  out[2]=(uint8_t)(X[3]^rk->roundKey[133]);
  out[3]=X[4];
  out[4]=(uint8_t)(X[5]+rk->roundKey[134]);
  out[5]=X[6];
  out[6]=(uint8_t)(X[7]^rk->roundKey[135]);
  out[7]=X[0];
}

void hight_decrypt_block(const hight_key_t *rk, const uint8_t in[8], uint8_t out[8]) {
  uint8_t X[8];
  X[1]=(uint8_t)(in[0]-rk->roundKey[132]);
  X[2]=in[1];
  X[3]=(uint8_t)(in[2]^rk->roundKey[133]);
  X[4]=in[3];
  X[5]=(uint8_t)(in[4]-rk->roundKey[134]);
  X[6]=in[5];
  X[7]=(uint8_t)(in[6]^rk->roundKey[135]);
  X[0]=in[7];

  for (int r=31;r>=0;r--){
    uint8_t t7=X[7],t6=X[6],t5=X[5],t4=X[4],t3=X[3],t2=X[2],t1=X[1],t0=X[0];
    X[0]=t1;
    X[1]=t2;
    X[2]=t3;
    X[3]=(uint8_t)(t4 ^ (f0(t5) + rk->roundKey[4+4*r+0]));
    X[4]=(uint8_t)((t5 - (f1(t6)^rk->roundKey[4+4*r+1])) & 0xFF);
    X[5]=(uint8_t)(t6 ^ (f0(t7) + rk->roundKey[4+4*r+2]));
    X[6]=(uint8_t)((t7 - (f1(t0)^rk->roundKey[4+4*r+3])) & 0xFF);
    X[7]=t0;
  }

  out[0]=X[0];
  out[1]=(uint8_t)(X[1]-rk->roundKey[0]);
  out[2]=X[2];
  out[3]=(uint8_t)(X[3]^rk->roundKey[1]);
  out[4]=X[4];
  out[5]=(uint8_t)(X[5]-rk->roundKey[2]);
  out[6]=X[6];
  out[7]=(uint8_t)(X[7]^rk->roundKey[3]);
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
uint64_t time_key_schedule(const uint8_t *key, hight_key_t *rk, uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) hight_key_schedule(key,rk);
  return esp_timer_get_time()-t0;
}
uint64_t time_encrypt(const hight_key_t *rk,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) hight_encrypt_block(rk,in,out);
  return esp_timer_get_time()-t0;
}
uint64_t time_decrypt(const hight_key_t *rk,const uint8_t *in,uint8_t *out,uint32_t loops){
  uint64_t t0=esp_timer_get_time();
  for(uint32_t i=0;i<loops;i++) hight_decrypt_block(rk,in,out);
  return esp_timer_get_time()-t0;
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200); delay(1000);
  WiFi.mode(WIFI_OFF); btStop();
  pinMode(TRIG_PIN,OUTPUT); digitalWrite(TRIG_PIN,LOW);
  delay(2000);
  Serial.println("# HIGHT-64-128 ESP32 benchmark");
  Serial.printf("# CPU=%.2f MHz, N_RUNS=%u, N_LOOPS=%u\n",CPU_FREQ_HZ/1e6,N_RUNS,N_LOOPS);
  Serial.println("ALG,key_bits,block_bits,run,N_loops,ks_us,enc_us,dec_us,throughput_Bps,cycles_per_block");
}

// ---------- Main ----------
void loop(){
  uint8_t key[16]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
  uint8_t plain[8]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
  uint8_t cipher[8], recovered[8];
  hight_key_t rk;

  // Sanity check
  hight_key_schedule(key,&rk);
  hight_encrypt_block(&rk,plain,cipher);
  hight_decrypt_block(&rk,cipher,recovered);
  if(memcmp(plain,recovered,8)!=0){Serial.println("# ERROR: HIGHT self-test fail!");while(true)delay(1000);}
  else Serial.println("# HIGHT self-test PASS");

  for(uint32_t run=0;run<N_RUNS;run++){
    trig_high(); uint64_t ks_us=time_key_schedule(key,&rk,N_LOOPS); trig_low(); ks_times_us[run]=ks_us; delay(50);
    trig_high(); uint64_t enc_us=time_encrypt(&rk,plain,cipher,N_LOOPS); trig_low(); enc_times_us[run]=enc_us; delay(50);
    trig_high(); uint64_t dec_us=time_decrypt(&rk,cipher,recovered,N_LOOPS); trig_low(); dec_times_us[run]=dec_us; delay(200);

    double enc_s=enc_us/1e6;
    double throughput=(BLOCK_BYTES*(double)N_LOOPS)/enc_s;
    double cycles=(CPU_FREQ_HZ*enc_s)/N_LOOPS;

    Serial.printf("HIGHT-64-128,128,64,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f\n",
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
  Serial.printf("HIGHT-64-128,128,64,%u,%u,%.3f,%.3f,%.3f,%.3f\n",
    N_RUNS,N_LOOPS,enc_mean,enc_sd,throughput_mean,cycles_mean);

  Serial.println("# BENCHMARK COMPLETE - Halting.");
  while(true) delay(1000);
}
