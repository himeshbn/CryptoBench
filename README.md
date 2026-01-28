# **🔐 CryptoBench**

## **Benchmarking Lightweight Cryptographic Algorithms on ESP32**

A hardware-based benchmarking framework to evaluate the performance and energy efficiency of lightweight cryptographic algorithms on resource-constrained IoT devices.

**📌 Overview**

- CryptoBench is an experimental benchmarking project that evaluates eight lightweight cryptographic algorithms on the ESP32 DevKit v1 platform.
- The project focuses on real-time performance and energy analysis, making it highly relevant for battery-powered IoT systems.
- Unlike simulation-based studies, CryptoBench uses a dual-microcontroller setup with an INA219 power sensor to obtain accurate, real-world measurements.

## **🎯 Objectives**

- Implement lightweight cryptographic algorithms in pure C/C++
- Measure encryption, decryption, and key schedule time
- Capture real-time power and energy consumption
- Compare algorithms using RR% (Relative Reference) and Weighted Scoring Models
- Identify the best algorithm for IoT deployments

**🔑 Algorithms Benchmarked**
| Algorithm | Type                | Block / Key Size |
| --------- | ------------------- | ---------------- |
| AES       | Standard Cipher     | 128 / 128        |
| LEA       | ARX                 | 128 / 128        |
| SPECK     | ARX Feistel         | 64 / 128         |
| XTEA      | Feistel             | 64 / 128         |
| HIGHT     | Generalized Feistel | 64 / 128         |
| SCHWAEMM  | AEAD (NIST LWC)     | 128              |
| TinyJAMBU | AEAD                | 128              |
| XOODYAK   | Sponge-based AEAD   | 128              |

## **🛠️ Hardware & Software Stack**

### **Hardware**

- ESP32 DevKit v1 – Cryptographic execution
- Arduino UNO – Power measurement controller
- INA219 Sensor – Voltage, current & power monitoring
- Breadboard & jumper wires

### **Software**

- ESP-IDF / Arduino IDE
- Tera Term (serial logging)
- Python (NumPy, Pandas, Matplotlib)

**✅ All tools used are free and open-source**

## **🏗️ System Architecture**

- Architecture Highlights
- ESP32 executes cryptographic algorithms
- Arduino UNO + INA219 measures energy
- GPIO trigger ensures precise synchronization
- Two Tera Term instances log data independently

## **🔌 Hardware Setup**

- ESP32 power routed through INA219 shunt resistor
- Arduino samples current & voltage during execution
- GPIO trigger controls measurement window

## **📊 Performance Metrics**

The following metrics are collected over **200,000 iterations** per run:

- Encryption time
- Decryption time
- Key schedule time
- Throughput (MB/s)
- Average power (mW)
- Energy per block (µJ)
- Energy per byte (µJ/byte)
- RR% (Relative Reference vs AES)

## **📈 Graphical Results**

<img width="900" height="590" alt="RR_encryption" src="https://github.com/user-attachments/assets/81e07371-8ef5-4e50-b999-2eb6d367f321" />

<img width="900" height="590" alt="RR_energy" src="https://github.com/user-attachments/assets/bc12e674-a0f3-41c8-9de8-d6655073d0df" />

<img width="900" height="590" alt="RR_keyschedule" src="https://github.com/user-attachments/assets/cea47290-e278-45cc-b87b-0fd86bb4f2bc" />

<img width="900" height="590" alt="RR-decryption" src="https://github.com/user-attachments/assets/cba9e1e0-84ca-4365-ba8a-93b76022f1b3" />

<img width="790" height="390" alt="throughput_MBps" src="https://github.com/user-attachments/assets/cdfb6a3b-70cf-4756-8e99-243564b9e49c" />

## **🧮 Algorithm Ranking (WSM)**

_**Two evaluation models are applied:**_

- Battery-Priority Model → Energy focused
- Speed-Priority Model → Latency focused

_**Key Findings**_

- SPECK-64-128 → Best overall performer
- LEA-128-128 → Best 128-bit block cipher
- AES-128 → Highest energy consumption

<img width="620" height="535" alt="Radar_Graph_top_algorithms" src="https://github.com/user-attachments/assets/7ea17678-3d86-4b1b-823f-d0942c68a0fa" />


## **▶️ How to Run**

**1️⃣ Upload Firmware**

- Upload ESP32 cryptographic code
- Upload Arduino UNO power logger code

**2️⃣ Start Logging**

- Open two Tera Term windows
- ESP32 → performance logs
- Arduino → power logs

**3️⃣ Run Benchmark**

**Each algorithm runs:**

- 20 runs × 200,000 iterations
- Logs exported and analyzed in Python

## **📌 Applications**

- Secure IoT sensor networks
- Battery-powered embedded devices
- Smart meters & wearables
- Industrial IoT (IIoT)
- Lightweight secure communication systems

## **🚀 Future Enhancements**

- Benchmarking on STM32 / ARM Cortex-M
- Hardware acceleration comparison
- Integration with MQTT/CoAP
- Automated benchmarking pipeline
- Battery discharge modeling

