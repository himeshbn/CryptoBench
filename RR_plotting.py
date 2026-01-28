import matplotlib.pyplot as plt

# ---------------------------------------------------------
# Final benchmark data (from your uploaded table)
# ---------------------------------------------------------
algorithms = [
    "AES-128-128", "HIGHT-64-128", "LEA-128-128", "SCHWAEMM-128-128",
    "SPECK-64-128", "TinyJAMBU-128-128", "XOODYAK-128-128", "XTEA-64-128"
]

# Updated final performance values
ks_us = [
    11.9145, 5.5511, 6.9649, 0.1927,
    2.8988, 0.5145, 0.9286, 1.05
]

enc_us = [
    31.26, 20.71, 3.69, 25.19,
    1.51, 156.01, 30.48, 3.69
]

dec_us = [
    344.6327, 20.3488, 3.9239, 25.19,
    1.64, 156.01, 30.48, 3.71
]

# UPDATED ENERGY PER BLOCK (µJ)
energy_blk = [
    2.743838209, 0.24627213, 0.140080935, 0.587048869,
    0.024187814, 4.87681122, 0.591770156, 0.098310917
]

# ---------------------------------------------------------
# AES baseline values
# ---------------------------------------------------------
AES_KS   = ks_us[0]
AES_ENC  = enc_us[0]
AES_DEC  = dec_us[0]
AES_ENBL = energy_blk[0]

# ---------------------------------------------------------
# RR% Calculations
# ---------------------------------------------------------
RR_KS   = [(AES_KS / x) * 100 for x in ks_us]
RR_ENC  = [(AES_ENC / x) * 100 for x in enc_us]
RR_DEC  = [(AES_DEC / x) * 100 for x in dec_us]
RR_ENBL = [(AES_ENBL / x) * 100 for x in energy_blk]

# ---------------------------------------------------------
# Plot: RR% for Encryption Time
# ---------------------------------------------------------
plt.figure(figsize=(9,4))
plt.bar(algorithms, RR_ENC, color='skyblue')
plt.ylabel("RR% (Encryption Time vs AES)")
plt.title("Relative Reference – Encryption Time")
plt.xticks(rotation=45, ha="right")
plt.tight_layout()
plt.show()

# ---------------------------------------------------------
# Plot: RR% for Decryption Time
# ---------------------------------------------------------
plt.figure(figsize=(9,4))
plt.bar(algorithms, RR_DEC, color='lightgreen')
plt.ylabel("RR% (Decryption Time vs AES)")
plt.title("Relative Reference – Decryption Time")
plt.xticks(rotation=45, ha="right")
plt.tight_layout()
plt.show()

# ---------------------------------------------------------
# Plot: RR% for Key Schedule Time
# ---------------------------------------------------------
plt.figure(figsize=(9,4))
plt.bar(algorithms, RR_KS, color='salmon')
plt.ylabel("RR% (Key Schedule Time vs AES)")
plt.title("Relative Reference – Key Schedule")
plt.xticks(rotation=45, ha="right")
plt.tight_layout()
plt.show()

# ---------------------------------------------------------
# Plot: RR% for Energy per Block
# ---------------------------------------------------------
plt.figure(figsize=(9,4))
plt.bar(algorithms, RR_ENBL, color='gold')
plt.ylabel("RR% (Energy per Block vs AES)")
plt.title("Relative Reference – Energy per Block")
plt.xticks(rotation=45, ha="right")
plt.tight_layout()
plt.show()
