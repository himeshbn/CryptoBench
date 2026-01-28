#!/usr/bin/env python3
"""
WSM Ranking Script for Lightweight Crypto Algorithms (UPDATED)

- Uses the final benchmark table you provided (hard-coded here).
- Computes Weighted Scoring Model (WSM) scores for two models:
  A) IoT Battery-Priority Model (energy-focused)
  B) Speed-Priority Model (time-focused)

- Normalization:
    - For metrics where LOWER is better -> normalized = min(value_col) / value
    - For metrics where HIGHER is better -> normalized = value / max(value_col)

- Toggle plotting with PLOT_RADAR (default: True)

Run:
    python wsm_ranking.py
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ----------------------------
# Reference to uploaded file (if needed)
# ----------------------------
# local path of an uploaded file (adjust if you want to attach / export)
UPLOADED_FILE = "/mnt/data/major_project_report (9)_TrueMedia.pdf"

# ----------------------------
# 1) Final dataset (from your last message)
# ----------------------------
algorithms = [
    "AES-128-128","HIGHT-64-128","LEA-128-128","SCHWAEMM-128-128",
    "SPECK-64-128","TinyJAMBU-128-128","XOODYAK-128-128","XTEA-64-128"
]

# Exact final values you provided (units in comments)
ks_us =      [11.9145, 5.5511, 6.9649, 0.1927, 2.8988, 0.5145, 0.9286, 1.05]       # key schedule time (µs, lower better)
enc_us =     [31.26, 20.71, 3.69, 25.19, 1.51, 156.01, 30.48, 3.69]               # encryption time per block (µs, lower better)
dec_us =     [344.6327, 20.3488, 3.9239, 25.19, 1.64, 156.01, 30.48, 3.71]        # decryption time per block (µs, lower better)
throughput_Bps = [511691.978, 386239.373, 4326504.532, 635076.26, 5268041.23, 102557.507, 524852.422, 2163389.447] # higher better
energy_block = [2.743838209, 0.24627213, 0.140080935, 0.587048869, 0.024187814, 4.87681122, 0.591770156, 0.098310917]    # energy per block (µJ, lower better)
energy_byte =  [0.171489888, 0.030784016, 0.008755058, 0.036690554, 0.003023477, 0.304800701, 0.036985635, 0.012288865]  # energy per byte (µJ/byte, lower better)

# Put into DataFrame for pretty printing later
df = pd.DataFrame({
    "ALG": algorithms,
    "ks_us": ks_us,
    "enc_us": enc_us,
    "dec_us": dec_us,
    "throughput_Bps": throughput_Bps,
    "energy_block_uJ": energy_block,
    "energy_byte_uJ": energy_byte
})

# ----------------------------
# 2) Normalization helpers (robust)
# ----------------------------
def norm_lower_better(arr):
    arr = np.array(arr, dtype=float)
    mn = arr.min()
    # avoid division by zero: if arr == 0 -> give 0 (worst)
    with np.errstate(divide='ignore', invalid='ignore'):
        out = np.where(arr <= 0, 0.0, mn / arr)
    return out

def norm_higher_better(arr):
    arr = np.array(arr, dtype=float)
    mx = arr.max()
    if mx == 0:
        return np.zeros_like(arr)
    return arr / mx

# Compute normalized arrays
norms = {}
norms['ks']   = norm_lower_better(df['ks_us'])
norms['enc']  = norm_lower_better(df['enc_us'])
norms['dec']  = norm_lower_better(df['dec_us'])
norms['thr']  = norm_higher_better(df['throughput_Bps'])
norms['eblk'] = norm_lower_better(df['energy_block_uJ'])
norms['ebyte']= norm_lower_better(df['energy_byte_uJ'])

# ----------------------------
# 3) Define WSM weights (A and B)
# ----------------------------
# Option A — IoT Battery-Priority Model (recommended)
weights_A = {
    'energy_block_uJ': 0.35,
    'enc_us': 0.30,
    'throughput_Bps': 0.20,
    'ks_us': 0.05,
    'dec_us': 0.05,
    'energy_byte_uJ': 0.05
}

# Option B — Speed-Priority Model
weights_B = {
    'enc_us': 0.40,
    'throughput_Bps': 0.30,
    'energy_block_uJ': 0.20,
    'ks_us': 0.05,
    'dec_us': 0.05
}

# ----------------------------
# 4) Compute WSM scores
# ----------------------------
def compute_wsm_score(weights):
    score = np.zeros(len(df), dtype=float)
    for k,w in weights.items():
        if k == 'ks_us':
            score += w * norms['ks']
        elif k == 'enc_us':
            score += w * norms['enc']
        elif k == 'dec_us':
            score += w * norms['dec']
        elif k == 'throughput_Bps':
            score += w * norms['thr']
        elif k == 'energy_block_uJ':
            score += w * norms['eblk']
        elif k == 'energy_byte_uJ':
            score += w * norms['ebyte']
        else:
            raise ValueError("Unknown metric key: " + str(k))
    return score

df['WSM_A'] = compute_wsm_score(weights_A)
df['WSM_B'] = compute_wsm_score(weights_B)

# ----------------------------
# 5) Ranking and display
# ----------------------------
def ranked_table(df, col):
    tmp = df[['ALG','ks_us','enc_us','dec_us','throughput_Bps','energy_block_uJ','WSM_A','WSM_B']].copy()
    return tmp.sort_values(col, ascending=False).reset_index(drop=True)

print("\n=== Input dataset ===")
print(df[['ALG','ks_us','enc_us','dec_us','throughput_Bps','energy_block_uJ','energy_byte_uJ']].to_string(index=False))

print("\n=== Normalized values (sample) ===")
norm_df = pd.DataFrame({
    'ALG': df['ALG'],
    'norm_ks': norms['ks'],
    'norm_enc': norms['enc'],
    'norm_dec': norms['dec'],
    'norm_thr': norms['thr'],
    'norm_eblk': norms['eblk'],
    'norm_ebyte': norms['ebyte']
})
print(norm_df.round(4).to_string(index=False))

# Rankings
rankA = df.sort_values('WSM_A', ascending=False).reset_index(drop=True)
rankB = df.sort_values('WSM_B', ascending=False).reset_index(drop=True)

print("\n=== WSM Model A (IoT Battery-Priority) Results ===")
print(rankA[['ALG','WSM_A']].round(6).to_string(index=False))

print("\n=== WSM Model B (Speed-Priority) Results ===")
print(rankB[['ALG','WSM_B']].round(6).to_string(index=False))

# winners
winner_A = rankA.iloc[0]['ALG']
winner_B = rankB.iloc[0]['ALG']
print("\nWinner (Model A - IoT Battery-Priority):", winner_A)
print("Winner (Model B - Speed-Priority):", winner_B)

# ----------------------------
# 6) Final side-by-side ranking table
# ----------------------------
final = pd.DataFrame({
    'ALG': df['ALG'],
    'WSM_A': df['WSM_A'],
    'WSM_B': df['WSM_B'],
})
final['Rank_A'] = final['WSM_A'].rank(ascending=False, method='min').astype(int)
final['Rank_B'] = final['WSM_B'].rank(ascending=False, method='min').astype(int)
final = final.sort_values(['Rank_A','Rank_B', 'ALG']).reset_index(drop=True)
print("\n=== Final Ranking Table (both models) ===")
print(final[['ALG','WSM_A','Rank_A','WSM_B','Rank_B']].round(6).to_string(index=False))

# ----------------------------
# 7) Optional: Radar chart for the top 4 algorithms by Model A
# ----------------------------
PLOT_RADAR = True
if PLOT_RADAR:
    # choose top 4 by Model A score
    topN = rankA.head(4)['ALG'].tolist()
    # metrics to plot (normalized, higher better)
    metrics = ['norm_enc','norm_eblk','norm_thr','norm_ks','norm_dec']
    labels = ['Enc(norm)','EnergyBlk(norm)','Thr(norm)','KS(norm)','Dec(norm)']
    # build data for radar
    import math
    N = len(labels)
    angles = np.linspace(0, 2 * math.pi, N, endpoint=False).tolist()
    angles += angles[:1]
    fig, ax = plt.subplots(figsize=(6,6), subplot_kw=dict(polar=True))
    for alg_name in topN:
        row = norm_df[norm_df['ALG']==alg_name].iloc[0]
        vals = [row['norm_enc'], row['norm_eblk'], row['norm_thr'], row['norm_ks'], row['norm_dec']]
        vals += vals[:1]
        ax.plot(angles, vals, label=alg_name)
        ax.fill(angles, vals, alpha=0.1)
    ax.set_thetagrids(np.degrees(angles[:-1]), labels)
    ax.set_title("Radar: top algorithms (normalized metrics)")
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    plt.show()

# End