import pandas as pd
import numpy as np
from scipy.signal import welch
import matplotlib.pyplot as plt

# -------------------- Configuration --------------------
# Path to your CSV file
csv_file = r"C:\Users\AMREEN\Downloads\both hand.csv"

# Sampling frequency (Hz) - adjust this based on your OpenBCI configuration
sfreq = 250  

# Define EEG frequency bands (in Hz)
bands = {
    'delta': (1, 4),
    'theta': (4, 8),
    'alpha': (8, 13),
    'beta':  (13, 30),
    'gamma': (30, 50)
}

# -------------------- Data Reading --------------------
# Read the CSV file (assumes header row with channel names or numeric indices)
df = pd.read_csv(csv_file)
# Convert the DataFrame to a NumPy array and transpose so that rows = channels, columns = samples.
data = df.values.T  
n_channels, n_samples = data.shape
print(f"Data shape: {n_channels} channels x {n_samples} samples.")

# -------------------- Feature Extraction --------------------
# Dictionary to hold the average power in each band per channel
band_power = {band: [] for band in bands.keys()}

# Use Welch's method to compute the power spectral density for each channel
# You can adjust nperseg (segment length) depending on your data length and desired resolution
for ch in range(n_channels):
    # Compute PSD for channel ch
    f, psd = welch(data[ch], fs=sfreq, nperseg=sfreq*2)
    
    # For each frequency band, calculate average power within that band
    for band, (fmin, fmax) in bands.items():
        idx_band = np.logical_and(f >= fmin, f <= fmax)
        avg_power = np.mean(psd[idx_band])
        band_power[band].append(avg_power)

# -------------------- Display and Plot Features --------------------
# Print the average power for each band and channel
for band in bands.keys():
    print(f"\n{band.capitalize()} band power per channel:")
    for ch, power in enumerate(band_power[band]):
        print(f"Channel {ch+1}: {power:.4e}")

# Plotting the band power across channels for visual comparison
plt.figure(figsize=(10, 6))
for band in bands.keys():
    plt.plot(band_power[band], marker='o', label=f'{band.capitalize()} Power')
plt.xlabel('Channel Index')
plt.ylabel('Average Power')
plt.title('EEG Band Power across Channels')
plt.legend()
plt.grid(True)
plt.show()
