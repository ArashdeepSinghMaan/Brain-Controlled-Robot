import mne
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert

# -------------------- Configuration --------------------
sfreq = 250  # sampling frequency in Hz
n_channels = 2  # adjust based on your data
ch_names = ['EEG1', 'EEG2']
ch_types = ['eeg'] * n_channels

# For demonstration, we'll simulate 30 seconds of data:
times = np.arange(0, 30, 1/sfreq)

# Simulate two channels as sine waves with a little noise (representing, e.g., alpha band activity)
data = np.array([np.sin(2 * np.pi * 10 * times) + 0.1*np.random.randn(len(times)),
                 np.sin(2 * np.pi * 11 * times) + 0.1*np.random.randn(len(times))])

# Create an MNE RawArray object
info = mne.create_info(ch_names=ch_names, sfreq=sfreq, ch_types=ch_types)
raw = mne.io.RawArray(data, info)

# -------------------- Preprocessing and Hilbert Transform --------------------
# Define the frequency band of interest (e.g., alpha: 8-12 Hz)
fmin, fmax = 8, 12

# Copy and filter the raw data to isolate the frequency band
raw_band = raw.copy().filter(fmin, fmax, method='iir', verbose=False)

# Get the filtered data as a NumPy array (shape: [n_channels, n_samples])
filtered_data = raw_band.get_data()

# Compute the analytic signal using Hilbert transform
# Note: mne.filter.hilbert returns the analytic signal (complex)
analytic_signal = hilbert(filtered_data, axis=1)
#mne.filter.hilbert(filtered_data, envelope=False)

# Compute the amplitude envelope (magnitude of the analytic signal)
envelope = np.abs(analytic_signal)

# -------------------- Baseline Calculation --------------------
# Suppose you have an event at t = 10 seconds.
# Define a baseline period and an event period (in seconds)
baseline_tmin, baseline_tmax = 9.8, 10.0   # baseline period (e.g., 200 ms before event)
event_tmin, event_tmax = 10.0, 10.5          # event period (e.g., 500 ms following the event)

# Convert time intervals to sample indices using the time vector
baseline_idx = (times >= baseline_tmin) & (times < baseline_tmax)
event_idx = (times >= event_tmin) & (times < event_tmax)

# Compute the mean amplitude envelope over the baseline period for each channel
baseline_mean = envelope[:, baseline_idx].mean(axis=1)
print("Baseline mean envelope (per channel):", baseline_mean)

# Compute the mean amplitude envelope over the event period for each channel
event_mean = envelope[:, event_idx].mean(axis=1)
print("Event mean envelope (per channel):", event_mean)

# Compute the relative change (e.g., percentage increase or decrease)
relative_change = (event_mean - baseline_mean) / baseline_mean
print("Relative change in envelope (per channel):", relative_change)

# -------------------- Visualization --------------------
# Plot the envelope for one channel along with the baseline and event intervals
channel = 0  # for example, plot for channel 1
plt.figure(figsize=(10, 4))
plt.plot(times, envelope[channel], label='Amplitude Envelope')
plt.axvspan(baseline_tmin, baseline_tmax, color='green', alpha=0.3, label='Baseline')
plt.axvspan(event_tmin, event_tmax, color='red', alpha=0.3, label='Event')
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.title(f"Channel {ch_names[channel]}: Hilbert Envelope with Baseline and Event")
plt.legend()
plt.show()
