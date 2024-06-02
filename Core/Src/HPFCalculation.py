import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import firwin, freqz

# Filter design parameters
fs = 18181  # Sampling frequency
cutoff = 250  # Cutoff frequency
numtaps = 51  # Number of filter taps

# Generate filter coefficients
coeffs = firwin(numtaps, cutoff, fs=fs, pass_zero=False)

# Frequency response
w, h = freqz(coeffs, worN=8000)

# Plot frequency response
plt.figure(figsize=(10, 6))

# Plot the magnitude response
plt.subplot(2, 1, 1)
plt.plot(0.5 * fs * w / np.pi, np.abs(h), 'b')
plt.title('High-Pass Filter Frequency Response')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Gain')
plt.grid()

# Desired attenuation in the stop band (cyan) and desired ripple bounds in the pass band (red)
plt.axhline(0.1, color='cyan', linestyle='dashed', linewidth=1)  # Stop band attenuation
plt.axhline(0.9, color='red', linestyle='dashed', linewidth=1)   # Pass band ripple bounds
plt.axhline(1.1, color='red', linestyle='dashed', linewidth=1)   # Pass band ripple bounds

# Transition band (green)
plt.axvline(cutoff, color='green', linestyle='dashed', linewidth=1)  # Cutoff frequency

# Plot the phase response
plt.subplot(2, 1, 2)
plt.plot(0.5 * fs * w / np.pi, np.angle(h), 'b')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Phase (radians)')
plt.grid()

plt.tight_layout()
plt.show()
