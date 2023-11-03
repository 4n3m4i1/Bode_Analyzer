import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, fft

num_samples = 10000

noise = np.random.normal(0, 1, size=num_samples)

# plt.plot(noise)
# plt.show()

LPF_sos = signal.butter(5, 0.5, btype='lowpass', output='sos')
w, h = signal.sosfreqz(LPF_sos)

D = signal.sosfilt(sos=LPF_sos, x=noise)

plt.plot(w, np.abs(h), 'b')
plt.show()

num_taps = 64
mu = 1/1024

h_hat = np.zeros(num_taps)

y = np.zeros(num_samples)
e = np.zeros(num_samples)

for i in range (num_taps, num_samples):
    u = noise[i:i-num_taps:-1]
    y[i] = np.dot(h_hat, u)
    e[i] = D[i] - y[i]
    h_hat = h_hat + mu * u * e[i]

y_fft = fft.fft(h_hat)

y_fft = np.abs(y_fft)
y_fft = fft.fftshift(y_fft)


plt.plot(y_fft)
plt.show()