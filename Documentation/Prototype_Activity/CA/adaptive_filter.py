import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, fft

num_samples = 10000

noise = np.random.normal(0, 1, size=num_samples)


LPF_sos = signal.butter(5, 0.5, btype='lowpass', output='sos')
w, h = signal.sosfreqz(LPF_sos)

D = signal.sosfilt(sos=LPF_sos, x=noise)



num_taps = 128
mu = 1/1024

h_hat = np.zeros(num_taps)

y = np.zeros(num_samples)
e = np.zeros(num_samples)

for i in range (num_taps, num_samples):
    u = noise[i:i-num_taps:-1]
    y[i] = np.dot(h_hat, u)
    e[i] = D[i] - y[i]
    h_hat = h_hat + mu * u * e[i]
print(h_hat)
y_fft = fft.fft(h_hat)

y_fft = np.abs(y_fft)
y_fft = fft.fftshift(y_fft)


# plt.subplot(6, 1, 1)
# plt.plot(noise)
# plt.subplot(6, 1, 2)
# plt.plot(w, np.abs(h), 'b')
# plt.subplot(6, 1, 3)
# plt.plot(D)
# plt.subplot(6, 1, 4)
# plt.plot(y)
# plt.subplot(6, 1, 5)
# plt.plot(e)
# plt.subplot(6, 1, 6)
# plt.plot(y_fft)
# plt.show()