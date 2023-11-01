clc;
clear;
close all;

Fs = 10000;
LPF = dsp.LowpassFilter;
x = wgn(Fs+1,1,0);
x1= x(1:Fs);
x2= x(2:(Fs+1));

d = LPF(x1);

Taps = 64;
mu = 1/1024;

h_hat = zeros(Taps,1);
h_hat1 = zeros(Taps,1);


for i = Taps:Fs
    u = x1(i:-1:i-Taps+1);
    y(i) = h_hat' * u;
    e(i) = d(i) - y(i);
    h_hat = h_hat + mu * u * e(i);
end


for i = Taps:Fs
    u1 = x2(i:-1:i-Taps+1);
    y1(i) = h_hat1' * u1;
    e1(i) = d(i) - y1(i);
    h_hat1 = h_hat1 + mu * u1 * e1(i);
end

figure(1);
subplot(5,1,1), plot(x), title('Noise');
subplot(5,1,2), plot(d), title('Noise after LPF');
subplot(5,1,3), plot(y), title('Noise from Adaptive Filter');
subplot(5,1,4), plot(e1), title('error 1');
subplot(5,1,5), plot(e), title('Error Signal');

Y = fft(h_hat);
Y = abs(Y);
Y = fftshift(Y);

Y1 = fft(h_hat1);
Y1 = abs(Y1);
Y1 = fftshift(Y1);

figure(2);
subplot(2,1,1),plot(Y), title('FFT of h.hat');
subplot(2,1,2),plot(Y1), title('FFT of h.hat1');







