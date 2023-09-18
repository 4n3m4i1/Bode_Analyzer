clear all;
clear global;

% Lock in Amplifier Theory Verification
% Joseph A. De Vico
% 9/16/2023
%
%

% Decimate() for AWGN generation but not 100% necessary
pkg load signal

% Setup signal and simulated sampling environment
freq = 40000;
window_time = 1.024e-3;
num_periods = window_time * freq;

Fs = 1e6;
pts_per_period = int32(Fs / freq);

radfreq = deg2rad(freq);

x_ax = linspace(0,num_periods*2*pi,pts_per_period*num_periods);

input_phase = deg2rad(10);

Vsig = sin(radfreq*x_ax + input_phase);

Vsig_Amp = 0.1;

% Oversample noise generation just for fun,
%   decimate() includes built in low pass emulating front end AA filter
NGen_OSR = 2;
Noise_amp = 0.1;

% Scale Vsig amplitude
Vsig = (Vsig .* Vsig_Amp);
% Apply AWGN 
Vsig = Vsig .+ (Noise_amp .* decimate(randn(1, NGen_OSR * pts_per_period * num_periods), NGen_OSR));

% Generate reference signals
ref_amp = 2;
sinref = ref_amp * sin(radfreq*x_ax);
cosref = ref_amp * cos(radfreq*x_ax);

% Vector components from FIR LP / Avg
X = mean(cosref .* Vsig);
Y = mean(sinref .* Vsig);

% Plot sin(wt), cos(wt), and Vs (at double thickness)
plot(x_ax, sinref, "-", x_ax, cosref, "-", x_ax, Vsig, "linewidth",2);
xlabel("w");
ylabel("A");
legend("sine ref", "cos ref", "Vsig");
title("Vsig vs. Reference Waves");

% Final Results
Mag = (2 / ref_amp) * sqrt(X^2 + Y^2);
DB = 20 * log10(Mag/ref_amp);
SNR = 10 * log10( ((Vsig_Amp ^ 2)/2) / ((Noise_amp ^ 2) / 2));
Theta = 90 - rad2deg(atan(Y / X));