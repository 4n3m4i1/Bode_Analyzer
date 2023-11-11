%%
%   Joseph A. De Vico
%   26-10-2023
%   Multirate Downsampling Example
%%

clear all;
clear global;

pkg load signal;
pkg load ltfat;

NGen_OSR = 4;
freq = 40000;
window_time = 1.024e-3;
num_periods = window_time * freq;

Fs = 500e3;
pts_per_period = int32(Fs / freq);

radfreq = deg2rad(freq);

x_ax = linspace(0,num_periods*2*pi,pts_per_period*num_periods);
x_ax_2 = linspace(0, (num_periods * 2 * pi)/NGen_OSR, (pts_per_period * num_periods) / NGen_OSR);

Vsig = sin(radfreq*x_ax);

Vsig_dec_filt = decimate(Vsig, NGen_OSR, "fir");

Vsig_dec_nofilt = Vsig(1:NGen_OSR:end);

figure(1);
subplot(3,2,1);
plot(Vsig);
subplot(3,2,2);
fyt = fft(Vsig);
plotfft(Vsig,Fs,'posfreq');

subplot(3,2,3);
plot(Vsig_dec_nofilt);
subplot(3,2,4);
plotfft(Vsig_dec_nofilt,Fs/NGen_OSR,'posfreq');

subplot(3,2,5);
plot(Vsig_dec_filt);
subplot(3,2,6);
plotfft(Vsig_dec_filt,Fs/NGen_OSR,'posfreq');

%figure(2);
%plot(Vsig_dec_nofilt);

%figure(3);
%plot(Vsig_dec_filt);

%plot(length(Vsig) / 2, fft(Vsig), "-", length(Vsig_dec_filt) / 2, fft(Vsig_dec_filt), "-", length(Vsig_dec_nofilt) / 2, fft(Vsig_dec_nofilt), "linewidth",1);
%xlabel("t");
%ylabel("A");
%legend("Signal", "Proper Decimation", "No Filter Decimation");
%title("Vsig Decimation");