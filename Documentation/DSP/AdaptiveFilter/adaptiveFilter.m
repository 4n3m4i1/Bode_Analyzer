%   Adaptive Filter Test
%   Joseph A. De Vico
%   10/03/2023
%

clear global;
clearvars;


ideal_taps = importdata("test_ideal_tap_weights.txt","\n",0);
ideal_taps = transpose(ideal_taps);

firsize = size(ideal_taps);
%firsize([1 2]) = firsize([2 1]);

adaptive_taps = zeros(firsize);

nze = zeros(firsize);
nze = awgn(nze,20)


