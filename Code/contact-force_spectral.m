




%% Check psd
addpath(genpath('C:\Users\John\Projects_Git\vma'))
dt = 2.64E-02;
fs = 1/dt; 
nAvg = 4;  % number of averages
perc = 20;  % percent overlap
nfft = [];  % use default nfft lines

% get psd
[PSD,F] = getpsd(dat2,nAvg,perc,nfft,fs);
figure

% loglog(ff,pxx)
% hold all
loglog(F,PSD)
xlabel('frequency [Hz]')
ylabel('Displacement PSD [lb3]')
ylim([1e-15,1])
xlim([1e-3 20])