%load accel data
% accel data as variable: "accel"
% corresponding time vector as variable: "tt"


%filter out high frequency content
dt = mean(diff(tt));
fs = 1/dt;

begin_ind = 1;
end_ind = length(tt);

% plot(tt(begin_ind:end_ind),defl(begin_ind:end_ind))
%% Compute PSD
addpath(genpath('C:\Users\John\Projects_Git\vma'))

nAvg = 3;  % number of averages
perc = 20;  % percent overlap
nfft = [];  % use default nfft lines

% get psd
[pxx,ff] = getpsd(accel2(begin_ind:end_ind),nAvg,perc,nfft,fs);
figure
semilogy(ff,pxx,'-o')
xlabel('Frequency [Hz]')
ylabel('Acceleration power spectral density')
xlim([0 10])
