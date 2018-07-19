%% assesses roadway unneveness based on PSD 

%% Load profile
pro_file = file();
pro_file.name = 'EB_right_1_R.csv';
pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right';
data = dlmread(pro_file.fullname,',');
profile = data(:,2);
dx = mean(diff(data(:,1)))/12;

%% Compute PSD
addpath(genpath('C:\Users\John\Projects_Git\vma'))
fs = 1/dx;
nAvg = 1;  % number of averages
perc = 20;  % percent overlap
nfft = [];  % use default nfft lines

% get psd
[pxx,ff] = getpsd(profile,nAvg,perc,nfft,fs);
figure
semilogy(ff,pxx)
xlabel('Spatial frequency [cycles/ft]')
ylabel('Displacement power spectral density [in3]')
xlim([0 .1])
% get psd in metric units
[pxx,ff] = getpsd(profile*0.0254,nAvg,perc,nfft,fs/0.3048);
figure
loglog(ff,pxx)
xlabel('Spatial frequency (n) [cycles/m]')
ylabel('Displacement PSD (G_{d}) [m3]')

%% Fit with expression (matlab curve fitting tool used)
% ISO 8608 Gd(n) = Cn^(-w)
% log_psd = log(pxx);
% log_ff = log(ff);
% cftool

C = exp(-14.81);
w = 2.495;
fit = C*ff.^(-w);
hold all
loglog(ff,fit)
ylim([1e-15,1])
xlim([1e-3 20])
txt1 = ['fit: G_{d}(n) = ' num2str(C,3) '*n^{-' num2str(w) '}'];

% for waviness (w) equal to 2:
C = exp(-15.8);
w = 2;
fit = C*ff.^(-w);
loglog(ff,fit)
txt2 = ['fit: G_{d}(n) = ' num2str(C,3) '*n^{-' num2str(w) '}'];

legend({'PSD'; txt1; txt2})