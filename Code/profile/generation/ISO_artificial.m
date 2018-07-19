%% Setup profile parameters
% uses results from "spectral_density for comparison plots

N  = 30240; %  Number of data points
L  = 755;  % Length Of Road Profile (m)
B  = L/N ; % Sampling Interval (m)
C = exp(-14.81); % roughness coefficient
w = 2.495; % waviness

%% Generate profile
% Construct amplitudes and phase angles for each spatial frequency
dn = 1/L;  % Frequency Band
n  = 6*dn : dn : N*dn; % Spatial Frequency Band (start higher in frequency to avoid large global elevation changes)
nangle = n*2*pi; % angular frequency band
psd = C*n.^(-w); % fitted psd value for each frequency band
del_angle = (nangle(end)-nangle)/(N-1);
Amp1 = sqrt(psd.*del_angle/pi); % amplitude

rng('shuffle') %shuffle random number generator
phi =  2*pi*rand(size(n)); % Random Phase Angle (uniformly ditributed)

% Sum sinusoids at each profile step
x = 0:B:L-B; % Abscissa Variable from 0 to L
hx = zeros(size(x));
for ii=1:length(x)
    hx(ii) = sum(Amp1.*sin(nangle*x(ii) - phi));
end

% plot
figure
plot(data(:,1)/12,profile)
hold all
plot(x*3.28084, hx*39.3701) % plot with meters converted to feet and inched

xlabel('Longitudinal location [ft.]');
ylabel('Profile elevation [in.]');
ylim([-2.7 2]);
legend({'measured profile';'artificial profile'});

%% Check psd
addpath(genpath('C:\Users\John\Projects_Git\vma'))
fs = 1/B;
nAvg = 1;  % number of averages
perc = 20;  % percent overlap
nfft = [];  % use default nfft lines

% get psd
[PSD,F] = getpsd(hx',nAvg,perc,nfft,fs);
figure

loglog(ff,pxx)
hold all
loglog(F,PSD)
fit = C*ff.^(-w);
loglog(ff,fit)

xlabel('Spatial frequency (n) [cycles/m]')
ylabel('Displacement PSD (G_{d}) [m3]')
ylim([1e-15,1])
xlim([1e-3 20])
legend({'measured profile';'artificial profile'; 'curve fit'});

%% Save profile to file
save_file = file();
save_file.name = ['ISO_e' num2str(14.81) 'w' num2str(2.495) '.csv'];
save_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\ISO8608';
prof_dist = x*39.3701; % in.
prof_elev = hx*39.3701; % in.

dlmwrite(save_file.fullname, [prof_dist' prof_elev'], ',');


