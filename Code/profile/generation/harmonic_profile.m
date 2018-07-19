%% Setup profile parameters
% uses results from "spectral_density for comparison plots

N  = 21600; %  Number of data points
L  = 1800*12;  % Length Of Road Profile (in)
B  = L/N ; % Sampling Interval (in)
wave = 20*12; % wavelength of profile features (in)
amp = 0.1; % amplitude of feature (in)

%% Generate profile
% Construct amplitudes and phase angles for each spatial frequency
nangle = 2*pi./(wave); % angular frequency band
rng(2); % control random number generator for repeatability
phi =  0; %2*pi*rand(size(nangle)); % Random Phase Angle (uniformly ditributed)
% Sum sinusoids at each profile step
x = 0:B:L-B; % Abscissa Variable from 0 to L
hx = zeros(size(x));
for ii=1:length(x)
    hx(ii) = sum(amp.*sin(nangle*x(ii) - phi));
end

% plot
figure
% plot(data(:,1)/12,profile)
% hold all
plot(x*unitsratio('ft','in'), hx) % plot with meters converted to feet and inche

xlabel('Longitudinal location [ft.]');
ylabel('Profile elevation [in.]');
ylim([-2.7 2]);


%% Check psd
addpath(genpath('C:\Users\John\Projects_Git\vma'))
fs = 1/B;
nAvg = 1;  % number of averages
perc = 20;  % percent overlap
nfft = [];  % use default nfft lines

% get psd
[PSD,F] = getpsd(hx',nAvg,perc,nfft,fs);
figure

% loglog(ff,pxx)
% hold all
loglog(F,PSD)
xlabel('Spatial frequency (n) [cycles/in]')
ylabel('Displacement PSD (G_{d}) [in3]')
ylim([1e-15,1])
xlim([1e-3 20])


%% Save profile to file
save_file = file();
save_file.name = ['pure-' num2str(wave) 'in_' num2str(amp) 'in' '.csv'];
save_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic2';
prof_dist = x; % in.
prof_elev = hx; % in.

dlmwrite(save_file.fullname, [prof_dist' prof_elev'], ',');


