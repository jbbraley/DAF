%% Setup profile parameters
% uses results from "spectral_density for comparison plots
iter = 1; %iteration number for filename
N  = 7875; %  Number of data points
L  = 200;  % Length Of Road Profile (m)
dn = 1/L;  % Frequency Band
B  = L/N ; % Sampling Interval (m)
C10 = 600e-6; % roughness coefficient
w = 2.0; % waviness
freq_lb = 0.05; % upper bound for frequency band of included  frequency content
freq_ub = 10; % lower bound for frequency band of included frequency content

%% Generate profile
% Construct amplitudes and phase angles for each spatial frequency
n  = dn:dn:N*dn ; % Spatial Frequency Band (start higher in frequency to avoid large global elevation changes)
sub_ind = find(n<freq_lb,1,'last'):find((n-freq_ub)>0,1,'first'); 
n_sub = n(sub_ind); %
nangle = 2*pi*n_sub; % angular frequency band
psd = C10*(10*n_sub).^(-w); % fitted psd value for each frequency band
del_angle = (nangle(end)-nangle)/(length(n_sub)-1);
Amp1 = sqrt(psd.*del_angle/pi); % amplitude

rng('shuffle') %shuffle random number generator
% rng(55); % control random number generator for repeatability
phi =  2*pi*rand(size(n)); % Random Phase Angle (uniformly ditributed)
phi = phi(sub_ind);
% Sum sinusoids at each profile step
x = 0:B:L-B; % Abscissa Variable from 0 to L
hx = zeros(size(x));
for ii=1:length(x)
    hx(ii) = sum(Amp1.*sin(nangle*x(ii) - phi));
end

% plot
figure
% plot(data(:,1)/12,profile)
% hold all
plot(x*3.28084, hx*39.3701) % plot with meters converted to feet and inched

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
hold all
fit = C10*(10*F).^(-w);
loglog(F,fit)

xlabel('Spatial frequency (n) [cycles/m]')
ylabel('Displacement PSD (G_{d}) [m3]')
ylim([1e-15,1])
xlim([1e-3 20])


%% Save profile to file
save_file = file();
save_file.name = ['ISO_C10-' num2str(C10*1e6) 'e-06_w-' num2str(w) '_' num2str(iter) '.csv'];
save_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\model_validation';
prof_dist = x*39.3701; % in.
prof_elev = (hx-hx(1))*39.3701; % in.

dlmwrite(save_file.fullname, [prof_dist' prof_elev'],'delimiter', ',', 'precision', 5);


