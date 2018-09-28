%% Setup profile parameters
% uses results from "spectral_density for comparison plots

B  = 1 ; % Sampling Interval (in)
L  = 300;  % Length Of Road Profile (ft)
N  = L*12/B; %  Number of data points
C = exp(-15); % roughness coefficient (not important)
w = 3.5; % waviness
IRI = 100; % in/mi
iter = 1;

% convert to meters
B= B*0.0254;
L = L*0.3048;

%% Generate profile
% Construct amplitudes and phase angles for each spatial frequency
dn = 1/(L);  % Frequency Band
n  = dn : dn : N*dn; % Spatial Frequency Band (start higher in frequency to avoid large global elevation changes. Min freq>=1/20m)
n = n(find(1./n>=20,1,'last'):end);
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



%% compute IRI and scale to specified value
% analyze pre-profile with golden-car ss model
[IRI0, y0] = getIRI(hx(1:find(x>=30.48,1,'first'))*39.3701,x(1:find(x>=30.48,1,'first'))*39.3701);
% get final state values
state_inds = [3 5 2 4];
x0 = y0(end,state_inds)';

[IRI_prof, ~] = getIRI(hx(find(x>=30.48,1,'first')+1:end)*39.3701,x(find(x>=30.48,1,'first')+1:end)*39.3701,[],x0);
scal = IRI/IRI_prof/63360;
profile = scal*hx;

% plot
% figure
% plot(x*3.28084, profile*39.3701) % plot with meters converted to feet and inched
% 
% xlabel('Longitudinal location [ft.]');
% ylabel('Profile elevation [in.]');
% ylim([-1 1]);

% % check IRI
% [~, y0] = getIRI(profile(1:find(x>=30.48,1,'first'))*39.3701,x(1:find(x>=30.48,1,'first'))*39.3701);
% % get final state values
% x0 = y0(end,state_inds)';
% 
% [IRI_prof, ~] = getIRI(profile(find(x>=30.48,1,'first')+1:end)*39.3701,x(find(x>=30.48,1,'first')+1:end)*39.3701,[],x0);
% IRI_prof*63360;
% 
% %% Check psd
% addpath(genpath('C:\Users\John\Projects_Git\vma'))
% fs = 1/B;
% nAvg = 1;  % number of averages
% perc = 20;  % percent overlap
% nfft = [];  % use default nfft lines
% 
% % get psd
% [PSD,F] = getpsd(profile',nAvg,perc,nfft,fs);
% [PSD0, F0] = getpsd(hx',nAvg,perc,nfft,fs);
% 
% figure
% loglog(F,PSD)
% hold all
% loglog(F0,PSD0)
% fit = C*F.^(-w);
% loglog(F,fit)
% 
% xlabel('Spatial frequency (n) [cycles/m]')
% ylabel('Displacement PSD (G_{d}) [m3]')
% ylim([1e-15,1])
% xlim([1e-3 20])

%% Save profile to file
% save_file = file();
% save_file.name = ['ISO_IRI' num2str(IRI) '-w' num2str(w) '_' num2str(iter) '.csv'];
% save_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\ISO_IRI';
% prof_dist = x*39.3701; % in.
% prof_elev = profile*39.3701; % in.
% 
% dlmwrite(save_file.fullname, [prof_dist' prof_elev'], ',');

% plot 2 different w
figure
plot(x*3.28084, profile*39.3701) % plot with meters converted to feet and inched

xlabel('Longitudinal location [ft.]');
ylabel('Profile elevation [in.]');
ylim([-0.5 0.5]);
