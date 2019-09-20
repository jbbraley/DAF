function [C,W] = ISO8608(profile, dist, w)
%% ISO8608
% 
% 
% 
% author: John Braley
% create date: 2018-09-10 02:45:18.487
	
if nargin<3; w = []; end

dx = mean(diff(dist));

%% Compute PSD
addpath(genpath('C:\Users\John\Projects_Git\vma'))
fs = 1/dx;
nAvg = max(1,floor(length(dist)*dx/(528*12)));  % number of averages (1/10th mile length of averages)
perc = 20;  % percent overlap
nfft = [];  % use default nfft lines

% get psd in metric units
[pxx,ff] = getpsd(profile*0.0254,nAvg,perc,nfft,fs/0.0254);

%% Fit with expression
% ISO 8608 Gd(n) = Cn^(-w)
% linear fit to log of predictors and results
if isempty(w)
    c = polyfit(log(ff(15:1500)),log(pxx(15:1500)),1);
    C = exp(c(2));
    W = -c(1);
else
    c = mean(log(pxx(15:1500))+w*log(ff(15:1500)));
    C = exp(c);
	W = w;
end

% % plotting example
% figure
% loglog(ff,pxx)
% hold all
% loglog(ff,C*ff.^(-W))
% ylim([1e-15,1])
% xlim([ff(2) ff(end)])
% txt1 = ['ISO 8608 fit: G_{d}(n) = ' num2str(C,3) '*n^{-' num2str(W) '}'];
% legend({'PSD'; txt1})
% xlabel('Spatial frequency (n) [cycles/m]')
% ylabel('Displacement PSD (G_{d}) [m^{3}]')
end
