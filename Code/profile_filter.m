%% Import profile data from file
pro_file_name = 'EB_Right_3_R.csv';

pro_file = file();
pro_file.name =pro_file_name;

pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right';

prof_raw = dlmread(pro_file.fullname, ',');

dist_ind = 1;
elev_ind = 2;
ramp_dist = 20; % feet

%% sampling info
dd = mean(diff(prof_raw(:,1)));
fs = 1/dd;

% find index of end of ramp
ramp_ind = find(prof_raw(:,1)-ramp_dist*12 >=0, 1, 'first');

%% examine spectra of raw profile
L = length(prof_raw(ramp_ind:end,2));
xdft = fft(prof_raw(ramp_ind:end,2));
P2 = abs(xdft/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = fs*12*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
xlim([0 .1])

%% Set up band stop filter
