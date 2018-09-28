%% assesses roadway unneveness based on PSD 

%% Load profile
pro_file = file();

pro_file.name = 'I76-span3.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\Code\profile\vehicle_sim\@ss_bridge_vehicle';
file_cont = dlmread(pro_file.fullname,',');

dist = file_cont(:,1);
profile = file_cont(:,2);
dx = mean(diff(dist));

% filter out frequency content corresponding to 1.2 Hz (golden-car natural
% frequency) and scale to have same IRI as original IRI
fs = 1/dx;
freq_filter = 3; % Hz
vel = 80*10.9361;
wav=vel./freq_filter;
bounds = [0.8 1.2];
d = fdesign.bandstop('N,F3dB1,F3dB2',20,1/wav*bounds(1),1/wav*bounds(2),fs);
Hd = design(d,'butter');
prof_filt = filter(Hd,profile);

%% try different filtering options
% freq_filter = [0.8 1.0 1.2 3 5 7 9 11]; % Hz
% wav=vel./freq_filter;
% stop_bands = bounds./wav';
% 
% prof_filt = zeros(length(profile),size(stop_bands,1));
% for ii = 1:size(stop_bands,1)
%     d = fdesign.bandstop('N,F3dB1,F3dB2',20,stop_bands(ii,1),stop_bands(ii,2),fs);
%     Hd = design(d,'butter');
%     prof_filt(:,ii) = filter(Hd,profile);
% end
% figure
% hold all
% for ii = 1:size(prof_filt,2)
% %     plot(prof_filt(:,ii))
%    [IRI_filt(ii), yy(:,:,ii)] = getIRI(prof_filt(:,ii),dist,[],x0);
% end

%% get IRI of original and filtered profile
% load pre-profile for accurate initial conditions
pre_file = pro_file.clone;
pre_file.name = 'I76-span2.csv';
pre_file_cont = dlmread(pre_file.fullname,',');

dist0 = pre_file_cont(:,1);
prefile = pre_file_cont(:,2);

% analyze pre-profile with golden-car ss model
[IRI, y0] = getIRI(prefile,dist0);
IRI*63360;
% get final state values
state_inds = [3 5 2 4];
x0 = y0(end,state_inds)';

% set initial conditions and compute IRIs
[IRI_1, yy1] = getIRI(profile,dist,[],x0);
[IRI_filt1, yy2] = getIRI(prof_filt,dist,[],x0);

prof_filt2 = prof_filt*IRI_1/IRI_filt1;
[IRI_filt2, ~]  = getIRI(prof_filt2, dist, [], x0);

figure
plot([profile prof_filt2])
%% Get bridge response to original profile and filtered profile
% initiate coupled vehicle bridge state space model object
vb_ss = ss_bridge_vehicle();

% model parameters
% golden car model used first
fnb = 3; % natural frequency of bridge
fnv = 1.2; % natural frequency of vehicle
dt = 0.2; % vehicle damping ratio

wnb = fnb*2*pi;
wnv = fnv*2*pi;

vb_ss.mt = 19841.6+2976.241; % (lb)
vb_ss.vel = 80*10.9361; % 80km/hr (in/sec)
vb_ss.L = 140*12;
vb_ss.kt = (wnv)'.^2.*vb_ss.mt/vb_ss.gravity; % suspension stiffness for vehicle
vb_ss.mb = 2000*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = vb_ss.mb/vb_ss.gravity*vb_ss.L^3*wnb^2/pi^4;

% set vehicle initial conditions
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt+y0(end,1);
vb_ss.x0(3) = y0(end,2);


% compare dynamic amplification generated from two profiles
vb_ss.dist = dist;
vb_ss.profile = profile;
% simulate
yvb1 = vb_ss.simulate;
b_disp1 = yvb1(:,1);

% simulate with filtered profile
vb_ss.profile = prof_filt2;
% simulate
yvb2 = vb_ss.simulate;
b_disp2 = yvb2(:,1);

% compute dynamic amplification
LL_disp = -vb_ss.mt*vb_ss.L^3/(48*vb_ss.EI);

% plot amplification 
figure('Position', [488 342 700 250])
plot(vb_ss.time,[yvb1(:,1) yvb2(:,1)]/LL_disp)
xlabel('time (sec)')
ylabel('bridge displacement amplification')
legend({'original profile'; 'filtered profile'})
max([yvb1(:,1) yvb2(:,1)]/LL_disp)

%% look at bridge amplification with different vehicle
vb_ss.mt = 200*vb_ss.gravity; % (lb)
vb_ss.kt = (3.2*2*pi)'.^2.*vb_ss.mt/vb_ss.gravity; % suspension stiffness for vehicle
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);

car = qcarSDF();
car.ms = vb_ss.mt;
car.k = vb_ss.kt; 
car.c =vb_ss.ct ;
car.vel = vb_ss.vel;
car.profile = prefile;
car.dist = dist0;
% simulate
yv = car.simulate;

vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt+yv(end,1);
vb_ss.x0(3) = yv(end,2);


vb_ss.profile = profile;
% simulate
yvb3 = vb_ss.simulate;
b_disp3 = yvb3(:,1);

% simulate with filtered profile
vb_ss.profile = prof_filt2;
% simulate
yvb4 = vb_ss.simulate;
b_disp4 = yvb4(:,1);

% compute dynamic amplification
LL_disp = -vb_ss.mt*vb_ss.L^3/(48*vb_ss.EI);

% plot amplification 
figure('Position', [488 342 700 250])
plot(vb_ss.time,[yvb3(:,1) yvb4(:,1)]/LL_disp)
xlabel('time (sec)')
ylabel('bridge displacement amplification')
legend({'original profile'; 'filtered profile'})
max([yvb3(:,1) yvb4(:,1)]/LL_disp)

%% Good IRI, large amp
% create profile with most of spatial frequency content corresponding to
% bridge natural frequency
% pass band filter profile
% filter out frequency content corresponding to 1.2 Hz (golden-car natural
% frequency) and scale to have same IRI as original IRI
freq_pass = 3; % Hz
vel = 720;
wav=vel./freq_pass;
bounds = [0.8 1.2];
d2 = fdesign.bandpass('N,F3dB1,F3dB2',20,1/wav*bounds(1),1/wav*bounds(2),fs);
Hd2 = design(d2,'butter');
prof_filt3 = filter(Hd2,profile);

% get IRI
[IRI_filt3, ~] = getIRI(prof_filt3,dist,[],x0);
prof_filt4 = 55/63360/IRI_filt3*prof_filt3;

[IRI_filt4, ~] = getIRI(prof_filt4, dist,[],x0);
IRI_filt4*63360;

% simulate with filtered profile
vb_ss.profile = prof_filt4;
vb_ss.vel = vel;
% simulate
yvb5 = vb_ss.simulate;
b_disp4 = yvb5(:,1);

% compute dynamic amplification
LL_disp = -vb_ss.mt*vb_ss.L^3/(48*vb_ss.EI);

% plot amplification 
figure('Position', [488 342 700 250])
plot(vb_ss.time,yvb5(:,1)/LL_disp)
xlabel('time (sec)')
ylabel('bridge displacement amplification')
max(yvb4(:,1)/LL_disp)

