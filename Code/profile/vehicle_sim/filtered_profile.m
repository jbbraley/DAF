%% Initiate state space models
vb_ss = ss_bridge_vehicle();

% model parameters
fnb = 4.5; % natural frequency of bridge
fnv = 3.2; % natural frequency of vehicle
mb = 2000; % mass of bridge (slinch)
blength = 140; % ft
mt = 200; % mass of vehicle (slinch)
dt = 0.2; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)

wnb = fnb*2*pi;
wnv = fnv*2*pi;


%% Load profile
pro_file = file();

pro_file.name = 'I76-span3.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\Code\profile\vehicle_sim\@ss_bridge_vehicle';
file_cont = dlmread(pro_file.fullname,',');

dist = file_cont(:,1);
profile = file_cont(:,2);

%% Vehicle-Bridge
% populate parameters:
vb_ss.L = blength*12;
vb_ss.vel = vel;
vb_ss.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ss.mt = mt*vb_ss.gravity;
vb_ss.mb = mb*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = mb*vb_ss.L^3*wnb^2/pi^4;

vb_ss.dist = dist;
vb_ss.profile = profile;

LL_disp = -vb_ss.mt*vb_ss.L^3/(48*vb_ss.EI);

% make vehicle pre-displaced by self-weight
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;

% simulate
yvb = vb_ss.simulate;
b_disp1 = yvb(:,1);

% response with no profile
vb_ss.profile = zeros(size(vb_ss.profile));
yvb_0 = vb_ss.simulate;


%% Filter profile (stop gap)
dx = mean(diff(dist));
fs = 1/dx;
stop_bands = [0.0013 0.002; 0.0022 0.0033; 0.0033 0.005; 0.0067 0.01; 0.0133 0.02];

prof_filt = zeros(length(profile),size(stop_bands,1));
for ii = 1:size(stop_bands,1)
    d = fdesign.bandstop('N,F3dB1,F3dB2',20,stop_bands(ii,1),stop_bands(ii,2),fs);
    Hd = design(d,'butter');
    prof_filt(:,ii) = filter(Hd,profile);
end
    
%% Simulate with filtered profiles
vb_ss.vel = 1080;

vb_ss.profile = profile;
yvb = vb_ss.simulate;
vb_ss.profile = zeros(size(vb_ss.profile));
yvb_0 = vb_ss.simulate;

figure
plot(vb_ss.time,yvb(:,1)/LL_disp)
hold all
for  ii = 1:size(stop_bands,1)
vb_ss.profile =prof_filt(:,ii);
yvb_filt{ii} = vb_ss.simulate;
plot(vb_ss.time,yvb_filt{ii}(:,1)/LL_disp)
end
plot(vb_ss.time,yvb_0(:,1)/LL_disp)
legend({'original profile'; '50ft removed';'30ft removed';'20ft removed';'10ft removed';'5ft removed'; 'smooth roadway'})
xlabel('time (sec)')
ylabel('bridge disp amplification')

%% plot

figure('Position', [488 342 700 250])
plot(dist,[profile prof_filt])
legend({'original profile'; '50ft removed';'30ft removed';'20ft removed';'10ft removed';'5ft removed'})
xlabel('distance (in.)')
ylabel('profile elevation (in.)')
legend({})

max_amp = max([yb1(:,1) yb2(:,1) yvb(:,1) yvb_0(:,1)]/LL_disp,[],1);



