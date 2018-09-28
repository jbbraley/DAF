%% Initiate state space models
veh = qcarSDF();
bm = ss_bridge();
vb_ss = ss_bridge_vehicle();

% model parameters
fnb = 3; % natural frequency of bridge
fnv = 3.2; % natural frequency of vehicle
mb = 2000; % mass of bridge (slinch)
length = 140; % ft
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

%% Vehicle
% populate parameters
veh.ms = mt*veh.gravity;
veh.k = (wnv)'.^2.*mt; 
veh.c = dt*2*sqrt(veh.k*veh.ms/veh.gravity);
veh.vel = vel;
veh.profile = profile;
veh.dist = dist;
% simulate
yv = veh.simulate;
c_force1 = yv(:,3);

%% Vehicle-Bridge
% populate parameters:
vb_ss.L = length*12;
vb_ss.vel = vel;
vb_ss.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ss.mt = mt*vb_ss.gravity;
vb_ss.mb = mb*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = mb*vb_ss.L^3*wnb^2/pi^4;

vb_ss.dist = dist;
vb_ss.profile = profile;

% make vehicle pre-displaced by self-weight
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;

% simulate
yvb = vb_ss.simulate;
c_force2 = yvb(:,5);
b_disp2 = yvb(:,1);

%% Compare contact force
figure('Position', [488 342 640 200])
plot(veh.time,c_force1)
hold all
plot(vb_ss.time,c_force2)
legend({'vehicle only'; 'vehicle with bridge'})
xlim([0 2.3])
xlabel('time (sec)')
ylabel('contact force (lb)')

%% bridge state-space model
% compare output of bridge state space model with 2 contact forces (should
% be equal
bm.ms = mb*bm.gravity;
bm.L = length*12;
bm.vel = vel;
bm.EI = mb*bm.L^3*wnb^2/pi^4;
bm.dt = diff(veh.time(1:2));
bm.p = -c_force1;

yb1 = bm.simulate;

bm.p = -c_force2;
yb2 = bm.simulate;

% plot comparison
figure('Position', [488 342 700 250])
plot(bm.time,[yb1(:,1) yb2(:,1) yvb(:,1)])
xlabel('time (sec)')
ylabel('bridge displacement (in)')
legend({'contact force: vehicle only'; 'contact force: vehicle w/ bridge'; 'vehicle-bridge model'})

max_disp = min([yb1(:,1) yb2(:,1) yvb(:,1)],[],1);
err = (max_disp(3)-max_disp)/max_disp(3);

%% Amplification
LL_disp = -veh.ms*bm.L^3/(48*bm.EI);
LL_disp_amp = -max(c_force1)*bm.L^3/(48*bm.EI);

% response with no profile
vb_ss.profile = zeros(size(vb_ss.profile));
yvb_0 = vb_ss.simulate;

figure('Position', [488 342 700 250])
plot(bm.time,[yb1(:,1) yb2(:,1) yvb(:,1) yvb_0(:,1)]/LL_disp)
xlabel('time (sec)')
ylabel('bridge displacement amplification')
legend({'contact force: vehicle only'; 'contact force: vehicle w/ bridge'; 'vehicle-bridge model'; 'no profile'})

max_amp = max([yb1(:,1) yb2(:,1) yvb(:,1) yvb_0(:,1)]/LL_disp,[],1);

%% Double bridge
% x0 = yv(end,[2 1]);
% veh.x0 = x0;
% yv2 = veh.simulate;
% c_force3 = yv2(:,3);

bm.p = -c_force1;
bm.x0 = yvb(end,[2 1])';
yb3 = bm.simulate;

vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;
vb_ss.x0(1:2) = yvb(end,[2 1]);
vb_ss.profile = profile;
yvb2 = vb_ss.simulate;

figure('Position', [488 342 700 250])
plot(bm.time,[yb3(:,1) yvb2(:,1)]/LL_disp)
xlabel('time (sec)')
ylabel('bridge displacement amplification')
legend({'contact force: vehicle only'; 'contact force: vehicle w/ bridge'})


DL_disp = 5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

%% Bridge initial conditions
% vehicle has stationary initial conditions
% bridge has initial conditions

x0 = yvb(1590,[2 1 4 3]);
x0(3:4) = 0;


vb_ss.x0 = x0';
% make vehicle pre-displaced by self-weight
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;

vb_res1=vb_ss.simulate;
plot(vb_ss.time,vb_res1(:,1)/LL_disp)
hold all
vb_ss.x0(1:2) = vb_res1(1590,[2 1]);
vb_res2 = vb_ss.simulate;
plot(vb_ss.time,vb_res2(:,1)/LL_disp)
vb_ss.x0(1:2) = vb_res2(1590,[2 1]);
vb_res3 = vb_ss.simulate;
plot(vb_ss.time,vb_res3(:,1)/LL_disp)

plot(vb_ss.time,yvb(:,1)/LL_disp)

%% Try profile with smoothness criteria
%1/8" over 10 ft.
s_len = 10*12; s_dev = 1/8;
prof_filt1 = straightedge_filter(profile, dist, s_len,s_dev);
figure
plot(dist,[profile prof_filt1])
xlabel('distance (in.)')
ylabel('profile elevation (in.)')
legend({'original profile'; 'smoothed profile (1/8"/10ft)'})


vb_ss.x0 = [0 0 0 0]';
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;

vb_ss.profile = prof_filt1;
vb_res4 = vb_ss.simulate;

figure('Position', [488 342 700 250])
plot(vb_ss.time,[yvb(:,1) vb_res4(:,1) yvb_0(:,1)]/LL_disp)
legend({'real profile'; 'smoothed profile (1/8"/20ft)'; 'no profile'})
xlabel('time (sec)')
ylabel('bridge disp amplification')

% try tougher criteria
s_dev = 1/16;
prof_filt2 = straightedge_filter(profile, dist, s_len,s_dev);

vb_ss.profile = prof_filt2;
vb_res5 = vb_ss.simulate;

figure('Position', [488 342 700 250])
plot(vb_ss.time,[yvb(:,1) vb_res5(:,1) yvb_0(:,1)]/LL_disp)
legend({'real profile'; 'smoothed profile (1/16"/10ft)'; 'no profile'})
xlabel('time (sec)')
ylabel('bridge disp amplification')

