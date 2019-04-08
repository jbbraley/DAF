%% Initiate state space models
veh = qcarSDF();
bm = ss_bridge();
vb_ss = sgl_bridge_vehicle();
dist_past = 0.5;
k_mid = 222278;
vb_ss.db = 0;

% model parameters
fnb = 2.08; % natural frequency of bridge
fnv = 2.5; % natural frequency of vehicle
span = 140; % ft
mt = 200; % mass of vehicle (slinch)
dt = 0.10; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)
bridge_start = 3840;

[bmass, EI] = bridge2beam(fnb, k_mid, span*12, 1);
wnv = fnv*2*pi;

%% Load profile
pro_file = file();

pro_file.name = 'ISO_C10-600e-06_w-2_1.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');

dist = file_cont(:,1);
profile = file_cont(:,2);

%% Vehicle
% populate parameters
veh.mt = mt*veh.gravity;
veh.kt = (wnv)'.^2.*mt; 
veh.ct = dt*2*sqrt(veh.kt*veh.mt/veh.gravity);
veh.vel = vel;
veh.profile = profile;
veh.dist = dist;
veh.bridge_start = bridge_start;
% simulate
yv = veh.simulate;
c_force1 = yv(:,3);

%% Vehicle-Bridge
% populate parameters:
vb_ss.L = span*12;
vb_ss.vel = vel;
vb_ss.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ss.mt = mt*vb_ss.gravity;
vb_ss.mb = bmass*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = EI;
vb_ss.bridge_start = bridge_start;

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
bm.ms = bmass*bm.gravity;
bm.L = span*12;
bm.vel = vel;
bm.EI = EI;
bm.dt = diff(veh.time(1:2));
% grab only contact force for vehicle on bridge
bm.p = -c_force1(find(veh.time>=0,1,'first'):find(veh.time>=span*12/vel,1,'first'));

yb1 = bm.simulate;
b_disp2 = yb1(:,1);

% plot comparison
figure('Position', [488 342 700 250])
plot(bm.time,[yb1(:,1)])
hold all
plot(vb_ss.time,yvb(:,1))
xlabel('time (sec)')
ylabel('bridge displacement (in)')
legend({'uncoupled model'; 'coupled model'})
xlim([0 2.3])

%% quantify differences
% max differences
% align
dat_uncoup = yb1(:,1);
dat_coup = interp1(vb_ss.time,yvb(:,1),bm.time)';
force_uncoup = c_force1;
force_coup = c_force2;
temp = veh.time';


diff_dat=(dat_uncoup-dat_coup);
mae_dat = sum(abs(diff_dat))/length(diff_dat);
mae_perc = mae_dat/abs(min(dat_coup));
max_diff = min(dat_uncoup)-min(dat_coup);
max_diff_perc = max_diff/(min(dat_coup));
fh = plotter('thesis_large');
plot(bm.time,100*diff_dat/min(dat_coup));
xlim([bm.time(1) max(bm.time)]);
fh.xlabel = 'time (sec)';
fh.ylabel = 'Percent Difference';
ytickformat(fh.ah, 'percentage');
fh.refresh

dat_copy = [min(dat_coup) min(dat_uncoup) max_diff_perc mae_dat mae_perc];


%% end of use



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

