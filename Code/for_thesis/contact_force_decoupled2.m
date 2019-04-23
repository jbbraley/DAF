%% Initiate state space models
vb_dc = sgl_dc();
vb_ss = sgl_bridge_vehicle();
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

% %% Vehicle
% % populate parameters
% veh.mt = mt*veh.gravity;
% veh.kt = (wnv)'.^2.*mt; 
% veh.ct = dt*2*sqrt(veh.kt*veh.mt/veh.gravity);
% veh.vel = vel;
% veh.profile = profile;
% veh.dist = dist;
% veh.bridge_start = bridge_start;
% % simulate
% yv = veh.simulate;
% c_force1 = yv(:,3);

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
c_force = yvb(:,5);
b_disp = yvb(:,1);

%% Decoupled Model
vb_dc = vb_ss.clone(vb_dc);
ydc = vb_dc.simulate();
c_force_dc = ydc(:,5);
b_disp_dc = ydc(:,1);

%% Compare contact force
figure('Position', [488 342 640 200])
plot(vb_dc.time,c_force_dc)
hold all
plot(vb_ss.time,c_force)
legend({'vehicle only'; 'vehicle with bridge'})
xlim([0 2.3])
xlabel('time (sec)')
ylabel('contact force (lb)')

% plot displacement comparison
fh = plotter('thesis_large');
plot(vb_dc.time,b_disp_dc)
hold all
plot(vb_ss.time,b_disp)
xlabel('time (sec)')
ylabel('bridge displacement (in)')
legend({'decoupled model'; 'coupled model'})
xlim([0 2.3])

%% quantify differences
% max differences
% align
% c_force_dc;
% c_force;
temp = vb_ss.time';


diff_dat=(b_disp_dc-b_disp);
mae_dat = sum(abs(diff_dat))/length(diff_dat);
mae_perc = mae_dat/abs(min(b_disp));
max_diff = min(b_disp_dc)-min(b_disp);
max_diff_perc = max_diff/(min(b_disp));

fh = plotter('thesis_large');
plot(vb_ss.time,100*diff_dat/min(b_disp));
xlim([vb_ss.time(1) max(vb_ss.time)]);
fh.xlabel = 'time (sec)';
fh.ylabel = 'Percent Difference';
ytickformat(fh.ah, 'percentage');
fh.refresh


dat_copy = [min(b_disp) min(b_disp_dc) max_diff_perc mae_dat mae_perc];

%% end of use