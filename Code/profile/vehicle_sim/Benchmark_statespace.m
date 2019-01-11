%% Initiate
vb_ss = ss_bridge_vehicle();
vb_ssmodel = sgl_bridge_vehicle();

fnb = 3; % natural frequency of bridge
fnv = 3.2; % natural frequency of vehicle
mb = 2000; % mass of bridge (slinch)
length = 140; % ft
mt = 200; % mass of vehicle (slinch)
dt = 0.2; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)

wnb = fnb*2*pi;
wnv = fnv*2*pi;
% parameters:
vb_ss.L = length*12;
vb_ss.vel = vel;
vb_ss.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ss.mt = mt*vb_ss.gravity;
vb_ss.mb = mb*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = mb*vb_ss.L^3*wnb^2/pi^4;

vb_ssmodel.L = length*12;
vb_ssmodel.vel = vel;
vb_ssmodel.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ssmodel.mt = mt*vb_ssmodel.gravity;
vb_ssmodel.mb = mb*vb_ssmodel.gravity;
vb_ssmodel.ct = dt*2*sqrt(vb_ssmodel.kt*vb_ssmodel.mt/vb_ssmodel.gravity);
vb_ssmodel.EI = mb*vb_ss.L^3*wnb^2/pi^4;

%% Load profile
pro_file = file();

pro_file.name = 'ISO_C10-300e-06_w-2_1.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');

% % chop profile into segments equal to bridge length.
% dx = mean(diff(file_cont(1:50,1)));
% prof_start = round(((500+111)*12+8)/dx);
% block_size = round(vb_ss.L/dx);
% prof_end = floor((size(file_cont,1)-prof_start+1)/block_size)*block_size+prof_start-1;
% block_inds = prof_start:block_size:size(file_cont,1);
% 
% profile = reshape(file_cont(prof_start:prof_end,2),block_size,[]);
% vb_ss.dist = (1:size(profile,1))*dx;
% 
% %start profile at zero amplitude 
% profile = profile-profile(1,:);
vb_ssmodel.bridge_start = 320*12;

vb_ssmodel.dist =  file_cont(:,1);
vb_ssmodel.profile = file_cont(:,2);
start_ind = find(file_cont(:,1)>=vb_ssmodel.bridge_start,1,'first')-1;
vb_ss.dist = file_cont(start_ind:end,1)-file_cont(start_ind,1);
vb_ss.profile = file_cont(start_ind:end,2);

DL_disp = 5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

%% Simulate

yy2 = vb_ssmodel.simulate;
vb_ss.x0 = yy2(start_ind,[2 1 4 3])';
yy = vb_ss.simulate();
    
figure
plot(vb_ss.time,yy(:,1))
    hold all
plot(vb_ssmodel.time,yy2(:,1))

%% compare with FE
dat1 = [];
% paste data from FE results [time bm_disp veh_accel contact_force]

% beam displacement
figure('Position', [488 342 640 200])
plot(vb_ssmodel.time,yy2(:,1))
hold all
plot(dat1(:,1)+vb_ssmodel.time(1),dat1(:,2))
xlabel('time (sec)')
ylabel('beam displacement (in)')
legend({'state-space model'; 'FE model'})
xlim([0 2.3])


%% Try with different bridge
fnb = 9.94; % natural frequency of bridge
fnv = 10.5; % natural frequency of vehicle
length = 40; % ft
kt = 879292; %lb/in
mt = 200; % mass of vehicle (slinch)
ct = 2652.23; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)
[mb, EI] = bridge2beam(fnb, 587723, length*12, 1);
start = 3864;


vb_ssmodel.L = length*12;
vb_ssmodel.vel = vel;
vb_ssmodel.kt = kt; % suspension stiffness for vehicle
vb_ssmodel.mt = mt*vb_ssmodel.gravity;
vb_ssmodel.mb = mb*vb_ssmodel.gravity;
vb_ssmodel.ct = ct;
vb_ssmodel.EI = EI;

vb_ssmodel.bridge_start = start;

vb_ssmodel.dist =  file_cont(:,1);
vb_ssmodel.profile = file_cont(:,2);

DL_disp = 5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

%% Simulate

yy2 = vb_ssmodel.simulate;
plot(vb_ssmodel.time,yy2(:,1))

dat2 = [];

% beam displacement
figure('Position', [488 342 640 200])
plot(vb_ssmodel.time,yy2(:,1))
hold all
plot(dat2(:,1)+vb_ssmodel.time(1),dat2(:,2))
xlabel('time (sec)')
ylabel('beam displacement (in)')
legend({'state-space model'; 'FE model'})
xlim([0 1])
