%% set up Run parameters

fnb = 3; % natural frequency of bridge
fnv = 3.6; % natural frequency of vehicle
mb = 2000; % mass of bridge (slinch)
mt = 200; % mass of vehicle (slinch)
db = .02; %:0.01:0.03; % bridge damping ratio
dt = 0.2; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)

wnb = fnb*2*pi;
wnv = fnv*2*pi;
kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
kb = (wnb)'.^2.*mb; % stiffness of bridge (idealized as sprung mass)
%% Load profile
pro_file = file();

pro_file.name = 'EB_right_1_R.csv';
pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right';
file_cont = dlmread(pro_file.fullname,',');

profile = file_cont(:,2);
%start profile at zero amplitude and zero slope
flat_ind = 1;
if sign(diff(profile(1:2)))>0
    flat_ind = find(diff(profile)<=0,1,'first');
else
    flat_ind = find(diff(profile)>=0,1,'first');
end
profile = profile(flat_ind:end)-profile(flat_ind);

%% Build vehicle-bridge state space model
% initiate vehicle-bridge object
vbs = VB();
vbs.profile = profile; 
vbs.dist = file_cont(flat_ind:end,1);

% populate dependent parameters
% bridge mass
vbs.mb = mb*386.09;
% bridge parameters
vbs.kb = kb;

%calculate bridge damping coefficient
vbs.cb = db*2*sqrt(vbs.kb*vbs.mb/386.09);  
% vehicle parameters
vbs.kt = kt; % lb/in
vbs.mt = mt*386.09;  % vehicle weight
vbs.ct = dt*2*sqrt(vbs.kt*vbs.mt/386.09); % lb.s/in
%% simulate 
vbs.vel = vel;
yy = vbs.simulate;

%% %% Gather results
bridge_accel = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
truck_accel = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
bridge_disp = yy(:,3);
truck_disp = yy(:,1);
c_force = yy(:,5);

tt = vbs.time;

%% Build vehicle state space model
veh = qcarSDF(); %initiate model object
veh.profile = profile; 
veh.dist = file_cont(flat_ind:end,1);

% populate dependent parameters
% vehicle parameters
veh.vel = vel;
veh.k = kt; % lb/in
veh.ms = mt*386.09;  % vehicle weight
veh.c = dt*2*sqrt(veh.k*veh.ms/386.09); % lb.s/in
%% Run Vehicle SDOF state-space model
y2 = veh.simulate;

% Gather results
truck_accel2 = [0; diff(y2(:,2))/diff(veh.time(1:2))];
c_force2 = y2(:,3); % Record contact force

t2 = veh.time;

%% Build bridge state space model
bridge = SDOF_P(); %initiate model object
bridge.p = c_force; % applied force
bridge.dt = diff(t2(1:2)); % time step

% populate dependent parameters
% bridge parameters
bridge.k = kb; % lb/in
bridge.ms = mb*386.09;  % vehicle weight
bridge.c = db*2*sqrt(bridge.k*bridge.ms/386.09); % lb.s/in

%% Run Bridge SDOF state-space model
y3 = bridge.simulate;

% record bridge displacement
b_disp_2 = y3(:,1); 
t3 = bridge.time;

%% Compare coupled system with uncoupled system results
% compare contact force

%filter out high frequency content
fs = 1000;
forder = 6; % Order of filter function
rip = 0.5; % Pass band ripple
atten_stop = 40; % Stop attenuation in dB
flim = 20; % Frequency pass upper limit
[b,a] = ellip(forder,rip, atten_stop, flim/(fs/2),'low');

cf1_filt = filter(b,a,c_force);
cf2_filt = filter(b,a,c_force2);

figure
plot(tt,cf1_filt)
hold all
plot(t2,cf2_filt)
xlabel('time (sec)');
ylabel('vehicle contact force (lb)');
legend({'coupled'; 'uncoupled'});
% compare bridge force
figure
plot(tt,(bridge_disp-b_disp_2)./max(bridge_disp))
hold all
plot(t3,b_disp_2)
xlabel('time (sec)');
ylabel('bridge displacement (in)');
xlim([10 30])

%% Compare with simply supported bridge model
% test quantities
L = 1200; % bridge length
EI = (wnb*b2.L^2/pi^2)^2*b2.ms/b2.gravity/b2.L;
dt = diff(t2(1:2));
p = 1;
td = L/vel;


b2 = ss_bridge(); 
% populate parameters
b2.L = L; % in
b2.dt = dt;  % time step
b2.EI = (wnb*b2.L^2/pi^2)^2*b2.ms/b2.gravity/b2.L;
b2.ms = mb*386.09;  % vehicle weight
b2.vel = vel;
b2.p = c_force2; % applied force
%% Run Bridge SDOF state-space model
y4 = b2.simulate;

% record bridge displacement
b_disp_3 = y4(:,1); 
t4 = b2.time;
