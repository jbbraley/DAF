%% set up Run parameters
prof_amp = 0.2;
fn = 3; % natural frequency of profile
mb = 2000; % mass of bridge (slinch)
mt = 200; % mass of vehicle (slinch)
db = .02; %:0.01:0.03; % bridge damping ratio
dt = 0.2; % vehicle damping ratio
freq_factor = 1.0; %linspace(0.8,1,5); % factor to be applied to the specified profile frequency
wn = fn*2*pi;

%% Load profile
pro_file = file();
prof_wav = 360; %in wavelength
pro_file.name = 'pure-360in_0.5in.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic2';
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

% initiate vehicle-bridge object
vbs = VB();
vbs.profile = profile; 
vbs.dist = file_cont(flat_ind:end,1);
prof_amplitude = max(vbs.profile)-mean(vbs.profile);

%% populate dependent parameters
kt = (wn*1)'.^2.*mt; % suspension stiffness for vehicle
kb = (wn)'.^2.*mb; % stiffness of bridge (idealized as sprung mass)


elev_factor = prof_amp/prof_amplitude;
vbs.profile = profile*elev_factor;
% bridge mass
vbs.mb = mb*386.09;
% bridge parameters
vbs.kb = kb;

%calculate bridge damping coefficient
vbs.cb = db*2*sqrt(vbs.kb*vbs.mb/386.09);  
% vehicle parameters
vbs.kt = kt; % lb/in
vbs.mt = mt*386.09;  % vehicle weight
freq_range = fn*freq_factor; %
vbs.ct = dt*2*sqrt(vbs.kt*vbs.mt/386.09); % lb.s/in
%% simulate 
vbs.vel = prof_wav*freq_range;
yy = vbs.simulate;

%% %% Gather results
bridge_accel = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
truck_accel = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
bridge_disp = yy(:,3);
truck_disp = yy(:,1);
c_force = yy(:,5);

tt = vbs.time;

%% Run again with very stiff bridge
% set bridge stiffness
vbs.kb = kb*1000;
%re-calculate bridge damping coefficient
vbs.cb = db*2*sqrt(vbs.kb*vbs.mb/386.09);  

% Simulate
y2 = vbs.simulate;

%% %% Gather results
bridge_accel2 = [0; diff(y2(:,4))/diff(vbs.time(1:2))];
truck_accel2 = [0; diff(y2(:,2))/diff(vbs.time(1:2))];
bridge_disp2 = y2(:,3);
truck_disp2 = y2(:,1);
c_force2 = y2(:,5);

t2 = vbs.time;

% Compare contact force time histories
figure
plot(tt,c_force)
hold all
plot(t2,c_force2)
xlim([0 7])
ylim([-5 5]*10^4)
xlabel('time (sec)')
ylabel('Contact Force (lb)')
legend({'over bridge'; 'over rigid-road'})

% compare bridge force time-histories (kb*bridge_disp)
figure
plot(tt,bridge_disp*kb)
hold all
plot(t2,bridge_disp2*kb*1000)
xlim([0 7])
ylim([-2.5 2.5]*10^5)
xlabel('time (sec)')
ylabel('Bridge Force (lb)')
legend({'over bridge'; 'over rigid-road'})

% numerical comparison
mcf(1) = max(c_force);
mcf(2) = max(c_force2);
picf = (mcf(1)-mcf(2))/mcf(2);
mbf(1) = max(bridge_disp*kb);
mbf(2) = max(bridge_disp2*kb*1000);
pibf = (mbf(1)-mbf(2))/mbf(2);

% %% plot system motion
% truck_pos = [5 8];
% truck_dim = [3 2];
% bridge_pos = [5 3];
% bridge_dim = [6 2];
% road_dim = [1 1];
% 
% figure
% ah = axes;
% th = rectangle('Position',[truck_pos-truck_dim/2 truck_dim]);
% bh = rectangle('Position',[bridge_pos-bridge_dim/2 bridge_dim]);
% rh = rectangle('Position',[bridge_pos(1)-road_dim(1)/2 bridge_pos(2)+bridge_dim(2)/2 road_dim(1) road_dim(2)]);
% lth = line([truck_pos(1) truck_pos(1)],[bridge_pos(2)+bridge_dim(2)/2+road_dim(2) truck_pos(2)-truck_dim(2)/2]);
% lbh = line([truck_pos(1) truck_pos(1)],[0 bridge_pos(2)-bridge_dim(2)/2]);
% % rh = line([bridge_pos(1)-bridge_dim(1)/2 bridge_pos(1)+bridge_dim(1)/2],[road_pos road_pos]);
% axis([0 10 0 10])
% 
% 
% % animate displacement time history
% % pull shape positions
% truck_elev = th.Position(2);
% bridge_elev = bh.Position(2);
% for ii = 1:length(vbs.time)/6
%     th.Position(2) = truck_elev+t_disp(ii);
%     bh.Position(2) = bridge_elev+b_disp(ii);
%     rh.Position(2) = bridge_elev+bridge_dim(2)+b_disp(ii);
%     rh.Position(4) = road_dim(2)+profile(ii);   
%     lth.YData = [bridge_elev+bridge_dim(2)+b_disp(ii)+road_dim(2)+profile(ii) truck_elev+t_disp(ii)];
%     lbh.YData(2) = bridge_elev+b_disp(ii);
%     drawnow
%     pause(2*diff(vbs.time(1:2)))
% end
% 
% figure
% plot(vbs.time,[t_disp b_disp]);

%% Filter some ST7 results
% copy and paste in data
dat = [];

%filter out high frequency content
fs = 1000;
forder = 6; % Order of filter function
rip = 0.5; % Pass band ripple
atten_stop = 40; % Stop attenuation in dB
flim = 10; % Frequency pass upper limit
[b,a] = ellip(forder,rip, atten_stop, flim/(fs/2),'low');

dat_filt = filter(b,a,dat);

