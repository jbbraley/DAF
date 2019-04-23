%% Initiate state space models
veh = qcarSDF();

% model parameters
fnv = 2.5; % natural frequency of vehicle
mt = 200; % mass of vehicle (slinch)
dt = 0.10; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)
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
% simulate
yv = veh.simulate;
c_force1 = yv(:,3);

[maxf ind] = max(c_force1);
[maxv indd] = max(yv(:,1))
[maxd indv] = max(yv(:,2))


veh.x0 = yv(indd,[2 1]);
veh.profile = zeros(size(profile));
yv2 = veh.simulate;

fh = plotter('thesis_large');
plot(veh.time,yv2(:,3)/veh.mt)
fh.xlabel = 'time (sec)';
fh.ylabel = 'Contact Force Amplification';
xlim([0 max(veh.time)])
fh.refresh()

peak = findpeaks(yv2(:,3)-veh.mt);
decrement = peak/veh.mt;
fall_off_ind = find(decrement<0.1,1,'first');
time_ind = find(yv2(:,3)==peak(fall_off_ind)+veh.mt);
time2falloff = veh.time(time_ind);
dist2falloff = veh.vel*time2falloff/12;




