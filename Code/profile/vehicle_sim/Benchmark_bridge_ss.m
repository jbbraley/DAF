%% Initiate
vb_ss = ss_bridge_vehicle();

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

%% Load profile
pro_file = file();

pro_file.name = 'EB_right_1_R.csv';
pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right';
file_cont = dlmread(pro_file.fullname,',');

% chop profile into segments equal to bridge length.
dx = mean(diff(file_cont(1:50,1)));
prof_start = round(((500+111)*12+8)/dx);
block_size = round(vb_ss.L/dx);
prof_end = floor((size(file_cont,1)-prof_start+1)/block_size)*block_size+prof_start-1;
block_inds = prof_start:block_size:size(file_cont,1);

profile = reshape(file_cont(prof_start:prof_end,2),block_size,[]);
vb_ss.dist = (1:size(profile,1))*dx;

%start profile at zero amplitude 
profile = profile-profile(1,:);

% make vehicle pre-displaced by self-weight
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;

DL_disp = 5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

%% Simulate
for span = 1:5
    vb_ss.profile = profile(:,span);
    yy(:,:,span) = vb_ss.simulate;
    
    % save profile for use in FE software
    dlmwrite(['C:\Users\John\Projects_Git\DAmp\Code\profile\vehicle_sim\@ss_bridge_vehicle\I76-span' num2str(span+1) '.csv'], [vb_ss.dist' profile(:,span)],',');
end

figure
for ii = 1:size(yy,3)
    plot(vb_ss.time,yy(:,4,ii))
    hold all
end

%% compare with FE
dat1 = [];
% paste data from FE results [time bm_disp veh_accel contact_force]

% beam displacement
figure('Position', [488 342 640 200])
plot(vb_ss.time,yy(:,1,2))
hold all
plot(dat1(:,1),dat1(:,2))
xlabel('time (sec)')
ylabel('beam displacement (in)')
legend({'state-space model'; 'FE model'})
xlim([0 2.3])

% vehicle accel
figure('Position', [488 342 640 200])
plot(vb_ss.time,[0; diff(yy(:,4,2))/vb_ss.dt])
hold all
plot(dat1(:,1),dat1(:,3))
xlabel('time (sec)')
ylabel('vehicle accel (in/sec^2)')
legend({'state-space model'; 'FE model'})
xlim([0 2.3])

% contact force
figure('Position', [488 342 640 200])
plot(vb_ss.time, yy(:,5,2))
hold all
plot(dat1(:,1),dat1(:,4))
xlabel('time (sec)')
ylabel('contact force (lb)')
legend({'state-space model'; 'FE model'})
ylim([0 1.5e5])
xlim([0 2.3])

% error
