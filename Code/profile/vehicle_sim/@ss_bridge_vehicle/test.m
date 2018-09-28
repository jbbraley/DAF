%% for the testing and benchmarking of this class

% compare with model for moving point load first

%% Initiate
vb_ss = ss_bridge_vehicle();


% parameters:
vb_ss.L = 200;
vb_ss.EI = 576000*700;
vb_ss.vel = 80.67;
vb_ss.kt = 1000; % kip/ft 
vb_ss.mt = 1;
vb_ss.mb = 11*vb_ss.L;
vb_ss.gravity = 32.174;
vb_ss.ct = 2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);

% generate flat profile
dt = 0.001;
vb_ss.dist = 0:dt*vb_ss.vel:vb_ss.L;
vb_ss.profile = zeros(length(vb_ss.dist),1);
% make vehicle pre-displaced by self-weight
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;
yy = vb_ss.simulate;


truth = (-sin(1.267*vb_ss.time)+1.267/8.477*sin(8.477*vb_ss.time))/2400;

figure
plot(vb_ss.time,[yy(:,1) truth'])

%% Benchmark with profile
% make profile
wav = 30; %ft
vb_ss.profile = 0.02*(cos(2*pi/wav*vb_ss.dist)-1)';
% save profile to disk
dlmwrite('30ft_harmonic.csv',[vb_ss.dist' vb_ss.profile]);
vb_ss.mt = 100; %kip
vb_ss.kt = 10; % kip/ft 
vb_ss.ct = 2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.x0(4) = -vb_ss.mt/vb_ss.kt;

DL_disp = 5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

yy = vb_ss.simulate;
figure
plot(vb_ss.time,yy(:,1))

dat = [];
tt = [];

% bridge displacement
figure
plot(vb_ss.time,yy(:,1))
hold all
plot(dat(:,1),dat(:,2))
xlabel('time sec')
ylabel('bridge displacement (ft)')

% vehicle contact point force
figure
plot(vb_ss.time,yy(:,5))
hold all
plot(dat(:,1),dat(:,3))
xlabel('time (sec)')
ylabel('contact force (kip)')
ylim([95 105])

% bridge displacement with multiple modes included
figure
plot(vb_ss.time,yy(:,1))
hold all
plot(dat2(:,1),dat2(:,2))
xlabel('time (sec)')
ylabel('bridge displacement (ft)')

% vehicle displacement
figure
plot(vb_ss.time,yy(:,3)-vb_ss.x0(4))
hold all
plot(dat2(:,1),dat2(:,3))
xlabel('time (sec)')
ylabel('vehicle displacement (ft)')


%% try with lower vehicle damping
vb_ss.ct = 3;
y2 = vb_ss.simulate;
dat3 = [];

% bridge displacement
figure
plot(vb_ss.time,y2(:,1))
hold all
plot(dat3(:,1),dat3(:,2))
