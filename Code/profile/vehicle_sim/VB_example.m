
%% Run parameters
speed = 720; % in/sec
kt = 35530; % suspension stiffness for vehicle
kb = 710611; % stiffness of bridge (idealized as sprung mass)
dt = 0.10; % damping ratio of truck
db = 0.01; % damping ratio of bridge
mt = 100; % mass of vehicle
mb = 2000; % effective mass of bridge slinch

%% Simulates a 2-DOF system subjected to a given profile
% initiate vehicle-bridge object
vbs = VB();

%% Load profile
pro_file = file();
prof_wav = 240; %in wavelength
pro_file.name = 'pure-240in_0.5in.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic2';
file_cont = dlmread(pro_file.fullname,',');

profile = file_cont(:,2);
%start profile at zero
if profile(1)~=0
    if sign(profile(1))>0
        start_ind = find(profile<0,1,'first');
    else
        start_ind = find(profile>0,1,'first');
    end
    profile = vertcat(0,profile(start_ind:end));
else
    start_ind = 2;
end

vbs.profile = profile; 
vbs.dist = file_cont(start_ind-1:end,1);
prof_amplitude = max(vbs.profile);

%% vehicle  and bridge parameters
vbs.kb = kb;
vbs.cb = db*2*sqrt(kb*mb);
vbs.mb = mb*386.09;
vbs.kt = kt; % lb/in
vbs.ct = dt*2*sqrt(kt*mt);            % lb.s/in
vbs.mt = mt*386.09;  % vehicle weight (lb)

vbs.vel = speed;        % vehicle velocity (in/s)
%% simulate
yy = vbs.simulate;

%% Gather results
t_disp = yy(:,1);
t_accel = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
b_disp = yy(:,3);
b_accel = [0; diff(yy(:,4))/diff(vbs.time(1:2))];

%% plot system motion
truck_pos = [5 8];
truck_dim = [3 2];
bridge_pos = [5 3];
bridge_dim = [6 2];
road_dim = [1 1];

figure
ah = axes;
th = rectangle('Position',[truck_pos-truck_dim/2 truck_dim]);
bh = rectangle('Position',[bridge_pos-bridge_dim/2 bridge_dim]);
rh = rectangle('Position',[bridge_pos(1)-road_dim(1)/2 bridge_pos(2)+bridge_dim(2)/2 road_dim(1) road_dim(2)]);
lth = line([truck_pos(1) truck_pos(1)],[bridge_pos(2)+bridge_dim(2)/2+road_dim(2) truck_pos(2)-truck_dim(2)/2]);
lbh = line([truck_pos(1) truck_pos(1)],[0 bridge_pos(2)-bridge_dim(2)/2]);
% rh = line([bridge_pos(1)-bridge_dim(1)/2 bridge_pos(1)+bridge_dim(1)/2],[road_pos road_pos]);
axis([0 10 0 10])


% animate displacement time history
% pull shape positions
truck_elev = th.Position(2);
bridge_elev = bh.Position(2);
for ii = 1:length(vbs.time)/6
    th.Position(2) = truck_elev+t_disp(ii);
    bh.Position(2) = bridge_elev+b_disp(ii);
    rh.Position(2) = bridge_elev+bridge_dim(2)+b_disp(ii);
    rh.Position(4) = road_dim(2)+profile(ii);   
    lth.YData = [bridge_elev+bridge_dim(2)+b_disp(ii)+road_dim(2)+profile(ii) truck_elev+t_disp(ii)];
    lbh.YData(2) = bridge_elev+b_disp(ii);
    drawnow
    pause(2*diff(vbs.time(1:2)))
end

figure
plot(vbs.time,[t_disp b_disp]);
%% compare with just truck
truck = qcarSDF();
truck.k = kt; % lb/in
truck.c = dt*2*sqrt(kt*mt);            % lb.s/in
truck.ms = mt*386.09;  % vehicle weight (lb)
truck.profile = profile; 
truck.dist = file_cont(start_ind-1:end,1);
truck.vel = speed;

vy = truck.simulate;
vehicle_accel = [0; diff(vy(:,2))/diff(truck.time(1:2))];

figure
plot(vbs.time,[t_accel vehicle_accel]);
figure
plot(vbs.time,[t_disp vy(:,1)]);

