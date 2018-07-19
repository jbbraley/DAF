
%% Run parameters
speed_range = 300:20:2160; %[500 640 880 1220 1300]; %400:20:1500; % range of speeds for each vehicle simulation
ks = [1.2106E+04 2.4354E+04 9.0220E+03 8.8507E+04 2.8893E+05 1.1706E+06 8.8507E+04]; % suspension stiffness for each vehicle
mass = [119.14 119.14 51.80 119.14 119.14 119.14 119.14]; % mass of each vehicle
cd = [168.11 238.46 95.7 454.53 821.41 1653.09 974.15]; % damping coefficient
kus = ones(1,7)*80000; kus(5:6) = 400000;

%% Simulates a quarter car model over a given profile
% initiate quarter car object
truck = qcar2();

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

truck.profile = profile; 
truck.dist = file_cont(start_ind-1:end,1);
prof_amplitude = max(truck.profile);

%% vehicle parameters

truck.mus = 5.1791*386.09;               % 1/4 unsprung mass (lb)


for vehicle = 1:length(ks)
    truck.k = ks(vehicle); % lb/in
    truck.c = cd(vehicle);            % lb.s/in
    truck.ms = mass(vehicle)*386.09;  % 1/4 sprung mass (lb)
    truck.kus = kus(vehicle);          % tire stiffness (lb/in)
    for speed = 1:length(speed_range)
        truck.vel = speed_range(speed);        % vehicle velocity (in/s)
        %% simulate
        yy = truck.simulate;

        %% Convert to acceleration
        vel = yy(:,2);
%         accel{speed}(:,vehicle) = [0; diff(vel)/diff(truck.time(1:2))];
        max_accel(speed,vehicle) = max(diff(vel)/diff(truck.time(1:2))); %(in/sec^2)
        max_theo_accel = prof_amplitude*(speed_range(speed)*2*pi/prof_wav)^2; % profile acceleration
        max_amp(speed,vehicle)  = max_accel(speed,vehicle)/max_theo_accel; % vehicle dynamic amplification
        tt(:,speed) = truck.time;
    end
end
