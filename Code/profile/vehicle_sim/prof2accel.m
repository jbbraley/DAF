
%% Simulates a quarter car model over a given profile
% initiate quarter car object
truck = qcar();

%% vehicle parameters
% Spring stiffness
truck.k = 7415253; %k/in to N/m
truck.c = 118017.3; % Viscocity coefficient lbf-s/in to N.s/m
truck.ms = 20865;               % 1/4 sprung mass (kip to kg)
truck.mus = 907;               % 1/4 unsprung mass (kip to kg)
truck.kus = 14010160;          % tire stiffness (k/in to N/m)
truck.vel = 600*0.0254;                % vehicle velocity (in/sec to m/s)

%% Load profile
pro_file = file();
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
truck.dist = file_cont(start_ind-1:end,1)*unitsratio('m','in'); %convert inches to meters
truck.profile = [0; diff(profile)/diff(truck.time(1:2))]*unitsratio('m','in'); %convert inches to meters
%% simulate
yy = truck.simulate;

%% Convert to acceleration
vel = yy(:,4);
accel = [0; diff(vel)/diff(truck.time(1:2))]/truck.gravity; %(m/sec^2 to g)

figure
plot(truck.time, accel)

tt = truck.time';