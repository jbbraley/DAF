
%% Simulates a quarter car model over a given profile
% initiate quarter car object
truck = qcar();

%% vehicle parameters
% Spring stiffness
truck.k = 7415253;             % N/m
truck.c = 118017.3;            % N.s/m
truck.ms = 20865;              % 1/4 sprung mass (kg)
truck.mus = 907;               % 1/4 unsprung mass (kg)
truck.kus = 14010160;          % tire stiffness (N/m)
truck.vel = 600*0.0254;        % vehicle velocity (m/s)

truck2 = qcar2();
truck.fill_empty_class(truck2);

truck2.k = truck.k*0.22481*unitsratio('m','in');             % lb/in
truck2.c = truck.c*0.22481*unitsratio('m','in');            % lb.s/in
truck2.ms = truck.ms*2.20462;              % 1/4 sprung mass (lb)
truck2.mus = truck.mus*2.20462;               % 1/4 unsprung mass (lb)
truck2.kus = truck.kus*0.22481*unitsratio('m','in');          % tire stiffness (lb/in)
truck2.vel = truck.vel*unitsratio('in','m');        % vehicle velocity (in/s)

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
truck2.profile = profile; 
truck2.dist = file_cont(start_ind-1:end,1);
%% simulate


yy = truck.simulate;
y2 = truck2.simulate;

%% Convert to acceleration
vel = yy(:,4);
accel = [0; diff(vel)/diff(truck.time(1:2))]/truck.gravity; %(m/sec^2 to g)

vel2 = y2(:,2);
accel2 = [0; diff(vel2)/diff(truck2.time(1:2))]/truck2.gravity; %(in/sec^2 to g)

figure
plot(truck.time, [accel accel2])

tt = truck.time';