
%% Simulates a quarter car model over a given profile
% initiate quarter car object
truck = qcarSDF();

%% vehicle parameters
truck.k = 169368.83;             % lb/in
truck.c = 628.97;            % lb.s/in
truck.ms = 119.142*386.09;              % 1/4 sprung mass (lb)
truck.vel = 600;        % vehicle velocity (in/s)

%% Load profile
pro_file = file();
pro_file.name = 'pure-240in_0.1in.csv';
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

%% simulate
yy = truck.simulate;

%% Convert to acceleration
vel = yy(:,2);
accel = [0; diff(vel)/diff(truck.time(1:2))]/truck.gravity; %(in/sec^2 to g)


% figure
% plot(truck.time, [accel])

acc = accel*truck.gravity; % convert to in/sec2
tt = truck.time;