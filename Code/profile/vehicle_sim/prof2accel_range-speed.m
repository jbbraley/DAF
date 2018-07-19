
%% Simulates a quarter car model over a given profile
% initiate quarter car object
truck = qcar();

velocity = [60 90 120]*unitsratio('m','ft');                % vehicle velocity (in/sec to m/s)
%% vehicle parameters
% Spring stiffness
truck.k = 29661012; %k/in to N/m
truck.c = 110150; % Viscocity coefficient lbf-s/in to N.s/m
truck.ms = 20865;               % 1/4 sprung mass (kip to kg)
truck.mus = 2*453.592;               % 1/4 unsprung mass (kip to kg)
truck.kus = 80*175.127E3;          % tire stiffness (k/in to N/m)
% truck.vel = 715*0.0254;                % vehicle velocity (in/sec to m/s)
wav = [20 30 40 60];

for jj = 1:length(wav)
    name = num2str(wav(jj));
    %% Load profile
    pro_file = file();
    pro_file.name = ['pure-' name 'ft_0.5in.csv'];
    pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic';
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
    truck.dist = file_cont(start_ind-1:end,1)*0.0254; %convert inches to meters

 clear accel
    for ii = 1:length(velocity)
        %% simulate
        truck.vel = velocity(ii);
        truck.profile = [0; diff(profile)/diff(truck.time(1:2))]*0.0254; %convert inches to meters
        yy = truck.simulate;

        %% Convert to acceleration
        vel = yy(:,4);
        accel(:,ii) = [0; diff(vel)/diff(truck.time(1:2))]/truck.gravity; %(m/sec^2 to g)
        accel_max(ii,jj) = max(accel(:,ii));
    end
end
