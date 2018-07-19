%% Generates equivalent force tables to be applied to sprung masses
%% load profile
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

% vbs.dist = file_cont(start_ind-1:end,1);
% prof_amplitude = max(vbs.profile);
%% Vehicle-Bridge idealised system: 2 sprung masses with varying road thickness (i.e. profile)
% global parameters
speed = 720; %in/sec

time = file_cont(start_ind-1:end,1)/speed;
%system parameters
kt = 35530; %vehicle spring stiffness (lb/in)
dt = 0.10; % damping ratio of truck
mt = 100; % vehicle mass (slinch)
ct = dt*2*sqrt(kt*mt); % damping coefficient of vehicle (lb.s./in)

%vehicle equivalent load
tload = kt*(profile)+ct*([0; diff(profile)/diff(time(1:2))]);
bload = -tload;