% Set parameters
speed = 720; % in/sec

%% Load profile
pro_file = file();
pro_file.name = 'EB_right_1_R.csv';
pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right';
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

dist = file_cont(:,1);
time = dist/speed;
dd = diff(dist);
dt = dd/speed;
vel = diff(profile)./dt;
accel = diff(vel)./dt(1:end-1);

figure
plot(time(1:end-2),accel)