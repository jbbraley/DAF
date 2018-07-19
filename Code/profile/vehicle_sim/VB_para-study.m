
%% Load Run parameters from file
speed_range = 180:60:2400; % in/sec
t_file = file();
t_file.path = 'C:\Users\John\Projects_Git\DAmp\Modeling\Simple\state-space';
t_file.name = 'trucks.csv';
t_params = dlmread(t_file.fullname,',');
b_file = file();
b_file.path = t_file.path;
b_file.name = 'bridges.csv';
b_params = dlmread(b_file.fullname,',');

kt = t_params(:,2); % suspension stiffness for vehicle
kb = b_params(:,2); % stiffness of bridge (idealized as sprung mass)
ct = t_params(:,3); % damping ratio of truck
cb = b_params(:,3); % damping ratio of bridge
mt = t_params(:,1); % mass of vehicle
mb = b_params(:,1); % effective mass of bridge slinch

%% Simulates a 2-DOF system subjected to a given profile
% initiate vehicle-bridge object
vbs = VB();

%% Load profile
pro_file = file();
prof_wav = 360; %in wavelength
pro_file.name = 'pure-360in_0.5in.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic2';
file_cont = dlmread(pro_file.fullname,',');

profile = file_cont(:,2);
%start profile at zero amplitude and zero slope
flat_ind = 1;
if sign(diff(profile(1:2)))>0
    flat_ind = find(diff(profile)<=0,1,'first');
else
    flat_ind = find(diff(profile)>=0,1,'first');
end
profile = profile(flat_ind:end)-profile(flat_ind);


vbs.profile = profile; 
vbs.dist = file_cont(flat_ind:end,1);
prof_amplitude = max(vbs.profile)-mean(vbs.profile);
prof_k = prof_amplitude*600^2;

for bridge = 1:length(kb)
    % bridge parameters
    vbs.kb = kb(bridge);
    vbs.mb = mb(bridge)*386.09;
    vbs.cb = cb(bridge); %*2*sqrt(vbs.kb*vbs.mb);    
    for vehicle = 1:length(kt)
        % vehicle parameters
        vbs.kt = kt(vehicle); % lb/in
        vbs.mt = mt(vehicle)*386.09;  % vehicle weight
        vbs.ct = ct(vehicle); %*2*sqrt(vbs.kt*vbs.mt);            % lb.s/in
        for speed = 1:length(speed_range)
            vbs.vel = speed_range(speed);        % vehicle velocity (in/s)
            elev_factor = prof_k/(vbs.vel)^2/prof_amplitude;
            vbs.profile = profile*elev_factor;
            %% simulate for each speed
            yy = vbs.simulate;

            %% %% Gather results
            bridge_accel(:,speed,vehicle,bridge) = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
            truck_accel(:,speed,vehicle,bridge) = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
            bridge_disp(:,speed,vehicle,bridge) = yy(:,3);
            truck_disp(:,speed,vehicle,bridge) = yy(:,1);
            % response length
            r_length = 720; %in
%             b_disp_transient(speed,vehicle,bridge) = max(yy(1:(r_length/speed_range(speed)/diff(vbs.time(1:2))),3),[],1);
%             max_theo_accel = prof_amplitude*(speed_range(speed)*2*pi/prof_wav)^2; % profile acceleration
%             max_amp(speed,vehicle)  = max_accel(speed,vehicle)/max_theo_accel; % vehicle dynamic amplification
            tt(:,speed) = vbs.time;
        end
    end
end

%% Examine maximum values

b_accel_max = permute(max(bridge_accel,[],1),[2 3 4 1]);
t_accel_max = permute(max(truck_accel,[],1),[2 3 4 1]);
b_disp_max = permute(max(bridge_disp,[],1),[2 3 4 1]);
t_disp_max = permute(max(truck_disp,[],1),[2 3 4 1]);
freq = speed_range/(prof_wav);


figure
y_range = [0.1 0.4 1 1.5 2 ];
for ii=1:size(b_accel_max,2)
    for jj = 1:size(b_accel_max,3)
        plot_num = (ii-1)*size(b_accel_max,3)+jj;
        subplot(size(b_accel_max,2),size(b_accel_max,3),plot_num)
        yyaxis left
        plot(freq,(t_accel_max(:,ii,jj)./((prof_k./speed_range'.^2).*(speed_range'*2*pi/prof_wav).^2)));
        ylim([0 6])
        %ylabel('vehicle acceleration (in/sec^2)')
        yyaxis right
        plot(freq,b_disp_max(:,ii,jj)/(mb(jj)*386.06/kb(jj)));
        %ylabel('bridge acceleration (in/sec^2)')
        %xlabel('profile forcing freqency (Hz)');
        ylim([0 0.4]);
        xlim([0 7]);
        title(['bridge-' num2str(jj) 'Hz | vehicle-' num2str(ii) 'Hz'])
    end
end


