
%% Load Run parameters from file
speed_range = 180:60:2400; % in/sec
t_file = file();
t_file.path = 'C:\Users\John B\Projects_Git\Damp\Data\for_thesis\vb_param';
t_file.name = 'trucks.csv';
t_params = dlmread(t_file.fullname,',');
b_file = file();
b_file.path = t_file.path;
b_file.name = 'bridges.csv';
b_params = dlmread(b_file.fullname,',');

kt = t_params(:,2); % suspension stiffness for vehicle
EI = b_params(:,2); % stiffness of bridge (idealized as sprung mass)
ct = t_params(:,3); % damping ratio of truck
mt = t_params(:,1); % mass of vehicle
mb = b_params(:,1); % effective mass of bridge slinch
span = b_params(:,3);
%% Simulates a 2-DOF system subjected to a given profile
% initiate vehicle-bridge object
vbs = sgl_bridge_vehicle();
vbs.db = 0.01;

%% Load profile
pro_file = file();
prof_wav = 360; %in wavelength
pro_file.name = 'pure-360in_0.5in.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\DAmp\profiles\artificial\harmonic2';
file_cont = dlmread(pro_file.fullname,',');

profile = file_cont(:,2);
%start profile at zero amplitude and zero slope
flat_ind = 1;
if sign(diff(profile(1:2)))>0
    flat_ind = find(diff(profile)<=0,1,'first');
else
    flat_ind = find(diff(profile)>=0,1,'first');
end
profile = profile(flat_ind:(flat_ind+6000))-profile(flat_ind);


vbs.profile = profile; 
vbs.dist = file_cont(flat_ind:(flat_ind+6000),1);
prof_amplitude = 0.5; %max(vbs.profile)-mean(vbs.profile);
% prof_k = prof_amplitude*600^2;
vbs.bridge_start = 3840;

for bridge = 1:length(EI)
    % bridge parameters
    vbs.EI = EI(bridge);
    vbs.mb = mb(bridge)*vbs.gravity;
    vbs.L = span(bridge);
    for vehicle = 1:length(kt)
        % vehicle parameters
        vbs.kt = kt(vehicle); % lb/in
        vbs.mt = mt(vehicle)*386.09;  % vehicle weight
        vbs.ct = ct(vehicle); %*2*sqrt(vbs.kt*vbs.mt);            % lb.s/in
        for speed = 1:length(speed_range)
            vbs.vel = speed_range(speed);        % vehicle velocity (in/s)
%             elev_factor = prof_k/(vbs.vel)^2/prof_amplitude;
%             vbs.profile = profile*elev_factor;
            %% simulate for each speed
            % reset initial conditions so default is applied
            vbs.x0 = [];
            yy(:,:,speed,vehicle,bridge) = vbs.simulate;

            %% %% Gather results
            bridge_accel(:,speed,vehicle,bridge) = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
            truck_accel(:,speed,vehicle,bridge) = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
            bridge_disp(:,speed,vehicle,bridge) = yy(:,1);
            truck_disp(:,speed,vehicle,bridge) = yy(:,3);
            % response length
            tt(:,speed) = vbs.time;
        end
    end
end

%% Examine maximum values

b_accel_max = permute(max(bridge_accel,[],1),[2 3 4 1]);
t_accel_max = permute(max(truck_accel,[],1),[2 3 4 1]);
b_disp_max = permute(min(bridge_disp,[],1),[2 3 4 1]);
t_disp_max = permute(min(truck_disp,[],1),[2 3 4 1]);

b_disp_max = permute(min(yy(:,1,:,:,:),[],1),[3 4 5 1 2]);
cforce_max = permute(max(yy(:,5,:,:,:),[],1),[3 4 5 1 2]);
freq = speed_range/(prof_wav);

%% compute amplification terms
% profile acceleration
prof_accel = 0.5*(2*pi/prof_wav*speed_range).^2;
LL_disp = mt(1)*vbs.gravity*span.^3./(48*EI);
DL_disp = 5*vbs.mb*vbs.L^3./(384*EI);

figure
y_range1 = [2.5 4 7 12 18];
y_range2 = [1.5 3 4 6 10];
for ii=2:size(b_accel_max,2)
    for jj = 2:size(b_accel_max,3)
        plot_num = (ii-2)*(size(b_accel_max,3)-1)+(jj-1);
        subplot(size(b_accel_max,2)-1,size(b_accel_max,3)-1,plot_num)
        yyaxis right
        plot(freq(7:end),(cforce_max(7:end,ii,jj)./vbs.mt),'-.');
        ylim([1 y_range2(ii)])
        %ylabel('vehicle acceleration (in/sec^2)')
        yyaxis left
        plot(freq(7:end),-b_disp_max(7:end,ii,jj)/LL_disp(jj),'-');
        %ylabel('bridge acceleration (in/sec^2)')
        %xlabel('profile forcing freqency (Hz)');
        ylim([1 y_range1(jj)]);
        xlim([0 7]);
        title(['bridge-' num2str(jj) 'Hz | vehicle-' num2str(ii) 'Hz'])
    end
end

legend({'Bridge Response'; 'Vehicle Response'})

vv = 5;
bb = 5;
figure
for ii = 1:length(speed_range)
    yyaxis left
    plot(vbs.time,yy(:,5,ii,vv,bb)/vbs.mt); %contact force
    yyaxis right
    plot(vbs.time,yy(:,1,ii,vv,bb)/LL_disp(bb)); % bridge_disp
    pause
end

%% look at positioning of profile
% load iso profile
pro_file = file();
pro_file.name = 'ISO_C10-300e-06_w-2_2.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');
vbs.dist = file_cont(:,1);
vbs.profile = file_cont(:,2);

profile_offset = (0:5:100);
vbs.EI = EI(2);
vbs.mb = mb(2)*vbs.gravity;
vbs.L = span(2);
% vehicle parameters
vbs.kt = kt(2); % lb/in
vbs.mt = mt(2)*386.09;  % vehicle weight
vbs.ct = ct(2); %*2*sqrt(vbs.kt*vbs.mt);            % lb.s/in
vbs.vel = 720;     
vbs.x0 = [];

for pos = 1:length(profile_offset)
    vbs.bridge_start = 3840+profile_offset(pos)*12;
    yp(:,:,pos) = vbs.simulate;
end
LLdisp = -mt(2)*vbs.gravity*span(2).^3./(48*EI(2));


figure
plot(profile_offset, permute(min(yp(:,1,:),[],1),[3 2 1])/LLdisp,'-ok');

xlabel('Profile Offset (ft.)')
ylabel('Bridge Displacement Amplification')
ylim([1 2.4])

%% Look at vehicle frequency as a function of beam frequency
% (i.e.) is matching frequencies worst-case scenario?
