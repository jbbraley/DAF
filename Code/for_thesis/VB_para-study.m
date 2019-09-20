
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
LL_disp = 2*mt(1)*vbs.gravity*span.^3./(pi^4*EI);
DL_disp = 5*vbs.mb*vbs.L^3./(384*EI);

% 4 - 4x4 plots
fh{1} = plotter('thesis_large');
y_range1 = [2.5 4 7 12 18];
y_range2 = [1.5 3 4 6 10];
row_start = 4;
column_start = 4;
for ii=row_start:(row_start+1)
    for jj = column_start:(column_start+1)
        plot_num = (ii-row_start)*(2)+(jj-column_start+1);
        subplot(2,2,plot_num)
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

% one big plot
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
LLdisp = -vbs.max_deflection; %mt(2)*vbs.gravity*span(2).^3./(48*EI(2));


figure
plot(profile_offset, permute(min(yp(:,1,:),[],1),[3 2 1])/LLdisp,'-ok');

xlabel('Profile Offset (ft.)')
ylabel('Bridge Displacement Amplification')
ylim([1 2.4])

% contact force 
vbs.bridge_start = 3840+profile_offset(9)*12
figure
plot(vbs.time,yp(:,5,9))
%% Look at vehicle frequency as a function of beam frequency
% (i.e.) is matching frequencies worst-case scenario?
% initiate vehicle-bridge object
vbs = sgl_bridge_vehicle();
vbs.db = 0.01;

EI = b_params(:,2); % stiffness of bridge (idealized as sprung mass)
mt = 200; % mass of vehicle
mb = b_params(:,1); % effective mass of bridge slinch
span = b_params(:,3);

vbs.vel = 960; %IN/SEC
vbs.bridge_start = 320*12;
freq_fact = [(-0.2:0.02:1) 1.1:0.1:3]+1 ;
% load iso profile
pro_file = file();
pro_file.name = 'ISO_C10-300e-06_w-2_2.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');
vbs.dist = file_cont(:,1);
vbs.profile = file_cont(:,2);

%% Loop through bridges and vehicles
for ii = 1:length(EI)
    % bridge parameters
    vbs.EI = EI(ii);
    vbs.mb = mb(ii)*vbs.gravity;
    vbs.L = span(ii);
    for vehicle = 1:length(freq_fact)
        % vehicle parameters        
        vbs.mt = mt*386.09;  % vehicle weight
        vbs.kt = (2*pi*ii*freq_fact(vehicle))^2*mt; % lb/in (bridge frequency = 1:5)
        vbs.ct = 2*0.1*sqrt(vbs.kt*mt); %*2*sqrt(vbs.kt*vbs.mt); 
        
        vbs.x0 = [];
        yy(:,:,vehicle,ii) = vbs.simulate;
        
        disp_amp = yy(:,1,vehicle,ii)/vbs.max_deflection;

%             %% %% Gather results
%             bridge_accel(:,vehicle,ii) = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
%             truck_accel(:,vehicle,ii) = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
%             bridge_disp(:,vehicle,ii) = yy(:,1);
%             truck_disp(:,vehicle,ii) = yy(:,3);
    end
end
% response length
            tt = vbs.time;
            
%% plot
figure
plot(freq_fact,permute(min(yy(:,1,:,:),[],1),[3 4 1 2]))

b_num = 2
figure
    plot(tt,permute(yy(:,1,:,b_num),[1 3 4 2]))
    
% freq index that causes max displacement
[disp,min_ind] = min(permute(min(yy(:,1,:,:),[],1),[3 4 1 2]),[],1);
freq_fact(min_ind)

fh = plotter('thesis_large')
plot(tt,permute(yy(:,1,[find(freq_fact==1) min_ind(b_num)],b_num),[1 3 2 4]))

fh = plotter('thesis_large')
plot(tt,permute(yy(:,3,find(freq_fact==1),b_num),[1 3 2 4])-yy(1,3,find(freq_fact==1),b_num))
hold all
plot(tt,permute(yy(:,3,min_ind(b_num),b_num),[1 3 2 4])-yy(1,3,min_ind(b_num),b_num))

fh = plotter('thesis_large')
plot(tt,permute(yy(:,5,[find(freq_fact==1) min_ind(b_num)],b_num),[1 3 2 4]))

for ii = 1:length(EI)
    vbs.EI = EI(ii);
    vbs.mb = mb(ii)*vbs.gravity;
    vbs.L = span(ii);
    disp_amp(:,:,ii) = permute(yy(:,1,:,ii),[1 3 4 2])/vbs.max_deflection;
end

fh = plotter('thesis_large')
max_amp=-permute(min(disp_amp,[],1),[2 3 1]);
plot(freq_fact,max_amp(:,2:end),'-','Marker','.')

legend({'bridge1'; 'bridge2';'bridge3';'bridge4'}); %;'bridge5'});
xlim([freq_fact(1) freq_fact(end)])
xlabel('Frequency Factor')
ylabel('Bridge Displacement Amplification')