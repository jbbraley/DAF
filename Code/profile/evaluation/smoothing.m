%% smooth real profile with rolling straight-edge requirements
% straightedge specifications
s_length = 10:24; % ft
s_dev = 0.125:0.125:0.5; % in

% directory to save filtered profiles
save_file = file();
save_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\artificial\straightedge';

%% Load profile
pro_file = file();

pro_file.name = 'I76-span3.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\I76';
file_cont = dlmread(pro_file.fullname,',');

dist = file_cont(:,1);
profile = file_cont(:,2);

% perform cursory filter and plot
[prof_filt exitf] = straightedge_filter(profile,dist,s_length(1)*12,s_dev(1));
figure
plot(dist/12,profile)
hold all
plot(dist/12,prof_filt)
xlabel('Distance (ft.)')
ylabel('Elevation (in.)')
legend({'Profile as measured'; 'Smoothed to 1/8" over 10ft.'})

%Define state space model parameters
vb = sgl_bridge_vehicle();
vb.L = 140*12;
vb.EI = 2.196E+13; % 576000*700*1000*12^2;
vb.mb = 2641*386.09; %46e4;
vb.vel = 720;
vb.bridge_start = 0; 

vb.mt = 200*386.09; %lb
vb.kt = 49846; % lb/in 
vb.ct = 631.48;

vb.dist = dist;
vb.profile = profile;

% get responses for unmodified profile
y0 = vb.simulate;

%% Filter profile
% loop through lengths and deviations
for ii = 1:length(s_length)
    for jj = 1:length(s_dev)
        %filter
        [prof2 exitf] = straightedge_filter(profile,dist,s_length(ii)*12,s_dev(jj));
        % save
        save_file.name = ['rse_' num2str(s_dev(jj),3) 'in-over' num2str(s_length(ii),2) 'ft.txt'];
%         dlmwrite(save_file.fullname, [dist prof2], ',');
        % simulate with SS model
        vb.profile = prof2;
        yy(:,:,ii,jj) = vb.simulate();
    end
end

% plot amplification
damp_raw = min(y0(:,1))/-vb.max_deflection;
amp = permute(min(yy(:,1,:,:),[],1),[3 4 1 2])/-vb.max_deflection;
amp_red = (damp_raw-amp)/damp_raw;
figure
plot(s_length,amp)
legend(num2str(s_dev',3))