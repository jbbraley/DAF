%% This script attempts to determine what percentage of ...
% calculated stiffness (EI) and mass should be used 
% for single-line girder analysis of moving dynamic loads

%% load FE data
FE_dat = []; %copypasta

%% choose data window
event_window = [3582 4000];
FE_time = FE_dat(:,1);
FE_disp = FE_dat(:,2);
FE_static = -0.13485;

%% Load profile 
pro_file = file();

pro_file.name = 'ISO_C10-300e-06_w-3_2.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');

% Initialize state-space model
vb_ssmodel = sgl_bridge_vehicle();

% Populate properties
fnb = 9.94; % natural frequency of bridge
span_length = 40; % ft
% Determine stiffness and mass base values
[mb, EI] = bridge2beam(fnb, 587723, span_length*12, 1);
fnv = 10.5; % natural frequency of vehicle
kt = 879292; %lb/in
mt = 200; % mass of vehicle (slinch)
ct = 2652.23; % vehicle damping ratio
vel = 720; % vehicle velocity (in/sec)

start = 3864; %distance (in) to start of bridge


vb_ssmodel.L = span_length*12;
vb_ssmodel.vel = vel;
vb_ssmodel.kt = kt; % suspension stiffness for vehicle
vb_ssmodel.mt = mt*vb_ssmodel.gravity;
vb_ssmodel.mb = mb*vb_ssmodel.gravity;
vb_ssmodel.ct = ct;
vb_ssmodel.EI = EI;
vb_ssmodel.bridge_start = start;

vb_ssmodel.dist =  file_cont(:,1);
vb_ssmodel.profile = file_cont(:,2);


%% interpolate FE displacement values at Matlab time values
FE_disp_interp = interp1(FE_time,FE_disp,vb_ssmodel.time-vb_ssmodel.time(1));
end_ind = find(isnan(FE_disp_interp),1,'first')-1;
if isempty(end_ind)
    end_ind = length(FE_disp_interp);
end
FE_disp_interp = FE_disp_interp(1:end_ind,1);
% clip profile data to match FE results
vb_ssmodel.dist = vb_ssmodel.dist(1:end_ind);
vb_ssmodel.profile = vb_ssmodel.profile(1:end_ind,1);
% store indices of event window
winds(1) = find((vb_ssmodel.time-vb_ssmodel.time(1))>=FE_time(event_window(1)),1,'first');
winds(2) = find((vb_ssmodel.time-vb_ssmodel.time(1))>=FE_time(event_window(2)),1,'first');
vb_ssmodel.window_report = winds;

%% Build model function
% encapsulate state-space model simulation
modelfun = @(ssmodel,X) ss_simulate(ssmodel,X); % X is a vector containing factor values

%% initialize model object that will be used to perform optimization
bridge1 = model();
bridge1.fun = modelfun;
bridge1.y = FE_disp_interp(vb_ssmodel.window_report(1):vb_ssmodel.window_report(2),1);
% store state-space info in X
bridge1.X = vb_ssmodel;
% set up starting values and bounds
bridge1.num_c = 2;
bridge1.lb = [0.2 0.1];
bridge1.ub = [5 5];
bridge1.c_start = [1 1];

% Run initial simulation

yy = vb_ssmodel.simulate;
SS_disp_initial = yy(:,1);
% Run
[factors, resid, exit_flag] = bridge1.fit;
% new displacement prediction
new_ss = clone(vb_ssmodel);
new_ss.mb = factors(1)*vb_ssmodel.mb;
new_ss.EI = factors(2)*vb_ssmodel.EI;
y_final = new_ss.simulate;
SS_disp_final = y_final(:,1);

% compare time histories
figure
plot(vb_ssmodel.time,FE_disp_interp)
hold all
plot(vb_ssmodel.time,SS_disp_initial)
plot(vb_ssmodel.time,SS_disp_final)
ylabel('Displacement (in)')
xlabel('time (sec)')
legend({'3D FE'; 'Prelim state-space'; 'Fit state-space'})

figure
plot(vb_ssmodel.time,FE_disp_interp/FE_static)
hold all
plot(vb_ssmodel.time,SS_disp_initial/min(yy(:,6)))
plot(vb_ssmodel.time,SS_disp_final/min(y_final(:,6)))
ylabel('Displacement (in)')
xlabel('time (sec)')
legend({'3D FE'; 'Prelim state-space'; 'Fit state-space'})


%% Brute force
% run through different stiffness values, calculate mass
bounds = [0.8 5];
steps = 100;
X_k = linspace(bounds(1),bounds(2),steps);

responses = [];
for ii = 1:steps
    [mb, EI] = bridge2beam(fnb, 587723*X_k(ii), span_length*12, 1);
    new_ss.mb = mb*new_ss.gravity;
    new_ss.EI = EI;
    responses(:,:,ii) = new_ss.simulate;
    display(['iteration' num2str(ii)]);
end

[val ind] = max(permute(min(responses(:,1,:),[],1)-min(responses(:,6,:),[],1),[3 1 2])-(min(FE_disp_interp)-FE_static));
 figure
plot(new_ss.time,FE_disp_interp)
hold all
plot(new_ss.time,responses(:,1,95))
xlim([0 1]);    
% amplfication 
% (displacement-quasi-static-displacement)-
fe_slg = []; %copy paste
plot(fe_slg(:,1),fe_slg(:,2))

center_inds = find(new_ss.time>=new_ss.L/new_ss.vel/2,1,'first')+[-1 1]*floor(0.03*new_ss.L/vel/diff(new_ss.time(1:2)));
center_range = center_inds(1):center_inds(2);
plot(new_ss.time(center_inds(1):center_inds(2)),zeros(1,diff(center_inds)+1))

plot(abs(permute(min(responses(center_range,1,:),[],1),[3 1 2])-min(FE_disp_interp(center_range))))
[val2 ind2] = min(abs(permute(min(responses(center_range,1,:),[],1),[3 1 2])-min(FE_disp_interp(center_range))));
fh = figure
% plot(fe_slg(:,1),fe_slg(:,2))
hold all
plot(new_ss.time,FE_disp_interp)
ah = gca;
lh = plot(new_ss.time,permute(responses(:,1,1),[1 3 2]));
xlim([0 1]);    
for ii = 1:steps
    lh.YData = responses(:,1,ii);
    ah.Title.String = num2str(ii);
    drawnow
    pause(.2)
end
plot(c,[-1 1])

ind3 = 77
lh.YData = responses(:,1,ind3);


figure
plot((permute(min(responses(center_range,1,:),[],1)./min(responses(center_range,6,:),[],1),[3 1 2])-...
    min(FE_disp_interp(center_range))/FE_static))
figure
lh = line();
lh.YData = plot(new_ss.time,responses(:,1,5)/min(responses(center_range,6,1),[],1));
hold all
plot(new_ss.time,FE_disp_interp/FE_static)
plot(new_ss.time(center_inds(1):center_inds(2)),zeros(1,diff(center_inds)+1))
xlim([0 0.8])
legend({'State-space'; '3D FE'})


