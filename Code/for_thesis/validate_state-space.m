%% for the testing and benchmarking of this class

% compare with FE model

%% Initiate
vb_dbl = dbl_bridge_vehicle();
vb_sgl = sgl_bridge_vehicle();

% parameters:
vb_dbl.L = 100*12;
vb_dbl.EI = 15e2*5e9; % 576000*700*1000*12^2;
vb_dbl.mb = 10*.099286*1200*386.09; %46e4;
vb_dbl.vel = 720;
vb_dbl.bridge_start = 100*12; 

vb_dbl.mt = 100*386.09; %lb
vb_dbl.kt = 63.1655e3; % lb/in 
vb_dbl.ct = 0.1*2*sqrt(vb_dbl.kt*vb_dbl.mt/vb_dbl.gravity);

nat_freq = sqrt(vb_dbl.EI*pi^2/(4*(vb_dbl.mb/vb_dbl.gravity)*vb_dbl.L^3));


% load  profile
pro_file = file();

pro_file.name = 'ISO_C10-300e-06_w-2_1.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');

vb_dbl.dist = file_cont(:,1);
vb_dbl.profile = file_cont(:,2)-file_cont(1,2);

vb_sgl = vb_dbl.clone(vb_sgl);
y2 = vb_dbl.simulate;
y1 = vb_sgl.simulate;

%% Visualize Single Span
figure
plot(vb_sgl.time,y1(:,1))

%% Visualize Double span

%% compute amplifications
% DL and LL disp
DL_disp = vb_dbl.DL_disp; % 1/185*vb_dbl.mb*vb_dbl.L^3/(vb_dbl.EI);
LL_disp = vb_dbl.max_deflection % -0.015*vb_dbl.mt*vb_dbl.L^3/vb_dbl.EI; % expected LL disp
LL2 = -vb_dbl.mt*vb_dbl.L^3/(pi^4*vb_dbl.EI).*sin(pi*(0:4800)'/vb_dbl.L); % displacement based on shape function
FE_LL = -.34037;

%plot midspan displacement
disp_fig = plotter('thesis_single');
disp_fig.plot(vb_dbl.time,(yy(:,1)))
ylim([-.4 .3])
xlim([-1.5 8])
hold all
disp_fig.plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,2))
disp_fig.xlabel = 'time (sec)';
disp_fig.ylabel = 'mid-span displacement (in)';
disp_fig.legend = {'state-space model'; 'FE model'};
disp_fig.refresh

% plot dynamic displacement component only
disp_fig = plotter('thesis_single');
disp_fig.plot(vb_dbl.time,(yy(:,1)-yy(:,6)))
ylim([-.04 .04])
xlim([-1.5 8])
hold all
disp_fig.plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,2)-dat(:,5))
disp_fig.xlabel = 'time (sec)';
disp_fig.ylabel = 'mid-span dynamic disp. (in)';
disp_fig.legend = {'state-space model'; 'FE model'};
disp_fig.refresh

figure
plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,((yy(1:length(dat),1)-yy(1:length(dat),6)+dat(:,5))))
hold all
plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,2))

%plot amplification (deflection/max static deflection)
amp_fig = plotter('thesis_single');
amp_fig.plot(vb_dbl.time,(yy(:,1))./min(LL2))
hold all
amp_fig.plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,2)./FE_LL)
ylim([-1.2 1.2])
xlim([-1.5 8])
amp_fig.xlabel = 'time (sec)';
amp_fig.ylabel = 'mid-span disp. amplification';
amp_fig.legend = {'state-space model'; 'FE model'};
amp_fig.refresh


% figure
% plot(vb_dbl.time,yy(:,1)-yy(:,6))
% hold all
% plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,2)./FE_LL)


%plot vehicle motion
figure
plot(vb_dbl.time,yy(:,3)-yy(1,3))
hold all
plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,3))

% plot contact force
cf_fig = plotter('thesis_single');
cf_fig.plot(vb_dbl.time,yy(:,5))
hold all
cf_fig.plot(dat(:,1)-vb_dbl.bridge_start/vb_dbl.vel,dat(:,4))
xlim([-1.5 8])
cf_fig.xlabel = 'time (sec)';
cf_fig.ylabel = 'contact force (lb)';
cf_fig.legend = {'state-space model'; 'FE model'};
cf_fig.refresh
