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
vb_sgl2 = vb_sgl.clone(vb_sgl);
vb_sgl2.dt = vb_sgl2.dt/2;

vb_sgl3 = vb_sgl.clone(vb_sgl);
vb_sgl3.dt = vb_sgl3.dt/4;

y2 = vb_dbl.simulate;
y1 = vb_sgl.simulate;
y12 = vb_sgl2.simulate;

y13 = vb_sgl3.simulate;

%% Visualize Single Span
figure
plot(vb_sgl.time(vb_sgl.bridge_inds(1):vb_sgl.bridge_inds(2)),y1(vb_sgl.bridge_inds(1):vb_sgl.bridge_inds(2),1))
hold all
plot(vb_sgl2.time(vb_sgl2.bridge_inds(1):vb_sgl2.bridge_inds(2)),y12(vb_sgl2.bridge_inds(1):vb_sgl2.bridge_inds(2),1));
plot(vb_sgl3.time,y13(:,1));

%% mean absolute error
y12_dec = interp1(vb_sgl2.time,y12(:,1),vb_sgl.time);
diff12=abs(y12_dec(1:end-1)-y1((1:end-1),1));
mae1 = sum(diff12(vb_sgl.bridge_inds(1):vb_sgl.bridge_inds(2)))/(numel(y1(vb_sgl.bridge_inds(1):vb_sgl.bridge_inds(2),1))-1)
mae1_perc = mae1/abs(min(y12(vb_sgl2.bridge_inds(1):vb_sgl2.bridge_inds(2),1)))
mae1_max_perc = max(diff12(vb_sgl.bridge_inds(1):vb_sgl.bridge_inds(2)))/abs(min(y12(vb_sgl2.bridge_inds(1):vb_sgl2.bridge_inds(2),1)));
figure
plot(vb_sgl.time(1:end-1),diff12/abs(min(y12(:,1))))

%% loop through a bunch of time step sizes
steps = .00005*(1:10);
for ii = 1:10
    vb_study = vb_sgl.clone(vb_sgl);
    vb_study.dt = steps(ii);
    y_study = vb_study.simulate();
    ys_dec(:,ii) = interp1(vb_study.time,y_study(:,1),vb_sgl.time);
end
figure
plot(vb_sgl.time,ys_dec);
figure
plot(steps,min(ys_dec,[],1));

%% LUSAS time steps
dat1 = [];
dat2 = [];
start_ind = find(dat2(:,1)>=1200/720,1,'first')
dat1_dec = interp1(dat1(:,1),dat1(:,2),dat2(:,1));
diff_lusas=abs(dat1_dec-dat2(:,2));
mae_lusas = sum(diff_lusas(start_ind:end))/length(diff_lusas(start_ind:end))
mae_lusas_perc = mae_lusas/abs(min(dat1(:,2)))
max_diff = max(diff_lusas);
max_diff_perc = max(diff_lusas)/abs(min(dat1(:,2)));
fh = plotter('thesis_large')
plot(dat2(:,1),100*diff_lusas/abs(min(dat1(:,2))))
xlim([dat2(start_ind,1) max(dat1(:,1))]);
fh.xlabel = 'time (sec)';
fh.ylabel = 'Percent Difference';
ytickformat(fh.ah, 'percentage');
fh.refresh


dat_copy = [max_diff max_diff_perc mae_lusas mae_lusas_perc];

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
