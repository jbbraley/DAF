%% computes errors between FEM and state space predictions

% 
dat1 = []; % LUSAS

%% state-space
vb_sgl = sgl_bridge_vehicle();

% parameters:
vb_sgl.L = 100*12;
vb_sgl.EI = 15e2*5e9; % 576000*700*1000*12^2;
vb_sgl.mb = 10*.099286*1200*386.09; %46e4;
vb_sgl.vel = 720;
vb_sgl.bridge_start = 100*12; 

vb_sgl.mt = 100*386.09; %lb
vb_sgl.kt = 63.1655e3; % lb/in 
vb_sgl.ct = 0.1*2*sqrt(vb_sgl.kt*vb_sgl.mt/vb_sgl.gravity);

nat_freq = sqrt(vb_sgl.EI*pi^2/(4*(vb_sgl.mb/vb_sgl.gravity)*vb_sgl.L^3));


% load  profile
pro_file = file();

pro_file.name = 'ISO_C10-300e-06_w-2_1.csv';
pro_file.path = 'C:\Users\John B\Projects_Git\Damp\profiles\artificial\model_validation';
file_cont = dlmread(pro_file.fullname,',');

vb_sgl.dist = file_cont(:,1);
vb_sgl.profile = file_cont(:,2)-file_cont(1,2);
yy = vb_sgl.simulate();

dat2 = [vb_sgl.time yy(:,1)];

%% differences
figure
plot(dat1(:,1),dat1(:,2))
hold all
plot(dat2(:,1),dat2(:,2))

start_ind = find(dat1(:,1)>=1200/720,1,'first')
dat2_dec = interp1(dat2(:,1)-dat2(1,1),dat2(:,2),dat1(:,1));
diff_dat=abs(dat2_dec-dat1(:,2));
mae_dat = sum(diff_dat(start_ind:end))/length(diff_dat(start_ind:end));
mae_perc = mae_dat/abs(min(dat1(:,2)));
max_diff = min(dat2(:,2))-min(dat1(:,2));
max_diff_perc = abs(max_diff)/abs(min(dat1(:,2)));
fh = plotter('thesis_large')
plot(dat1(:,1),100*diff_dat/abs(min(dat1(:,2))))
xlim([dat1(start_ind,1) max(dat1(:,1))]);
fh.xlabel = 'time (sec)';
fh.ylabel = 'Percent Difference';
ytickformat(fh.ah, 'percentage');
fh.refresh

dat_copy = [min(dat2(:,2)) min(dat1(:,2)) max_diff_perc mae_dat mae_perc];

%% 2-span

%% Initiate
vb_dbl = dbl_bridge_vehicle();
vb_dbl = vb_sgl.clone(vb_dbl);
vb_dbl.x0 = [];

y2 = vb_dbl.simulate();

dat1 = [];
dat2 = [vb_dbl.time y2(:,1)];

%% differences
figure
plot(dat1(:,1),dat1(:,2))
hold all
plot(dat2(:,1),dat2(:,2))

start_ind = find(dat1(:,1)>=1200/720,1,'first')
dat2_dec = interp1(dat2(:,1)-dat2(1,1),dat2(:,2),dat1(:,1));
diff_dat=abs(dat2_dec-dat1(:,2));
mae_dat = sum(diff_dat(start_ind:end))/length(diff_dat(start_ind:end));
mae_perc = mae_dat/abs(min(dat1(:,2)));
max_diff = min(dat2(:,2))-min(dat1(:,2));
max_diff_perc = abs(max_diff)/abs(min(dat1(:,2)));
fh = plotter('thesis_large')
plot(dat1(:,1),100*diff_dat/abs(min(dat1(:,2))))
xlim([dat1(start_ind,1) max(dat1(:,1))]);
fh.xlabel = 'time (sec)';
fh.ylabel = 'Percent Difference';
ytickformat(fh.ah, 'percentage');
fh.refresh

dat_copy = [min(dat2(:,2)) min(dat1(:,2)) max_diff_perc mae_dat mae_perc];

%% 1 mode and 5 modes FEM
dat1 = [];
dat2 = [];

figure
plot(dat1(:,1),dat1(:,2))
hold all
plot(dat2(:,1),dat2(:,2))

start_ind = find(dat1(:,1)>=1200/720,1,'first')
diff_dat=abs(dat2(:,2)-dat1(:,2));
mae_dat = sum(diff_dat(start_ind:end))/length(diff_dat(start_ind:end));
mae_perc = mae_dat/abs(min(dat1(:,2)));
max_diff = min(dat2(:,2))-min(dat1(:,2));
max_diff_perc = abs(max_diff)/abs(min(dat1(:,2)));
fh = plotter('thesis_large')
plot(dat1(:,1),100*diff_dat/abs(min(dat1(:,2))))
xlim([dat1(start_ind,1) max(dat1(:,1))]);
fh.xlabel = 'time (sec)';
fh.ylabel = 'Percent Difference';
ytickformat(fh.ah, 'percentage');
fh.refresh

dat_copy = [min(dat2(:,2)) min(dat1(:,2)) max_diff_perc mae_dat mae_perc];

