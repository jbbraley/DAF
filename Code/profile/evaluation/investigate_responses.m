load('responses.mat');

FE_dat = [];

bridge = 5;
vehicle = 1;
lpath = 1;
profile = pp;

temp = famp{bridge}(:,:,lpath,vehicle);
temp = IRI{bridge}(:,:,lpath,vehicle)*63360;

figure
plot(vb_ssmodel.time,responses{bridge,profile}(:,1,lpath,vehicle))
hold all
plot(FE_dat(:,1)+vb_ssmodel.time(1),FE_dat(:,2))

figure
plot(vb_ssmodel.time,-1*responses{bridge,profile}(:,1,lpath,vehicle))
hold all
plot(FE_dat(:,1)+vb_ssmodel.time(1),FE_dat(:,3))


%% recompute displacement and amplification


disp{bridge}(profile,path,vehicle)