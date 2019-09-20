% Test getIRI function

%% Load profile
lane = 1;
dist_ind = 2*lane-1;
elev_ind = 2*lane;

pro_file = file();
pro_file.name = 'EB_right_1.txt';
pro_file.path = 'F:\research_backup\I76\Profiles\measured\profiles';
data = dlmread(pro_file.fullname,'\t',1,0);
profile = data(:,elev_ind);
dist = data(:,dist_ind)*12;

[IRI,yy] = getIRI(profile,dist,528*12);
% convert to in/mi
IRI = IRI*63360;

[IRI2,yy2] = getIRI(profile*2,dist,528*12);
IRI2 = IRI2*63360;

diff([yy(:,1) yy2(:,1)],1,2)
%% look at natural frequencies of golden car
m1 = 19841.6;
m2 =  2976.241;
k1 = 224.8089431/39.37007874*569.7;
k2 = 224.8089431/39.37007874*5877;
freqs = wn_2dof(m1/386.09, m2/386.09, k1, k2);


