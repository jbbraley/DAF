
iter = 3; % iteration number for filename purposes
%% load profile
pro_file_name = 'EB_Right_1_R.csv';

pro_file = file();
pro_file.name =pro_file_name;
pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right';

prof_raw = dlmread(pro_file.fullname, ',');

dist_ind = 1;
elev_ind = 2;
dist = prof_raw(:,dist_ind);
elev = prof_raw(:,elev_ind);

%% start and end indices
prof_length = 650*12; %inches
bridge_start = 500*12; %inches
approach = 320*12; %inches
bridge_ind = find(dist>=bridge_start,1,'first');
start_ind = find(dist>=(bridge_start-approach),1,'first');
end_ind = find(dist>=(dist(start_ind)+prof_length),1,'first');

dist_new = dist(start_ind:end_ind)-dist(start_ind);
elev_new = elev(start_ind:end_ind)-elev(start_ind);

%% save clipped profile
save_file = file();
save_file.name = ['real_' num2str(iter) '.csv'];
save_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\model_validation';
prof_dist = dist_new; % in.
prof_elev = elev_new; % in.

dlmwrite(save_file.fullname, [prof_dist prof_elev],'delimiter', ',', 'precision', 5);


