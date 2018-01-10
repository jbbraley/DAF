% path with LUSAS resusts textfile output
path = 'C:\Users\John\Desktop\dat';
write_path = 'C:\Users\John\Projects_Git\DAmp\profiles\set1';
write_name = 'LL_disp_1';
dirnames = dir(path); 
dirnames = dirnames(3:end);
% row and column where data point is stored
dat_loc = [2 10];

% read in all the data
for ii = 1:length(dirnames)
    dat(ii) = dlmread([path '\' dirnames(ii).name],'\t',[dat_loc-1 dat_loc-1]);
end

%create distance vector
dist1 = 0:1340/56:1340;
dist2 = 0:24:1680;
dist3 = 0:24:1680*2;
dist = [dist1 dist2+dist1(end)];
dist = [dist dist3+dist(end)];
dist = [dist dist3+dist(end)];
dist = [dist dist3+dist(end)];
dist = [dist dist3+dist(end)];
dist = [dist dist1+dist(end)];

% offset for path beginning
pre_dist = 700;
dist = [0 dist+pre_dist];
dat = [0 dat];

% remove duplicate locations
[dist,rm_ind,~] = unique(dist);
dat = dat(rm_ind);

% save profile to text file
Profile_Table = horzcat(dist',dat');
savename = file();
savename.path = write_path;
savename.name = write_name;
dlmwrite(savename.fullname, Profile_Table);