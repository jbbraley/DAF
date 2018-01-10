% File to write resulting traffic csv file to
putfile = file();
putfile.name = '6in_ramp.csv';
putfile.path = 'C:\Users\John\Projects_Git\DAmp\profiles\set1';


ramp_height = 2; % inch
ramp_start = -35; % feet
ramp_end = 15; % feet
ramp_peak = 0; %feet
path_start = -60; % feet
path_end = 1484; % feet
profile_discretization = 1; % feet

% convert to inches
ramp_end = ramp_end*12;
ramp_start = ramp_start*12;
ramp_peak = ramp_peak*12;
path_start = path_start*12;
path_end = path_end*12;
total_length = (path_end-path_start);
resolution = profile_discretization*12;


% Build traffic csv file
[dist, disp] = build_profile_ramp(ramp_height, ramp_start, ramp_end, ramp_peak, path_start, total_length, resolution, putfile.fullname);
