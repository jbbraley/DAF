% File to write resulting traffic csv file to
putfile = file();
putfile.name = '1in_bump.csv';
putfile.path = 'C:\Users\John\Projects_Git\DAmp\profiles\set1';

bump_spacing = 240; % feet
bump_height = 1; % inch
path_length = 1440; % feet
profile_discretization = 1; % feet

% convert to inches
spacing = bump_spacing*12;
total_length = path_length*12;
resolution = profile_discretization*12;


% Build traffic csv file
build_profile_bump( spacing, height, total_length, resolution, putfile.fullname)
