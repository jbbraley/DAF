% File containing vehicle information
vfile = file();
vfile.name = 'vehicles.csv';
vfile.path = 'C:\Users\John\Projects_Git\DAmp\Code';

% File to write resulting traffic csv file to
putfile = file();
putfile.name = 'traffic_100_20-100.csv';
putfile.path = 'C:\Users\John\Projects_Git\DAmp\traffic\2lanes_2';

% vehicle spacing range to sample between
maxspacing = 100; % feet
minspacing = 20; % feet

% Number of vehicles in traffic segment 
NumVehicles = 100;

% Number of lanes
numLanes = 2;

% Build traffic csv file
[locations, ids] = BuildTraffic(vfile.fullname, putfile.fullname, NumVehicles, minspacing, maxspacing, numLanes);