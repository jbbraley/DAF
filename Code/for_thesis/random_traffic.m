% traffic density factor
fact = 1:5;
% File containing vehicle information
vfile = file();
vfile.name = 'vehicles.csv';
vfile.path = 'C:\Users\John B\Projects_Git\Damp\Code'; %C:\Users\John\Projects_Git\DAmp\Code';

% File to write resulting traffic csv file to
putfile = file();
% putfile.name = 'traffic_100_20-100.csv';
putfile.path = 'C:\Users\John B\Projects_Git\Damp\traffic\for-thesis';

% vehicle spacing range to sample between
maxspacing = 100*fact; % feet
minspacing = 20*fact; % feet

% Number of vehicles in traffic segment 
NumVehicles = 35;

% Number of lanes
numLanes = 1;

% Build traffic csv file
for ii = 1:length(fact)
    putfile.name = ['traffic_' num2str(NumVehicles,3) '_' num2str(minspacing(ii)) '-' num2str(maxspacing(ii)) '.csv'];
    [locations{ii}, ids{ii}] = BuildTraffic(vfile.fullname, putfile.fullname, NumVehicles, minspacing(ii), maxspacing(ii), numLanes);
end