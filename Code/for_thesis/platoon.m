
% File containing vehicle information
vfile = file();
vfile.name = 'vehicles.csv';
vfile.path = 'C:\Users\John\Projects_Git\Damp\Code'; %C:\Users\John\Projects_Git\DAmp\Code';

% File to write resulting traffic csv file to
putfile = file();
% putfile.name = 'traffic_100_20-100.csv';
putfile.path = 'C:\Users\John\Projects_Git\DAmp\traffic\for-thesis\platoons';

% vehicle type
veh_num = 5;

% vehicle spacing 
spacing = 380:20:500; % feet

% Number of vehicles in traffic segment 
NumVehicles = 4;

% Number of lanes
numLanes = 1;

% Build traffic csv file
for ii = 1:length(spacing)
    putfile.name = ['platoon_' num2str(NumVehicles) '@' num2str(spacing(ii)) 'ft.csv'];
    [locations{ii}, ids{ii}] = BuildPlatoon(vfile.fullname, putfile.fullname, NumVehicles, spacing(ii), veh_num);
end