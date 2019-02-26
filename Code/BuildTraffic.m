function [axle_loc, axle_id] = BuildTraffic(vehicle_fname, write_name, numVehicles, minspace, maxspace, lanes)
%Builds a csv file containing axle spring-mass system spacing
veh_ind_offset = 3;
loc_offset = -(minspace+20)*12;
distr = 'uniform';

if nargin < 3
    minspace = 0; %feet
    maxspace = 50; %feet
    lanes = 1;
end
% Read in vehicle parameters
vfile = file();
vfile.name = vehicle_fname;
vdat = readtable(vfile.fullname);
% parse individual vehicle parameters
vnames = unique(vdat.vehicle);
for jj = 1 : length(vnames)
    vehicle(jj).name = vnames(jj);
    vehicle(jj).axles = vdat.axle(vdat.vehicle==vnames(jj));
    vehicle(jj).location = vdat.location(vdat.vehicle==vnames(jj));
end

axle_loc = [];
axle_id = [];
for kk = 1:lanes
    % Randomly choose vehicles
    traffic(:,kk) = randsample(vnames,numVehicles,true);

    % create spacing between vehicles
    if strcmp(distr,'norm')
        space{kk}(:) = nearest((randn(numVehicles-1,1)+2)/4*(maxspace - minspace)+minspace);
    else
        space{kk}(:) = nearest(rand(numVehicles-1,1)*(maxspace - minspace)+minspace);
    end
    space{kk} = vertcat(0,space{kk}');



    % Create array of locations and axle ids
    a_loc{kk} = 0;
    a_id{kk} = [];
    for pp = 1:length(traffic)
        a_loc{kk} = vertcat(a_loc{kk},vehicle(traffic(pp,kk)).location+space{kk}(pp)+a_loc{kk}(end));
        a_id{kk} = vertcat(a_id{kk},vehicle(traffic(pp,kk)).axles);
    end
    axle_loc = vertcat(axle_loc,a_loc{kk}(2:end));
    axle_id = vertcat(axle_id,a_id{kk});
end

%put all axles into single column
%offset axle id's 
axle_id = axle_id+veh_ind_offset;

axle_loc = (axle_loc-axle_loc(end))*12+loc_offset; % position all vehicles before location 0, convert to inches

% Create and write table to file
Traffic_Table = table(axle_loc,axle_id);
savename = file();
savename.name = write_name;
writetable(Traffic_Table,savename.fullname);

end

