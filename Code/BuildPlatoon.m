function [axle_loc, axle_id] = BuildPlatoon(vehicle_fname, write_name, numVehicles, spacing, veh_ind)
%Builds a csv file containing axle spring-mass system spacing
veh_ind_offset = 3;
loc_offset = 0; %-(minspace+20)*12;
distr = 'uniform';

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

% choose vehicles
traffic = veh_ind*ones(numVehicles,1);

% create spacing between vehicles
space = spacing*ones(numVehicles-1,1);
space = vertcat(0,space);

% Create array of locations and axle ids
axle_loc = 0;
axle_id = [];
for pp = 1:length(traffic)
    axle_loc = vertcat(axle_loc,vehicle(traffic(pp)).location+space(pp)+axle_loc(end));
    axle_id = vertcat(axle_id,vehicle(traffic(pp)).axles);
end


%offset axle id's 
axle_id = axle_id+veh_ind_offset;

axle_loc = (axle_loc(2:end)-axle_loc(end))*12+loc_offset; % position all vehicles before location 0, convert to inches


% Create and write table to file
axle_loc_in = axle_loc(end:-1:1);
axle_id_num = axle_id(end:-1:1);
Traffic_Table = table(axle_loc_in,axle_id_num);
savename = file();
savename.name = write_name;
writetable(Traffic_Table,savename.fullname);

end

