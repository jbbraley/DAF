function [dist, disp] = build_profile_ramp( ramp_height, ramp_start, ramp_end, ramp_peak, path_start, total_length, resolution, write_name)

num_points = nearest(total_length/resolution)+1;

ramp_start = nearest(ramp_start/resolution)*resolution;
ramp_end = nearest(ramp_end/resolution)*resolution;
ramp_peak = nearest(ramp_peak/resolution)*resolution;
total_length = nearest(total_length/resolution)*resolution;

dist = path_start:resolution:(total_length+path_start);
disp = zeros(num_points, 1);

ramp_start_ind = find((dist-ramp_start)>=0,1,'first');
ramp_peak_ind = find((dist-ramp_peak)>=0,1,'first');
ramp_end_ind = find((dist-ramp_end)>=0,1,'first');

disp(ramp_start_ind:ramp_peak_ind) = linspace(0,ramp_height,(ramp_peak-ramp_start)/resolution+1);
disp(ramp_peak_ind:ramp_end_ind) = linspace(ramp_height,0,(ramp_end-ramp_peak)/resolution+1);

% make sure path starts at 0 distance
dist = dist-dist(1);

% Create and write table to file
Profile_Table = horzcat(dist',disp);
savename = file();
savename.name = write_name;
dlmwrite(savename.fullname, Profile_Table);
end

