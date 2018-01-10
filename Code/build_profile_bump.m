function build_profile_bump( spacing, height, total_length, resolution, write_name)

num_points = nearest(total_length/resolution)+1;
% num_bumps = ceil((total_length-resolution)/spacing);

spacing = nearest(spacing/resolution)*resolution;
total_length = nearest(total_length/resolution)*resolution;

dist = 0:resolution:total_length;
disp = zeros(num_points, 1);

bump_index = 2:spacing/resolution:num_points;
disp(bump_index) = height;

% Create and write table to file
Profile_Table = horzcat(dist',disp);
savename = file();
savename.name = write_name;
dlmwrite(savename.fullname, Profile_Table);
end

