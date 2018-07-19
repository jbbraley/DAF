

%% vehicle parameters
% Spring stiffness

wav = [20 30 40 60];

for jj = 1:length(wav)
    name = num2str(wav(jj));
    %% Load profile
    pro_file = file();
    pro_file.name = ['pure-' name 'ft_0.5in.csv'];
    pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic';
    file_cont = dlmread(pro_file.fullname,',');

    profile = file_cont(:,2);
    
    start_ind = find(profile/sign(profile(1))<0,1,'first');
    profile = [0; profile(start_ind:end)];
    %start profile at zero
    
    dx = round(mean(diff(file_cont(:,1))),3);
    dist = 0:dx:(length(profile)-1)*dx;
    
    % save file again
    dlmwrite(pro_file.fullname, [dist' profile], ',');
end

pro_file = file();
pro_file.name = 'EB_right_1_L.csv';
pro_file.path = 'C:\Users\John\Projects_Git\I76\Profiles\measured\for_simulation\EB_right'
