% UNPACK LUSAS IMD PLUS RESULT FILE

%% Load file
results_file = file();
results_file.path = 'C:\Users\John\Documents\RutgersResearch\Dynamic Impact\3D\Associated Model Data\2sp_140ft';
results_file.name = 'n0000001191_Speed_720-000000000000.dsp';

res_name = {'DZ'}; % Time DX DY	DZ THX THY THZ RSLT

%% Read file
dat = dlmread(results_file.fullname,'\t',1,0);
head = strsplit(results_file.read_line(1));

%% Store data of interest
for result = 1:length(res_name)
    res_out(:,result) = dat(:,strcmp(res_name{result},head));
end

