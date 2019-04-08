%% assesses roadway unneveness based on ISO Standard using ISO8608 code 

%% Load profiles
dirname = uigetdir();
%% get all files in folder
prof_files = ls([dirname '\*.csv']);
for ii = 1:size(prof_files,1)
    proff = file([dirname '\' prof_files(ii,:)]);
    prof_dat = dlmread(proff.fullname,',');
    dist{ii} = prof_dat(:,1);
    elev{ii} = prof_dat(:,2);
end

%% Compute ISO 8608 Parameters
for ii = 1:length(elev)
    [C(ii),W(ii)] = ISO8608(elev{ii},dist{ii});
    [C2(ii),W2(ii)]= ISO8608(elev{ii},dist{ii},2);
end

% fit = @(n) C*n.^(-W);