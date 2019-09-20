%% assesses roadway unneveness based on ISO Standard using ISO8608 code 

%% Load profile
pro_file = file();
pro_file.name = 'ISO_IRI50-w1.5_1.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\ISO_IRI';
data = dlmread(pro_file.fullname,',');
profile = data(:,2);
dist = data(:,1);

%% Compute ISO 8608 Parameters
[C,W] = ISO8608(profile,dist,1.5);
fit = @(n) C*n.^(-W);
