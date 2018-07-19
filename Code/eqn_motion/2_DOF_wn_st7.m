%% find the natural frequencies of a 2-dof sprung mass system (FE model in st7)

%% Load St7 API library
addpath(genpath('C:\Users\John\Projects_Git\st7api'));

%% Load model
FE_file = file();
FE_file.name = '2dof_sprung-mass.st7';
FE_file.path = 'C:\Users\John\Projects_Git\DAmp\Modeling\Simple\state-space';
scratchpath = 'C:\Temp';



%% setup spring/mass parametric study
m1 = 100; % slinch (top mass)
m2 = 2000; % slinch (bottom mass)
k1 = [3947.8	15791.4	35530.6	63165.5	98696.0]; % spring stiffness of top spring (lb/in)
k2 = [78956.84	315827.3	710611.5	1263309	1973921]; % spring stiffness of bottom spring (lb/in)
d1 = 0.10; % damping ratio of top spring 
d2 = 0.02; % damping ratio of bottom spring


% Instantiate st7 model
sys = st7model();
sys.filename = [FE_file.name FE_file.ext];
sys.pathname = FE_file.path;
sys.scratchpath = scratchpath;

% setup nfa info
nfa = NFA();
nfa.name = [FE_file.fullname(1:end-4)];
nfa.nmodes = 2; % set number of modes to compute

% add spring elements to parameters

K1 = parameter();
K1.name = 'ka';
K1.obj = spring_damper
K1.obj.propNum = 1;

K2 = parameter();
K2.name = 'ka';
K2.obj = spring_damper();
K2.obj.propNum = 2;

% Populate empty parameter fields
% api options
APIop = apiOptions();
APIop.keepLoaded = 1;
APIop.keepOpen = 1;
% start logger
logg = logger('Spring Stiffness');

% run shell
logg.task('Getting model props');
apish(@getModelProp,sys,{K1;K2},APIop);

for kk = 1:length(k1)
    for mm = 1:length(k2)
        
        model(mm).sys = sys;
        solver = NFA();
        solver.name = [nfa.name '_bridge' num2str(mm) '-truck' num2str(kk)];
        solver.nmodes = nfa.nmodes;
        solver.run = 1;
        
        model(mm).solvers = solver;
        model(mm).options.populate = 0;

        % Spring properties
        % Create new instance of spring_damper class
        % Create new instance of parameter class
        Para{1} = parameter();
        Para{1}.obj = K1.obj.clone; % create clone of previously defined beam class
        Para{1}.obj.(K1.name) = k1(kk); % overwrite with step value
        Para{1}.obj.ca = d1*2*sqrt(k1(kk)*m1);
        Para{1}.name = K1.name; % must correspond to the property being altered

        Para{2} = parameter();
        Para{2}.obj = K2.obj.clone; % create clone of previously defined beam class
        Para{2}.obj.(K2.name) = k2(mm); % overwrite with step value
        Para{2}.obj.ca = d2*2*sqrt(k2(mm)*m2);
        Para{2}.name = K2.name; % must correspond to the property being altered
        
        
        model(mm).params = Para;
    end

        %% run the shell
    APIop.keepOpen = 1;

    tic
    % Run sensitivity shell
    apish(@sensitivity,sys,model,APIop);
    toc
    
    % save results
    for mm = 1:length(model)
       [~, freq_ind] = max(model(mm).solvers.modal(:,6)); % return modes with highest mass participation factor
        Freq(mm,kk) = model(mm).solvers.freq(freq_ind); % bridge x vehicle
    end

%     % save last step model
%     new_filename = [sys.filename(1:end-4) '_step' num2str(ii) '.st7'];
%     api.saveas(sys.uID,sys.pathname,new_filename);

end
%% Close Model
api.closeModel(sys.uID)
sys.open=0;

end
