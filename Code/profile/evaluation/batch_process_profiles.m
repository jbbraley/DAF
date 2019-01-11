%% select folder containing profiles
dirname = uigetdir();

%% set bridge and vehicle parameters
% vehicle
load('vehicles.mat');

% get bridge parameters from stiffness value and 1st nat. freq.
load('bridges.mat')
stiff_factors = [1 1.2 1.4];
for ii = 1:length(bridges.nf)
    for jj = 1:length(bridges.K_mid{ii})
        for kk = 1:length(stiff_factors)
            [bmass{ii}(jj,kk), EI{ii}(jj,kk)] = bridge2beam(bridges.nf(ii), bridges.K_mid{ii}(jj)*stiff_factors(kk), bridges.span_length(ii), bridges.numspans(ii));
        end
    end
end

%% get all files in folder
prof_files = ls([dirname '\*.csv']);
for ii = 1:size(prof_files,1)
    proff = file([dirname '\' prof_files(ii,:)]);
    prof_dat = dlmread(proff.fullname,',');
    dist{ii} = prof_dat(:,1);
    elev{ii} = prof_dat(:,2);
end

%% setup state space model
for ii = 1:length(bridges.nf)
     % initialize
     switch bridges.numspans(ii)
         case 1
            vb_ssmodel = sgl_bridge_vehicle();
            dist_past = 0.5;
            vb_ssmodel.db = 0.01;
         case 2
            vb_ssmodel = dbl_bridge_vehicle();
            dist_past = [0.5 1.5];
            vb_ssmodel.db = 0.01;

     end
    
    for jj = 1:length(bridges.K_mid{ii})
        % assign parameters
        % bridge
        vb_ssmodel.EI = EI{ii}(jj,1);
        vb_ssmodel.L = bridges.span_length(ii);
        vb_ssmodel.mb = bmass{ii}(jj,1)*vb_ssmodel.gravity;
        
        %vehicle
        v_ind = bridges.veh_ind{ii};
        for kk = 1:length(v_ind)
           
            vb_ssmodel.mt = vehicles.mass(v_ind(kk))*vb_ssmodel.gravity;
            vb_ssmodel.kt = vehicles.k(v_ind(kk));
            vb_ssmodel.ct = vehicles.c(v_ind(kk));
            vb_ssmodel.vel = 720; % in/sec        

            %% Loop through profiles
            % preallocate for results
            for pp = 1:length(dist)
               % load profile
                vb_ssmodel.dist = dist{pp};
                vb_ssmodel.profile = elev{pp};
                vb_ssmodel.bridge_start = bridges.start{ii}(jj); % 320' approach
                veh_model = clone(vb_ssmodel,qcarSDF());
               % run
                yy = vb_ssmodel.simulate;
                yv = veh_model.simulate;
                % get indices for bridge to 5% past mid-span
               center_inds = [find(vb_ssmodel.time>=0,1,'first') ...
                   find(vb_ssmodel.time>=vb_ssmodel.L/vb_ssmodel.vel/2,1,'first')+floor(dist_past*vb_ssmodel.L/vb_ssmodel.vel/vb_ssmodel.dt)];
               clear center_range
               for seg=1:length(center_inds)-1; center_range{seg} = center_inds(seg):center_inds(seg+1);    end
               bridge_inds = find(vb_ssmodel.time>=0,1,'first')+ ...
                   [0 1]*floor(vb_ssmodel.L*bridges.numspans(ii)/vb_ssmodel.vel/vb_ssmodel.dt);
               bridge_range = bridge_inds(1):bridge_inds(2);
                % record displacement
%                 responses{ii,pp}(:,:,jj,kk) = yy;
%                 veh_response{ii,pp}(:,:,jj,kk) = yv;
%                 force_window = zeros(size(yv,1),1);
                
               % compute amplification
               for span = 1:length(center_range)
                   disp{ii}(pp,span,jj,kk) = min((-1)^(span+1)*yy(center_range{span},1));
                   damp{ii}(pp,span,jj,kk) = min((-1)^(span+1)*yy(center_range{span},1))/-vb_ssmodel.max_deflection;
%                    window and compute maximum contact force
                   force_window(bridge_range) = abs(sin(pi*bridges.numspans(ii)*(bridge_range-bridge_range(1))/diff(bridge_inds)));
                   c_force{ii}(pp,span,jj,kk) = max(yv(center_range{span},3).*force_window(center_range{span}));
                   famp{ii}(pp,span,jj,kk) = max(yv(center_range{span},3).*force_window(center_range{span}))/veh_model.mt;
%                    grab IRI value over bridge-inds
                   IRI{ii}(pp,span,jj,kk) = getIRI(elev{pp},dist{pp},center_inds(span:span+1))*63360; %converted to in/mi

               end
               
               display(['running profile ' num2str(pp)]);
            end
        end
    end
end

save('responses.mat','responses','disp','damp')
temp = disp{ii}(:,:,jj,kk);
temp = damp{ii}(:,:,jj,kk);
temp = famp{ii}(:,:,jj,kk);
FE_dat = [];
figure
plot(vb_ssmodel.time,responses{ii,pp}(:,1,jj,kk)))

%% working
Kf = 1:0.1:2;
for stiff = 1:length(Kf)
[mb, EI] = bridge2beam(bridges.nf(ii), bridges.K_mid{ii}(jj)*Kf(stiff), bridges.span_length(ii), bridges.numspans(ii));
vb_ssmodel.EI = EI;
vb_ssmodel.mb = mb*vb_ssmodel.gravity; 
yy{stiff} = vb_ssmodel.simulate;
end