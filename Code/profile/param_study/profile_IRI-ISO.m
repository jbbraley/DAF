%% Parametric study of the two parameters for profile (IRI and ISO 8608 waviness)

% Establish parameter ranges
IRI_range = [50 75 100 150 200 250 300 400];
wav_range = 1.5:0.5:4;
vel = [720 960 1200];

% establish remaining model parameters
fnb = 3.0; % natural frequency of bridge
fnv = 3.2; % natural frequency of vehicle
mb = 2000; % mass of bridge (slinch)
approach_length = 100; % length of approach profile (ft)
blength = 140; % ft
mt = 200; % mass of vehicle (slinch)
dt = 0.2; % vehicle damping ratio

wnb = fnb*2*pi;
wnv = fnv*2*pi;

%% Vehicle-Bridge
% Initiate state space models
vb_ss = ss_bridge_vehicle();

% populate parameters:
vb_ss.L = blength*12;

vb_ss.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ss.mt = mt*vb_ss.gravity;
vb_ss.mb = mb*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = mb*vb_ss.L^3*wnb^2/pi^4;



%% Vehicle model (for initial conditions)
veh = qcarSDF();
veh.ms = mt*veh.gravity;
veh.k = (wnv)'.^2.*mt; 
veh.c = dt*2*sqrt(veh.k*veh.ms/veh.gravity);

LL_disp = -vb_ss.mt*vb_ss.L^3/(48*vb_ss.EI);
DL_disp = -5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

pro_file = file();
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\ISO_IRI';


for ii = 1:length(wav_range)    
    %% Load profile
    pro_file.name = ['ISO_IRI50-w' num2str(wav_range(ii)) '_1.csv'];    
    file_cont = dlmread(pro_file.fullname,',');
    dist = file_cont(:,1);
    profile = file_cont(:,2);
    approach_ind = 1:find(dist>=approach_length*12,1,'first');
    bridge_ind = find(dist>=approach_length*12,1,'first'):find(dist>=(approach_length+blength)*12,1,'first');
    vb_ss.dist = dist(bridge_ind)-dist(bridge_ind(1));
    veh.dist = dist(approach_ind);

    for jj = 1:length(IRI_range)
        IRI_scale = IRI_range(jj)/50;       
        veh.profile = profile(approach_ind)*IRI_scale;          
        vb_ss.profile = profile(bridge_ind)*IRI_scale; 
        % Store ISO Parameter Values
        [C(ii,jj), W(ii,jj)] = ISO8608(profile*IRI_scale,dist);
        ISOfit = @(n) C(ii,jj)*n.^(-W(ii,jj));
        C10(ii,jj) = ISOfit(1/10);      
        
        for kk = 1:length(vel)            
            % simulate approach
            veh.vel = vel(kk);
            yv0 = veh.simulate;
            % set initial conditions
            vb_ss.x0(3) = yv0(end,2); 
            vb_ss.x0(4) = yv0(end,1)-vb_ss.mt/vb_ss.kt; % make vehicle pre-displaced by self-weight
            
            % record profile power at bridge natural frequency
            ff = fnb/vel(kk)/0.0254; %cycles/meter
            Cfn(ii,jj,kk) = ISOfit(ff); % m^3
            
            % simulate
            vb_ss.vel = vel(kk);
            yvb(:,:,kk,jj,ii) = vb_ss.simulate;
            Damp_LL(kk,jj,ii) = min(yvb(:,1,kk,jj,ii))/LL_disp;
            Damp_DL(kk,jj,ii) =  min(yvb(:,1,kk,jj,ii))/DL_disp;
        end
    end
end


figure
plot(vel,Damp_LL(:,:,4))
legend([padarray('IRI - ',[length(IRI_range)-1 0],'replicate','post') num2str(IRI_range') padarray(' (in/mi)',[length(IRI_range)-1 0],'replicate','post') ])

figure
plot(IRI_range,permute(Damp_LL(2,:,:),[2 3 1]))
legend([padarray('waviness - ',[length(wav_range)-1 0],'replicate','post') num2str(wav_range')])

figure
plot(wav_range,permute(Damp_LL(2,:,:),[3 2 1]))
legend([padarray('IRI - ',[length(IRI_range)-1 0],'replicate','post') num2str(IRI_range') padarray(' (in/mi)',[length(IRI_range)-1 0],'replicate','post') ])

figure
plot(IRI_range,permute(Damp_LL(:,:,4),[2 1 3]))
legend([padarray('velocity - ',[length(vel)-1 0],'replicate','post') num2str(vel') padarray(' (in/sec)',[length(vel)-1 0],'replicate','post')] )


figure
plot(vel,permute(Damp_LL(:,6,:),[1 3 2]))
legend([padarray('waviness - ',[length(wav_range)-1 0],'replicate','post') num2str(wav_range')])

figure
plot(wav_range,permute(Damp_LL(:,6,:),[3 1 2]))
legend([padarray('velocity - ',[length(vel)-1 0],'replicate','post') num2str(vel') padarray(' (in/sec)',[length(vel)-1 0],'replicate','post')] )


%% plot
figure
plot(vb_ss.time,yvb(:,1)/LL_disp)
hold all
for  ii = 1:size(stop_bands,1)
vb_ss.profile =prof_filt(:,ii);
yvb_filt{ii} = vb_ss.simulate;
plot(vb_ss.time,yvb_filt{ii}(:,1)/LL_disp)
end
plot(vb_ss.time,yvb_0(:,1)/LL_disp)
legend({'original profile'; '50ft removed';'30ft removed';'20ft removed';'10ft removed';'5ft removed'; 'smooth roadway'})
xlabel('time (sec)')
ylabel('bridge disp amplification')





