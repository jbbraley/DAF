%% solving the equations for the natural frequency of a 2-dof system

kt = [3947.8	15791.4	35530.6	63165.5	98696.0]; % spring stiffness of top spring (lb/in); % spring stiffness of top sprung mass
kb = [78956.84	315827.3	710611.5	1263309	1973921]; % spring stiffness of bottom spring (lb/in); % spring stiffness for bottom sprung mass

m1 = 200; % slinch (top mass); % mass of top sprung mass
m2 = 2000; % slinch (bottom mass); % mass of bottom sprung mass

kt = (1:0.1:2)'*(kb/m2)*m1;
sqrt(kt/m1)/(2*pi)

for ii = 1:length(kb)
    for jj = 1:size(kt,1)
        k1 = kt(jj,ii);  k2 = kb(ii);
        ff(jj,ii,:) = wn_2dof(m1,m2,k1,k2,0);
    end
end

figure
plot(1:0.1:2,ff(:,:,2))
legend({[num2str(sqrt(kb/m2)'/(2*pi)) padarray('Hz Bridge',length(kb)-1,'replicate','post')]})
ylabel('First Mode Frequency (Hz)')
xlabel('Ratio of Vehicle Frequency to Bridge Frequency')


figure
plot(1:0.1:2,ff(:,:,1))
legend({[num2str(sqrt(kb/m2)'/(2*pi)) padarray('Hz Bridge',length(kb)-1,'replicate','post')]})
ylabel('Second Mode Frequency (Hz)')
xlabel('Ratio of Vehicle Frequency to Bridge Frequency')