%% solving the equations for the natural frequency of a 2-dof system

kt = [3947.8	15791.4	35530.6	63165.5	98696.0]; % spring stiffness of top spring (lb/in); % spring stiffness of top sprung mass
kb = [78956.84	315827.3	710611.5	1263309	1973921]; % spring stiffness of bottom spring (lb/in); % spring stiffness for bottom sprung mass
m1 = 100; % slinch (top mass); % mass of top sprung mass
m2 = 2000; % slinch (bottom mass); % mass of bottom sprung mass

for ii = 1:length(kt)
    for jj = 1:length(kb)
        k1 = kt(ii);  k2 = kb(jj);
        ff(ii,jj,:) = wn_2dof(m1,m2,k1,k2,1);
    end
end
