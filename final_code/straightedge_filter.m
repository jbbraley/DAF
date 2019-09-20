function [filtered_profile, exitflag] = straightedge_filter(profile,dist,straightedge_length, straightedge_deviation)
%% filter out straightedge deviation
% filtered profile
% exitflag = 1 if modifications were made (profile "smoothed")

exitflag = 0;
ind2 = 1;
ind1 = 0;
while ind2<length(profile)
    ind1 = ind1+1; ind2 = find(dist>=(straightedge_length+dist(ind1)),1,'first');
    if isempty(ind2); ind2 = length(profile); end
    slope = interp1([dist(ind1) dist(ind2)],[profile(ind1) profile(ind2)],dist(ind1:ind2));
    dev = straightedge_deviation * sqrt(diff(profile([ind1 ind2]))^2+diff(dist([ind1 ind2]))^2)/diff(dist([ind1 ind2]));
%     push_ind = find((profile(ind1:ind2)-slope+dev)<0);
%     profile(push_ind+ind1-1) = slope(push_ind)-dev;
    pull_ind = find(profile(ind1:ind2)-slope-dev>0);
    profile(pull_ind+ind1-1) = slope(pull_ind)+dev;
    
    exitflag = any([~isempty(pull_ind) exitflag]); %~isempty(push_ind)
end

filtered_profile = profile;
end