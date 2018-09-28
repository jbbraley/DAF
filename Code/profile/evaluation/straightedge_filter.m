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
    push_ind = find(profile(ind1:ind2)-slope+straightedge_deviation<0);
    profile(push_ind+ind1-1) = slope(push_ind)-straightedge_deviation;
    pull_ind = find(profile(ind1:ind2)-slope-straightedge_deviation>0);
    profile(pull_ind+ind1-1) = slope(pull_ind)+straightedge_deviation;
    
    exitflag = any([~isempty(push_ind) ~isempty(pull_ind) exitflag]);
end

filtered_profile = profile;
end