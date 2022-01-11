n_samples  = [20, 12, 8];

cal_time = [38.82, 23.25, 15.39];
% cal_cost = []
our_time = [17.69, 15.01, 15.4];

bar([3,2,1], [cal_time; our_time])
xlabel("number of samples")
set(gca,'XTickLabel',{'8', '12', '20'})
ylabel("calculation time (ms) ")
legend("our methods", "sample-based methods")
title("Optimization time between our iterative methods and sampling methods")