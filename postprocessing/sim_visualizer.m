clc
clear
close all
%put absolute path 

truthtable = readtable('..\build\sim_truths');
ekftable = readtable('..\build\sim_estimates');

t = truthtable{:,'t'};
truth_pos = truthtable{:,{'n','e','d'}};
ekf_pos = ekftable{:,{'n','e','d'}};
plot3(truth_pos(:,1),truth_pos(:,2),-truth_pos(:,3))
hold on
plot3(ekf_pos(:,1),ekf_pos(:,2),-ekf_pos(:,3))
xlabel("North")
ylabel("East")
zlabel("Up")
title("Drone Simulation")
legend('True Pos','EKF State')