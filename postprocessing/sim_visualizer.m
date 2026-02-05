clc
clear
close all
%put absolute path 

simulation = 2;

switch simulation
    case 0

        truthtable = readtable('..\build\sim_truths');
        ekftable = readtable('..\build\sim_estimates');
        
        t = truthtable{:,'t'};
        truth_pos = truthtable{:,{'n','e','d'}};
        ekf_pos = ekftable{:,{'n','e','d'}};
        scatter3(truth_pos(:,1),truth_pos(:,2),-truth_pos(:,3),5)
        hold on
        scatter3(ekf_pos(:,1),ekf_pos(:,2),-ekf_pos(:,3),5)
        xlabel("North")
        ylabel("East")
        zlabel("Up")
        title("EKF Isolated Simulation")
        legend('True Pos','EKF State')

    case 1

        controltable = readtable('..\build\control_test.csv');
        t = controltable{:,'t'};
        pos = controltable{:,{'n','e','d'}};
        plot3(pos(:,1),pos(:,2),-pos(:,3))
        xlabel("North")
        ylabel("East")
        zlabel("Up")
        title("Controls Isolated Simulation")
    case 2
        controltable = readtable('..\build\sim_results.csv');
        t = controltable{:,'t'};
        true_pos = 3.048 * controltable{:,{'n','e','d'}};
        e_pos = 3.048 * controltable{:,{'n_est','e_est','d_est'}}; 
        plot3(true_pos(:,1),true_pos(:,2),-true_pos(:,3), color='k')
        hold on
        plot3(e_pos(:,1),e_pos(:,2),-e_pos(:,3))

        legend('True', 'Estimated')
        xlabel("North")
        ylabel("East")
        zlabel("Up")
        title("Full Simulation")

end