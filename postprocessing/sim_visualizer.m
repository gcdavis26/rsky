clc
clear
close all
%put absolute path 
live = true;

if live == false
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
else
    %% UDP SETUP
    u = udpport("datagram","LocalPort",5000);
    packetSize = 136;
    
    disp("Waiting for telemetry...")
    
    %% FIGURE
    figure('Color','w')
    tiledlayout(2,2)
    
    % 3D View
    nexttile([2 1])
    axis equal
    grid on
    view(3)
    xlabel("North")
    ylabel("East")
    zlabel("Up")
    title("Live Drone Visualization (NED)")
    xlim([-.5 5.5])
    ylim([-.5,10.5])
    zlim([0 3])
    hold on
    
    armLength = 0.2;
    
    % Drone body
    body = plot3(0,0,0,'LineWidth',3);
    
    % Trajectory
    traj = animatedline('Color','b');
    
    % Attitude plots
    nexttile
    rollPlot = animatedline('Color','r');
    title("Roll (rad)")
    grid on
    
    nexttile
    yawPlot = animatedline('Color','k');
    title("Yaw (rad)")
    grid on
    
    %% LOOP
    while true
        
        if u.NumDatagramsAvailable > 0
        
            raw = read(u, 1, "uint8");   % read ONE full datagram
        
            if numel(raw) == 136   % verify correct packet size
            
                %% Decode
                doubles = typecast(raw(1:128), "double");
                time = doubles(1);
                state = doubles(2:end);
    
                roll  = state(1);
                pitch = state(2);
                yaw   = state(3);
    
                north = state(4);
                east  = state(5);
                down  = state(6);
        
                %% Convert NED → Plot Frame
                x = north;
                y = east;
                z = -down;   % flip sign
        
                %% Update trajectory
                addpoints(traj, x, y, z)
        
                %% Update attitude plots
                addpoints(rollPlot, time, roll)
                addpoints(yawPlot,  time, yaw)
        
                %% Rotation matrices (NED convention)
                % Aircraft body-to-NED (ZYX)
                Rz = [ cos(yaw) -sin(yaw) 0;
                       sin(yaw)  cos(yaw) 0;
                       0         0        1 ];
        
                Ry = [ cos(pitch) 0 sin(pitch);
                       0          1 0;
                      -sin(pitch) 0 cos(pitch) ];
        
                Rx = [ 1 0 0;
                       0 cos(roll) -sin(roll);
                       0 sin(roll)  cos(roll) ];
        
                R_ned = Rz * Ry * Rx;
        
                % Convert NED to plotting frame (flip Z)
                T = diag([1 1 -1]);
                R_plot = T * R_ned;
        
                %% Body arms in body frame
                arm1 = R_plot * [-armLength;0;0];
                arm2 = R_plot * [ armLength;0;0];
                arm3 = R_plot * [0;-armLength;0];
                arm4 = R_plot * [0; armLength;0];
        
                %% Update drawing
                set(body, ...
                    'XData',[arm1(1)+x arm2(1)+x NaN arm3(1)+x arm4(1)+x], ...
                    'YData',[arm1(2)+y arm2(2)+y NaN arm3(2)+y arm4(2)+y], ...
                    'ZData',[arm1(3)+z arm2(3)+z NaN arm3(3)+z arm4(3)+z])
        
                drawnow limitrate
            end
      
        end
    end
end
