%% UDP Telemetry Receiver + 3D Drone Position
clear; clc;

% --------------------------
% Configuration
% --------------------------
PORT = 5000;           % Must match C++ sender
PACKET_SIZE = 136;     % sizeof(TelemetryPacket
STATE_SIZE = 15;       % Total state vector

% --------------------------
% Open UDP port
% --------------------------
u = udpport("datagram", "IPV4", "LocalPort", PORT);
disp("Listening for telemetry...");

% --------------------------
% 3D plot setup
% --------------------------
figure('Name','Drone Position');
axis equal;
grid on; hold on;
xlabel('North'); ylabel('East'); zlabel('Up');
xlim([-10 10]); ylim([-10 10]); zlim([0 10]); % adjust to your sim
view(3);

% Plot handles
hDrone = plot3(NaN, NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b'); % current drone position
hPath  = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1.5); % flight path

% --------------------------
% Storage for logging
% --------------------------
time_log = zeros(0,1);
state_log = zeros(STATE_SIZE,0);

% --------------------------
% Main loop
% --------------------------
while true
    
    if u.NumDatagramsAvailable > 0
        
        % Read one UDP datagram
        d = read(u,1);
        data = d.Data(:); % ensure column
        
        if numel(data) ~= PACKET_SIZE
            warning("Unexpected packet size: %d bytes", numel(data));
            continue;
        end
        
        % --------------------------
        % Decode packet
        % --------------------------
        idx = 1;
        time = double(typecast(uint8(data(idx:idx+7)),'double')); idx = idx+8;
        state = double(typecast(uint8(data(idx:idx+8*STATE_SIZE-1)),'double')); state = state(:); idx = idx+8*STATE_SIZE;
        flags = uint32(typecast(uint8(data(idx:idx+3)),'uint32')); idx=idx+4;
        counter = uint32(typecast(uint8(data(idx:idx+3)),'uint32'));
        
        % Store logs
        time_log(end+1,1) = time;
        state_log(:,end+1) = state;
        
        % --------------------------

        pos = state(4:6); % [n,e,d]
        
        % --------------------------
        % Update plots
        % --------------------------
        set(hDrone, 'XData', pos(1), 'YData', pos(2), 'ZData', -pos(3));
        set(hPath,  'XData', state_log(4,:), 'YData', state_log(5,:), 'ZData', state_log(6,:));
        
        drawnow limitrate;
        
    end
    
end
