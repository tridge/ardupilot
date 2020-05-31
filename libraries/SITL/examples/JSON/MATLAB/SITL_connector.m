% this function handles all the UDP connection to SITL using the TCP/UDP/IP
% Toolbox 2.0.6 by Peter Rydesäter
% https://uk.mathworks.com/matlabcentral/fileexchange/345-tcp-udp-ip-toolbox-2-0-6

function SITL_connector(state,init_function,physics_function,delta_t)
try
    pnet('closeall') % close any connections left open from past runs
catch
    warning('Could not execute pnet mex, trying to compile')
    if ispc
        % running on windows
        mex -O -outdir ../tcp_udp_ip_2.0.6 ../tcp_udp_ip_2.0.6/pnet.c ws2_32.lib -DWIN32
    else
        % running on unix or mac
        mex -O -outdir ../tcp_udp_ip_2.0.6 ../tcp_udp_ip_2.0.6/pnet.c
    end
    try
        pnet('closeall')
    catch
        error('Failed to compile pnet mex file, see tcp_udp_ip_2.0.6/pnet.c for instructions on manual complication')
    end
end

% init physics
state = init_function(state);

% Init the UDP port
u = pnet('udpsocket',9002);
pnet(u,'setwritetimeout',1);
pnet(u,'setreadtimeout',0);

frame_time = tic;
frame_count = 0;
physics_time_us = 0;
last_SITL_frame = -1;
print_frame_count = 1000; % print the fps every x frames
connected = false;
bytes_read =  4 + 16*2; % the number of bytes received in each packet
re_connect_timeout = 1; % after this time we close the socket and reopen to listen on any port, a AP can change ports after a re-start 
while true

    % Wait for data
    wait_start = tic;
    while true
         in_bytes = pnet(u,'readpacket',bytes_read);
         if in_bytes > 0
             break;
         end
         if connected && toc(wait_start) > re_connect_timeout
             pnet('closeall')
             u = pnet('udpsocket',9002);
             pnet(u,'setwritetimeout',1);
             pnet(u,'setreadtimeout',0);
             connected = false;
         end
     end

    % if there is another frame waiting, read it straight away
    if in_bytes > bytes_read
        if in_bytes == u.InputBufferSize
            % buffer got full, reset
            % should only happen if we have been paused in Matlab for some time
            fprintf('Buffer reset\n')
            continue;
        end
        continue;
    end

    % read in the current SITL frame and PWM
    SITL_frame = pnet(u,'read',1,'UINT32','intel');
    pwm_in = double(pnet(u,'read',16,'UINT16','intel'))';
    % Check if the fame is in expected order
    if SITL_frame < last_SITL_frame
        % Controller has reset, reset physics also
        state = init_function(state);
        fprintf('Controller reset\n')
    elseif SITL_frame == last_SITL_frame
        % duplicate frame, skip
        fprintf('Duplicate input frame\n')
        continue;
    elseif SITL_frame ~= last_SITL_frame + 1 && connected
        fprintf('Missed %i input frames\n',SITL_frame - last_SITL_frame - 1)
    end
    last_SITL_frame = SITL_frame;
    physics_time_us = physics_time_us + delta_t * 10^6;

    if ~connected
        % use port -1 to indicate connection to address of last recv pkt
        pnet(u,'udpconnect',"",-1);
        connected = true;
    end
    frame_count = frame_count + 1;

    % Do a physics time step
    state = physics_function(pwm_in,state);

    % build structure representing the JSON string to be sent
    JSON.timestamp = physics_time_us;
    JSON.imu.gyro = state.gyro;
    JSON.imu.accel_body = state.accel;
    JSON.position = state.position;
    JSON.attitude = state.attitude;
    JSON.velocity = state.velocity;

    % Report to AP  
    pnet(u,'printf',sprintf('\n%s\n',jsonencode(JSON)));
    pnet(u,'writepacket');

    % print a fps and runtime update
    if rem(frame_count,print_frame_count) == 0
        total_time = toc(frame_time);
        frame_time = tic;
        time_ratio = (print_frame_count*delta_t)/total_time;
        fprintf("%0.2f fps, %0.2f%% of realtime\n",print_frame_count/total_time,time_ratio*100)
    end
end

