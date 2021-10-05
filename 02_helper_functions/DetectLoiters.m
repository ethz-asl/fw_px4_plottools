function [t_starts, t_ends] = DetectLoiters(local_position, airspeed_raw, commander_state, config)
t_starts = [];
t_ends = [];

% resample the data to a regular interval
t_log_start = local_position.x.Time(1);
t_log_end = local_position.x.Time(end);
dt_rs = 0.1;
time_resampled = t_log_start:dt_rs:t_log_end;
x = resample(local_position.x, time_resampled);
y = resample(local_position.y, time_resampled);
z = resample(local_position.z, time_resampled);
airspeed = resample(airspeed_raw, time_resampled);
if config.only_use_auto_loiter
    commander_state = resample(commander_state, time_resampled);
end

% segment the data into the windows
N_window = round(config.window_size / dt_rs);
N_overlap = round(N_window * config.overlap);
x_segments = buffer(x.Data,N_window,N_overlap);
y_segments = buffer(y.Data,N_window,N_overlap);
z_segments = buffer(z.Data,N_window,N_overlap);
airspeed_segments = buffer(airspeed.Data,N_window,N_overlap);
time_segments = buffer(x.Time,N_window,N_overlap);
commander_state_segments = buffer(commander_state.Data,N_window,N_overlap);

x_centers = zeros(size(x_segments,2), 1);
y_centers = zeros(size(x_segments,2), 1);
radii = zeros(size(x_segments,2), 1);
avg_errors = zeros(size(x_segments,2), 1);
max_errors = zeros(size(x_segments,2), 1);
avg_airspeed = zeros(size(x_segments,2), 1);
std_airspeed = zeros(size(x_segments,2), 1);
std_altitude = zeros(size(x_segments,2), 1);

% compute the circle fit for the individual segments
for i=1:size(x_segments,2)
    [x_centers(i), y_centers(i), radii(i), avg_errors(i), max_errors(i)] = FitCircle(x_segments(:,i), y_segments(:,i));
    avg_airspeed(i) = mean(airspeed_segments(:, i));
    std_airspeed(i) = std(airspeed_segments(:, i));
    std_altitude(i) = std(z_segments(:, i));
end

% detect the loiters based on the segments
loiter_active = false;
current_t_start = 0;
current_t_end = 0;
current_airspeed = 0;
current_radius = 0;
for i=1:size(x_segments,2)
    current_segment_loiter = false;
    % radius check
    if (radii(i) > config.min_radius && radii(i) < config.max_radius)
        % airspeed check
        if avg_airspeed(i) > config.min_airspeed
            % error check
            if max_errors(i) < config.max_circle_fit_error
                % altitude check
                if std_altitude(i) < config.max_altitude_std
                    if std_airspeed(i) < config.max_airspeed_std
                        current_segment_loiter = true;
                    end
                end
            end
        end
        
        if current_segment_loiter
            if config.only_use_auto_loiter
                if ~all(commander_state_segments(:,i) == 3)
                    current_segment_loiter = false;
                    
                    if config.verbose
                        disp('Loiter rejected because not in auto mode')
                    end
                end
            end
        end
        
        if loiter_active
            if current_segment_loiter
                if abs((radii(i)/current_radius) - 1.0) > config.max_radius_change
                    % radius changed significantly save previous segment if
                    % it was long enough
                    n_loiters = (current_t_end - current_t_start) / (2*pi*radii(i) / avg_airspeed(i));
                    if (n_loiters > config.n_loiters_required)
                        t_starts(end+1) = current_t_start;
                        t_ends(end+1) = current_t_end;
                        disp(['Detected ', num2str(n_loiters), '  loiters with airspeed: ', num2str(current_airspeed),...
                            ', radius: ', num2str(current_radius), ' and duration: ', num2str(current_t_end-current_t_start), ...
                            ' [', num2str(current_t_start), '->', num2str(current_t_end), ']']) 
                    elseif config.verbose
                        disp(['Segment rejected because too few full loiters: ', num2str(n_loiters)])
                    end
                    
                    % reset the times since this is still detected as a
                    % loiter
                    current_t_start = time_segments(1,i);
                    current_t_end = time_segments(end,i);
                    current_airspeed = avg_airspeed(i);
                    current_radius = radii(i);
                else
                    current_t_end = time_segments(end,i);
                end
                
            else 
                loiter_active = false;
                
                n_loiters = (current_t_end - current_t_start) / (2*pi*radii(i) / avg_airspeed(i));
                if (n_loiters > config.n_loiters_required)
                    t_starts(end+1) = current_t_start;
                    t_ends(end+1) = current_t_end;
                    disp(['Detected ', num2str(n_loiters), '  loiters with airspeed: ', num2str(current_airspeed),...
                        ', radius: ', num2str(current_radius), ' and duration: ', num2str(current_t_end-current_t_start), ...
                        ' [', num2str(current_t_start), '->', num2str(current_t_end), ']']) 
                elseif config.verbose
                    disp(['Segment rejected because too few full loiters: ', num2str(n_loiters)])
                end
                
                
                
                if config.verbose
                    disp('Loiter ended:')
                    if ~(radii(i) > config.min_radius && radii(i) < config.max_radius)
                       disp(['Radius check failed: ', num2str(radii(i))]) 
                    end
                    if ~(avg_airspeed(i) > config.min_airspeed)
                       disp(['Airspeed check failed: ', num2str(avg_airspeed(i))]) 
                    end
                    if ~(max_errors(i) < config.max_circle_fit_error)
                       disp(['Circle fit check failed: ', num2str(max_errors(i))]) 
                    end
                    if ~(std_altitude(i) < config.max_altitude_std)
                       disp(['Altitude check failed: ', num2str(std_altitude(i))]) 
                    end
                    if ~(std_airspeed(i) < config.max_airspeed_std)
                       disp(['Airspeed std check failed: ', num2str(std_airspeed(i))]) 
                    end
                end
            end
            
        elseif current_segment_loiter
            current_t_start = time_segments(1,i);
            current_t_end = time_segments(end,i);
            current_airspeed = avg_airspeed(i);
            current_radius = radii(i);
            loiter_active = true;
        end        
    end
end



