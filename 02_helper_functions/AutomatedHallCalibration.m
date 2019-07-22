function cal_data = AutomatedHallCalibration(sysvector, topics, sensor_index, cal_opt)

switch sensor_index
   case 0
      sensor_hall_mag_T = sysvector.sensor_hall_0.mag_T;
   case 1
      sensor_hall_mag_T = sysvector.sensor_hall_1.mag_T;
   case 2
      sensor_hall_mag_T = sysvector.sensor_hall_2.mag_T;
   case 3
      sensor_hall_mag_T = sysvector.sensor_hall_3.mag_T;
   otherwise
      disp('HALL SENS PREV: Invalid sensor_index')
end

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% AUTOMATED DATA PROCESSING / CALIBRATION

% start index
idx_st = find(sensor_hall_mag_T.Time > cal_opt.t_st_cal, 1, 'first');

% steps in degrees
if (cal_opt.angle_st > cal_opt.angle_ed)
    cal_data = [(cal_opt.angle_st:-cal_opt.large_step_size:cal_opt.small_step_range)'; ...
        (cal_opt.small_step_range-cal_opt.small_step_size:-cal_opt.small_step_size:-cal_opt.small_step_range)'; ...
        (-cal_opt.small_step_range-cal_opt.large_step_size:-cal_opt.large_step_size:cal_opt.angle_ed)'];
else
    cal_data = [(cal_opt.angle_st:cal_opt.large_step_size:-cal_opt.small_step_range)'; ...
        (-cal_opt.small_step_range+cal_opt.small_step_size:cal_opt.small_step_size:cal_opt.small_step_range)'; ...
        (cal_opt.small_step_range+cal_opt.large_step_size:cal_opt.large_step_size:cal_opt.angle_ed)'];
end
len_cal_data = length(cal_data);

% calibration data matrix [deg, mean mT, st dev mT, idx_st, idx_ed]
cal_data = [cal_data, zeros(len_cal_data, 4)]; 

len_t = length(sensor_hall_mag_T.Time(idx_st:end));
dmT_filt = 0;
t_step = 0;
recording_step = false;
i_cal = 0;
for k = idx_st+1:idx_st+len_t-1
    
    if i_cal == len_cal_data
        break;
    end
    
    % calculate delta magnetic field strength from previous measurement
    dmT_k = sensor_hall_mag_T.Data(k) - sensor_hall_mag_T.Data(k-1);
    dmT_filt = dmT_filt * (1 - cal_opt.k_filt) + cal_opt.k_filt * dmT_k;
    
    % check mag deviation threshold
    if (abs(dmT_filt) > cal_opt.mag_dev)
        % the step deviation has been exceeded
        
        if (t_step > cal_opt.t_step_min)
            % the previous step time satisfies criteria
            
            % set current calibration step
            i_cal = i_cal + 1;
            
            % crop tail and set end index
            cal_data(i_cal, 5) = max(cal_data(i_cal,4)+1, k-cal_opt.idx_ed_crop);
            
            % compute mean magnetic field strength
            cal_data(i_cal, 2) = ...
                mean(sensor_hall_mag_T.Data(cal_data(i_cal,4):cal_data(i_cal, 5)));
            
            % compute standard deviation of measurements
            cal_data(i_cal, 3) = ...
                std(sensor_hall_mag_T.Data(cal_data(i_cal,4):cal_data(i_cal, 5)));
        end
        
        % reset step time
        t_step = 0;
        
        % reset recording state
        recording_step = false;
        
    else
        % we are within the deviation limits for successive measurements
        
        if (~recording_step)
            % check if current data is sufficiently far from previously
            % logged step mean (this check prevents repeat measurements
            % from same angle)
            if (i_cal == 0)
                % protect agains 0 index on first step
                last_mean = 0;
            else
                last_mean = cal_data(i_cal, 2);
            end
            if (abs(sensor_hall_mag_T.Data(k) - last_mean) > cal_opt.step_thres || i_cal == 0)
                % start recording step
                recording_step = true;
                cal_data(i_cal+1, 4) = k;
            end
        end
        
        if (recording_step)
            % time since started recording
            t_step = sensor_hall_mag_T.Time(k) - sensor_hall_mag_T.Time(cal_data(i_cal+1, 4));
        end
    end
end

%% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% plot

figure('color','w','name','Hall Calibration Data');
hold on; grid on; box on;

% check for unpopulated rows..
last_populated_idx = find(cal_data(:,4) ~= 0 & cal_data(:,5) ~= 0, 1, 'last');

plot(sensor_hall_mag_T.Time(cal_data(1,4):cal_data(last_populated_idx,5)), ...
    sensor_hall_mag_T.Data(cal_data(1,4):cal_data(last_populated_idx,5)), '.', 'color', [0.3 0.3 0.3]);
for i = 1:len_cal_data
    if (i <= last_populated_idx)
        plot(sensor_hall_mag_T.Time([cal_data(i,4), cal_data(i,5)]), ...
            ones(1,2) * cal_data(i, 2), 'linewidth', 2, 'color', 'r');
    end
end
ylabel('Magnetic field strength [mT]');
legend('raw data', 'step means');
xlabel('Time [s]');
xlim(sensor_hall_mag_T.Time([cal_data(1,4), cal_data(last_populated_idx,5)]));

if (last_populated_idx == len_cal_data)
    disp('HALL SENS CAL: All calibration steps populated.');
end

