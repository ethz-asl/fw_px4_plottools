%% ************************************************************************
%  EstimatorStatusPlots
%  ************************************************************************
%  Display the estimator status data from the log file. Assumes that
%  successfully estimator data was logged.

function EstimatorStatusPlots(sysvector, topics)
if topics.estimator_status.logged
    % innovation checkbit
    inno_check_bits = de2bi(sysvector('estimator_status_0.innovation_check_flags').Data,12);
    fig1 = figure();
    fig1.Name = 'Innovation Check Flags';

    titles = [string('Reject VEL NED'), string('Reject POS NE'), string('Reject POS D'),...
        string('Reject Mag X'), string('Reject Mag Y'), string('Reject Mag Z'),...
        string('Reject Yaw'), string('Reject Airspeed'), string('Reject Sideslip'),...
        string('Reject HAGL'), string('Reject Optical Flow X'), string('Reject Optical Flow Y')];
    
    for i = 1:12
        inno_check(i) = subplot(3,4,i);
        plot(sysvector('estimator_status_0.innovation_check_flags').Time,inno_check_bits(:,i));
        title(titles(i))
    end

    linkaxes(inno_check(:),'x');
    set(inno_check(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig1);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
    
    % gps_check_fail_flags
    gps_check_bits = de2bi(sysvector('estimator_status_0.gps_check_fail_flags').Data,10);
    fig2 = figure();
    fig2.Name = 'GPS Check Fail Flags';
    
    titles = [string('Fix Type Insufficient'), string('Number of Satellites Insufficient'), ...
        string('GDOP Insufficient'), string('Horizontal Accuracy Insufficient'),...
        string('Vertical Accuracy Insufficient'), string('Speed Accuracy Insufficient'),...
        string('Excessive Horizontal Drift (only used on ground)'),...
        string('Excessive Vertical Drift (only used on ground)'),...
        string('Excessive Horizontal Speed (only used on ground)'),...
        string('Excessive Vertical Speed')];
    
    for i = 1:10
        gps_check(i) = subplot(5,2,i);
        plot(sysvector('estimator_status_0.gps_check_fail_flags').Time,gps_check_bits(:,i));
        title(titles(i))
    end

    linkaxes(gps_check(:),'x');
    set(gps_check(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig2);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
    
    % control_mode_flags
    control_mode_bits = de2bi(sysvector('estimator_status_0.control_mode_flags').Data,20);
    fig3 = figure();
    fig3.Name = 'Control Mode Flags';
    
    titles = [string('Tilt Alignment Complete'), string('Yaw Alignment Complete'), ...
        string('GPS Fused'), string('Optical Flow Fused'), string('Mag Yaw Fused'),...
        string('Mag 3D Fused'), string('Mag Decl Fused'), string('Vehicle in Air'),...
        string('Wind Vel Estimated'), string('Baro Height is Primary'),...
        string('Range Finder is Primary'), string('GPS Height is Primary'),...
        string('LPOS from Ext Vision Fused'), string('Yaw from Ext Vision Fused'),...
        string('Height from Ext Vision Fused'), string('Synthetic Sideslip Fused'),...
        string('Magnetometer updates only Mag States'), string('Fixed Wing'),...
        string('Mag Fault'), string('Airspeed Fused')];
    
    for i = 1:20
        control_modes(i) = subplot(5,4,i);
        plot(sysvector('estimator_status_0.control_mode_flags').Time,control_mode_bits(:,i));
        title(titles(i))
    end
    
    linkaxes(control_modes(:),'x');
    set(control_modes(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig3);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
        
    % filter_fault_flags
    filter_fault_flags = de2bi(sysvector('estimator_status_0.filter_fault_flags').Data,16);
    fig4 = figure();
    fig4.Name = 'Filter Fault Flags';
    
    titles = [string('Mag X Num Error'), string('Mag y Num Error'), ...
        string('Mag Z Num Error'), string('Mag Heading Num Error'), string('Mag Decl Num Error'),...
        string('Airspeed Num Error'), string('Sideslip Num Error'), string('Opt Flow X Num Error'),...
        string('Opt Flow Y Num Error'), string('Vel N Num Error'),...
        string('Vel E Num Error'), string('Vel D Num Error'),...
        string('Pos N Num Error'), string('Pos E Num Error'),...
        string('Pos D Num Error'), string('Bad Acc Bias')];
    
    for i = 1:16
        filter_fault(i) = subplot(4,4,i);
        plot(sysvector('estimator_status_0.filter_fault_flags').Time,filter_fault_flags(:,i));
        title(titles(i))
    end
    
    linkaxes(filter_fault(:),'x');
    set(filter_fault(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig4);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
    
    % solution_status_flags
    solution_status_flags = de2bi(sysvector('estimator_status_0.solution_status_flags').Data,12);
    fig5 = figure();
    fig5.Name = 'Solution Status Flags';
    
    titles = [string('Good Attitude'), string('Good Horizontal Velocity'), ...
        string('Good Vertical Velocity'), string('Good Horizontal Position (Relative)'),...
        string('Good Horizontal Position (absolute)'),...
        string('Good Vertical Position (relative)'),...
        string('Good Vertical Position (absolute)'),...
        string('Const Pos Mode'),...
        string('Predict Horizontal Position (relative)'),...
        string('Predict Horizontal Position (absolute)'),...
        string('GPS Glitch'), string('Bad Accelerometer Data')];
    
    for i = 1:12
        filter_fault(i) = subplot(4,3,i);
        plot(sysvector('estimator_status_0.solution_status_flags').Time,solution_status_flags(:,i));
        title(titles(i))
    end
    
    linkaxes(filter_fault(:),'x');
    set(filter_fault(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig5);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
    
    % test ratios
    fig6 = figure();
    fig6.Name = 'Estimator Test Ratios';
    
    test_ratios(1) = subplot(3,2,1);
    plot(sysvector('estimator_status_0.mag_test_ratio').Time,sysvector('estimator_status_0.mag_test_ratio').Data);
    title('Mag Test Ratio')
    
    test_ratios(2) = subplot(3,2,2);
    plot(sysvector('estimator_status_0.vel_test_ratio').Time,sysvector('estimator_status_0.vel_test_ratio').Data);
    title('Vel Test Ratio')
    
    test_ratios(3) = subplot(3,2,3);
    plot(sysvector('estimator_status_0.pos_test_ratio').Time,sysvector('estimator_status_0.pos_test_ratio').Data);
    title('Pos Test Ratio')
    
    test_ratios(4) = subplot(3,2,4);
    plot(sysvector('estimator_status_0.hgt_test_ratio').Time,sysvector('estimator_status_0.hgt_test_ratio').Data);
    title('Hgt Test Ratio')
    
    test_ratios(5) = subplot(3,2,5);
    plot(sysvector('estimator_status_0.tas_test_ratio').Time,sysvector('estimator_status_0.tas_test_ratio').Data);
    title('Tas Test Ratio')
    
    test_ratios(6) = subplot(3,2,6);
    plot(sysvector('estimator_status_0.hagl_test_ratio').Time,sysvector('estimator_status_0.hagl_test_ratio').Data);
    title('Hagl Test Ratio')

    linkaxes(test_ratios(:),'x');
    set(test_ratios(:), 'XGrid','on','YGrid','on','ZGrid','on');
    dcm_obj = datacursormode(fig6);
    set(dcm_obj,'UpdateFcn',@HighPrecisionTooltipCallback);
else
    disp('EstimatorStatusBitsPlots: The estimator status was not logged so no plots are displayed.');
end