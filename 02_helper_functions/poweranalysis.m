%%
% AircraftPowerPolar_Analysis.m
%
% Purpose: Analyses the aircraft power consumption vs. flight speed from
% recorded log data.
% Note: You have to set the time intervals yourself in t1 and t2!

%% Setup
clearvars -except sysvector time

% Settings
UseLocalTime = 0;

% Flight data
%SS2A TF20180621
t1=[1173 1479 1696 2007 2374 2750 3100 3432 3709 4057 4318 4676];
t2=[1440 1666 1973 2343 2727 3069 3396 3679 4010 4276 4640 5465];
%SS2A TF20180626 F2 13x12 prop
t1=[916 1326 1769 2317];
t2=[1312 1741 2283 2646];
%SS2A TF20180626 F3 13x10 prop
t1=[3264 3615 3970 4281 4585 4930 5422];
t2=[3585 3951 4246 4559 4893 5388 5795];
%SS2A TF20180626 F1 13x11 prop
t1=[1640 2306 2596 2848 3182 3601 3984];
t2=[2271 2557 2822 3142 3550 3957 4375];
%SS2A TF20180627 attempted 6h flight with lots of turbulence
t1=[1608];
t2=[3932];
%SS2A TF20180701 actually achieved 5:30 hour flight with many downdrafts
t1=[856];
t2=[20434];
%cal1=922.0;
%cal2=2306.0;

h1 = t1;

PWRboard = true;
if(PWRboard==true)
    %Precalcs
    m_SystemPower = sysvector.sensor_pwr_brd_0.pwr_brd_mot_l_amp.Data .* sysvector.sensor_pwr_brd_0.pwr_brd_system_volt.Data + ...
            sysvector.sensor_pwr_brd_0.pwr_brd_analog_amp.Data .* sysvector.sensor_pwr_brd_0.pwr_brd_servo_volt.Data + ...
            sysvector.sensor_pwr_brd_0.pwr_brd_digital_amp.Data .* sysvector.sensor_pwr_brd_0.pwr_brd_digital_volt.Data + ...
            sysvector.sensor_pwr_brd_0.pwr_brd_ext_amp.Data .* sysvector.sensor_pwr_brd_0.pwr_brd_system_volt.Data + ...
            sysvector.sensor_pwr_brd_0.pwr_brd_aux_amp.Data .* sysvector.sensor_pwr_brd_0.pwr_brd_digital_volt.Data;
    
    
    
    %mot_l_amp *system_volt + analog_amp * servo_volt + digital_amp * digital_volt + ext_amp * system_volt + aux_amp * digital_volt;
    
%     sysvector.POWB_SERVO1_CUR(sysvector.POWB_SERVO1_CUR>1.0) = 0.0; %Correct for bug in power board
%     sysvector.POWB_SERVO1_CUR(sysvector.POWB_SERVO1_CUR<-0.4) = 0.0; %Correct for bug in power board
%     sysvector.p_servos = sysvector.POWB_SERVO_volt / 1000.0 .* (sysvector.POWB_SERVO1_CUR+sysvector.POWB_SERVO2_CUR+sysvector.POWB_SERVO3_CUR+sysvector.POWB_SERVO4_CUR);
%     sysvector.p_aux = sysvector.POWB_SERVO_volt / 1000.0 .* sysvector.POWB_AUX_CUR;
%     sysvector.p_motor = sysvector.POWB_SYS_volt / 1000.0 .* (sysvector.POWB_MOTL_CUR + sysvector.POWB_MOTR_CUR);
%     %sysvector.p_pixhawk = 0.0; %TODO: We really need to add this measurement!
%     sysvector.p_system = sysvector.p_motor + sysvector.p_servos + sysvector.p_aux;
%     P_batteries = (sysvector.BAT0_I_bat.* sysvector.BAT0_V_bat +sysvector.BAT1_I_bat.* sysvector.BAT1_V_bat +sysvector.BAT2_I_bat.* sysvector.BAT2_V_bat)/1.0E6;
    
    %"Calibrate" our measurements
%     i1=find(time>cal1,1,'first');
%     i2=find(time>cal2,1,'first');
%     P_System_AtRest_mean = mean(sysvector.p_system(i1:i2));
%     P_Batmon_AtRest_mean = mean(P_batteries(i1:i2));
%     P_Bias = -P_Batmon_AtRest_mean - P_System_AtRest_mean;
end

str='';
for i=1:numel(t1)
    idx1bat = find(sysvector.sensor_bat_mon_0.voltage.Time > t1(i),1,'first');
    idx2bat = find(sysvector.sensor_bat_mon_0.voltage.Time > t2(i),1,'first');
    idx1pwr = find(sysvector.sensor_pwr_brd_0.pwr_brd_system_volt.Time > t1(i),1,'first');
    idx2pwr = find(sysvector.sensor_pwr_brd_0.pwr_brd_system_volt.Time > t2(i),1,'first');
    idx1tecs = find(sysvector.tecs_status_0.airspeedSp.Time > t1(i),1,'first');
    idx2tecs = find(sysvector.tecs_status_0.airspeedSp.Time > t2(i),1,'first');
    idx1aspd = find(sysvector.airspeed_0.true_airspeed_m_s.Time > t1(i),1,'first');
    idx2aspd = find(sysvector.airspeed_0.true_airspeed_m_s.Time > t2(i),1,'first');
        
    dt(i) = sysvector.sensor_bat_mon_0.voltage.Time(idx2bat)-sysvector.sensor_bat_mon_0.voltage.Time(idx1bat);

    %mean airspeed, power, etc
    airspeed_ref(i) = mean(sysvector.tecs_status_0.airspeedSp.Data(idx1tecs:idx2tecs));
    v_mean_tas(i) = mean( sysvector.airspeed_0.true_airspeed_m_s.Data(idx1aspd:idx2aspd));
    v_mean_ias(i) = mean( sysvector.airspeed_0.indicated_airspeed_m_s.Data(idx1aspd:idx2aspd));
%     if(newPWRboard == false)
%         U_mean(i) = mean(sysvector.POWS_Main_Volt(idx1:idx2));
%         I_mean(i) = mean(sysvector.POWS_Main_Cur(idx1:idx2));
%         P_mean(i) = mean(sysvector.POWS_Main_Volt(idx1:idx2).*sysvector.POWS_Main_Cur(idx1:idx2));
%     else
%         % Note: For calculations with the new power board, you need to run the Plot/Draw script once to
%         % have the sysvector.p_system, sysvector.p_servos etc. values calculated and stored back into the 
%         % sysvector!
%         
%         U_mean(i) = mean(sysvector.POWB_SYS_volt(idx1:idx2));
%         I_mean(i) = 0.0;
%         P_mean(i) = mean(sysvector.p_system(idx1:idx2))+P_Bias;
%     end
    P_mean_batmon(i) = mean(sysvector.sensor_bat_mon_0.voltage.Data(idx1bat:idx2bat).*sysvector.sensor_bat_mon_0.current.Data(idx1bat:idx2bat)) /1.0E6;
    P_mean_pwrbrd(i) = mean(m_SystemPower(idx1pwr:idx2pwr));
%     uThrot_mean(i) = mean(sysvector.ASLD_uT(idx1:idx2));
%     pitch_mean(i) = mean(sysvector.ASLD_P(idx1:idx2))*180.0/pi();
%     %v_gps_mean(i)=mean(sysvector.v_gps(idx1:idx2))
%     roll_mean(i) = mean(sysvector.ASLD_R(idx1:idx2))*180.0/pi();
%     rollref_mean(i) = mean(sysvector.ASLD_RR(idx1:idx2))*180.0/pi();
%     dh(i) = sysvector.ASLD_h(idx2)-sysvector.ASLD_h(idx1);
%     dh_dot(i) = dh(i) / (t2(i)-t1(i));
%     dh_gps(i) = sysvector.GPS_Alt(idx2)-sysvector.GPS_Alt(idx1);
%     dh_baro(i) = sysvector.SENS_BaroAlt(idx2)-sysvector.SENS_BaroAlt(idx1);
%     
%     eta_climb = 0.72;
%     m_airplane = 6.81;
%     P_corr(i) = (P_mean(i)*dt(i)-dh(i)*m_airplane*9.81/eta_climb)/dt(i);

    %str = [str sprintf('#%u vRef=%.2f: h1/t1/t2/dt=%.2f/%.2f/%.2f/%.2f v/U/I/P/Pcorr=%.2f/%.2f/%.2f/%.2f/%.2f uT/uTSR/duT=%.2f/%.2f/%.2f Rref/R/P/hdot/dh/dh_gps/dh_baro=%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f\n',...
    %    i,airspeed_ref(i),h1(i),t1(i),t2(i),dt(i),v_mean(i),U_mean(i),I_mean(i),P_mean(i),P_corr(i),uThrot_mean(i),uThrotSlewrate(i),duT_mean(i),rollref_mean(i),roll_mean(i),pitch_mean(i),dh_dot(i),dh(i),dh_gps(i),dh_baro(i))];    
    
    str = [str sprintf('#%u vIASRef/vTASRef=%.2f/%.2f: t1/t2/dt=%.2f/%.2f/%.2f vTAS/vIAS/P_bat/P_brd/P_mppt=%.2f/%.2f/%.2f/%.2f/%.2f\n',...
        i,airspeed_ref(i)*v_mean_ias(i)/v_mean_tas(i),airspeed_ref(i),t1(i),t2(i),dt(i),v_mean_tas(i),v_mean_ias(i),P_mean_batmon(i),P_mean_pwrbrd(i),P_mean_pwrbrd(i)-(-P_mean_batmon(i)))];    
 end

display(str);

% 
% 
% %% Plot vs time (preliminary plot)
% figure
% [~,hdl1,hdl2] = plotyy(h1,[P_mean;P_mean_batmon],h1,v_mean);
% legend('Power[W]','v_{mean}[m/s]');
% set(hdl1,'LineWidth',3);
% set(hdl1,'marker','o','color','red');
% set(hdl2,'marker','s','color','green'); 
% 
% %% PLOTTING METHOD 1
% % Note: This plots (1) the raw airspeed vs. power consumption (which
% % therefore does not compensate for altitude changes!) and (2) plots a
% % mean-averaged line through this raw data.
% 
% figure
% P = sysvector.POWS_Main_Volt.*sysvector.POWS_Main_Cur; %Pre-calculate power consumption for every index
% idx1=find(time(:,1)>min(t1),1,'first');
% idx2=find(time(:,1)>max(t2),1,'first');
% altdiff(idx1:idx2) = sysvector.ASLD_h(idx1:idx2)-sysvector.ASLD_hR_t(idx1:idx2);
% idxpos=find(altdiff>0);
% idxneg=find(altdiff<0);
% altdiff(idxpos) = (altdiff(idxpos)) / (max(altdiff(idxpos)));
% altdiff(idxneg) = (altdiff(idxneg)) / (min(altdiff(idxneg)));
% color = zeros (idx2,3);
% color(idxneg,1) = 0.2 + 0.8*altdiff(idxneg);
% color(idxpos,3) = 0.2 + 0.8*altdiff(idxpos);
% 
% % Plot with color according to altitude-difference or not
% if(1)
%     % Constant color plot
%     ax(1) = plot(sysvector.AIRS_IndSpeed(idx1:idx2), P(idx1:idx2),'.','MarkerSize',1,'Color',[0.9 0.9 0.9]);%[0,0,0.1+0.8*altdiff(idx1:idx2)]);
% else
%     % Variable color plot, very slow!
%     for i = idx1:idx2;%18000
%         if(isfinite(altdiff(i)) && (sum(isnan(color(i,:))) == 0)) 
%             ax(1) = plot(sysvector.AIRS_IndSpeed(i), P(i),'.','Color',color(i,:));
%         end
%         if(mod(i,1000)==0) display(i);end
%         hold all
%     end
% end
% 
% % Get power consumption for every single unique airspeed value
% [airspeeds,idx_airspeeds_a,idx_airspeeds_c] = unique(sysvector.AIRS_IndSpeed(idx1:idx2));
% numidx = zeros(numel(airspeeds),1);
% for i = 1:numel(sysvector.AIRS_IndSpeed(idx1:idx2))
%     val = idx_airspeeds_c(i);
%     numidx(val) = numidx(val) + 1;
%     idx{val}(numidx(val)) = idx1 - 1 + i;
% end
% % Mean-average the power consumption over all found indices. 
% for i = 1:numel(airspeeds)
%     P_mean2(i) = mean(P(idx{i}(:)));
% end
% 
% filter_samples=1000;
% coeff_meanavrgfilter = ones(1, filter_samples)/filter_samples;
% P_mean2_avrg = filter(coeff_meanavrgfilter, 1, P_mean2);
% 
% vLimMin=7.05;
% ivLimMin=find(airspeeds>vLimMin,1,'first');
% hold all
% %ax(2) = plot(airspeeds(ivLimMin:end),P_mean2_avrg(ivLimMin:end),'Or');
% 
% 
% %% PLOTTING METHOD 2
% % Note: This is plotting of the per-time-interval pre-averaged value
% % calculated above in the beginning
% 
% %figure
% hold on
% ax(3) = plot(v_mean,P_mean,'-s','MarkerSize',7,'LineWidth',2,'Color','r','MarkerFaceColor','r','MarkerEdgeColor','k');
% hold all
% ax(4) = plot(v_mean,P_mean_batmon,'-s','MarkerSize',7,'LineWidth',2,'Color','g','MarkerFaceColor','g','MarkerEdgeColor','k');
% %ax(4) = plot(v_mean,P_corr,'x','MarkerSize',25,'LineWidth',3,'MarkerEdgeColor','k','MarkerEdgeColor','g');
% legend('P_{mean}','P_{mean,Batmon}');
% hold off
% 
% %legend([ax(1) ax(2) ax(3) ax(4)],'Raw Data','Average over airspeed (from raw data)','Average over airspeed (from time intervals)','Average over airspeed & altitude-corrected (from time intervals)');
% legend('Raw Data','Average over time-interval');
% xlabel('True Airspeed [m/s]');
% ylabel('P_{out} [W]');
% 
% % Plotting options for paper only
% ylim([25 95]);
% 
% figure
% plot(v_mean,uThrot_mean);
% xlabel('Airspeed [m/s]');
% ylabel('u-Throttle [m/s]');


%export_fig Power.png -m8