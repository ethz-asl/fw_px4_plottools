function MissionResultPlots(sysvector)
% Display the mission result

fig1 = figure();
fig1.Name = 'Mission Result';

mission_results(1) = subplot(3,1,1);
hold on;
plot(sysvector('mission_result_0.seq_reached'));
plot(sysvector('mission_result_0.seq_current'));
plot(sysvector('mission_result_0.seq_total'));
hold off;
legend('reached', 'current', 'total');
title('Mission Item Index')

mission_results(2) = subplot(3,1,2);
plot(sysvector('mission_result_0.item_do_jump_remaining').Time,sysvector('mission_result_0.item_do_jump_remaining').Data);
title('Jumps Remaining')

mission_results(3) = subplot(3,1,3);
hold on;
plot(sysvector('mission_result_0.valid'));
plot(sysvector('mission_result_0.warning'));
plot(sysvector('mission_result_0.finished'));
plot(sysvector('mission_result_0.failure'));
plot(sysvector('mission_result_0.stay_in_failsafe'));
plot(sysvector('mission_result_0.flight_termination'));
plot(sysvector('mission_result_0.item_do_jump_changed'));
hold off;
legend('valid', 'warning', 'finished', 'failure', 'stay_in_failsafe', 'flight_termination', 'item_do_jump_changed');
title('Flags')
ylim([-0.1, 1.1])

linkaxes(mission_results(:),'x');
set(mission_results(:), 'XGrid','on','YGrid','on','ZGrid','on');
end

