function IridiumSBDStatusPlots(sysvector, topics)
% Display the power plots

if ~topics.iridiumsbd_status.logged
    return
end

fig1 = figure();
fig1.Name = 'Iridium SatCom Plots';

isbd(1) = subplot(2,3,1);
hold on;
stairs(sysvector.iridiumsbd_status_0.failed_sbd_sessions.Time, sysvector.iridiumsbd_status_0.failed_sbd_sessions.Data);
stairs(sysvector.iridiumsbd_status_0.successful_sbd_sessions.Time, sysvector.iridiumsbd_status_0.successful_sbd_sessions.Data);
hold off;
legend('failed sessions', 'successful sessions');

isbd(end+1) = subplot(2,3,2);
hold on;
stairs(sysvector.iridiumsbd_status_0.ring_pending.Time, sysvector.iridiumsbd_status_0.ring_pending.Data);
stairs(sysvector.iridiumsbd_status_0.tx_session_pending.Time, sysvector.iridiumsbd_status_0.tx_session_pending.Data);
stairs(sysvector.iridiumsbd_status_0.rx_session_pending.Time, sysvector.iridiumsbd_status_0.rx_session_pending.Data);
stairs(sysvector.iridiumsbd_status_0.tx_buf_write_pending.Time, sysvector.iridiumsbd_status_0.tx_buf_write_pending.Data);
stairs(sysvector.iridiumsbd_status_0.rx_read_pending.Time, sysvector.iridiumsbd_status_0.rx_read_pending.Data);
hold off;
legend('ring pending', 'tx session pending', 'rx session pending', 'tx buf write pending', 'rx read pending');

isbd(end+1) = subplot(2,3,3);
hold on;
stairs(sysvector.iridiumsbd_status_0.rx_buf_end_index.Time, sysvector.iridiumsbd_status_0.rx_buf_end_index.Data);
stairs(sysvector.iridiumsbd_status_0.rx_buf_read_index.Time, sysvector.iridiumsbd_status_0.rx_buf_read_index.Data);
hold off;
legend('rx end index', 'rx read index');

isbd(end+1) = subplot(2,3,4);
hold on;
stairs(sysvector.iridiumsbd_status_0.num_tx_buf_reset.Time, sysvector.iridiumsbd_status_0.num_tx_buf_reset.Data);
stairs(sysvector.iridiumsbd_status_0.tx_buf_write_index.Time, sysvector.iridiumsbd_status_0.tx_buf_write_index.Data);
hold off;
legend('num tx buf reset', 'tx buf index');

isbd(end+1) = subplot(2,3,5);
session_pending = or(sysvector.iridiumsbd_status_0.tx_session_pending.Data, sysvector.iridiumsbd_status_0.rx_session_pending.Data);
hold on;
stairs(sysvector.iridiumsbd_status_0.failed_sbd_sessions.Time(2:end), diff(sysvector.iridiumsbd_status_0.failed_sbd_sessions.Data.*2.0));
stairs(sysvector.iridiumsbd_status_0.successful_sbd_sessions.Time(2:end), diff(sysvector.iridiumsbd_status_0.successful_sbd_sessions.Data.*1.5));
stairs(sysvector.iridiumsbd_status_0.successful_sbd_sessions.Time, session_pending);
hold off;
legend('failed session', 'successful session', 'session pending');

isbd(end+1) = subplot(2,3,6);
hold on;
stairs(sysvector.iridiumsbd_status_0.signal_quality.Time, sysvector.iridiumsbd_status_0.signal_quality.Data);
hold off;
legend('signal quality');

linkaxes(isbd(:),'x');
set(isbd(:), 'XGrid','on','YGrid','on','ZGrid','on');
end

