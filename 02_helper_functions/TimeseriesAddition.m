function [ret] = TimeseriesAddition(ts1, ts2, dt)
% resample the timeseries and then add the data from ts2 and ts1

tstart = max(ts1.Time(1), ts2.Time(1));
tend = min(ts1.Time(end), ts2.Time(end));
t_interpolation = tstart:dt:tend;

ts1_resampled = resample(ts1, t_interpolation);
ts2_resampled = resample(ts2, t_interpolation);

ret = timeseries(ts1_resampled.Data + ts2_resampled.Data, ts1_resampled.Time);
% TODO determine if the interpolation method should be copied from one of
% the two time series.