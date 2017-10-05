function [ret] = TimeseriesSubtraction(ts1, ts2, dt)
% resample the timeseries and then subtract the data in ts2 from ts1

ts2_neg = timeseries(-ts2.Data, ts2.Time);
ret = TimeseriesAddition(ts1, ts2_neg, dt);