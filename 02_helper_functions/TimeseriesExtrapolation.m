function [ ts_resampled ] = TimeseriesExtrapolation( ts1, ts2 )
% Extrapolates time series 1 (ts1) data via ZOH to bounds of time series 2
% (ts2)

idx0 = find(ts1.Time < ts2.Time(1), 1, 'last');
idxf = find(ts1.Time < ts2.Time(end), 1, 'last');
if idx0==idxf
    ts_resampled = timeseries( ...
        [ts1.Data(idxf); ts1.Data(idxf)], ...
        ts2.Time([1 end]));
else
    ts_resampled = timeseries( ...
        [ts1.Data(idx0:idxf); ts1.Data(idxf)], ...
        [ts2.Time(1); ts1.Time(idx0+1:idxf); ts2.Time(end)]);
end

end

