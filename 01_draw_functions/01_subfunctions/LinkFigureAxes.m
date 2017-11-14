%% ************************************************************************
%  LinkFigureAxes
%  ************************************************************************
%  Link the axes of different figures.

function LinkFigureAxes(plotvector)
    ax = [];
    for n_fig = plotvector.figures_list
        try
            ax(end + 1) = gca(n_fig);
        catch
            % do nothing (could display here that figure does not exist)
        end
    end
    linkaxes(ax);
end