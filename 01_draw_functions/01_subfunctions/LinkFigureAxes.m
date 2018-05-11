%% ************************************************************************
%  LinkFigureAxes
%  ************************************************************************
%  Link the axes of different figures.

function LinkFigureAxes()
    figHandles = get(groot, 'Children');
    axHandles = [];
    for n_fig = 1:size(figHandles)
        axes = findall(figHandles(n_fig),'type','axes')';

        for i = 1:size(axes)
            [~,el] = view(axes(i));
            if ((logical(sum(campos(axes(i))-camtarget(axes(i))==0)==2)) && ...
                (el == 90))
                axHandles = [axHandles axes(i)];
            end
        end
    end
    linkaxes(axHandles, 'x');
end