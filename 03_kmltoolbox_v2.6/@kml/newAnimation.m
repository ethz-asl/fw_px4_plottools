function f = newAnimation(this,animName)
%KML.NEWANIMATION(folderName) Creates an animation storyboard inside an kml (or another folder).
%  Example of use:
%
%   Copyright 2013 Rafael Fernandes de Oliveira (rafael@rafael.aero)
%   $Revision: 2.6 $  $Date: 2013/05/17 17:17:17 $

    if nargin < 2
        animName = 'Unnamed Animation';
    end
    f = kmlAnimation(this,animName);
end