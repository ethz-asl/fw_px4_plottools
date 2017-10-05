function kmldoc(topic)
    if nargin==0 || isempty(topic)
        topic = 'kml.contents';
    end
    if length(topic) > 2  && ischar(topic) && strcmpi(topic(1:3),'kml')
        html = [topic '.html'];
        htmlFile = which(html);
        % Display the results.
        if isempty(htmlFile)
            error('"%s.html" not found, check the instalation of your KML toolbox.',topic);
        else
            web(htmlFile,'-helpbrowser');
        end        
    else
        doc(topic);
    end
end