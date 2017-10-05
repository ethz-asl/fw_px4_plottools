if isempty(which('kml'))
    disp(repmat('_',1,80))
    disp('     _  ____  __ _      _____         _ _             ')
    disp('    | |/ /  \/  | |    |_   _|__  ___| | |__  _____ __')
    disp('    | '' <| |\/| | |__    | |/ _ \/ _ \ | ''_ \/ _ \ \ /')
    disp('    |_|\_\_|  |_|____|   |_|\___/\___/_|_.__/\___/_\_\')
    disp(sprintf('\n'));
    disp('          Thanks for downloading the KML toolbox v1.4!');
    disp(sprintf('\n'));
    disp('For some cool examples on how to use the KML toolbox,');
    disp(sprintf('please check the file RunTests.m located in the folder %s',pwd));
    disp(repmat('_',1,80));
else
    tmp = getpref('kmltoolbox','ShowDisclaimer',true);
    setpref('kmltoolbox','ShowDisclaimer',true);
    display(kml);
    setpref('kmltoolbox','ShowDisclaimer',tmp);
end