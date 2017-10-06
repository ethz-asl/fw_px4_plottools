k = kml('Model Tour');
f = k.createFolder('Above');
t = linspace(0,4*pi,1000);
long = cos(t)*20;
lat = sin(t)*20;
alt = 100000 + 0.*t;

heading = 360-mod((t+pi)*180/pi,720);
tilt    = 0.*t;
roll    = 0.*t -20;
f.modelTour(t*1,long,lat,alt,heading,tilt,roll,'model',fullfile(fileparts(mfilename('fullpath')),'A320.dae'),'scale',10000,'cameraMode','above','cameraDistance',1e7)

f = k.createFolder('Behind');
f.modelTour(t,long,lat,alt,heading,tilt,roll,'model',fullfile(fileparts(mfilename('fullpath')),'A320.dae'),'scale',100,'cameraMode','behind','cameraDistance',1e5)

k.run;