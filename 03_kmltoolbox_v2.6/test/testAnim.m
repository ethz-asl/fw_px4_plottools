time = linspace(0,4*pi,1000);
longitude = cos(time)*1;
latitude = sin(time)*1;
altitude = 1e3 + 0.*time;

yaw   = 360-mod((time)*180/pi,720);
gamma = 0.*time;
roll  = 0.*time -20;

k = kml('testAnimation');
k.useDegrees;

N = numel(longitude);

yaw = yaw - 180; %This is to correct the mis-orientation in the A320 model, you can ignore this for other models - or fix the model
modelA320 = k.model(longitude(1),latitude(1),altitude(1),yaw(1),gamma(1),roll(1),'model','A320.dae','scale',10);
anim = k.newAnimation('Flight');

time = time.*100;

for i = 2:N
   dT = time(i) - time(i-1); 
   anim.updateLocation(modelA320,dT,longitude(i),latitude(i),altitude(i));
   anim.updateOrientation(modelA320,dT,yaw(i),gamma(i),roll(i));
   
   anim.flyToLookAt(dT, longitude(i), latitude(i),1e4)
end

anim.flyToLookAt(10, mean(longitude),mean(latitude),1e5);

k.run;
