%% create a KML toolbox object
k = kml('kml toolbox tests using radians'); %create an kmltoolbox object
k.useRadians;
%% plot
f = k.createFolder('kml.plot');
t = linspace(0,2*pi,1000);
f.plot(20*t, cos(t/2), 'altitude', 1e6, 'altitudeMode','absolute','lineWidth',10, 'name','plot test');

%% plot3
f = k.createFolder('kml.plot3');
t = linspace(0,2*pi,1000);
f.plot3(10*t, cos(t/8), sin(t/2)*1e6, 'altitudeMode','absolute','lineWidth',5,'lineColor','FF00FF00', 'name','plot3 test');

%% point
f = k.createFolder('kml.point');
f.point(rand(100,1)*2*pi-pi,rand(100,1)*pi-pi/2,ones(100,1)*1e6,'iconURL','wht-stars','iconScale',10);

%% poly
f = k.createFolder('kml.poly');
% tp = linspace(0,2*pi,50);
% for i =1:50
%     r = rand(2,1)*2-1;
%     f.poly(r(1)*180 + 2.8*sin(tp),r(2)*60 + 2*sin(2*tp),'altitude', 1e5,'lineColor','FFFFFFFF','lineWidth',5,'polyColor','FF0000FF','extrude',true);
% end

tp = linspace(0,2*pi,20);
cities = load('cities.mat'); %Population Data Copyright notice: © by Stefan Helders www.world-gazetteer.com
cities = cities.cities;

cmap = jet(20);
pops = linspace(min([cities(:).Population]),max([cities(:).Population]),20);

for i = 1:numel(cities)
    color = [interp1(pops,cmap(:,1),cities(i).Population) interp1(pops,cmap(:,2),cities(i).Population) interp1(pops,cmap(:,3),cities(i).Population)];

    color = min(max(floor(color*255),0),255);
    [r,g,b,a] = deal(color(1),color(2),color(3),255); 
    [rhex, ghex, bhex, ahex ]= deal(dec2hex(r),dec2hex(g),dec2hex(b),dec2hex(a));
    if length(rhex)==1,rhex=['0' rhex];end
    if length(ghex)==1,ghex=['0' ghex];end
    if length(bhex)==1,bhex=['0' bhex];end
    if length(ahex)==1,ahex=['0' ahex];end

    colorHex = [ahex bhex ghex rhex];
    
    f.poly(cities(i).Longitude*pi/180 + 0.01*sin(tp),cities(i).Latitude*pi/180 + 0.01*cos(tp),'altitude',sqrt(cities(i).Population)*1e2,'altitudeMode','relativeToGround','extrude',true,'name',[cities(i).Name '[' cities(i).Country ']'],'lineWidth',0,'polyColor',colorHex);
end

%% poly3
f = k.createFolder('kml.poly3');
t = linspace(0,2*pi,50);
for i = 1:numel(t)-1;
    f.poly3([t(i) t(i+1) t(i+1) t(i) t(i)],([0 0 1 1 0]+5)*pi/180, [1 1 3 3 1]*1e6,'polyColor','BB00FF00','extrude',false,'lineWidth',0);
    f.poly3([t(i) t(i+1) t(i+1) t(i) t(i)],([0 0 1 1 0]-5)*pi/180, [1 1 3 3 1]*1e6,'polyColor','BB00FF00','extrude',false,'lineWidth',0);
end
%     

%% quiver
f = k.createFolder('kml.quiver');
[x,y] = meshgrid(-5:.2:5,-2:.15:2);
z = x .* exp(-x.^2 - y.^2) + y.*sin(x);
[px,py] = gradient(z,.2,.15);
f.quiver(x*pi/36,y*pi/18,px*pi/22,py*pi/22);


%% quiver3d
f = k.createFolder('kml.quiver3d');
[x,y] = meshgrid(-2:.2:2,-1:.15:1);
z = x .* exp(-x.^2 - y.^2);
[px,py] = gradient(z,.2,.15);

f.quiver3d(x*pi/18,y*pi/18,1e5+x*0,1000*px*pi/180,1000*py*pi/180,-cosd(x)*1e2*pi/180,'scale',1500);
     
%% contour
f = k.createFolder('kml.contour');
[x,y,z] = peaks(200);
f.contour(pi*x/3,pi*y/6,z,'numberOfLevels',40);

%% contour3
f = k.createFolder('kml.contour3');
[x,y,z] = peaks(200);
f.contour3(pi*x/3,pi*y/6,1e5*z+1e4,'numberOfLevels',40,'altitudeMode','absolute');
 
%% scatter
f = k.createFolder('kml.scatter');
f.scatter(2*pi*rand(100,1)-pi,pi*rand(100,1)-pi/2,'iconScale',linspace(5,10,100).','iconColor',[linspace(0,1,100).' zeros(100,2) linspace(0,1,100).']);

%% scatter3
f = k.createFolder('kml.scatter3');
f.scatter3(2*pi*rand(100,1)-pi,pi*rand(100,1)-pi/2,linspace(5,10,100).','iconScale',linspace(5,10,100).','iconColor',[zeros(100,1)  linspace(0,1,100).' zeros(100,1) linspace(0,1,100).']);

%% text
f = k.createFolder('kml.text');
for lat = -80:20:80
    for long = -180:20:170
        f.text(long*pi/180,lat*pi/180,1e5,sprintf('Lat %g° Long %g°',lat,long),'labelScale',3);
    end
end

%% model
f = k.createFolder('kml.model');
f.model(-32*pi/180,10*pi/180,50000,-145,0,0,'model',fullfile(fileparts(mfilename('fullpath')),'A320.dae'),'scale',10000);

%% overlay
f = k.createFolder('kml.overlay');
dimg = pow2(get(0,'DefaultImageCData'),47);
ee = reshape([47,42,37,36,35,34,33,28,23,18,13,9,5,1,-1,51,46,41,36,35,34,33,32,27,22,16,12,8,4,-1,33.8562,43.3,41.3,54.7167,23.35,17,21.17,33.8562,49.45,41.3,40.4296,35.7143,33.8562,35.0839,-22.007,-85.2163,-70.3504,-72.3504,20.5167,90.7083,17,72.83,-83.2163,11.0833,-70.3504,-79.9191,-83.5102,-84.2163,-106.6186,-47.8974],[15 4]);
en = reshape([111,[1:2]*0+77,72,77,116,97,121,77,56,46,87,97,116,111,108,[1:2]*0+84,105,84,104,32,111,101,45,46,105,110,104,32,100,[1:2]*0+87,108,87,101,83,117,108,98,46,108,100,101,112,101,[1:2]*0+32,98,[1:2]*0+32,105,110,97,105,32,107,[1:2]*0+32,111,115,[1:2]*0+100,101,111,110,97,103,110,116,111,105,116,111,114,116,[1:2]*0+111,114,108,117,109,101,99,115,[1:2]*0+110,104,114,99,32,[1:2]*0+103,116,100,109,101,115,111,[1:2]*0+32,115,101,105,111,108,[1:4]*0+32,98,115,116,108,115,116,111,32,103,32,105,49,50,109,108,[1:2]*0+101,32,105,116,104,110,98,105,97,116,[1:2]*0+-1,97,111,114,32,108,97,105,101,44,105,110,109,116,[1:2]*0+-1,116,103,-1,109,105,44,108,[1:2]*0+32,103,[1:2]*0+97,108,[1:2]*0+-1,114,111,-1,97,116,32,108,97,71,32,108,114,101,[1:2]*0+-1,105,[1:2]*0+-1,103,116,98,32,114,105,69,32,101,32,[1:2]*0+-1,120,[1:2]*0+-1,105,108,121,114,116,118,100,119,108,69,[1:5]*0+-1,99,101,32,117,32,101,100,97,111,100,[1:5]*0+-1,[1:2]*0+32,65,108,111,110,105,115,44,100,[1:5]*0+-1,115,69,108,101,102,115,110,[1:2]*0+32,105,[1:5]*0+-1,113,100,98,115,[1:2]*0+32,115,97,109,110,[1:5]*0+-1,117,100,114,-1,77,38,-1,108,97,115,[1:5]*0+-1,97,105,101,-1,65,32,-1,119,115,[1:6]*0+-1,114,110,99,-1,84,70,-1,97,32,[1:6]*0+-1,101,115,104,-1,76,111,-1,121,110,[1:8]*0+-1,116,-1,65,114,-1,115,227,[1:8]*0+-1,32,-1,66,115,-1,32,111,[1:8]*0+-1,68,[1:2]*0+-1,121,-1,116,32,[1:8]*0+-1,117,[1:2]*0+-1,116,-1,104,111,[1:8]*0+-1,114,[1:2]*0+-1,104,-1,101,32,[1:8]*0+-1,101,[1:2]*0+-1,101,-1,114,100,[1:8]*0+-1,114,[1:4]*0+-1,101,111,[1:14]*0+-1,32,[1:14]*0+-1,67,[1:14]*0+-1,65,[1:14]*0+-1,65,[1:14]*0+-1,83,[1:14]*0+-1,79],[15 35]);
for i = 1:size(ee,1)
    fn = sprintf('ee%i.png',i);
    if i<size(ee,1)
        img = bitslice(dimg,ee(i,1),ee(i,2));
        img = img-min(img(:))./(max(img(:))-min(img(:)));
        imwrite(img*2^6,gray,fn);    
    else
        r = bitslice(dimg,0,0);
        g = bitslice(dimg,17,17);
        b = bitslice(dimg,34,34);
        
        imwrite(cat(3,r,g,b),fn)
    end
    n = char(en(i,:));
    f.overlay((ee(i,4)-0.5)*pi/180,(ee(i,4)+0.5)*pi/180,(ee(i,3)-0.5/secd(ee(i,3)))*pi/180,(ee(i,3)+0.5/secd(ee(i,3)))*pi/180,'file',fn,'color','FFFFFFFF','name',n);
end

%% transfer
f = k.createFolder('kml.transfer');
fh = figure;
ax = gca;

%plot the Butterfly curve, en.wikipedia.org/wiki/Butterfly_curve_(transcendental)
t = linspace(0,20*pi,10000);
ph = plot(ax,pi/180*sin(t).*(exp(cos(t))-2*cos(4*t)-sin(t/12).^5), pi/180*cos(t).*(exp(cos(t))-2*cos(4*t)-sin(t/12).^5));
xlabel('longitude');
ylabel('latitude');

%transfer the figure with the axes information
f.transfer(ax,'keepAxis',true);

set(ph,'Color','r','Marker','o','LineStyle','none')
f.transfer(ax,'keepAxis',false,'transparentBG',true,'altitudeMode','absolute','altitude',10000);

close(fh);

%% Run the KML file in Google Earth
k.run;