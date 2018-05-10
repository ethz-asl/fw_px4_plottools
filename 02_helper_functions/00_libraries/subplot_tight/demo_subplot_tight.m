% DEMO of subplot Vs subplot_tight
func_hndl={@subplot;@subplot_tight};
filePath = mfilename('fullpath');
[filePath, ~, ~] = fileparts(filePath);
dirFilesJPEG=dir( fullfile(filePath,'*.jpg') );
for p=1:length(dirFilesJPEG)
   img=imread(dirFilesJPEG(p).name);
   for func_ind=1:length(func_hndl)
      figure(func_ind)
      func_hndl{func_ind}(3,3,p);
      imshow(img);
   end
end
peaks_data=peaks(50);
for func_ind=1:length(func_hndl);
   figure(func_ind)
   func_hndl{func_ind}(3,3,[8,9]);
   surf(peaks_data);
   axis tight;
   xlabel('x');
   ylabel('y');
   zlabel('z');
   title('Peaks Plot','FontSize', 14);
end
set(1,'Name','Matlab SUBPLOT');
set(2,'Name','Our subplot_tight');
set(1:2,'MenuBar','none')