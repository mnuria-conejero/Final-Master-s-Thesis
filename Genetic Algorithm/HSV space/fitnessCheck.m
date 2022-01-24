function check=fitnessCheck(hueThresholdLow,hueThresholdHigh,saturationThresholdLow,saturationThresholdHigh,valueThresholdLow,valueThresholdHigh)
% hueThresholdLow = 0.1857;
% hueThresholdHigh = 0.3202;
% saturationThresholdLow = 0.2234;
% saturationThresholdHigh = 0.9992;
% valueThresholdLow = 0.4230;
% valueThresholdHigh = 0.9986;


f=0;
muestras_dataset=150;
av_similarity=[];
%mapped=map(T,0,255,0,1);

currentFolder = pwd;
path_rgb = strcat(currentFolder,'\datasets\realsense\All\color\');
path_binary = strcat (currentFolder,'\datasets\realsense\All\GT\');

for i=f:(muestras_dataset-1)
    
    %Image_rgb to grayscale
    image_rgb=imread (strcat(path_rgb,num2str(i),'.jpg'));
    image_HSV = rgb2hsv(image_rgb);
    H=image_HSV(:,:,1);
    S=image_HSV(:,:,2);
    V=image_HSV(:,:,3);
    
    % Now apply each color band's particular thresholds to the color band
	hueMask = (H >= hueThresholdLow) & (H <= hueThresholdHigh);
	saturationMask = (S >= saturationThresholdLow) & (S <= saturationThresholdHigh);
	valueMask = (V >= valueThresholdLow) & (V <= valueThresholdHigh);
    coloredObjectsMask = hueMask & saturationMask & valueMask;

    new_image_binary=rescale(coloredObjectsMask);
    imwrite(new_image_binary,strcat('image_binary',num2str(i),'.png'));
    
    % Cargamos imagen binaria dataset
    image_binary_rgb = imread (strcat(path_binary,num2str(i),'.jpg'));
    image_binary = rescale(imbinarize(im2gray(image_binary_rgb)));
    
    %Valores verdaderos blancos
    vvb=sum(image_binary(:));
    
    %Valores acertados blancos
    bothTrue= new_image_binary & image_binary;
    vab=sum(bothTrue(:));
    
    %Valores falsos blancos
    vfb=sum(new_image_binary(:))-vab;
    
    %Fitness (vab/(vvb+vfb))
  
    av_similarity = cat(1,av_similarity,vab/(vvb+vfb));
    
end
check=(sum(av_similarity)/muestras_dataset)*100;
disp(strcat('Fitness: ',num2str(check),'%'));
end

