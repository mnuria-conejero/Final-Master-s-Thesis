function val = tracklsq3(params)

%Valores entrada: posible solución al problema
hueThresholdLow = params(1);
hueThresholdHigh = params(2);
saturationThresholdLow = params(3);
saturationThresholdHigh = params(4);
valueThresholdLow = params(5);
valueThresholdHigh = params(6);


%% Path imágenes del dataset
currentFolder = pwd;
path_rgb = strcat(currentFolder,'\datasets\realsense\All\color\');
path_binary = strcat (currentFolder,'\datasets\realsense\All\GT\');


%% RGB-grayscale-binary-correlacion
%mapped=map(T,0,255,0,1);
muestras_dataset=150;
n=0;
similarity=[];

for i=n:(muestras_dataset-1)
    
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
  
    similarity = cat(1,similarity,vab/(vvb+vfb));
    
end
%% Correlacion escala grises-binaria dataset
av_similarity=sum(similarity)/muestras_dataset;
%disp(av_similarity);

val = (-1*av_similarity);