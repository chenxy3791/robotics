% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
load('mu_sigma.mat','mu','sig'); 

thre = 0.00002;

sig_inv = inv(sig);
sig_det = det(sig);
sig_det_sqrt = sqrt(sig_det);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
D      = length(mu)
prob_y = zeros(size(I,1),size(I,2));
size(I)
assert( D == size(I,3) );
for m = 1:1:size(I,1)
    for n = 1:1:size(I,2)
        pixel = I(m,n,:);
        pixel = pixel(:);
        
        dev   = double(pixel) - mu;
        prob_y(m,n) = (1/(2*pi)^(D/3) * (1/sig_det_sqrt) * exp((-1/2)*(dev')*(sig_inv)*(dev)));
    end
end    

%figure;
%meshgrid = mesh([1:1:size(I,1)],[1:1:size(I,2)]);
%plot(meshgrid, prob_y)
%mesh(prob_y); title('Probability of each pixel belong to yellow ball');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
bw = prob_y > thre; % Black has smallest value. Pick up black part.
% figure, imshow(bw); 
% title('Image of possible ball area')

% create new empty binary image
bw_biggest = false(size(bw));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
% CC = bwconncomp(BW) returns the connected components CC found in the binary image BW. 
% bwconncomp uses a default connectivity of 8 for two dimensions, 26 for three dimensions, 
% and conndef(ndims(BW),'maximal') for higher dimensions.
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
%figure,
%imshow(bw_biggest); hold on;
%title('biggest circle in black-white');

% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
% Calculate centroids for connected components in the image using regionprops. The regionprops function returns the centroids in a structure array.
S    = regionprops(CC,'Centroid');
loc  = S(idx).Centroid;
%plot(loc(1), loc(2),'r+');

segI = bw_biggest;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
