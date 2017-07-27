%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Get Color Mask by HSV Threshold
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016, based on ColorThreshold toolbox       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [threshed_img, thresh_mask, H_mask, S_mask, V_mask] = ColorThreshold_thresh(Original_imgRGB, HSV_min, HSV_max, IsHSplit)

Hmin = HSV_min(1); Smin = HSV_min(2); Vmin = HSV_min(3);
Hmax = HSV_max(1); Smax = HSV_max(2); Vmax = HSV_max(3);

Original_imgHSV = Original_imgRGB;
for in = 1:numel(Original_imgRGB(:,1,1))
    for ij = 1:numel(Original_imgRGB(1,:,1))
        R = Original_imgRGB(in,ij,1);
        G = Original_imgRGB(in,ij,2);
        B = Original_imgRGB(in,ij,3);
        [H, S, V] = rgb2hsvGE423(R,G,B);
        Original_imgHSV(in,ij,1) = H;
        Original_imgHSV(in,ij,2) = S;
        Original_imgHSV(in,ij,3) = V;
    end
end

if IsHSplit
    H_mask = (Original_imgHSV(:,:,1) <= Hmin | Original_imgHSV(:,:,1) >= Hmax);
    S_mask = Original_imgHSV(:,:,2) >= Smin & Original_imgHSV(:,:,2) <= Smax;
    V_mask = Original_imgHSV(:,:,3) >= Vmin & Original_imgHSV(:,:,3) <= Vmax;
    thresh_mask = (Original_imgHSV(:,:,1) <= Hmin | Original_imgHSV(:,:,1) >= Hmax) ...
        & Original_imgHSV(:,:,2) >= Smin & Original_imgHSV(:,:,2) <= Smax ...
        & Original_imgHSV(:,:,3) >= Vmin & Original_imgHSV(:,:,3) <= Vmax;
else
    H_mask = Original_imgHSV(:,:,1) >= Hmin & Original_imgHSV(:,:,1) <= Hmax;
    S_mask = Original_imgHSV(:,:,2) >= Smin & Original_imgHSV(:,:,2) <= Smax;
    V_mask = Original_imgHSV(:,:,3) >= Vmin & Original_imgHSV(:,:,3) <= Vmax;
    thresh_mask = Original_imgHSV(:,:,1) >= Hmin & Original_imgHSV(:,:,1) <= Hmax ...
        & Original_imgHSV(:,:,2) >= Smin & Original_imgHSV(:,:,2) <= Smax ...
        & Original_imgHSV(:,:,3) >= Vmin & Original_imgHSV(:,:,3) <= Vmax;
end

threshed_img = cast(repmat(thresh_mask,[1,1,3]),'uint8').*Original_imgHSV; % threshold the HSV image
%image(threshed_img)

end


function[H,S,V] = rgb2hsvGE423(R,G,B)

red = (double(R)-16)*255/224;
green = (double(G)-16)*255/224;
blue = (min(double(B)*2,240)-16)*255/224;
minV = min(red,min(green,blue));
value = max(red,max(green,blue));
delta = value - minV;
if(value~=0)
    sat = (delta*255) / value;% s
    if (delta ~= 0)
        if( red == value )
            hue = 60*( green - blue ) / delta;		% between yellow & magenta
        elseif( green == value )
            hue = 120 + 60*( blue - red ) / delta;	% between cyan & yellow
        else
            hue = 240 + 60*( red - green ) / delta;	% between magenta & cyan
        end
        if( hue < 0 )
            hue = hue + 360;
        end
    else
        hue = 0;
        sat = 0;
    end
else
    % r = g = b = 0		// s = 0, v is undefined
    sat = 0;
    hue = 0;
end
H = max(min(floor(((hue*255)/360)),255),0);        % Hmax is 256 in rgb2hsvGE423
%H = max(min(floor(hue),359),0);
S = max(min(floor(sat),255),0);
V = max(min(floor(value),255),0);
end