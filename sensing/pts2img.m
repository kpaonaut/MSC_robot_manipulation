% Wenjie Chen, FANUC Corporation, 2016/02/23
% Convert 2D point clouds to bw image

function img = pts2img(pt2d, imgsize)

img = zeros(imgsize, imgsize);  % consider only the square image
ptN = size(pt2d, 1);
pt2d = pts_normalize(pt2d, 'center') * imgsize;

for i = 1:ptN
    x = ceil(pt2d(i, 1));  
    y = imgsize - floor(pt2d(i, 2));
    if x < 1,  x = 1;  end
    if y < 1,  y = 1;  end
    if x > imgsize,  x = imgsize;  end
    if y > imgsize,  y = imgsize;  end
    img(y, x) = img(y, x) + 1;
end

img = uint8(round(img/max(img(:))*255));

% figure; plot(pt2d(:,1), pt2d(:,2), 'o'); axis equal;
% figure; imshow(img);

end