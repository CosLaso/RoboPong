%% Load image of puzzle pieces
I = imread('Puzzle.png');
I = rgb2gray(I);

%% Generate binary image of piece regions
%Detect entire piece
[~,threshold] = edge(I,'sobel');
f_Factor = 0.1;
I_BW = edge(I,'sobel',threshold * f_Factor);

%Dilate region
SE_90 = strel('line',3,90);
SE_0 = strel('line',3,0);
I_BW_dilated = imdilate(I_BW,[SE_90 SE_0]);

%Fill gaps
I_BW_filled = imfill(I_BW_dilated,'holes');

%Remove boarders
I_BW_NoBoarders = imclearborder(I_BW_filled,4);

%Smooth regions
SE_D = strel('diamond',1);
I_BW_Final = imerode(I_BW_NoBoarders,SE_D);
I_BW_Final = imerode(I_BW_Final,SE_D);

%% Seperate pieces into individual images
%Get number of pieces and create a cell array
B = bwboundaries(I_BW_Final);
pieces_individual = cell(1,length(B));
pieces_individual_mask = cell(1,length(B));

%Create a mask for each piece
for k = 1:length(B)
    pieces_individual_mask{k} = uint8(bwareafilt(I_BW_Final,k));
end

for k = length(B) : -1 : 2
    pieces_individual_mask{k} = pieces_individual_mask{k} - pieces_individual_mask{k - 1};
end

%Use the mask to seperate the pieces
for k = 1:length(B)
    Image_holder = I;
    Image_holder = Image_holder .* pieces_individual_mask{k};
    pieces_individual{k} = Image_holder;
end

%% Feature matching
%Load completed image
I1gs = imread('Puzzle_complete.png');
I1gs = rgb2gray(I1gs);
imshow(I1gs)
hold on

for k = 1 : length(B)
    %load individual piece
    I2gs = pieces_individual{k};
    
    %Find centre of individual piece
    s = regionprops(pieces_individual_mask{k},'centroid');

    %Feature matching process
    points1 = detectORBFeatures(I1gs);
    points2 = detectORBFeatures(I2gs);
    [features1, validPoints1] = extractFeatures(I1gs, points1);
    [features2, validPoints2] = extractFeatures(I2gs, points2);
    indexPairs = matchFeatures(features1,features2);
    matchedPoints1 = validPoints1(indexPairs(:,1));
    matchedPoints2 = validPoints2(indexPairs(:,2));

    %Calculate centre of individual piece on completed puzzle image
    x_coordinate = matchedPoints1.Location(end,1) + (s.Centroid(1) - matchedPoints2.Location(end,1));
    y_coordinate = matchedPoints1.Location(end,2) + (s.Centroid(2) - matchedPoints2.Location(end,2));
    
    %Label each individual piece
    text_label = num2str(k);
    text(x_coordinate,y_coordinate,text_label,'Fontsize',20,'Color','red')
end

figure
%Display results
montage(pieces_individual, 'BackgroundColor', 'r', 'BorderSize', [1 1])