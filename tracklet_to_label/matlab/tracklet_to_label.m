function tracklet_to_label(base_dir,calib_dir)
output_dir = '/home/dieter/Documents/Results/0059/ground/data';

% clear and close everything
close all; dbstop error; clc;
disp('======= KITTI DevKit Tracklet to labels =======');

if nargin<1
  base_dir = '/home/dieter/Documents/Kitti/2011_09_26/2011_09_26_drive_0059_sync';
end
if nargin<2
  calib_dir = '/home/dieter/Documents/Kitti/2011_09_26';
end
cam = 2; % 0-based index

% get image sub-directory
image_dir = fullfile(base_dir, sprintf('/image_%02d/data', cam));

% get number of images for this dataset
nimages = length(dir(fullfile(image_dir, '*.png')));

% read calibration for the day
[veloToCam, K] = loadCalibration(calib_dir);

% read tracklets for the selected sequence
tracklets = readTracklets([base_dir '/tracklet_labels.xml']); % slow version
%tracklets = readTrackletsMex([base_dir '/tracklet_labels.xml']); % fast version

fprintf('Found %d tracklets\n',size(tracklets));
fprintf('Number of frames: %d\n',nimages-1);

% extract tracklets
% LOCAL OBJECT COORDINATE SYSTEM:
%   x -> facing right
%   y -> facing forward
%   z -> facing up
for it = 1:numel(tracklets)
  
  % shortcut for tracklet dimensions
  w = tracklets{it}.w;
  h = tracklets{it}.h;
  l = tracklets{it}.l;

  % set bounding box corners
  corners(it).x = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]; % front/back
  corners(it).y = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]; % left/right
  corners(it).z = [0,0,0,0,h,h,h,h];
  
  % get translation and orientation
  t{it} = [tracklets{it}.poses(1,:); tracklets{it}.poses(2,:); tracklets{it}.poses(3,:)];
  rz{it} = wrapToPi(tracklets{it}.poses(6,:));
  occlusion{it} = tracklets{it}.poses(8,:);
end

% 3D bounding box faces (indices for corners)
face_idx = [ 1,2,6,5   % front face
             2,3,7,6   % left face
             3,4,8,7   % back face
             4,1,5,8]; % right face

% main loop (start at first image of sequence)
img_idx = 0;
while img_idx ~= nimages
  fileID = fopen(sprintf('%s/%06d.txt', output_dir, img_idx) ,'a');
  fclose(fileID);
  % compute bounding boxes for visible tracklets
  for it = 1:numel(tracklets)
    
    % get relative tracklet frame index (starting at 0 with first appearance; 
    % xml data stores poses relative to the first frame where the tracklet appeared)
    pose_idx = img_idx-tracklets{it}.first_frame+1; % 0-based => 1-based MATLAB index

    % only draw tracklets that are visible in current frame
    if pose_idx<1 || pose_idx>(size(tracklets{it}.poses,2))
      continue;
    end

    % compute 3d object rotation in velodyne coordinates
    % VELODYNE COORDINATE SYSTEM:
    %   x -> facing forward
    %   y -> facing left
    %   z -> facing up
    R = [cos(rz{it}(pose_idx)), -sin(rz{it}(pose_idx)), 0;
         sin(rz{it}(pose_idx)),  cos(rz{it}(pose_idx)), 0;
                             0,                      0, 1];

    % rotate and translate 3D bounding box in velodyne coordinate system
    corners_3D      = R*[corners(it).x;corners(it).y;corners(it).z];
    corners_3D(1,:) = corners_3D(1,:) + t{it}(1,pose_idx);
    corners_3D(2,:) = corners_3D(2,:) + t{it}(2,pose_idx);
    corners_3D(3,:) = corners_3D(3,:) + t{it}(3,pose_idx);
    corners_3D      = (veloToCam{cam+1}*[corners_3D; ones(1,size(corners_3D,2))]);
    
    % generate an orientation vector and compute coordinates in velodyneCS
    orientation_3D      = R*[0.0, 0.7*l; 0.0, 0.0; 0.0, 0.0];
    orientation_3D(1,:) = orientation_3D(1,:) + t{it}(1, pose_idx);
    orientation_3D(2,:) = orientation_3D(2,:) + t{it}(2, pose_idx);
    orientation_3D(3,:) = orientation_3D(3,:) + t{it}(3, pose_idx);
    orientation_3D      = (veloToCam{cam+1}*[orientation_3D; ones(1,size(orientation_3D,2))]);
    
    % only draw 3D bounding box for objects in front of the image plane
    if any(corners_3D(3,:)<0.5) || any(orientation_3D(3,:)<0.5) 
      continue;
    end

    % project the 3D bounding box into the image plane
    corners_2D     = projectToImage(corners_3D, K);
    orientation_2D = projectToImage(orientation_3D, K);
    %drawBox3D(gh,occlusion{it}(pose_idx),corners_2D,face_idx,orientation_2D)
    
    % compute and draw the 2D bounding box from the 3D box projection
    box.x1 = min(corners_2D(1,:));
    box.x2 = max(corners_2D(1,:));
    box.y1 = min(corners_2D(2,:));
    box.y2 = max(corners_2D(2,:));
    %drawBox2D(gh,box,occlusion{it}(pose_idx),tracklets{it}.objectType)
    
    if tracklets{it}.objectType ~= "Car" 
        continue;
    end
    
    % Output tracklet to label file
    fileID = fopen(sprintf('%s/%06d.txt', output_dir, img_idx) ,'a');
    % Object label
    fprintf(fileID,'%s ',tracklets{it}.objectType);
    % Truncation
    fprintf(fileID,'-1 ');
    % Occlusion
    fprintf(fileID,'-1 ');
    % Alpha
    fprintf(fileID,'-10 ');
    % 2D boundingbox
    fprintf(fileID,'%.2f ',box.x1);
    fprintf(fileID,'%.2f ',box.y1);
    fprintf(fileID,'%.2f ',box.x2);
    fprintf(fileID,'%.2f ',box.y2);
    % 3D boundingbox
%     fprintf(fileID,'-1 ');
%     fprintf(fileID,'-1 ');
%     fprintf(fileID,'-1 ');
%     fprintf(fileID,'-1000 ');
%     fprintf(fileID,'-1000 ');
%     fprintf(fileID,'-1000 ');
%     fprintf(fileID,'-10 ');
    fprintf(fileID,'%.2f ',tracklets{it}.h);
    fprintf(fileID,'%.2f ',tracklets{it}.w);
    fprintf(fileID,'%.2f ',tracklets{it}.l);
    fprintf(fileID,'%.2f ',tracklets{it}.poses(1,pose_idx));
    fprintf(fileID,'%.2f ',tracklets{it}.poses(2,pose_idx));
    fprintf(fileID,'%.2f ',tracklets{it}.poses(3,pose_idx));
    fprintf(fileID,'-10 ');
    % Score
    fprintf(fileID,'1 \n');
    fclose(fileID);
  end
  img_idx = img_idx+1;

end

% clean up
close all;
