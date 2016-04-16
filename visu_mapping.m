function [lm, wp] = visu_mapping( input_path )
%VI Summary of this function goes here
%   Detailed explanation goes here
%Visual Mapping session: Create maps for SLAM system

w = warning ('off','all');

format compact
path(path, '../')

video = VideoReader('MOV_0104.mp4');

%Load calibrated matrix
load cameraParams.mat;
cameraParams1 = cameraParams;cameraParams2 = cameraParams1;

%Create two augmenting vectors to store the track of the pose and features
pose_track = [];
features_track = [];

%Waypoint vector
waypoints = [];

%Counter for feature selection
count = 1;

%Start iteration
while(hasFrame(video)),
    %Grab two consecutive frames from one video
    frame1 = readFrame(video);
    frame2 = readFrame(video);
%     if framecount == 12
%         pause;
%     end
%     frame1 = undistortImage(frame1, cameraParams1);
%     frame2 = undistortImage(frame2, cameraParams2);
        
    %Extract and match features using FAST
%     points1 = detectMinEigenFeatures(rgb2gray(frame1));
%     points2 = detectMinEigenFeatures(rgb2gray(frame2));
    points1 = detectFASTFeatures(rgb2gray(frame1));
    points2 = detectFASTFeatures(rgb2gray(frame2));
    featurepoints = points2;
    [features1, valid_points1] = extractFeatures(rgb2gray(frame1), points1);
    [features2, valid_points2] = extractFeatures(rgb2gray(frame2), points2);
    pairs = matchFeatures(features1, features2);
    
    %Apply SFM of Multiple Views to get 3D location of landmarks
    
    %Create point tracker
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
    
    % Initialize the point tracker
    points1 = points1.Location;
    if isempty(points1),continue;end;
    initialize(tracker, points1, frame1);
    
    % Track the points
    [points2, validIdx] = step(tracker, frame2);
    matchedPoints1 = points1(validIdx, :);
    matchedPoints2 = points2(validIdx, :);
    
    % If not enough matched points, move on to next frame
    dim = size(matchedPoints1);
    if dim(1) < 20
        continue;
    end
    
    % Estimate the fundamental matrix
    [fMatrix, epipolarInliers] = estimateFundamentalMatrix(...
        matchedPoints1, matchedPoints2);
    
    % Find epipolar inliers
    inlierPoints1 = matchedPoints1(epipolarInliers, :);
    inlierPoints2 = matchedPoints2(epipolarInliers, :);
    
    %Compute Camera Poses
    [R, t] = cameraPose(fMatrix, cameraParams1, inlierPoints1, inlierPoints2);
    
    if rem(count, 3) == 0,
        % Compute the camera matrices for each position of the camera
        % The first camera is at the origin looking along the X-axis. Thus, its
        % rotation matrix is identity, and its translation vector is 0.
        camMatrix1 = cameraMatrix(cameraParams1, eye(3), [0 0 0]);
        camMatrix2 = cameraMatrix(cameraParams2, R', -t*R');

        %Compute the 3D points using triangulation
        points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);
    %     points3D = triangulate(matchedPoints1, matchedPoints2, stereoParams);
        
        %Landmark Assignment
        dim = size(points3D);
        if dim(1) > 50,
            dim_y50 = points3D(1:50,2);
            lm = [points3D(1:50,1),dim_y50]'*10;
        else
            dim_y = points3D(1:dim(1),2);
            lm = [points3D(1:dim(1),1),dim_y]'*10;
        end
        
        point_2 = lm(:,floor(rand(1)*49+1));
        
        %Scaling point_2
        while norm(point_2) > 10
            point_2 = floor(point_2 / 10);
        end
        
        features_track = [features_track point_2];
        update_pose = t(1:2);
        way_sum = sum(waypoints');
        waypoints = [waypoints (way_sum+update_pose)'];
        
        size_way = size(waypoints);
        if norm(waypoints(:,size_way(2))) > 100
            break;
        end
    end
        
%     pose_size = size(pose_track);
%     update_pose = [pose_track(1,pose_size(2))+t(1,1); pose_track(2,pose_size(2))+t(1,2)];
% %     update_pose = [pose_track(1,pose_size(2))+x_estimate(1,1); pose_track(2,pose_size(2))+x_estimate(1,2)];
%     pose_track = [pose_track update_pose];
    
    count = count + 1;
    
    if count == 50,
        break;
    end
end

%Alignment
features_track = features_track*10 + waypoints;

%Assignment for landmarks and waypoints
lm = features_track;
wp = waypoints;

end

