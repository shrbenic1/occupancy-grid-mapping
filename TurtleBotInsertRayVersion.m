rosinit('192.168.153.128');
scanSubscriber = rossubscriber('scan');
odom = rossubscriber('/odom');
[velocityPublisher, velocityMessage] = rospublisher('/mobile_base/commands/velocity');
pause(1);
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
position = [pose.Position.X, pose.Position.Y];
path = [position; 2 1; 5 1; 3 -1; 6 0; 6 1; 5 1; 5 4; 6 4; 6.5 3.5; 4 3; -3 4; -1 4; -4 4; -5 0; -4 1; -2 1; -2 -1; -4 -1; -1 0; 1 1; 1 -1];
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.4;
controlRate = robotics.Rate(10);
goalRadius = 0.1;
robotCurrentLocation = path(1, :);
robotGoal = path(end, :);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
map = robotics.OccupancyGrid(14, 8, 20);
map.GridLocationInWorld = [-6.3 -2.3];
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid');
updateCounter = 1;
while( distanceToGoal > goalRadius )
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    position = [pose.Position.X, pose.Position.Y];
    quat = pose.Orientation;
    orientation = quat2eul([quat.W quat.X quat.Y quat.Z], 'XYZ');
    robotPose = [position, orientation(3)];
    scan = receive(scanSubscriber, 3);
    maxRange = 4.5;
    scan = lidarScan(scan);
    ranges = scan.Ranges;
    for i = 2 : size(ranges,1) - 1
        if isnan(ranges(i, 1)) && ~isnan(ranges(i - 1, 1)) && ~isnan(ranges(i + 1, 1))
            ranges(i, 1) = (ranges(i - 1, 1) + ranges(i + 1, 1)) ./ 2;
        elseif isnan(ranges(i, 1))
            ranges(i, 1) = maxRange;
        end
    end
    modifiedScan = lidarScan(ranges, scan.Angles);
    insertRay(map, robotPose, modifiedScan, maxRange);
    [v, w] = controller(robotPose);
    velocityMessage.Linear.X = v;
    velocityMessage.Angular.Z = w;
    send(velocityPublisher, velocityMessage);
    if ~mod(updateCounter, 2)
        mapHandle.CData = occupancyMatrix(map);
    end   
    updateCounter = updateCounter + 1;
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    waitfor(controlRate);
end
show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Final Map');
rosshutdown;