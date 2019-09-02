rosinit;
robotSimulator = ExampleHelperRobotSimulator('complexMap');
enableROSInterface(robotSimulator,true);
setRobotPose(robotSimulator, [2 2 -pi/2]);
robotSimulator.LaserSensor.NumReadings = 50;
scanSubscriber = rossubscriber('scan');
[velocityPublisher, velocityMessage] = rospublisher('/mobile_base/commands/velocity');
transformationTree = rostf;
pause(1);
path = [2 2; 1.5 2; 4 3; 2 4; 4 4; 8 5; 8 2; 6 2; 8 2; 8 8; 2 8; 2 14; 8 14; 8 18; 3 18; 8 18; 8 12; 7 12; 8 12; 8 14; 14 14; 14 12; 12 12; 14 12; 14 14; 14 18; 12 18; 14 18; 14 14; 16 14; 16 12; 18 12; 16 12; 16 14; 16 18; 18 18; 18 19; 22 19; 22 17; 24 18; 24 14; 21 14; 22 12; 23 7; 21 9; 18 7; 16 9; 16 5; 16 2; 18 2; 16 2; 16 5; 23 5; 23 1; 21 5; 14 5; 14 2; 12 2; 14 2; 14 5; 13 8; 11 8];
plot(path(:, 1),path(:, 2), 'r--o');
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.4;
controlRate = robotics.Rate(10);
goalRadius = 0.1;
robotCurrentLocation = path(1, :);
robotGoal = path(end, :);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
map = robotics.OccupancyGrid(26, 20.5, 20);
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid');
updateCounter = 1;
while( distanceToGoal > goalRadius )
    scanMessage = receive(scanSubscriber);
    pose = getTransform(transformationTree, 'map', 'robot_base', scanMessage.Header.Stamp, 'Timeout', 2);
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'XYZ');
    robotPose = [position, orientation(3)];
    scan = lidarScan(scanMessage);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = robotSimulator.LaserSensor.MaxRange;
    modifiedScan = lidarScan(ranges, scan.Angles);
    insertScan(map, robotPose, modifiedScan, robotSimulator.LaserSensor.MaxRange);
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