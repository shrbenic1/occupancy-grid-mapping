rosinit;
robotSimulator = ExampleHelperRobotSimulator('basicMap');
enableROSInterface(robotSimulator,true);
setRobotPose(robotSimulator, [2 3 -pi/2]);
robotSimulator.LaserSensor.NumReadings = 50;
scanSubscriber = rossubscriber('scan');
[velocityPublisher, velocityMessage] = rospublisher('/mobile_base/commands/velocity');
transformationTree = rostf;
pause(1);
path = [2 3; 2 1; 3.25 6.25; 1 7; 2 11; 6 7; 11 11; 12 11; 8 6; 10 5; 7 3; 11 4; 11.5 3];
plot(path(:, 1),path(:, 2), 'r--o');
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.4;
controlRate = robotics.Rate(10);
goalRadius = 0.1;
robotCurrentLocation = path(1, :);
robotGoal = path(end, :);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
map = robotics.OccupancyGrid(14, 13, 20);
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
    insertRay(map, robotPose, modifiedScan, robotSimulator.LaserSensor.MaxRange);
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