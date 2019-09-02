function endingPoints = getEndingPointsFromScan(robotPose, scan)
endingPoints = [robotPose(1) + scan.Ranges.*cos(scan.Angles + robotPose(3)), robotPose(2) + scan.Ranges.*sin(scan.Angles + robotPose(3))];
end