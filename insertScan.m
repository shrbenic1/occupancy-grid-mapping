function insertScan(map, robotPose, scan, maxRange)
%maxProbabilitySaturation 0.999, minProbabilitySaturation 0.001
probabilitySaturation = [0.001 0.999];
logOddsSaturation = probabilityToLogOdds(probabilitySaturation);
%free 0.4, occupied 0.7
inverseModelProbability = [0.4 0.7];
inverseModelLogOdds = probabilityToLogOdds(inverseModelProbability);
%robots position in the moment of taking a scan
centerPoint = [robotPose(1) robotPose(2)];
%ignore NaN range values
ranges = scan.Ranges;
angles = scan.Angles;
angles = angles(~isnan(ranges));
ranges = ranges(~isnan(ranges));
modifiedScan = lidarScan(ranges, angles);
%points reached by the lidar
endingPoints = getEndingPointsFromScan(robotPose, modifiedScan);
for i = 1:size(endingPoints, 1)
    [endingPoint, midPoints] = robotics.algs.internal.raycastCells(centerPoint, endingPoints(i, :), map.GridSize(1), map.GridSize(2), map.Resolution, map.GridLocationInWorld);
    if ~isempty(midPoints)
        freeCellsUpdate = probabilityToLogOdds(getOccupancy(map, midPoints, "grid")) + inverseModelLogOdds(1);
        freeCellsUpdate(freeCellsUpdate < logOddsSaturation(1)) = logOddsSaturation(1);
        setOccupancy(map, midPoints, logOddsToProbability(freeCellsUpdate), "grid");
    end
    if modifiedScan.Ranges(i) == maxRange
        continue;
    end
    if ~isempty(endingPoint)
        occupiedCellsUpdate = probabilityToLogOdds(getOccupancy(map, endingPoint, "grid")) + inverseModelLogOdds(2);
        occupiedCellsUpdate(occupiedCellsUpdate > logOddsSaturation(2)) = logOddsSaturation(2);
        setOccupancy(map, endingPoint, logOddsToProbability(occupiedCellsUpdate), "grid");
    end
end
end