package frc.robot.configurableAutos;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfindToPose {

    public Command generatePathfindCommand(Pose2d targetPose, Optional<Double> maxVelocityMPS,
            Optional<Double> maxAccelerationMPS, Optional<Double> maxAngularVelocityDPS,
            Optional<Double> maxAngularAccelerationDPS) {

        PathConstraints constraints = new PathConstraints(
                maxVelocityMPS.orElse(3.0), maxAccelerationMPS.orElse(4.0),
                Units.degreesToRadians(maxAngularVelocityDPS.orElse(360.0)),
                Units.degreesToRadians(maxAngularAccelerationDPS.orElse(500.0)));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);

        return pathfindingCommand;
    }
}
