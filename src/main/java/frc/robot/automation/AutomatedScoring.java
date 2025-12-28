package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotState;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.RobotConstants.ScoringConstants.Setpoints;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

public class AutomatedScoring {
    static Pose2d targetPose;

    private static Pose2d pathPlanToHP(int humanPlayerSide) {
        targetPose = ScoringConstants.HP_POSES.get(humanPlayerSide); // no -1 since 0 is left and 1 is
                                                                     // right
                                                                     // and indexing starts at 0'
        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        return targetPose;
    }

    public static int getClosesetReefSide(Pose2d currentPose) {
        int closestReefSide = 1; // 1-based index.
        // Compare using 0-based index for the array.
        for (int i = 1; i < 7; i++) {
            Pose2d comparisonPose = ScoringConstants.REEF_SIDE_POSES[i - 1][1];
            if (CowboyUtils.isRedAlliance()) {
                comparisonPose = FlippingUtil.flipFieldPose(comparisonPose);
            }
            Pose2d closestPose = ScoringConstants.REEF_SIDE_POSES[closestReefSide - 1][1];
            if (CowboyUtils.isRedAlliance()) {
                closestPose = FlippingUtil.flipFieldPose(closestPose);
            }
            // i is used as 1-based when updating, so access array with (i - 1)
            if (comparisonPose.getTranslation().getDistance(
                    currentPose.getTranslation()) < closestPose
                            .getTranslation()
                            .getDistance(currentPose.getTranslation())) {
                closestReefSide = i;
            }
        }
        System.out.println("Closest reef side: " + closestReefSide);
        return closestReefSide;
    }

    private static Pose2d getClosestReefPose(int position, Pose2d currentPose) {
        targetPose = ScoringConstants.REEF_SIDE_POSES[getClosesetReefSide(currentPose) - 1][position];

        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        targetPose = new Pose2d(targetPose.getX(), targetPose.getY(),
                new Rotation2d(Math.toRadians(targetPose.getRotation().getDegrees())));// +180

        return targetPose;
    }

    private static Pose2d getReefPose(int reefSide, int position, Pose2d currentPose) {
        targetPose = ScoringConstants.REEF_SIDE_POSES[reefSide - 1][position];

        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        targetPose = new Pose2d(targetPose.getX(), targetPose.getY(),
                new Rotation2d(Math.toRadians(targetPose.getRotation().getDegrees())));// +180

        return targetPose;
    }

    public static Command fullReefAutomationPerpendicularLineup(int reefSide, int position,
            Setpoints height, Supplier<Double> perpendicularSpeed,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {

        Pose2d pose = getClosestReefPose(position, drivesubsystem.getPose());
        RobotState.isAutoAlignActive = true;
        if (position == 1) {

            return new ParallelCommandGroup(

                    new AlignPerpendicularToPoseCommand(drivesubsystem, pose, perpendicularSpeed),
                    new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                            wristSubsystem.goToAlgaeGrabSetpoint(height)));
        } else {

            return new ParallelCommandGroup(

                    new AlignPerpendicularToPoseCommand(drivesubsystem, pose, perpendicularSpeed),
                    new SequentialCommandGroup(elevatorSubsystem.goToCoralScoreSetpoint(height),
                            wristSubsystem.goToCoralScoreSetpoint(height)));
        }

    }

    public static Command fullReefAutomationDynamicAuto(int reefSide, int position,
            int height,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        Pose2d pose = getReefPose(reefSide, position, drivesubsystem.getPose());
        RobotState.isAutoAlignActive = true;
        if (position == 1) {
            return new ParallelCommandGroup(

                    PPmoveToPose(pose),
                    new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                            wristSubsystem.goToAlgaeGrabSetpoint(height)),
                    clawSubsystem.intakeAlgae());
        } else {

            return new ParallelCommandGroup(

                    PPmoveToPose(pose),

                    new SequentialCommandGroup(elevatorSubsystem.goToCoralScoreSetpoint(height),
                            wristSubsystem.goToCoralScoreSetpoint(height)));
        }

    }

    public static Command scoreCoralNoPathing(Setpoints level, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        return new SequentialCommandGroup(elevatorSubsystem.goToCoralScoreSetpoint(level),
                wristSubsystem.goToCoralScoreSetpoint(level));
    }

    public static Command grabAlgaeNoPathing(Setpoints level, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        return new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(level),
                wristSubsystem.goToAlgaeGrabSetpoint(level)); // clawSubsystem.intakeAlgae());
    }

    public static Command homeSubsystems(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        return new ParallelCommandGroup(elevatorSubsystem.goToHomeSetpoint(), wristSubsystem.goToHomeSetpoint());

    };

    public static Command humanPlayerPickup(int humanPlayerSide, DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        return new ParallelCommandGroup(
                new AlignWithPose(drivesubsystem, pathPlanToHP(humanPlayerSide)),
                elevatorSubsystem.goToHumanPlayerPickup(), wristSubsystem.goToHumanPlayerSetpoint(),
                clawSubsystem.intakeCoral());
    }

    public static Command humanPlayerPickupNoPathing(DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        return new ParallelCommandGroup(elevatorSubsystem.goToHumanPlayerPickup(),
                wristSubsystem.goToHumanPlayerSetpoint(),
                clawSubsystem.intakeCoral());

    }

    public static Command PPmoveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
        );
        return Commands.deferredProxy(() -> pathfindingCommand);

    }
}