package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.WristConstants;

import java.util.function.Supplier;

import org.ejml.equation.IntegerSequence.Range;

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

    private static Pose2d getReefPose(int reefSide, int position, Pose2d currentPose) {
        targetPose = ScoringConstants.REEF_SIDE_POSES[getClosesetReefSide(currentPose) - 1][position];

        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        targetPose = new Pose2d(targetPose.getX(), targetPose.getY(),
                new Rotation2d(Math.toRadians(targetPose.getRotation().getDegrees() + 180)));
        // Field2d field = new Field2d();
        // field.setRobotPose(targetPose);
        // SmartDashboard.putData("e", field);

        // Determine the correct x & y offset(s) based on the position
        // double adjustedXOffset = xOffset;
        // if (position == 0) {
        // adjustedXOffset = xOffset;
        // } else if (position == 1) {
        // adjustedXOffset = 0;
        // } else {
        // adjustedXOffset = -xOffset;
        // }

        // Create a translation for the offsets
        // Translation2d translation = new Translation2d(0, adjustedXOffset);
        // System.out.println(adjustedXOffset);

        // Apply the translation to the target pose

        // targetPose = targetPose.transformBy(new Transform2d(translation, new
        // Rotation2d()));

        return targetPose;
    }

    public static Command fullReefAutomation(int reefSide, int position,
            int height, Supplier<Double> perpendicularSpeed,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {

        Pose2d pose = getReefPose(reefSide, position, drivesubsystem.getPose());
        if (position == 1) {
            RobotState.isAlgaeMode = true;
            return new ParallelCommandGroup(
                    
                    new AlignPerpendicularToPoseCommand(drivesubsystem, pose, perpendicularSpeed),
                    new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                            wristSubsystem.goToAlgaeGrabSetpoint(height)));
        } else {
            RobotState.isAlgaeMode = false;
            return new ParallelCommandGroup(
                    
                    new AlignPerpendicularToPoseCommand(drivesubsystem, pose, perpendicularSpeed),
                    new SequentialCommandGroup(elevatorSubsystem.goToCoralScoreSetpoint(height),
                            wristSubsystem.goToCoralScoreSetpoint(height)));
        }

    }

    public static Command scoreCoralNoPathing(int height, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {
        RobotState.isAlgaeMode = false;
        return new SequentialCommandGroup(elevatorSubsystem.goToCoralScoreSetpoint(height),
                wristSubsystem.goToCoralScoreSetpoint(height));
    }

    public static Command grabAlgaeNoPathing(int height, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {
        RobotState.isAlgaeMode = true;
        return new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                wristSubsystem.goToAlgaeGrabSetpoint(height)); // clawSubsystem.intakeAlgae());
    }

    public static Command stopClaw(ClawSubsystem clawSubsystem) {
        return new InstantCommand(() -> {
            clawSubsystem.moveAtSpeed(0);
        }, clawSubsystem);
    }

    public static Command homeSubsystems(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        return new InstantCommand(() -> {
            // System.out.println("HOMING");
            elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
            wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
        });
    }

    public static Command humanPlayerPickup(int humanPlayerSide, DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {
        RobotState.isAlgaeMode = false;
        return new ParallelCommandGroup(
                new AlignWithPose(drivesubsystem, pathPlanToHP(humanPlayerSide)),
                elevatorSubsystem.goToHumanPlayerPickup(), wristSubsystem.goToHumanPlayerSetpoint(),
                clawSubsystem.intakeCoral());
    }

    public static Command humanPlayerPickupNoPathing(DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {
        RobotState.isAlgaeMode = false;
        return new ParallelCommandGroup(elevatorSubsystem.goToHumanPlayerPickup(),
                wristSubsystem.goToHumanPlayerSetpoint(),
                clawSubsystem.intakeCoral());

    }

    public static Command PPmoveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(140), Units.degreesToRadians(120));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints // Rotation delay distance in meters. This is how far the robot should travel
                            // before attempting to rotate.
        );
        return Commands.deferredProxy(() -> pathfindingCommand);

    }
}