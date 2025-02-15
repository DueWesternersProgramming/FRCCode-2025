package frc.robot.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotState;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.ScoringConstants;
import frc.robot.RobotConstants.WristConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.util.FlippingUtil;

public class AutomatedScoring {
    static Pose2d targetPose;
    static double xOffset = .2;// left and right
    // double yOffset = -.25;// forward and back

    private static Pose2d pathPlanToHP(int humanPlayerSide) {
        targetPose = ScoringConstants.BlueAlliance.HP_POSES.get(humanPlayerSide); // no -1 since 0 is left and 1 is
                                                                                  // right
                                                                                  // and indexing starts at 0'
        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }
        return targetPose;
    }

    private static Pose2d pathPlanToReef(int reefSide, int position) {
        // System.out.println("Reef Side: " + reefSide.get());
        targetPose = ScoringConstants.BlueAlliance.REEF_SIDE_POSES.get(reefSide - 1);

        if (CowboyUtils.isRedAlliance()) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
            // System.out.println("Flipping pose");
            targetPose = new Pose2d(targetPose.getX(), targetPose.getY(),
                    new Rotation2d(Math.toRadians(targetPose.getRotation().getDegrees() - 90)));
            // // Not sure what to do
            // // about this
        }
        // Field2d field = new Field2d();
        // field.setRobotPose(targetPose);
        // SmartDashboard.putData("e", field);

        // Determine the correct x & y offset(s) based on the position
        double adjustedXOffset = xOffset;
        if (position == 0) {
            adjustedXOffset = -xOffset;
        } else if (position == 1) {
            adjustedXOffset = 0;
        } else {
            adjustedXOffset = xOffset;
        }

        // Create a translation for the offsets
        Translation2d translation = new Translation2d(adjustedXOffset, 0);

        // Apply the translation to the target pose
        targetPose = targetPose.transformBy(new Transform2d(translation, targetPose.getRotation()));

        return targetPose;
    }

    public static Command fullReefAutomation(int reefSide, int position,
            int height,
            DriveSubsystem drivesubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {

        Pose2d pose = pathPlanToReef(reefSide, position);
        if (position == 1) {
            RobotState.isAlgaeMode = true;
            return new ParallelCommandGroup(
                    AlignWithPose.pathToPoseCommand(pose, drivesubsystem),
                    new SequentialCommandGroup(elevatorSubsystem.goToAlgaeGrabSetpoint(height),
                            wristSubsystem.goToAlgaeGrabSetpoint(height)));
        } else {
            RobotState.isAlgaeMode = false;
            return new ParallelCommandGroup(
                    AlignWithPose.pathToPoseCommand(pose, drivesubsystem),
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
                wristSubsystem.goToAlgaeGrabSetpoint(height)); //clawSubsystem.intakeAlgae());
    }

    public static Command stopClaw(ClawSubsystem clawSubsystem) {
        return new InstantCommand(() -> {
            clawSubsystem.moveAtSpeed(0);
        }, clawSubsystem);
    }

    public static Command homeSubsystems(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        return new InstantCommand(() -> {
            elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
            wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
        });
    }

    public static Command humanPlayerPickup(int humanPlayerSide, DriveSubsystem drivesubsystem,
            ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem,
            ClawSubsystem clawSubsystem) {
        RobotState.isAlgaeMode = false;
        return new ParallelCommandGroup(
                AlignWithPose.pathToPoseCommand(pathPlanToHP(humanPlayerSide), drivesubsystem),
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
}
