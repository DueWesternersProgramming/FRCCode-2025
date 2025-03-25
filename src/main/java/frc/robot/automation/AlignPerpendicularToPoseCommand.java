package frc.robot.automation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignPerpendicularToPoseCommand extends Command {
    private final ProfiledPIDController angleController = new ProfiledPIDController(5.0, 0.0, 0.4,
            new TrapezoidProfile.Constraints(3.0, 5.0));

    private final PIDController parallelController = new PIDController(5.0, 0.0, 0.1);

    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPoseSupplier;
    private final Supplier<Double> perpendicularInput;

    private double perpendicularError = 0;

    public AlignPerpendicularToPoseCommand(
            DriveSubsystem driveSubsystem,
            Pose2d pose,
            Supplier<Double> perpendicularInput) {

        addRequirements(driveSubsystem);

        this.driveSubsystem = driveSubsystem;
        this.targetPoseSupplier = pose;
        this.perpendicularInput = perpendicularInput;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d targetPose = targetPoseSupplier;

        Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

        perpendicularError = CowboyUtils.getPerpendicularError(robotPose, targetPose);

        double parallelError = CowboyUtils.getParallelError(robotPose, targetPose);

        double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();

        double parallelSpeed = parallelController.calculate(-parallelError, 0);
        parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed : 0;

        double angularSpeed = angleController.calculate(thetaError, 0);
        angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

        ChassisSpeeds speeds = new ChassisSpeeds(
                perpendicularInput.get() * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                parallelSpeed,
                angularSpeed);

        driveSubsystem.runChassisSpeeds(speeds, false);
    }

    @Override
    public void end(boolean interrupt) {
        parallelController.reset();
        angleController.reset(0);
    }

    public Trigger atSetpoint(DoubleSupplier distance) {
        return new Trigger(
                () -> parallelController.atSetpoint()
                        && angleController.atSetpoint()
                        && perpendicularError < distance.getAsDouble());
    }
}