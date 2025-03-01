package frc.robot.automation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.CowboyUtils;

public class AlignWithPose extends Command {
    DriveSubsystem driveSubsystem;
    Pose2d pose;
    ProfiledPIDController xController;
    ProfiledPIDController yController;
    ProfiledPIDController rotController;

    public AlignWithPose(DriveSubsystem driveSubsystem, Pose2d pose) {
        this.driveSubsystem = driveSubsystem;
        this.pose = pose;
        addRequirements(driveSubsystem);

        xController = new ProfiledPIDController(0.245, 0, 0, new Constraints(2, 2));
        yController = new ProfiledPIDController(0.245, 0, 0, new Constraints(2, 2));
        rotController = new ProfiledPIDController(0.031, 0, 0, new Constraints(16, 4));

        xController.setTolerance(.001);
        yController.setTolerance(.001);
        rotController.setTolerance(0.005);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.clamp((xController.calculate(driveSubsystem.getPose().getX(), pose.getX())), -1,
                1);
        double ySpeed = MathUtil.clamp((yController.calculate(driveSubsystem.getPose().getY(), pose.getY())), -1,
                1);
        double rotSpeed = MathUtil.clamp(
                (rotController.calculate(driveSubsystem.getPose().getRotation().getDegrees(),
                        pose.getRotation().getDegrees())),
                -1,
                1);

        if (CowboyUtils.isRedAlliance()){
            driveSubsystem.drive(-xSpeed, -ySpeed, rotSpeed, true, true);
        }
        else{
            driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true, true);
        }
        
        
    }

    @Override
    public void initialize() {
        // Reset each controller using the current sensor readings
        xController.reset(driveSubsystem.getPose().getX());
        yController.reset(driveSubsystem.getPose().getY());
        rotController.reset(driveSubsystem.getPose().getRotation().getDegrees());

        // Optionally, stop the drive before starting
        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public boolean isFinished() {
        if (xController.atGoal() && yController.atGoal() && rotController.atGoal()) {
            return true;
        } else {
            return false;
        }
    }

}