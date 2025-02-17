package frc.robot.automation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class AlignWithTag extends Command {
    double position;
    DriveSubsystem driveSubsystem;
    ProfiledPIDController pidController;
    VisionSubsystem visionSubsystem;
    double speed;
    double offset = 0;

    public AlignWithTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double position) {
        this.position = position;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);

        pidController = new ProfiledPIDController(.01, 0, 0, new Constraints(.5, .5));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, true);
    }

    @Override
    public void execute() {
        if (position == 2){
            offset = 7;
        }
        if (position == 1){
            offset = 16;
        }
        else{
            offset = 28;
        }
        if (VisionSubsystem.cameras[0].hasResults()){
        speed = pidController.calculate(VisionSubsystem.cameras[0].getTargetYaw(), offset);
        speed = MathUtil.clamp(speed, -.05, 0.05);
        System.out.println(speed);
        }
        else{
            speed = 0;
        }

        driveSubsystem.drive(0, speed, 0, false, true);
    }

    @Override
    public void initialize() {
        driveSubsystem.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}