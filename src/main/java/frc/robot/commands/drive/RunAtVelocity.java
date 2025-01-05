package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RunAtVelocity extends Command {
    private final DriveSubsystem drive;
    private final double x;
    private final double y;
    private final double rot;

    public RunAtVelocity(DriveSubsystem drive, double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public void execute() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            drive.drive(-x, -y, -rot, DrivetrainConstants.FIELD_RELATIVE, true);
        } else {
            drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}