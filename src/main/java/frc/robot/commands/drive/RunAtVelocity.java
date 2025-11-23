package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotState;

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
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true, RobotState.isAntiTippingEnabled);
    }

    @Override
    public void execute() {
        drive.drive(-x, -y, -rot, DrivetrainConstants.FIELD_RELATIVE, true, RobotState.isAntiTippingEnabled);

    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}