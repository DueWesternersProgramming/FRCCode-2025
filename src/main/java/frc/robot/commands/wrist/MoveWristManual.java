package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.wrist.WristSubsystem;

public class MoveWristManual extends Command {
    Joystick joystick;
    WristSubsystem wristSubsystem;

    public MoveWristManual(WristSubsystem wristSubsystem, Joystick joystick) {
        this.joystick = joystick;
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.moveAtSpeed(0);
    }

    @Override
    public void execute() {
        wristSubsystem.moveAtSpeed(joystick.getRawAxis(Controller.WRIST_MANUAL_CONTROL));
    }

    @Override
    public void initialize() {
        wristSubsystem.moveAtSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}