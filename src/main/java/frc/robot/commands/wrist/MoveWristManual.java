package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveWristManual extends Command {
    Joystick joystick;

    public MoveWristManual(Joystick joystick) {
        this.joystick = joystick;

        addRequirements();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}