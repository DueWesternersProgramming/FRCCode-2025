package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SetClawSpeed extends Command {
    WristSubsystem wristSubsystem;
    double speed;

    public SetClawSpeed (WristSubsystem wristSubsystem, double speed) {
        this.wristSubsystem = wristSubsystem;
        this.speed = speed;
        addRequirements(wristSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.moveAtSpeed(0);
    }

    @Override
    public void execute() {
        wristSubsystem.moveAtSpeed(speed);
    }

    @Override
    public void initialize() {
        wristSubsystem.moveAtSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}