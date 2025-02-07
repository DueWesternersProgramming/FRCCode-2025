package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SetClawSpeed extends Command {
    ClawSubsystem clawSubsystem;
    double speed;

    public SetClawSpeed (ClawSubsystem clawSubsystem, double speed) {
        this.clawSubsystem = clawSubsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.moveAtSpeed(0);
    }

    @Override
    public void execute() {
        clawSubsystem.moveAtSpeed(speed);
    }

    @Override
    public void initialize() {
        clawSubsystem.moveAtSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}