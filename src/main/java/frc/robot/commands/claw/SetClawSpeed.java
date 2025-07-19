package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class SetClawSpeed extends Command {
    ClawSubsystem clawSubsystem;
    double speed;

    public SetClawSpeed(ClawSubsystem clawSubsystem, double speed) {
        this.clawSubsystem = clawSubsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawSpeed(0);
    }

    @Override
    public void execute() {
        clawSubsystem.setClawSpeed(speed);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}