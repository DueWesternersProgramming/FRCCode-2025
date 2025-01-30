package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class MoveElevatorManual extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    Joystick joystick;

    public MoveElevatorManual(ElevatorSubsystem elevatorSubsystem, Joystick joystick) {
        this.joystick = joystick;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        elevatorSubsystem.moveAtSpeed(joystick.getRawAxis(Controller.ELEVATOR_MANUAL_CONTROL));
    }

    @Override
    public void initialize() {
        elevatorSubsystem.moveAtSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}