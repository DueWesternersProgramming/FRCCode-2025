package frc.robot.subsystems.elevator;

public class ElevatorSubsystemIOSim implements ElevatorSubsystemIO {

    public ElevatorSubsystemIOSim() {

    }

    @Override
    public void goToSetpoint(double setpoint) {

    }

    @Override
    public void setEncoderValue(double value) {

    }

    @Override
    public void updateInputs(ElevatorSubsystemIOInputs inputs) {
        inputs.leftMotorTemp = 0;
        inputs.rightMotorTemp = 0;
        inputs.leftMotorPosition = 0;
        inputs.rightMotorPosition = 0;
    }

}
