package frc.robot.subsystems.wrist;

public class WristSubsystemIOSim implements WristSubsystemIO {

    public WristSubsystemIOSim() {
    }

    @Override
    public void goToSetpoint(double setpoint) {

    }

    @Override
    public void moveAtSpeed(double speed) {

    }

    @Override
    public void setEncoderValue(double value) {

    }

    @Override
    public double getEncoderValue() {
        return 0;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristPosition = getEncoderValue();
        inputs.wristTemp = 0.0;
    }

}