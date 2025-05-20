package frc.robot.subsystems.claw;

public class ClawIOSim implements ClawSubsystemIO {

    public ClawIOSim() {

    }

    @Override
    public void moveAtSpeed(double speed) {

    }

    @Override
    public double[] getCurrent() {

        return new double[0];
    }

    @Override
    public void intakeCoral() {

    }

    @Override
    public void outtakeCoral() {

    }

    @Override
    public void intakeAlgae() {

    }

    @Override
    public void outtakeAlgae() {

    }

    @Override
    public void updateInputs(ClawSubsystemIOInputs inputs) {
        // Called periodically by the ClawSubsystem to update inputs
        inputs.temp = 0.0;
        inputs.appliedVolts = 0.0;
        inputs.currentAmps = 0.0;
    }
}