package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.CowboyUtils;

public class GyroIOSim implements GyroIO {
    private double fakeGyro = 0;

    public GyroIOSim() {

    }

    @Override
    public double getGyroYawAngle() {
        return fakeGyro + (CowboyUtils.isRedAlliance() ? 180 : 0);
    }

    @Override
    public double getGyroPitchAngle() {
        return 0.0;
    }

    @Override
    public double getGyroRollAngle() {
        return 0.0;
    }

    @Override
    public Rotation2d getGyroYawRotation2d() {
        return Rotation2d.fromDegrees(getGyroYawAngle());
    }

    @Override
    public void setGyroAngle(double angle) {
        fakeGyro = angle;
    }

    @Override
    public void reset() {
        fakeGyro = 0;
    }

    @Override
    public double getVelocityX() {
        return 0;
    }

    @Override
    public double getVelocityY() {
        return 0;
    }

    @Override
    public double getRate() {
        return 0;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.gyroAngle = getGyroYawAngle();
    }
}
