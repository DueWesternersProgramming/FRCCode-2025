package frc.robot.subsystems.drive.gyro;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.utils.CowboyUtils;

public class GyroIOSim implements GyroIO {
    private double fakeGyro = 0;

    public GyroIOSim() {

    }

    @Override
    public double getGyroAngle() {
        return fakeGyro + (CowboyUtils.isRedAlliance() ? 180 : 0);
    }

    @Override
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getGyroAngle());
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
        inputs.gyroAngle = getGyroAngle();
    }
}
