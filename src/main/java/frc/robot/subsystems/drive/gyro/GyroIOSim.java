package frc.robot.subsystems.drive.gyro;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.utils.CowboyUtils;

//This file is basicly the same as @GyroIONAVX.java due to NAVX ha simulation options, but we want to apply our own red allinace 180 deg offset.

public class GyroIOSim implements GyroIO {
    private static AHRS m_gyro;

    public GyroIOSim() {
        m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    @Override
    public double getGyroAngle() {
        // THIS LINE IS THE KEY CHANGE
        return (m_gyro.getAngle() + (CowboyUtils.isRedAlliance() ? 180 : 0)) * DrivetrainConstants.GYRO_ORIENTATION;
    }

    @Override
    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    @Override
    public void setGyroAngle(double angle) {
        m_gyro.setAngleAdjustment(angle);
    }

    @Override
    public void reset() {
        m_gyro.reset();
        m_gyro.setAngleAdjustment(0);
    }

    @Override
    public double getVelocityX() {
        return m_gyro.getVelocityX();
    }

    @Override
    public double getVelocityY() {
        return m_gyro.getVelocityY();
    }

    @Override
    public double getRate() {
        return m_gyro.getRate() * DrivetrainConstants.GYRO_ORIENTATION;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = m_gyro.isConnected();
        inputs.gyroAngle = getGyroAngle();
    }
}
