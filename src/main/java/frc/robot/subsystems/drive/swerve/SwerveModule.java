// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerve;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.RobotConstants.SwerveModuleConstants;
import frc.robot.sensors.CanCoder;

/**
 * The {@code SwerveModule} class contains fields and methods pertaining to the
 * function of a swerve module.
 */
public class SwerveModule {
        public final SparkMax m_drivingSparkMax;
        private final SparkMax m_turningSparkMax;

        public final RelativeEncoder m_drivingEncoder;
        private final RelativeEncoder m_turningEncoder;
        private final CanCoder m_turningAbsoluteEncoder;

        private final SparkClosedLoopController m_drivingPIDController;
        private final SparkClosedLoopController m_turningPIDController;
        private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

        /**
         * Constructs a SwerveModule and configures the driving and turning motor,
         * encoder, and PID controller.
         */
        public SwerveModule(int drivingCANId, int turningCANId, int CANcoderID, boolean drivingInverted) {
                m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
                m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

                SparkMaxConfig m_drivingConfig = new SparkMaxConfig();
                SparkMaxConfig m_turningConfig = new SparkMaxConfig();

                // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
                m_drivingEncoder = m_drivingSparkMax.getEncoder();
                m_turningEncoder = m_turningSparkMax.getEncoder();
                m_turningAbsoluteEncoder = new CanCoder(CANcoderID);

                m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
                m_turningPIDController = m_turningSparkMax.getClosedLoopController();

                m_drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                m_turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

                // Apply position and velocity conversion factors for the driving encoder. The
                // native units for position and velocity are rotations and RPM, respectively,
                // but we want meters and meters per second to use with WPILib's swerve APIs.

                m_drivingConfig.encoder.positionConversionFactor(
                                SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION);
                m_drivingConfig.encoder.velocityConversionFactor(
                                SwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);

                // Apply position and velocity conversion factors for the turning encoder. We
                // want these in radians and radians per second to use with WPILib's swerve
                // APIs.

                m_turningConfig.encoder.positionConversionFactor(
                                SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION);
                m_turningConfig.encoder.velocityConversionFactor(
                                SwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM);

                // Invert the turning controller, since the output shaft rotates in the opposite
                // direction of
                // the steering motor.
                m_turningConfig.inverted(false);
                m_drivingConfig.inverted(drivingInverted);


                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                m_turningConfig.closedLoop.positionWrappingEnabled(true);
                m_turningConfig.closedLoop.positionWrappingMinInput(
                                SwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS);
                m_turningConfig.closedLoop.positionWrappingMaxInput(
                                SwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS);

                // Set the PID gains for the driving motor.
                m_drivingConfig.closedLoop.pidf(SwerveModuleConstants.DRIVING_P, SwerveModuleConstants.DRIVING_I,
                                SwerveModuleConstants.DRIVING_D, SwerveModuleConstants.DRIVING_FF);

                m_drivingConfig.closedLoop.outputRange(SwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED,
                                SwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);

                // Set the PID gains for the turning motor.
                m_turningConfig.closedLoop.pidf(SwerveModuleConstants.TURNING_P, SwerveModuleConstants.TURNING_I,
                                SwerveModuleConstants.TURNING_D, SwerveModuleConstants.TURNING_FF);

                m_turningConfig.closedLoop.outputRange(SwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED,
                                SwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED);

                m_drivingConfig.idleMode(SwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE);
                m_turningConfig.idleMode(SwerveModuleConstants.TURNING_MOTOR_IDLE_MODE);
                

                m_drivingConfig.smartCurrentLimit(SwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
                m_turningConfig.smartCurrentLimit(SwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

                // Save the SPARK MAX configurations. If a SPARK MAX browns out during
                // operation, it will maintain the above configurations.

                m_drivingSparkMax.configure(m_drivingConfig, ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters);
                m_turningSparkMax.configure(m_turningConfig, ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters);

                m_desiredState.angle = new Rotation2d(0);
                m_drivingEncoder.setPosition(0);
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
                return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                                new Rotation2d(m_turningEncoder.getPosition()));
        }

        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                                m_drivingEncoder.getPosition(),
                                new Rotation2d(m_turningEncoder.getPosition()));
        }

        /**
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
                // Apply chassis angular offset to the desired state.
                SwerveModuleState correctedDesiredState = new SwerveModuleState();
                correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
                correctedDesiredState.angle = desiredState.angle;

                // Optimize the reference state to avoid spinning further than 90 degrees.

                correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

                if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.001 // less than 1 mm per sec
                                && Math.abs(correctedDesiredState.angle.getRadians()
                                                - m_turningEncoder.getPosition()) < 0.1) // 10% of
                                                                                         // a
                                                                                         // radian
                {
                        m_drivingSparkMax.set(0); // no point in doing anything
                        m_turningSparkMax.set(0);
                } else {
                        // Command driving and turning SPARKS MAX towards their respective setpoints.
                        m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond,
                                        SparkMax.ControlType.kVelocity);
                        m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(),
                                        SparkMax.ControlType.kPosition);
                }

                m_desiredState = desiredState;

        }

        /** Zeroes all the SwerveModule relative encoders. */
        public void resetEncoders() {

                m_drivingEncoder.setPosition(0); // arbitrarily set driving encoder to zero
        
                // temp
                m_turningAbsoluteEncoder.resetVirtualPosition();
                // the reading and setting of the calibrated absolute turning encoder values is
                // done in the Drivetrain's constructor

                m_turningSparkMax.set(0); // no moving during reset of relative turning encoder

                m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getPosition()); // set relative position based on
                                                                                      // virtual absolute position
        }

        /**
         * Calibrates the virtual position (i.e. sets position offset) of the absolute
         * encoder.
         */
        // public void calibrateVirtualPosition(double angle) {
        //         m_turningAbsoluteEncoder.setPositionOffset(angle);
        // }

        public RelativeEncoder getDrivingEncoder() {
                return m_drivingEncoder;
        }

        public RelativeEncoder getTurningEncoder() {
                return m_turningEncoder;
        }

        public CanCoder getTurningAbsoluteEncoder() {
                return m_turningAbsoluteEncoder;
        }

        public SwerveModuleState getDesiredState() {
                return m_desiredState;
        }

}
