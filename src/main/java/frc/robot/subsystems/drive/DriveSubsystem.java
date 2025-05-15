package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.stream.IntStream;

import org.photonvision.EstimatedRobotPose;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotState;
import frc.robot.utils.CowboyUtils;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.questnav.TimestampedPose;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.SwerveUtils;

import com.fasterxml.jackson.core.format.InputAccessor.Std;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;

/**
 * The {@code Drivetrain} class contains fields and methods pertaining to the
 * function of the drivetrain.
 * 
 */
public class DriveSubsystem extends SubsystemBase {
    RobotConfig config;

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.ROTATIONAL_SLEW_RATE);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private Rotation2d m_trackedRotation = new Rotation2d();

    private static SwerveDrivePoseEstimator m_odometry;

    GyroIO gyroIO;
    GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    ModuleIO[] moduleIO;
    ModuleIOInputsAutoLogged[] moduleInputs;

    /** Creates a new Drivetrain. */
    public DriveSubsystem(ModuleIO[] moduleIOs, GyroIO gyroIO) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {

            this.moduleIO = moduleIOs;
            this.gyroIO = gyroIO;

            // Create one AutoLogged instance per module:
            moduleInputs = IntStream.range(0, moduleIOs.length)
                    .mapToObj(i -> new ModuleIOInputsAutoLogged())
                    .toArray(ModuleIOInputsAutoLogged[]::new);

            m_odometry = new SwerveDrivePoseEstimator(
                    DrivetrainConstants.DRIVE_KINEMATICS,
                    gyroIO.getGyroRotation2d(),
                    new SwerveModulePosition[] {
                            moduleIOs[0].getPosition(),
                            moduleIOs[1].getPosition(),
                            moduleIOs[2].getPosition(),
                            moduleIOs[3].getPosition()
                    }, new Pose2d());

            // swerveModules[0] = new SwerveModule(// Front Left
            // RobotConstants.PortConstants.CAN.FRONT_LEFT_DRIVING,
            // RobotConstants.PortConstants.CAN.FRONT_LEFT_TURNING,
            // RobotConstants.PortConstants.CAN.FRONT_LEFT_STEERING, false);

            // swerveModules[1] = new SwerveModule( // Front Right
            // RobotConstants.PortConstants.CAN.FRONT_RIGHT_DRIVING,
            // RobotConstants.PortConstants.CAN.FRONT_RIGHT_TURNING,
            // RobotConstants.PortConstants.CAN.FRONT_RIGHT_STEERING, false);

            // swerveModules[2] = new SwerveModule( // Rear Left
            // RobotConstants.PortConstants.CAN.REAR_LEFT_DRIVING,
            // RobotConstants.PortConstants.CAN.REAR_LEFT_TURNING,
            // RobotConstants.PortConstants.CAN.REAR_LEFT_STEERING, false);

            // swerveModules[3] = new SwerveModule( // Rear Right
            // RobotConstants.PortConstants.CAN.REAR_RIGHT_DRIVING,
            // RobotConstants.PortConstants.CAN.REAR_RIGHT_TURNING,
            // RobotConstants.PortConstants.CAN.REAR_RIGHT_STEERING, false);

            gyroIO.reset();
            resetEncoders();

            m_odometry = new SwerveDrivePoseEstimator(
                    DrivetrainConstants.DRIVE_KINEMATICS,
                    gyroIO.getGyroRotation2d(),
                    new SwerveModulePosition[] {
                            moduleIOs[0].getPosition(),
                            moduleIOs[1].getPosition(),
                            moduleIOs[2].getPosition(),
                            moduleIOs[3].getPosition()
                    }, new Pose2d(0, 0, new Rotation2d()));

        }

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> runChassisSpeeds(speeds, false), // Method that will drive the robot given ROBOT RELATIVE
                                                             // ChassisSpeeds. Also optionally outputs individual module
                                                             // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        PathfindingCommand.warmupCommand().schedule();

    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
        for (int i = 0; i < moduleIO.length; i++) {
            states[i] = moduleIO[i].getState();
        }

        return states;
    }

    private void updateOdometry() {
        // Update the odometry (Called in periodic)

        if (RobotBase.isSimulation()) {

            m_trackedRotation = m_trackedRotation.plus(new Rotation2d(
                    DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                            * ModuleIOSim.getPeriodicRate()));
            gyroIO.setGyroAngle(m_trackedRotation.getDegrees());
        }

        m_odometry.update(
                gyroIO.getGyroRotation2d(),
                new SwerveModulePosition[] {
                        moduleIO[0].getPosition(),
                        moduleIO[1].getPosition(),
                        moduleIO[2].getPosition(),
                        moduleIO[3].getPosition()
                });
        RobotState.robotPose = m_odometry.getEstimatedPosition();

        Logger.recordOutput("DriveSubsystem/OdometryPose", m_odometry.getEstimatedPosition());
    }

    private void updateOdometrySensorMeasurements() {

        // for (int i = 0; i < VisionSubsystem.getLengthOfCameraList(); i++) {
        // EstimatedRobotPose estimatedRobotPose = VisionSubsystem.getVisionPoses()[i];
        // try {
        // m_odometry.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
        // estimatedRobotPose.timestampSeconds);
        // if (i == 0) {
        // Logger.recordOutput("/VisionSubsystem/FL Adding Data", true);
        // }
        // if (i == 1) {
        // Logger.recordOutput("/VisionSubsystem/FR Adding Data", true);
        // }
        // if (i == 2) {
        // Logger.recordOutput("/VisionSubsystem/BL Adding Data", true);
        // }

        // } catch (Exception e) {
        // if (i == 0) {
        // Logger.recordOutput("/VisionSubsystem/FL Adding Data", false);
        // }
        // if (i == 1) {
        // Logger.recordOutput("/VisionSubsystem/FL Adding Data", false);
        // }
        // if (i == 2) {
        // Logger.recordOutput("/VisionSubsystem/BL Adding Data", false);
        // }
        // }
        // }

        // PV poses:

        TimestampedPose timestampedPose;
        while ((timestampedPose = RobotState.getAprilTagCameraMeasurments().poll()) != null) {
            m_odometry.addVisionMeasurement(
                    timestampedPose.pose(), timestampedPose.timestamp()
            // , QuestConstants.stdDevs
            );
        }

        // QuestNav poses:

        // if (DriverStation.isEnabled()) {

        //     while ((timestampedPose = RobotState.getQuestMeasurments().poll()) != null) {
        //         m_odometry.addVisionMeasurement(
        //                 timestampedPose.pose(), timestampedPose.timestamp()
        //         // , QuestConstants.stdDevs
        //         );
        //     }
        // }
    }

    @Override
    public void periodic() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            updateOdometry();
            Logger.recordOutput("Module states", getModuleStates());

            Logger.recordOutput("Gyro", gyroIO.getGyroAngle());

            for (int i = 0; i < moduleIO.length; i++) {
                moduleIO[i].updateInputs(moduleInputs[i]);
                // e.g. produces “DriveModule0”, “DriveModule1”, etc.
                // FL, FR, RL, RR
                Logger.processInputs("DriveSubsystem/DriveModule" + i, moduleInputs[i]);
            }
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("DriveSubsystem/Gyro", gyroInputs);
        }
        // Vision pose estimates are added into the main odometry filter if vision
        // subsystem is enabled.
        if (SubsystemEnabledConstants.VISION_SUBSYSTEM_ENABLED) {
            updateOdometrySensorMeasurements();
        }
    }

    public Pose2d getPose() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED ? m_odometry.getEstimatedPosition() : new Pose2d();
    }

    public void resetPose(Pose2d pose) {
        resetOdometry(pose);
    }

    public void resetOdometry(Pose2d pose) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            m_odometry.resetPosition(
                    gyroIO.getGyroRotation2d(),
                    new SwerveModulePosition[] {
                            moduleIO[0].getPosition(),
                            moduleIO[1].getPosition(),
                            moduleIO[2].getPosition(),
                            moduleIO[3].getPosition()
                    },
                    pose);
        }
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            double xSpeedCommanded;
            double ySpeedCommanded;

            if (rateLimit) {
                // Convert XY to polar for rate limiting
                double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
                double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

                // Calculate the direction slew rate based on an estimate of the lateral
                // acceleration
                double directionSlewRate;

                if (m_currentTranslationMag != 0.0) {
                    directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
                } else {
                    directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
                }

                double currentTime = WPIUtilJNI.now() * 1e-6;
                double elapsedTime = currentTime - m_prevTime;
                double angleDif = SwerveUtils.angleDifference(inputTranslationDir, m_currentTranslationDir);

                if (angleDif < 0.45 * Math.PI) {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                } else if (angleDif > 0.85 * Math.PI) {
                    if (m_currentTranslationMag > 1e-4) {
                        m_currentTranslationMag = m_magLimiter.calculate(0.0);
                    } else {
                        m_currentTranslationDir = SwerveUtils.wrapAngle(m_currentTranslationDir + Math.PI);
                        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                    }
                } else {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                }

                m_prevTime = currentTime;

                xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
                ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
                m_currentRotation = m_rotLimiter.calculate(rot);

            } else {
                xSpeedCommanded = xSpeed;
                ySpeedCommanded = ySpeed;
                m_currentRotation = rot;
            }

            if (Robot.isSimulation() && CowboyUtils.isRedAlliance()) {
                xSpeedCommanded = -xSpeedCommanded; // Away from the DS
                ySpeedCommanded = -ySpeedCommanded; // Away from long wall
                m_currentRotation = -m_currentRotation;
            }

            // Convert the commanded speeds into the correct units for the drivetrain
            double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

            Rotation2d rotation = gyroIO.getGyroRotation2d();

            var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation)
                            : speeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(
                    swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

            moduleIO[0].setDesiredState(swerveModuleStates[0]);
            moduleIO[1].setDesiredState(swerveModuleStates[1]);
            moduleIO[2].setDesiredState(swerveModuleStates[2]);
            moduleIO[3].setDesiredState(swerveModuleStates[3]);

        }
    }

    public void runChassisSpeeds(ChassisSpeeds speeds, Boolean fieldRelative) {
        Rotation2d rotation = Rotation2d.fromDegrees(gyroIO.getGyroAngle());

        var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation)
                        : speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

        moduleIO[0].setDesiredState(swerveModuleStates[0]);
        moduleIO[1].setDesiredState(swerveModuleStates[1]);
        moduleIO[2].setDesiredState(swerveModuleStates[2]);
        moduleIO[3].setDesiredState(swerveModuleStates[3]);

    }

    public ChassisSpeeds getTheoreticalSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            boolean rateLimit) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            double xSpeedCommanded;
            double ySpeedCommanded;

            if (rateLimit) {
                // Convert XY to polar for rate limiting
                double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
                double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

                // Calculate the direction slew rate based on an estimate of the lateral
                // acceleration
                double directionSlewRate;

                if (m_currentTranslationMag != 0.0) {
                    directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
                } else {
                    directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
                }

                double currentTime = WPIUtilJNI.now() * 1e-6;
                double elapsedTime = currentTime - m_prevTime;
                double angleDif = SwerveUtils.angleDifference(inputTranslationDir, m_currentTranslationDir);

                if (angleDif < 0.45 * Math.PI) {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                } else if (angleDif > 0.85 * Math.PI) {
                    if (m_currentTranslationMag > 1e-4) {
                        m_currentTranslationMag = m_magLimiter.calculate(0.0);
                    } else {
                        m_currentTranslationDir = SwerveUtils.wrapAngle(m_currentTranslationDir + Math.PI);
                        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                    }
                } else {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                }

                m_prevTime = currentTime;

                xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
                ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
                m_currentRotation = m_rotLimiter.calculate(rot);

            } else {
                xSpeedCommanded = xSpeed;
                ySpeedCommanded = ySpeed;
                m_currentRotation = rot;
            }

            // Convert the commanded speeds into the correct units for the drivetrain
            double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            Rotation2d rotation = gyroIO.getGyroRotation2d();

            return !fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                            rotation)
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        }
        return new ChassisSpeeds();
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            moduleIO[0].setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(45)));
            moduleIO[1].setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-45)));
            moduleIO[2].setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-45)));
            moduleIO[3].setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(45)));

        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    desiredStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
            moduleIO[0].setDesiredState(desiredStates[0]);
            moduleIO[1].setDesiredState(desiredStates[1]);
            moduleIO[2].setDesiredState(desiredStates[2]);
            moduleIO[3].setDesiredState(desiredStates[3]);

        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0 and sets the
     * turn encoders using the absolute encoders.
     */
    public void resetEncoders() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            moduleIO[0].resetEncoders();
            moduleIO[2].resetEncoders();
            moduleIO[1].resetEncoders();
            moduleIO[3].resetEncoders();
        }
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            gyroIO.reset();

        }
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Double getHeading() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED // if this is true
                ? (gyroIO.getGyroAngle())
                : 0;
    }

    public void pathFollowDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    private ChassisSpeeds getChassisSpeeds() {

        return RobotBase.isReal()
                ? ChassisSpeeds.fromRobotRelativeSpeeds(gyroIO.getVelocityX(), gyroIO.getVelocityY(),
                        Units.degreesToRadians(gyroIO.getRate()),
                        gyroIO.getGyroRotation2d())
                : ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d());
    }

    public Command gyroReset() {
        return Commands.startEnd(() -> {
            gyroIO.reset();
        }, () -> {
            // end
        });
    }

    public Command xCommand() {
        return new InstantCommand(() -> {
            // init
            RobotState.xLocked = !RobotState.xLocked;
        });

    }

}