package frc.robot.subsystems.drive;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents a subsystem responsible for controlling the drive system of a robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position and orientation, and other related tasks.
 */
public interface DriveSubsystem extends Subsystem {
    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getChassisSpeeds();

    double getGyroAngle();

    Pose2d getPose();

    void resetPose(Pose2d pose);

    void setX();
    
    void runChassisSpeeds(ChassisSpeeds speeds, Boolean fieldRelative);

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        resetPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);

    default void configurePPAutoBuilder() {
    
        try {
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds) -> this.runChassisSpeeds(speeds, false), // Method that will drive the robot given ROBOT RELATIVE
                                                                 // ChassisSpeeds. Also optionally outputs individual module
                                                                 // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                    // holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    RobotConfig.fromGUISettings(), // The robot configuration
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
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    default Command gyroReset() {
        return Commands.startEnd(() -> {
            // init
            if (RobotBase.isReal()) {
                zeroHeading();
            }
        }, () -> {
            // end
        });
    }

}