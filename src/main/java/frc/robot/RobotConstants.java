package frc.robot;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class RobotConstants {

        public static final class ScoringConstants {
                public static final class BlueAlliance {
                        public static final List<Pose2d> poses = List.of(
                                        new Pose2d(2.961, 4, new Rotation2d(Math.toRadians(0))),
                                        new Pose2d(3.712, 2.723, new Rotation2d(Math.toRadians(30))),
                                        new Pose2d(5.253, 2.656, new Rotation2d(Math.toRadians(-120))),
                                        new Pose2d(6, 4, new Rotation2d(Math.toRadians(180))),
                                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));

                }

        }

        public static final class DrivetrainConstants {
                public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0; // -We do not apply an offset to the
                                                                                   // CANcoder
                                                                                   // angle, we just zero the encoders
                                                                                   // with the
                                                                                   // wheels forward
                                                                                   // -In radians
                public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double MAX_SPEED_METERS_PER_SECOND = 6.0; // 4.42; //4.8;
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

                public static final double DIRECTION_SLEW_RATE = 25;// radians per second
                public static final double MAGNITUDE_SLEW_RATE = 25;// percent per second (1 = 100%)
                public static final double ROTATIONAL_SLEW_RATE = 10;// percent per second (1 = 100%)

                // Chassis configuration

                public static final double DRIVE_BASE_RADIUS_METERS = Units.inchesToMeters(17.5);; // measurement from
                                                                                                   // center point of
                                                                                                   // robot
                // to the
                // center of one of the wheels. (use the
                // CAD)

                public static final double LEFT_RIGHT_DISTANCE_METERS = Units.inchesToMeters(24.750000); // Distance
                                                                                                         // between
                                                                                                         // centers of
                                                                                                         // right
                // and left wheels on robot

                public static final double FRONT_BACK_DISTANCE_METERS = Units.inchesToMeters(24.750000);// Distance
                                                                                                        // between
                                                                                                        // front and
                                                                                                        // back
                // wheels on robot

                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(LEFT_RIGHT_DISTANCE_METERS / 2, FRONT_BACK_DISTANCE_METERS / 2),
                                new Translation2d(LEFT_RIGHT_DISTANCE_METERS / 2, -FRONT_BACK_DISTANCE_METERS / 2),
                                new Translation2d(-LEFT_RIGHT_DISTANCE_METERS / 2, FRONT_BACK_DISTANCE_METERS / 2),
                                new Translation2d(-LEFT_RIGHT_DISTANCE_METERS / 2, -FRONT_BACK_DISTANCE_METERS / 2));

                public static final int GYRO_ORIENTATION = -1; // 1 for upside down, -1 for right side up.

                public static final boolean FIELD_RELATIVE = true;
        }

        public static final class ElevatorConstants {
                public static final double ELEVATOR_MAX_HEIGHT = 0.0;
                public static final double ELEVATOR_MIN_HEIGHT = 0.0;

                public static final class HeightSetpoints {
                        public static final double L1 = 0;
                        public static final double L2 = 0;
                        public static final double L3 = 0;
                }

                public static final double MAX_MOTOR_RPM = 500.0;
                public static final double MAX_MOTOR_ACCELERATION = 500.0;

                public static final class SimConstants {
                        public static final double MIN_LIGMENT_LENGTH = 0.043885;
                        public static final double L1 = 0.65;
                        public static final double L2 = 0.85;
                        public static final double L3 = 1.4;
                }

        }

        public static final class WristConstants {
                public static final double WRIST_MIN_ANGLE = 0.0;
                public static final double ELEVATOR_MIN_HEIGHT = 30.0;

                public static final class AngleSetpoints {
                        public static final double L1 = 0;
                        public static final double L2 = 0;
                        public static final double L3 = 0;
                }

                public static final double MAX_MOTOR_RPM = 500.0;
                public static final double MAX_MOTOR_ACCELERATION = 500.0;

                public static final class SimConstants {
                        public static final double L1 = 85;
                        public static final double L2 = 87.5;
                        public static final double L3 = 120;
                }

        }

        public static final class SwerveModuleConstants {

                public static final double TRANSLATION_P = 1.0;
                public static final double ROT_MOTION_P = 0.0;

                public static final double TRANSLATION_I = 0.0;
                public static final double ROT_MOTION_I = 0.0;

                public static final double TRANSLATION_D = 0.0;
                public static final double ROT_MOTION_D = 0.0;

                public static final double FREE_SPEED_RPM = 5676;

                public static final int DRIVING_MOTOR_PINION_TEETH = 16;

                public static final boolean TURNING_ENCODER_INVERTED = false;

                public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
                public static final double WHEEL_DIAMETER_METERS = 0.1016;
                public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
                public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50)
                                / (DRIVING_MOTOR_PINION_TEETH * 15 * 27);
                public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                                * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

                public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS
                                * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
                public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
                                * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

                public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // Ratio between internal relative
                                                                                  // encoder and
                                                                                  // the absolute encoder

                public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI)
                                / TURNING_MOTOR_REDUCTION; // radians, per rotation
                public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI)
                                / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

                public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
                public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

                // These PID constants relate to the movement and acceleration of the swerve
                // motors themselfs.
                public static final double DRIVING_P = 0.07;
                public static final double DRIVING_I = 0;
                public static final double DRIVING_D = 0;
                public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
                public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
                public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

                public static final double TURNING_P = 1.25;
                public static final double TURNING_I = 0;
                public static final double TURNING_D = 0;
                public static final double TURNING_FF = 0;
                public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
                public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

                public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
                public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

                public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; // amps
                public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
        }

        public static interface PortConstants {

                public static class CAN {
                        public static final int FRONT_LEFT_DRIVING = 5;
                        public static final int REAR_LEFT_DRIVING = 7;
                        public static final int FRONT_RIGHT_DRIVING = 6;
                        public static final int REAR_RIGHT_DRIVING = 8;

                        public static final int FRONT_LEFT_TURNING = 9;
                        public static final int REAR_LEFT_TURNING = 11;
                        public static final int FRONT_RIGHT_TURNING = 10;
                        public static final int REAR_RIGHT_TURNING = 12;

                        public static final int FRONT_LEFT_STEERING = 1;
                        public static final int FRONT_RIGHT_STEERING = 2;
                        public static final int REAR_LEFT_STEERING = 3;
                        public static final int REAR_RIGHT_STEERING = 4;

                        public static final int ELEVATOR_MOTOR_1 = 13;
                        public static final int ELEVATOR_MOTOR_2 = 14;

                }

                public static class Controller {
                        public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
                        public static final int DRIVE_JOYSTICK = 0;
                        public static final int PANEL = 1;
                        public static final int OPERATOR_JOYSTICK = 1;

                        // Joystick Axis
                        public static final int DRIVE_COMMAND_X_AXIS = 0;
                        public static final int DRIVE_COMMAND_Y_AXIS = 1;
                        public static final int DRIVE_COMMAND_ROT_AXIS = 4;
                }
        }

        public static final class AutonomousConstants {
                public static final boolean FLIP_PATHPLANNER_AUTOS = false;

                public static final double X_CONTROLLER_P = 3.5;
                public static final double Y_CONTROLLER_P = 3.5;
                public static final double THETA_CONTROLLER_P = 5;

                public static final double X_CONTROLLER_I = 0;
                public static final double Y_CONTROLLER_I = 0;
                public static final double THETA_CONTROLLER_I = 0;

                public static final double X_CONTROLLER_D = 0;
                public static final double Y_CONTROLLER_D = 0;
                public static final double THETA_CONTROLLER_D = 0;

                public static final double FIELD_LENGTH_INCHES = 54 * 12 + 1; // 54ft 1in
                public static final double FIELD_WIDTH_INCHES = 26 * 12 + 7; // 26ft 7in

        }

        public static final class TeleopConstants {
                public static final double MAX_SPEED_PERCENT = 1; // ex: 0.4 -> 40%
        }

        public static final class PathPlannerConstants {
                // public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

                public static final double kMaxAngularAcceleration = 4 * Math.PI;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3.00;

                public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                                DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                                PathPlannerConstants.kMaxAccelerationMetersPerSecondSquared,
                                DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                5 * Math.PI);

                public static final double MAX_VELOCITY = 6.0; // Meters per second
                public static final double MAX_ACCELERATION = 6.0; // Meters per second squared
                public static final double MAX_ANGULAR_SPEED = 540.0; // Degrees per second
                public static final double MAX_ANGULAR_ACCELERATION = 720.0; // Degrees per second squared
        }

        public static final class VisionConstants {
                public static final Transform3d[] CAMERA_POSITIONS = {
                                new Transform3d(
                                                // Front Left
                                                new Translation3d(
                                                                Units.inchesToMeters(5.56), // forward+
                                                                Units.inchesToMeters(7.99), // left+
                                                                Units.inchesToMeters(9.67)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-20), // Note, these are all
                                                                                             // counter clockwise so to
                                                                                             // face up we
                                                                                             // need
                                                                                             // - ;)
                                                                Units.degreesToRadians(15))),

                                // Front Right
                                new Transform3d(
                                                new Translation3d(
                                                                Units.inchesToMeters(5.56), // forward+
                                                                Units.inchesToMeters(-7.99), // left+
                                                                Units.inchesToMeters(25.416)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-20), // Note, these are all
                                                                                             // counter clockwise so to
                                                                                             // face up we
                                                                                             // need -40 ;)
                                                                Units.degreesToRadians(-15))),
                                // Back
                                new Transform3d(
                                                new Translation3d(
                                                                Units.inchesToMeters(13.5), // forward+
                                                                Units.inchesToMeters(12.75), // left+
                                                                Units.inchesToMeters(9)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-40), // Note, these are all
                                                                                             // counter clockwise so to
                                                                                             // face up we
                                                                                             // need -40 ;)
                                                                Units.degreesToRadians(-15))) };
        }

        public static final class SubsystemEnabledConstants {
                public static final boolean DRIVE_SUBSYSTEM_ENABLED = true;
                public static final boolean VISION_SUBSYSTEM_ENABLED = false;
        }
}