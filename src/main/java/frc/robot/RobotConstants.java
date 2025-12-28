package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.simulation.VisionSystemSim;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.SimCameraConfig;
import frc.robot.utils.CowboyUtils.RobotModes;

public final class RobotConstants {

        public static final class SimMode {
                public enum SimModes {
                        REGULAR,
                        REPLAY,
                }

                public static final SimModes SIM_MODE = SimModes.REGULAR;
        }

        public static final class ScoringConstants {

                public enum Setpoints {
                        HOME,
                        HP,
                        L1,
                        L2,
                        L3
                }

                public static final Pose2d[][] REEF_SIDE_POSES;

                static {
                        REEF_SIDE_POSES = new Pose2d[6][3];

                        // Reef 1 LEFT
                        REEF_SIDE_POSES[0][0] = new Pose2d(3.146, 4.191, new Rotation2d(Math.toRadians(0)));

                        // Reef 1 ALGAE
                        REEF_SIDE_POSES[0][1] = new Pose2d(3.146, 4.025, new Rotation2d(Math.toRadians(0)));

                        // Reef 1 RIGHT
                        REEF_SIDE_POSES[0][2] = new Pose2d(3.146, 3.859, new Rotation2d(Math.toRadians(0)));

                        // Reef 2 LEFT
                        REEF_SIDE_POSES[1][0] = new Pose2d(3.659, 2.947, new Rotation2d(Math.toRadians(60)));

                        // Reef 2 ALGAE
                        REEF_SIDE_POSES[1][1] = new Pose2d(3.812, 2.861, new Rotation2d(Math.toRadians(60)));

                        // Reef 2 RIGHT
                        REEF_SIDE_POSES[1][2] = new Pose2d(3.978, 2.801, new Rotation2d(Math.toRadians(60)));

                        // Reef 3 LEFT
                        REEF_SIDE_POSES[2][0] = new Pose2d(5.009, 2.781, new Rotation2d(Math.toRadians(120)));

                        // Reef 3 ALGAE
                        REEF_SIDE_POSES[2][1] = new Pose2d(5.168, 2.868, new Rotation2d(Math.toRadians(120)));

                        // Reef 3 RIGHT
                        REEF_SIDE_POSES[2][2] = new Pose2d(5.302, 2.954, new Rotation2d(Math.toRadians(120)));

                        // Reef 4 LEFT
                        REEF_SIDE_POSES[3][0] = new Pose2d(5.827, 3.839, new Rotation2d(Math.toRadians(180)));

                        // Reef 4 ALGAE
                        REEF_SIDE_POSES[3][1] = new Pose2d(5.827, 4.025, new Rotation2d(Math.toRadians(180)));

                        // Reef 4 RIGHT
                        REEF_SIDE_POSES[3][2] = new Pose2d(5.827, 4.171, new Rotation2d(Math.toRadians(180)));

                        // Reef 5 LEFT
                        REEF_SIDE_POSES[4][0] = new Pose2d(5.308, 5.103, new Rotation2d(Math.toRadians(-120)));

                        // Reef 5 ALGAE
                        REEF_SIDE_POSES[4][1] = new Pose2d(5.155, 5.183, new Rotation2d(Math.toRadians(-120)));

                        // RIGHT
                        REEF_SIDE_POSES[4][2] = new Pose2d(5.022, 5.262, new Rotation2d(Math.toRadians(-120)));

                        // Reef 6 LEFT
                        REEF_SIDE_POSES[5][0] = new Pose2d(3.965, 5.269, new Rotation2d(Math.toRadians(-60)));

                        // Reef 6 ALGAE
                        REEF_SIDE_POSES[5][1] = new Pose2d(3.825, 5.183, new Rotation2d(Math.toRadians(-60)));

                        // Reef 6 RIGHT
                        REEF_SIDE_POSES[5][2] = new Pose2d(3.665, 5.096, new Rotation2d(Math.toRadians(-60)));
                }

                public static final List<Pose2d> HP_POSES = List.of(
                                new Pose2d(1.11, 7.125, new Rotation2d(Units.degreesToRadians(125))), // L
                                new Pose2d(1.007, 0.884, new Rotation2d(Units.degreesToRadians(-125))));// R

        }

        public static final class DrivetrainConstants {
                public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0; // -We do not apply an offset to the
                                                                                   // CANcoder
                                                                                   // angle, we just zero the encoders
                                                                                   // with the
                                                                                   // wheels forward with bolt side
                                                                                   // facing
                                                                                   // LEFT!!!
                                                                                   // -In radians not degrees
                public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double MAX_SPEED_METERS_PER_SECOND = 6.0; // 4.42; //4.8;
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

                public static final double DIRECTION_SLEW_RATE = 25;
                public static final double MAGNITUDE_SLEW_RATE = 25; // Responsiveness, or the "jerk" of the drivebase
                public static final double ROTATIONAL_SLEW_RATE = 10;

                // Chassis configuration

                public static final double DRIVE_BASE_RADIUS_METERS = Units.inchesToMeters(17.5); // measurement from
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
                public static final double ELEVATOR_MAX_HEIGHT = 71;
                public static final double ELEVATOR_MIN_HEIGHT = 2;

                public static final class HeightSetpoints {
                        public static final double HOME = 0.5;
                        public static final double HP = 12.4;

                        public static final class Coral {
                                public static final double L1 = 8.15;
                                public static final double L2 = 12;
                                public static final double L3 = 17.35;
                        }

                        public static final class Algae {
                                public static final double L2 = 8.4;
                                public static final double L3 = 17.2;
                        }

                        public static final class SimConstants {
                                public static final double HOME = 0;
                                public static final double HP = .35;
                                public static final double L1 = 0.2;
                                public static final double L2 = 0.3;
                                public static final double L3 = 0.55;
                        }
                }

        }

        public static final class ClawConstants {
                public static final double INTAKE_CORAL_SPEED = 0.5;
                public static final double OUTTAKE_CORAL_SPEED = -1;
                public static final double INTAKE_ALGAE_SPEED = -0.75;
                public static final double OUTTAKE_ALGAE_SPEED = 1;
        }

        public static final class WristConstants {
                public static final double WRIST_MIN_ANGLE = 0.0;
                public static final double ELEVATOR_MAX_ANGLE = 30.0;

                public static final class AngleSetpoints {
                        public static final double HOME = -5;
                        public static final double HP = -13.0;

                        public static final class Coral {
                                public static final double L1 = -9;
                                public static final double L2 = -3;
                                public static final double L3 = -7;
                        }

                        public static final class Algae {
                                public static final double L2 = 5;
                                public static final double L3 = 8.5;
                        }

                        public static final class SimConstants {
                                public static final double HOME = 0;
                                public static final double HP = 0;
                                public static final double L1 = 0;
                                public static final double L2 = 0;
                                public static final double L3 = 0;
                        }
                }

                public static final double MAX_MOTOR_RPM = 10000.0;
                public static final double MAX_MOTOR_ACCELERATION = 4000.0;
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

                // public static final boolean TURNING_ENCODER_INVERTED = false;

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

                public static final double TURNING_MOTOR_REDUCTION = 12.8; // Ratio between internal relative
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

                        public static final int WRIST_MOTOR = 15;

                        public static final int CLAW_MOTOR_1 = 16;
                        public static final int CLAW_MOTOR_2 = 17;

                        public static final int PDH = 18;

                }

                public static class Controller {
                        public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
                        public static final int DRIVE_JOYSTICK = 0;
                        public static final int OPERATOR_JOYSTICK = 1;

                        // Joystick Axis

                        public static final int DRIVE_COMMAND_X_AXIS = 0;
                        public static final int DRIVE_COMMAND_Y_AXIS = 1;
                        public static final int DRIVE_COMMAND_ROT_AXIS = 2; // 2 for the flight controller, 4 for
                                                                            // xbox/gamepad

                        // Manual control axis for operator
                        public static final int ELEVATOR_MANUAL_CONTROL = 1;
                        public static final int WRIST_MANUAL_CONTROL = 5;
                }
        }

        public static final class TeleopConstants {
                public static final double MAX_SPEED_PERCENT = 1; // ex: 0.4 -> 40%
        }

        public static final class PathPlannerConstants {
                // public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

                public static final double kMaxAngularAcceleration = 4 * Math.PI;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3.00;

                public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                                .3,
                                .3,
                                .2,
                                2 * Math.PI);

                public static final double MAX_VELOCITY = 6.0; // Meters per second
                public static final double MAX_ACCELERATION = 6.0; // Meters per second squared
                public static final double MAX_ANGULAR_SPEED = 540.0; // Degrees per second
                public static final double MAX_ANGULAR_ACCELERATION = 720.0; // Degrees per second squared
        }

        public static final class VisionConstants {
                public static enum VisionMode {
                        APRIL_TAG_ONLY,
                        QUEST_NAV_ONLY,
                        HYBRID
                }

                public static final int[] REEF_IDS = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

                public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {
                }

                public record VisionSource(String name, Transform3d robotToCamera) {
                }

                public static final Optional<VisionSystemSim> aprilTagSim = RobotModes.currentMode == RobotModes.simMode
                                ? Optional.of(new VisionSystemSim("AprilTagSim"))
                                : Optional.empty();

                // configure our camera objects here
                public static final List<AprilTagCameraConfig> CAMERA_CONFIGS = List.of(
                                // Front Left
                                new AprilTagCameraConfig(
                                                new VisionSource(
                                                                "frontLeftCamera",
                                                                new Transform3d(
                                                                                new Translation3d(
                                                                                                Units.inchesToMeters(
                                                                                                                5.125), // forward+
                                                                                                Units.inchesToMeters(
                                                                                                                7.99), // left+
                                                                                                Units.inchesToMeters(
                                                                                                                13.725000)), // up+
                                                                                new Rotation3d(
                                                                                                // Counter clockwise
                                                                                                // positive
                                                                                                Units.degreesToRadians(
                                                                                                                0),
                                                                                                Units.degreesToRadians(
                                                                                                                0),
                                                                                                Units.degreesToRadians(
                                                                                                                0)))),
                                                SimCameraConfig.ARDUCAM_OV9281_55),
                                // Front Right
                                new AprilTagCameraConfig(
                                                new VisionSource(
                                                                "frontRightCamera",
                                                                new Transform3d(
                                                                                new Translation3d(
                                                                                                Units.inchesToMeters(
                                                                                                                5.077711), // forward+
                                                                                                Units.inchesToMeters(
                                                                                                                -8.006511), // left+
                                                                                                Units.inchesToMeters(
                                                                                                                24.964102)), // up+
                                                                                new Rotation3d(
                                                                                                Units.degreesToRadians(
                                                                                                                0),
                                                                                                Units.degreesToRadians(
                                                                                                                -27.5),
                                                                                                Units.degreesToRadians(
                                                                                                                10)))),
                                                SimCameraConfig.ARDUCAM_OV9281_55),
                                // Back Right
                                new AprilTagCameraConfig(
                                                new VisionSource(
                                                                "backLeftCamera", new Transform3d(
                                                                                new Translation3d(
                                                                                                Units.inchesToMeters(
                                                                                                                -4.361060), // forward+
                                                                                                Units.inchesToMeters(
                                                                                                                9.375080), // left+
                                                                                                Units.inchesToMeters(
                                                                                                                24.57)), // up+
                                                                                new Rotation3d(
                                                                                                Units.degreesToRadians(
                                                                                                                0),
                                                                                                Units.degreesToRadians(
                                                                                                                -5),
                                                                                                Units.degreesToRadians(
                                                                                                                180 + 5)))),
                                                SimCameraConfig.ARDUCAM_OV9281_55));

                public static final Transform3d[] CAMERA_POSITIONS = {
                                new Transform3d(
                                                // Front Left
                                                new Translation3d(
                                                                Units.inchesToMeters(5.125), // forward+
                                                                Units.inchesToMeters(7.99), // left+
                                                                Units.inchesToMeters(13.725000)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(0), // Note, these are all
                                                                                           // counter clockwise
                                                                                           // so to
                                                                                           // face up we
                                                                                           // need
                                                                                           // - ;)
                                                                Units.degreesToRadians(0))),

                                // Front Right
                                new Transform3d(
                                                new Translation3d(
                                                                Units.inchesToMeters(5.077711), // forward+
                                                                Units.inchesToMeters(-8.006511), // left+
                                                                Units.inchesToMeters(24.964102)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-22.5), // Note, these are
                                                                                               // all
                                                                                               // counter clockwise
                                                                                               // so to
                                                                                               // face up we
                                                                                               // need - ;)
                                                                Units.degreesToRadians(0))),
                                // Back Left
                                new Transform3d(
                                                new Translation3d(
                                                                Units.inchesToMeters(-4.361060), // forward+
                                                                Units.inchesToMeters(9.375080), // left+
                                                                Units.inchesToMeters(24.57)), // up+
                                                new Rotation3d(
                                                                Units.degreesToRadians(0),
                                                                Units.degreesToRadians(-5), // Note, these are all
                                                                                            // counter clockwise so
                                                                                            // to
                                                                                            // face up we
                                                                                            // need - ;)
                                                                Units.degreesToRadians(180 + 5))) };
        }

        public static final class QuestNavConstants {
                public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
                                0.02, // Trust down to 2cm in X direction
                                0.02, // Trust down to 2cm in Y direction
                                0.035 // Trust down to 2 degrees rotational
                );
                public static final Transform2d ROBOT_TO_QUEST = new Transform2d(
                                new Translation2d(.0958, .20249092),
                                Rotation2d.kCCW_90deg);
        }

        public static final class ConfigurableAutonsConstants {
                public enum HumanPlayerPositions {
                        LEFT,
                        RIGHT
                }

                public enum StartingPositions {
                        LEFT,
                        MIDDLE,
                        RIGHT
                }

                public enum SidePosition {
                        LEFT,
                        ALGAE,
                        RIGHT
                }
        }

        public static final class SubsystemEnabledConstants {
                public static final boolean VISION_SUBSYSTEM_ENABLED = true;
        }
}
