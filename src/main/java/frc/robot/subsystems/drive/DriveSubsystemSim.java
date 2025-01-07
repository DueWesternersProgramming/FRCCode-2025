package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutMass;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystemSim implements DriveSubsystem {
    private final SwerveDriveSimulation simulatedDrive;
    private final Field2d field2d = new Field2d();

    public DriveSubsystemSim() {
        // For your own code, please configure your drivetrain properly according to the documentation
        // Create and configure a drivetrain simulation configuration
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofNav2X())
        .withSwerveModule(COTS.ofMark4(
                DCMotor.getNEO(1), // Drive motor is a Kraken X60
                DCMotor.getNEO(1), // Steer motor is a Falcon 500
                COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                2)) // L2 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(37), Inches.of(37));

        // Creating the SimplifiedSwerveDriveSimulation instance
        this.simulatedDrive = new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d()));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive);

        // A field2d widget for debugging
        SmartDashboard.putData("Simulation field", field2d);
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return simulatedDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    @Override
    public Rotation2d getHeading() {
        return simulatedDrive.getGyroSimulation().getGyroReading();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getSimulatedDriveTrainPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        // Method not implemented as SwerveDriveSimulation does not support addVisionMeasurement
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulationPeriodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getSimulatedDriveTrainPose());
        field2d.getObject("odometry").setPose(getPose());
    }

    @Override
    public double getGyroAngle() {
        return simulatedDrive.getGyroSimulation().getGyroReading().getDegrees();
        
    }

    @Override
    public void setX() {
    }

    @Override
    public void runChassisSpeeds(ChassisSpeeds speeds, Boolean fieldRelative) {
        simulatedDrive.setRobotSpeeds(speeds);
    }

}