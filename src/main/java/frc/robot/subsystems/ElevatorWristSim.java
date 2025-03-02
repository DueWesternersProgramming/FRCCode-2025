package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotConstants.ElevatorConstants;

//@Logged
public class ElevatorWristSim {

    // Get the default NetworkTable instance and a table (choose a table name of
    // your choice).
    private final static NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("3dSimulation");
    static StructPublisher<Pose3d> firstStagePublisher = networkTable.getStructTopic("FirstStage", Pose3d.struct)
            .publish();
    static StructPublisher<Pose3d> secondStagePublisher = networkTable.getStructTopic("SecondStage", Pose3d.struct)
            .publish();

    static Pose3d firstStage = new Pose3d(0, 0, 0,
            new Rotation3d(Units.degreesToRadians(90), Units.degreesToRadians(0), Units.degreesToRadians(90)));

    static Pose3d secondStage = new Pose3d(0, 0, 0,
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));

    static double elevatorLength = 0;

    public ElevatorWristSim() {
        // Ensure the table name is consistent; here we use "3dSimulation".

        firstStagePublisher.set(getFirstStagePositioning(0, firstStage));

        StructPublisher<Pose3d> secondStagePublisher = networkTable.getStructTopic("SecondStage", Pose3d.struct)
                .publish();
        secondStagePublisher.set(getSecondStagePositioning(0, secondStage));

        // Test:
        setElevatorToHeight(.25);
    }

    private static Pose3d getFirstStagePositioning(double lengthMeters, Pose3d currentPose3d) {
        double x = Math.cos(Units.degreesToRadians(85)) * lengthMeters;
        double z = Math.sin(Units.degreesToRadians(85)) * lengthMeters;
        return new Pose3d(x, 0, z, currentPose3d.getRotation());
    }

    private static Pose3d getSecondStagePositioning(double lengthMeters, Pose3d currentPose3d) {
        double x = Math.cos(Units.degreesToRadians(85)) * (2 * lengthMeters);
        double z = Math.sin(Units.degreesToRadians(85)) * (2 * lengthMeters); // 2:1 stage ratio
        return new Pose3d(x, 0, z, currentPose3d.getRotation());
    }

    public static void setElevatorToHeight(double lengthMeters) {
        firstStage = getFirstStagePositioning(lengthMeters, firstStage);
        firstStagePublisher.set(firstStage);

        secondStage = getSecondStagePositioning(lengthMeters, secondStage);
        secondStagePublisher.set(secondStage);

        elevatorLength = lengthMeters;
    }

    public static void goToScoreSetpoint(int level) {
        if (level == 1) {
            setElevatorToHeight(ElevatorConstants.SimConstants.L1);
        }
        if (level == 2) {
            setElevatorToHeight(ElevatorConstants.SimConstants.L2);
        }
        if (level == 3) {
            setElevatorToHeight(ElevatorConstants.SimConstants.L3);
        }
        if (level == 0) {
            setElevatorToHeight(0);
        }
    }

    public static double getElevatorSimLength() {
        return elevatorLength;
    }

}
