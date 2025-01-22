package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.WristConstants;

public class ElevatorWristSim {

    private static MechanismLigament2d m_elevator;
    private static MechanismLigament2d m_wrist;
    static double ElevatorLength = ElevatorConstants.SimConstants.MIN_LIGMENT_LENGTH;
    static double WristAngle = 90;

    public ElevatorWristSim() {

        Mechanism2d mech = new Mechanism2d(3, 6);
        // the mechanism root node
        MechanismRoot2d root = mech.getRoot("climber", (1.5 - Units.inchesToMeters(5)), 0);

        // MechanismLigament2d objects represent each "section"/"stage" of the
        // mechanism, and are based
        // off the root node or another ligament object
        m_elevator = root
                .append(new MechanismLigament2d("elevator", ElevatorConstants.SimConstants.MIN_LIGMENT_LENGTH, 85, 6,
                        new Color8Bit(Color.kRed)));

        m_wrist = m_elevator.append(
                new MechanismLigament2d("wrist", Units.inchesToMeters(14.5), -90, 6, new Color8Bit(Color.kBlue)));
        SmartDashboard.putData("mech2d", mech);
    }

    public static void update() {
        m_elevator.setLength(ElevatorLength);
        m_wrist.setAngle(-WristAngle);
    }

    public static void setElevatorSimSetpoint(int level) {
        if (level == 1) {
            ElevatorLength = ElevatorConstants.SimConstants.L1;
        } else if (level == 2) {
            ElevatorLength = ElevatorConstants.SimConstants.L2;
        } else if (level == 3) {
            ElevatorLength = ElevatorConstants.SimConstants.L3;
        } else {
            ElevatorLength = ElevatorConstants.SimConstants.L1;
        }

    }

    public static void setWristSimSetpoint(int level) {
        if (level == 1) {
            WristAngle = WristConstants.SimConstants.L1;
        } else if (level == 2) {
            WristAngle = WristConstants.SimConstants.L2;
        } else if (level == 3) {
            WristAngle = WristConstants.SimConstants.L3;
        } else {
            WristAngle = WristConstants.SimConstants.L1;
        }
    }

    private static void setWristAngle(double angle) {
        WristAngle = angle;
    }
}
