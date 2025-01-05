package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

    Mechanism2d arm;
    MechanismRoot2d root;
    MechanismLigament2d joint1;
    MechanismLigament2d joint2;

    public ExampleSubsystem() {
        if (RobotBase.isReal()) {

        } else {
            arm = new Mechanism2d(4, 4); // Max reachable area
            root = arm.getRoot("base", 2, 0);
            joint1 = root.append(new MechanismLigament2d("joint1", 1, 20 + 90));
            joint2 = joint1.append(new MechanismLigament2d("joint2", 0.5, -10 + 90));
            // An example to show how this could be done

        }
    }

    public Command setArmAngle(double angle) {
        return run(() -> {
            joint1.setAngle(angle);
            joint2.setAngle(-angle);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("test mechanism", arm);
    }
}