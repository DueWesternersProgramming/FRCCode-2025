package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class ModuleIOSim implements ModuleIO {
    /**
     * Main timer to simulate the passage of time.
     */
    private final Timer timer = new Timer();

    /**
     * Loop rate.
     */
    private static double dt;

    /**
     * Last time queried.
     */
    private double lastTime;

    /**
     * Fake motor position.
     */
    private double fakePos;

    /**
     * The fake speed of the previous state, used to calculate
     * {@link SwerveModuleSimulation#fakePos}.
     */
    private double fakeSpeed;

    /**
     * Current simulated swerve module state.
     */
    private SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0));

    public ModuleIOSim() {
        this.timer.start();
        this.lastTime = timer.get();
    }

    @Override
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState angularOffsetState = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle);

        // Optimize the reference state to avoid spinning further than 90 degrees.
        // SwerveModuleState optimizedState =
        // SwerveModuleState.optimize(angularOffsetState, state.angle);

        // Find timer stuff for updating the fake pos & speed
        double currentTime = timer.get();

        dt = currentTime - lastTime;
        lastTime = currentTime;
        state = angularOffsetState;
        // Set the state and fake pos & speed
        fakeSpeed = state.speedMetersPerSecond;
        fakePos += fakeSpeed * dt; // 20 ms
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(fakePos, state.angle);
    }

    @Override
    public SwerveModuleState getState() {
        return state;
    }

    public static double getPeriodicRate() {
        return dt;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

    }

}
