package frc.robot.subsystems;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double rollerGoalRps = 0.0;
    private double pivotGoalRotations = 0.0;

    public Intake(IntakeIO io) {
        this.io = io;
        setDefaultCommand(hold());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (DriverStation.isDisabled()) {
            io.stop();
        }

        Logger.recordOutput("Intake/RollerGoalRps", rollerGoalRps);
        Logger.recordOutput("Intake/PivotGoalRotations", pivotGoalRotations);
        Logger.recordOutput("Intake/AtGoal", atGoal());
    }

    public boolean atGoal() {
        return Math.abs(inputs.pivotPosition.getRotations() - pivotGoalRotations) <= PIVOT_TOLERANCE_ROTATIONS;
    }

    private void setGoal(double rollerRps, double pivotRotations) {
        rollerGoalRps = rollerRps;
        pivotGoalRotations = pivotRotations;
        io.setRollerVelocity(rollerRps);
        io.setPivotPosition(pivotRotations);
    }

    /** Rollers off, pivot holds the position it was at when this started. Runs by default. */
    public Command hold() {
        double[] holdRotations = new double[1];
        return startRun(
                        () -> holdRotations[0] = inputs.pivotPosition.getRotations(),
                        () -> setGoal(0.0, holdRotations[0]))
                .withName("Intake Hold");
    }

    /** Pivot to the deployed position, rollers off. */
    public Command deploy() {
        return goalCommand(0.0, PIVOT_DEPLOYED_ROTATIONS).withName("Intake Deploy");
    }

    /** Pivot to the stowed position, rollers off. */
    public Command stow() {
        return goalCommand(0.0, PIVOT_STOWED_ROTATIONS).withName("Intake Stow");
    }

    /** Pivot deployed and rollers pulling in; the pivot holds where it is when released. */
    public Command intake() {
        return goalCommand(ROLLER_INTAKE_RPS, PIVOT_DEPLOYED_ROTATIONS).withName("Intake Intake");
    }

    /** Pivot deployed and rollers pushing out; the pivot holds where it is when released. */
    public Command eject() {
        return goalCommand(ROLLER_EJECT_RPS, PIVOT_DEPLOYED_ROTATIONS).withName("Intake Eject");
    }

    /** Roller velocity (rotations/sec) and pivot position (rotations) while scheduled. */
    public Command goalCommand(double rollerRps, double pivotRotations) {
        return run(() -> setGoal(rollerRps, pivotRotations));
    }
}
