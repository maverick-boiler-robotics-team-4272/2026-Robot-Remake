package frc.robot.intake;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }


    public Command setIntake(double rollerRps, double pivotRotations) {
        return runEnd(() -> io.setIntakeState(rollerRps, pivotRotations), () -> io.defaultState());
    }
}