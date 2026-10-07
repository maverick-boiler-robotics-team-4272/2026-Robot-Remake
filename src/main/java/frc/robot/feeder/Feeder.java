package frc.robot.feeder;


import static frc.robot.constants.SubsystemConstants.FeederConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    FeederIO io;
    FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged(); 

    public Feeder(FeederIO io) {
            this.io = io;
        }
    
    @Override
    public void periodic() {
        io.updateInputs(feederInputs);
    }
    public Command feedRun(double rps) {
        return runEnd(
            () -> {
                io.setFeederState(rps);
            }, 
            () -> {
                io.defaultState();
            }
        );
    }

    public Command defaultCommand() {
        return run(() -> io.defaultState());
    }
}