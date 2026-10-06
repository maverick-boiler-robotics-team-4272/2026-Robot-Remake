package frc.robot.drum;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drum extends SubsystemBase {
    private final DrumIO io;
    private final DrumIOInputsAutoLogged inputs = new DrumIOInputsAutoLogged();
    
    private final Alert motorRDis;
    private final Alert motorLDis;
    private final Alert motorHoodDis;

    public Drum(DrumIO io) {
        this.io = io;
        motorRDis = new Alert("Right Shooter Motor has disconneted", AlertType.kError);
        motorLDis = new Alert("Left Shooter Motor has disconneted", AlertType.kError);
        motorHoodDis = new Alert("Hood Motor has disconneted", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drum", inputs);
        motorRDis.set(!inputs.motorRIsConnected);
        motorLDis.set(!inputs.motorLIsConnected);
        motorHoodDis.set(!inputs.motorHoodIsConnected);
    }

    public Command setDrumState(double rps, Rotation2d angle) {
        return runEnd(
            () -> {
                io.setDrumState(rps, angle);
            },
            () -> io.defaultState()
        );
    }
}
