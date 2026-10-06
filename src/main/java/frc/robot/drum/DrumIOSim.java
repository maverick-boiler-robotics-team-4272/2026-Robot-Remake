package frc.robot.drum;

import static frc.robot.constants.SubsystemConstants.DrumConstants.DRUM_GEARING;
import static frc.robot.constants.SubsystemConstants.DrumConstants.HOOD_GEARING;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class DrumIOSim extends DrumIOReal {
    private final FlywheelSim drumModel = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2), 0.1, DRUM_GEARING), DCMotor.getKrakenX60Foc(2));
    private final DCMotorSim hoodModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), 0.015, HOOD_GEARING), 
        DCMotor.getKrakenX44Foc(1)
    );

    private TalonFXSimState motorRSim;
    private TalonFXSimState motorLSim;
    private TalonFXSimState motorHoodSim;

    public DrumIOSim() {
        motorRSim = motorR.getSimState();
        motorLSim = motorL.getSimState();
        motorHoodSim = motorHood.getSimState();
    }

    @Override
    public void updateInputs(DrumIOInputs inputs) {
        motorRSim = motorR.getSimState();
        motorLSim = motorL.getSimState();
        motorHoodSim = motorHood.getSimState();

        motorRSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorLSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorHoodSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double volt = motorRSim.getMotorVoltage();
        drumModel.setInputVoltage(volt);
        drumModel.update(0.020);
        hoodModel.setInputVoltage(volt);
        hoodModel.update(0.020);

        motorRSim.setRotorVelocity(drumModel.getAngularVelocity().times(DRUM_GEARING));
        motorLSim.setRotorVelocity(drumModel.getAngularVelocity().times(DRUM_GEARING));
        motorHoodSim.setRotorVelocity(drumModel.getAngularVelocity().times(HOOD_GEARING));
        motorHoodSim.setRawRotorPosition(hoodModel.getAngularPositionRotations() * HOOD_GEARING);

        super.updateInputs(inputs);
    }

}
