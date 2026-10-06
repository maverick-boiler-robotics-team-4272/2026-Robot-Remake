package frc.robot.feeder;

import static frc.robot.constants.SubsystemConstants.FeederConstants.FEED_GEARING;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim extends FeederIOReal {
    private final DCMotorSim feederModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(2), 0.01, FEED_GEARING), 
            DCMotor.getKrakenX60Foc(2)
            );

    private TalonFXSimState motorLSim;
    private TalonFXSimState motorRSim;


    public FeederIOSim() {
        motorLSim = motorL.getSimState();
        motorRSim = motorR.getSimState();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        motorLSim = motorL.getSimState();
        motorRSim = motorR.getSimState();
        
        motorLSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorRSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double spinVolt = motorLSim.getMotorVoltage();

        feederModel.setInputVoltage(spinVolt);
        feederModel.update(0.02);

        motorLSim.setRotorVelocity(feederModel.getAngularVelocity().times(FEED_GEARING));

        super.updateInputs(inputs);

    }

}
