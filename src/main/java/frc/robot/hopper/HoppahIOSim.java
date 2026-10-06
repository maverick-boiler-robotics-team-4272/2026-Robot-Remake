package frc.robot.hopper;

import static frc.robot.constants.SubsystemConstants.HoppahConstants.BELT_GEARING;
import static frc.robot.constants.SubsystemConstants.HoppahConstants.TOP_BELT_GEARING;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoppahIOSim extends HoppahIOReal {
    private final DCMotorSim motorBModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(2), 0.015, BELT_GEARING), 
        DCMotor.getKrakenX60Foc(2)
    );
    private final DCMotorSim motorTModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 0.015, TOP_BELT_GEARING), 
        DCMotor.getKrakenX60Foc(1)
    );

    private TalonFXSimState motorBLSim;
    private TalonFXSimState motorBRSim;
    private TalonFXSimState motorLSim;

    public HoppahIOSim() {
        motorBLSim = motorBL.getSimState();
        motorBRSim = motorBR.getSimState();
        motorLSim = motorL.getSimState();
    }

    @Override
    public void updateInputs(HoppahIOInputs inputs) {
        motorBLSim = motorBL.getSimState();
        motorBRSim = motorBR.getSimState();
        motorLSim = motorL.getSimState();

        motorBLSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorBRSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorLSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        double topVolt = motorLSim.getMotorVoltage();
        double bottomVolt = motorBLSim.getMotorVoltage();

        motorBModel.setInputVoltage(bottomVolt);
        motorTModel.setInputVoltage(topVolt);
        motorBModel.update(0.020);
        motorTModel.update(0.020);

        motorBLSim.setRotorVelocity(motorBModel.getAngularVelocity().times(BELT_GEARING));
        motorLSim.setRotorVelocity(motorTModel.getAngularVelocity().times(TOP_BELT_GEARING));
        motorBRSim.setRotorVelocity(motorBModel.getAngularVelocity().times(BELT_GEARING));

        super.updateInputs(inputs);
    }
}