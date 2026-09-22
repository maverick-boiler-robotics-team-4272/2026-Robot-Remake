package frc.robot.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoppahIOReal implements HoppahIO {
   private static final int motorLeftID = 0;
   private static final int motorRightID = 0;
   private static final int motorBLID = 0;  //🙏🙏🙏
   private static final int motorBRID = 0;


   protected final TalonFX motorLeft;
   protected final TalonFX motorRight;
   protected final TalonFX motorBL;
   protected final TalonFX motorBR;

   //control requests

    protected final StatusSignal<Current> motorLeftStatorCurrent;
    protected final StatusSignal<Current> motorLeftSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorLeftVelocityRPS;
    protected final StatusSignal<Voltage> motorLeftSupplyVoltage;
    protected final StatusSignal<Voltage> motorLeftOutputVolts;

    protected final StatusSignal<Current> motorRightStatorCurrent;
    protected final StatusSignal<Current> motorRightSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorRightVelocityRPS;
    protected final StatusSignal<Voltage> motorRightSupplyVoltage;
    protected final StatusSignal<Voltage> motorRightOutputVolts;

    protected final StatusSignal<Current> motorBLStatorCurrent;
    protected final StatusSignal<Current> motorBLSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorBLVelocityRPS;
    protected final StatusSignal<Voltage> motorBLSupplyVoltage;
    protected final StatusSignal<Voltage> motorBLOutputVolts;

    protected final StatusSignal<Current> motorBRStatorCurrent;
    protected final StatusSignal<Current> motorBRSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorBRVelocityRPS;
    protected final StatusSignal<Voltage> motorBRSupplyVoltage;
    protected final StatusSignal<Voltage> motorBROutputVolts;

    private final Debouncer motorLeftIsConnected = new Debouncer(0.5);
    private final Debouncer motorRightIsConnected = new Debouncer(0.5);
    private final Debouncer motorBLIsConnected = new Debouncer(0.5);
    private final Debouncer motorBRIsConnected = new Debouncer(0.5);

    public HoppahIOReal() {
        motorLeft = new TalonFX(motorLeftID);
        motorRight = new TalonFX(motorRightID);
        motorBL = new TalonFX(motorBLID);
        motorBR = new TalonFX(motorBRID);

        final DutyCycleOut m_request = new DutyCycleOut(0);

        motorRight.setControl(new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Aligned));

         Slot0Configs beltSlot0Configs = new Slot0Configs()
            .withKP(BELT_KP)
            .withKD(BELT_KD)
            .withKV(BELT_KV)
            .withKS(BELT_KS);

        Slot0Configs feederSlot0Configs = new Slot0Configs()
            .withKP(FEEDER_KP)
            .withKD(FEEDER_KD)
            .withKV(FEEDER_KV)
            .withKS(FEEDER_KS);

        var beltConfig = new TalonFXConfiguration();
            beltConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true));
            beltConfig.withSlot0(beltSlot0Configs);
            beltConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(BELT_GEARING));
            beltConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(BELT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
            beltConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        var feederConfig = new TalonFXConfiguration();
            feederConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true));
                feederConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(FEEDER_GEARING));
            feederConfig.withSlot0(feederSlot0Configs);
            feederConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(FEEDER_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive));
            feederConfig.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast));

        motorLeft.getConfigurator().apply(beltConfig);
        motorRight.getConfigurator().apply(beltConfig);
        motorBL.getConfigurator().apply(feederConfig);
        motorBR.getConfigurator().apply(feederConfig);
        motorHood.setPosition(0);
    }
    
}
   