package frc.robot.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

public class HoppahIOReal implements HoppahIO {
   private static final int motorLeftID = 0;
   private static final int motorRightID = 0;
   private static final int motorUpID = 0;  //🙏🙏🙏

   protected final TalonFX motorLeft;
   protected final TalonFX motorRight;
   protected final TalonFX motorUp;

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

    protected final StatusSignal<Current> motorUpStatorCurrent;
    protected final StatusSignal<Current> motorUpSupplyCurrent;
    protected final StatusSignal<AngularVelocity> motorUpVelocityRPS;
    protected final StatusSignal<Voltage> motorUpSupplyVoltage;
    protected final StatusSignal<Voltage> motorUpOutputVolts;

    private final Debouncer motorLeftIsConnected = new Debouncer(0.5);
    private final Debouncer motorRightIsConnected = new Debouncer(0.5);
    private final Debouncer motorUpIsConnected = new Debouncer(0.5);

    public HoppahIOReal() {
        motorLeft = new TalonFX(motorLeftID);
        motorRight = new TalonFX(motorRightID);
        motorUp = new TalonFX(motorUpID);
    }
}
