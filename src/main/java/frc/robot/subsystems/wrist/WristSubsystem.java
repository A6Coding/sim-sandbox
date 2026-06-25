package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class WristSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX wristMotor = new TalonFX(WristConfig.WRIST_KRAKEN_ID);
    private final CANcoder wristEncoder = new CANcoder(WristConfig.WRIST_CANCODER_ID);
    private final PositionVoltage motionRequest = new PositionVoltage(0).withSlot(0);

    public WristSubsystem() {
        wristMotor.getConfigurator().apply(WristConfig.wristMotorConfigs);
        wristEncoder.getConfigurator().apply(WristConfig.wristEncoderConfigs);

        PhysicsSim.getInstance().addTalonFX(wristMotor, wristEncoder);
    }

    @Override
    public Angle getCurrentPosition() {
        return wristMotor.getPosition().getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(wristMotor.getClosedLoopReference().getValue());
    }
}
