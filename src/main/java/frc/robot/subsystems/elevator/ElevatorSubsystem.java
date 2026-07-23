package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class ElevatorSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID);
    private final TalonFX secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID);
    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);
    private final NeutralOut neutralOut = new NeutralOut();
    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    public ElevatorSubsystem() {
        primaryElevatorMotor.getConfigurator().apply(ElevatorConfig.primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(ElevatorConfig.secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(new Follower(ElevatorConfig.primaryElevatorMotorID, MotorAlignmentValue.Opposed));

        primaryElevatorMotor.setPosition(0);
        secondaryElevatorMotor.setPosition(0);

        primaryElevatorMotor.setControl(neutralOut);

        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
        PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
    }

    @Override
    public Angle getCurrentPosition() {
        return primaryElevatorMotor.getPosition().getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(primaryElevatorMotor.getClosedLoopReference().getValue());
    }

    public boolean getMagSwitch() {
        return !magSwitch.get();
    }

    public Command position() {
        return runEnd(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(2.7)), () -> primaryElevatorMotor.setControl(magicRequest.withPosition(0.1)));
    }

    public Command autoOne() {
        return runOnce(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(2.8)));
    }

    public Command autoTwo() {
        return runOnce(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(1.3)));
    }

    public Command autoThree() {
        return runOnce(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(2.8)));
    }

    public Command autoFour() {
        return runOnce(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(1.3)));
    }
}
