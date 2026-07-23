package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.constants.Constants.RIO_BUS;

@Logged
public class GrabberSubsystem {
    private final TalonFX claw = new TalonFX(GrabberConfig.CLAW_ID, RIO_BUS);
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
    private final Canandcolor clawSwitch = new Canandcolor(GrabberConfig.GRABBER_CANANDCOLOR);

    // Your job: initialize and create triggers for the grabber's Canandcolor!

    public GrabberSubsystem() {
        claw.getConfigurator().apply(GrabberConfig.coralMotorConfig);
    }

    // Define the trigger conditions!

    public boolean hasCoral() {
        return false;
    }

    public boolean doesNotHaveCoral() {
        return false;
    }

    public boolean hasAlgae() {
        return false;
    }

    public boolean doesNotHaveAlgae() {
        return false;
    }

    public Command intake() {
        return runEnd(()-> claw.setPosition(1), () -> claw.setPosition(0));
    }

    public Command autoIn() {
        return runOnce(() -> claw.setPosition(1));
    }

    
}
