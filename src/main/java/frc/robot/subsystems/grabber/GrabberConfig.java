package frc.robot.subsystems.grabber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class GrabberConfig {
    // Replace the 0s with your own values
    static final int CLAW_ID = 12;
    static final double OUTSPIT = 0;
    static final double INTAKE = 0;
    static final double ALGAE_INTAKE = 0;
    static final double ALL_IN = 0;
    static final double HOLD = 0;
    static final double ALGAE_HOLD = 0;
    static final double ALGAE_SHOOT = 0;
    static final TalonFXConfiguration coralMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKV(0).withKA(0))
            .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(0));

    static final int GRABBER_CANANDCOLOR = 0;
}
