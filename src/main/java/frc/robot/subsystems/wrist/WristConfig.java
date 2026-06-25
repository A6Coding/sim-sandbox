package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

class WristConfig {
    static final double WRIST_TOLERANCE = 0.1;
    static final int WRIST_KRAKEN_ID = 19;
    static final int WRIST_CANCODER_ID = 42;
    static final double MAGNET_OFFSET = -0.386963;

    // Tune to find these!
    static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKG(0);

    static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
            .withRotorToSensorRatio(5)
            .withSensorToMechanismRatio(2.75)
            .withFeedbackRemoteSensorID(WRIST_CANCODER_ID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    static final TalonFXConfiguration wristMotorConfigs = new TalonFXConfiguration()
            .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
            .withSlot0(SLOT_0_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS);

    static final CANcoderConfiguration wristEncoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(MAGNET_OFFSET)
                    .withAbsoluteSensorDiscontinuityPoint(0.625));
}
