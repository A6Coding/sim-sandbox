package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

class ArmConfig {
    static final int ARM_KRAKEN_ID = 10;
    static final int ARM_CANCODER_ID = 0;

    static final double MAGNET_OFFSET = -0.496826171875;
    static final double GEAR_RATIO = 75.6055;

    static final double ARM_DEPLOY_LOWER_BOUND = 0;

    // Tune to find these!
    private static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    // Tune to find these!
    static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicAcceleration(0)
            .withMotionMagicJerk(0);

    static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(ARM_CANCODER_ID)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(GEAR_RATIO))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(motionMagicConfigs);

    static final CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(MAGNET_OFFSET)
                    .withAbsoluteSensorDiscontinuityPoint(0.625));

    static final double ANGLE_TOLERANCE = 0.05;
}
