package frc.robot.subsystems.wrist;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum WristPosition {
    EXAMPLE_ENUM(0);

    private final Angle angle;

    WristPosition(double height) {
        this.angle = Units.Rotations.of(height);
    }

    public Angle get() {
        return angle;
    }
}
