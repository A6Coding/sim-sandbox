package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.State;

public enum ElevatorPosition implements State<Angle> {
    EXAMPLE_ENUM(0);

    private final Angle height;

    ElevatorPosition(double height) {
        this.height = Units.Rotations.of(height);
    }

    public Angle get() {
        return height;
    }
}
