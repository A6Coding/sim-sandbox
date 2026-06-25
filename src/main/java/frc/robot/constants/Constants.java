// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int PRIMARY_XBOX_CONTROLLER_PORT = 0;
    public static final int PRIMARY_JOYSTICK_PORT = 0;
    public static final String CANIVORE_BUS = "canivoreBus";
    public static final String RIO_BUS = "rio";
    public static final int SECONDARY_XBOX_CONTROLLER_PORT = 1;
    public static final int GIGA_PORT = 3;
    public static final Transform3d camTrans1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.5), Units.inchesToMeters(-8), Units.inchesToMeters(11)),
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-90)));

    public static final Transform3d camTrans2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.5), Units.inchesToMeters(10), Units.inchesToMeters(11)),
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(90)));
}
