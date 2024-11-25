package frc.robot.subsystems.shooter;

import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class ShooterConstants {
    public static final int MASTER_MOTOR_ID = 30;
    public static final String MASTER_MOTOR_CANBUS = "rio";
    public static final boolean MASTER_MOTOR_INVERTED = false;

    public static final int SLAVE_MOTOR_ID = 31;
    public static final String SLAVE_MOTOR_CANBUS = "rio";
    public static final boolean SLAVE_MOTOR_INVERTED = false;

    public static final Gains SHOOTER_GAINS = new Gains(0, 0, 0, 0, 0, 0);
    public static final TrapezoidalConstraints SHOOTER_CONSTRAINTS = new TrapezoidalConstraints(0, 0);

    public static final double GEAR_RATIO = 1;

    public static final double FLYWHEEL_CIRCUMFERENCE = 4.0 * Math.PI;

    public static final double SIM_FLYWHEEL_MOMENT_OF_INERTIA = 0.00599676919909 + 0.0001;
}
