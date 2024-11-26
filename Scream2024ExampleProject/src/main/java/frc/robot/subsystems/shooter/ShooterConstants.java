package frc.robot.subsystems.shooter;

import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class ShooterConstants {
    public static final int SHOOTER_MASTER_MOTOR_ID = 30;
    public static final String SHOOTER_MASTER_MOTOR_CANBUS = "rio";
    public static final boolean SHOOTER_MASTER_MOTOR_INVERTED = false;

    public static final int SHOOTER_SLAVE_MOTOR_ID = 31;
    public static final String SHOOTER_SLAVE_MOTOR_CANBUS = "rio";
    public static final boolean SHOOTER_SLAVE_MOTOR_INVERTED = false;

    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 40;
    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 50;

    public static final Gains SHOOTER_GAINS = new Gains(0, 0, 0, 10000, 0, 0);
    public static final TrapezoidalConstraints SHOOTER_CONSTRAINTS = new TrapezoidalConstraints(1000000, 1000000);

    public static final double SHOOTER_GEAR_RATIO = 1;

    public static final double SHOOTER_FLYWHEEL_CIRCUMFERENCE = 4.0 * Math.PI;

    public static final double SIM_FLYWHEEL_MOMENT_OF_INERTIA = 0.00599676919909 + 0.0001;
}
