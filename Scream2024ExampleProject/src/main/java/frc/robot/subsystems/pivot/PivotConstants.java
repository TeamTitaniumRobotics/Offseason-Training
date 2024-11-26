package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class PivotConstants {
    public static final int PIVOT_MOTOR_ID = 30;
    public static final String PIVOT_MOTOR_CANBUS = "rio";
    public static final boolean PIVOT_MOTOR_INVERTED = false;

    public static final double PIVOT_STATOR_CURRENT_LIMIT = 40;
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 50;

    public static final Gains PIVOT_GAINS = new Gains(0, 0, 0, 0, 0, 0);
    public static final TrapezoidalConstraints PIVOT_CONSTRAINTS = new TrapezoidalConstraints(8, 16);

    public static final double PIVOT_GEAR_RATIO = 125.0 * (72.0 / 22.0);

    public static final double SIM_PIVOT_MOMENT_OF_INERTIA = 0.6193;

    public static final Distance AXLE_DISTANCE_FROM_ELEVATOR_TOP = Inches.of(9.998565);
    public static final Distance SHOOTER_DISTANCE_FROM_AXLE = Inches.of(2.7361);
}
