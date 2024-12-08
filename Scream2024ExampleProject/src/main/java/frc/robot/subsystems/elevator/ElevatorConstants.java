package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.TitaniumUtils.Gains;
import frc.robot.utils.TitaniumUtils.TrapezoidalConstraints;

public class ElevatorConstants {
    public static final Distance MAX_HEIGHT = Inches.of(21.513);
    public static final Distance MIN_HEIGHT = Inches.of(0.0);
    public static final Distance PULLEY_CIRCUFERENCE = Inches.of(6.946136755);

    public static final double GEAR_RATIO = 14.0167;

    public static final int LEADER_MOTOR_ID = 40;
    public static final int FOLLOWER_MOTOR_ID = 41;

    public static final String LEADER_MOTOR_CANBUS = "Canivore";
    public static final String FOLLOWER_MOTOR_CANBUS = "Canivore";

    public static final boolean LEADER_MOTOR_INVERTED = false;

    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 60;
    public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;

    public static final Gains ELEVATOR_GAINS = new Gains(5, 0, 0, 0, 0, 0);
    public static final TrapezoidalConstraints ELEVATOR_CONSTRAINTS = new TrapezoidalConstraints(5, 10);
}
