package frc.robot.utils;

public class TitaniumUtils {
    public record Gains(double kP, double kI, double kD, double kS, double kV, double kG) {
        public Gains {
            if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kG < 0) {
                throw new IllegalArgumentException("Gains must be positive");
            }
        }
    }

    public record TrapezoidalConstraints(double maxVelocity, double maxAcceleration) {
        public TrapezoidalConstraints {
            if (maxVelocity < 0 || maxAcceleration < 0) {
                throw new IllegalArgumentException("Constraints must be positive");
            }
        }
    }
}
