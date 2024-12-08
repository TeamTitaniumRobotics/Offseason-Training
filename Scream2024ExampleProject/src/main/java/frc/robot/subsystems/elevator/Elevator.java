package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
    private final TalonFX leaderMotor, followerMotor;

    private final TalonFXConfiguration config;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final VoltageOut voltageOut = new VoltageOut(0);

    private State state = State.STOW;

    private final ElevatorSim sim = new ElevatorSim(DCMotor.getKrakenX60(2), GEAR_RATIO, Units.lbsToKilograms(31),
            Units.inchesToMeters(2.211 / 2.0), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters), true, 0.0);

    public enum State {
        STOW(() -> Inches.of(0.0)),
        INTAKE(() -> Inches.of(3.41)),
        SUB(() -> Inches.of(3.13)),
        AMP(() -> Inches.of(19.71)),
        TRAP(() -> MAX_HEIGHT),
        EJECT(() -> Inches.of(5.43)),
        TRACKING(() -> Inches.of(0)),
        HOMING(() -> Inches.of(0));

        private final DoubleSupplier targetRotations;

        private State(Supplier<Distance> targetHeight) {
            this.targetRotations = () -> (targetHeight.get().in(Meters) / PULLEY_CIRCUFERENCE.in(Meters)) * GEAR_RATIO;
        }

        @Override
        public String toString() {
            return name().toLowerCase() + " (" + targetRotations.getAsDouble() + " rotations)";
        }
    }

    public Elevator() {
        leaderMotor = new TalonFX(LEADER_MOTOR_ID, LEADER_MOTOR_CANBUS);
        followerMotor = new TalonFX(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_CANBUS);
        followerMotor.setControl(new Follower(LEADER_MOTOR_ID, true));

        config = new TalonFXConfiguration();

        config.Slot0.kP = ELEVATOR_GAINS.kP();
        config.Slot0.kI = ELEVATOR_GAINS.kI();
        config.Slot0.kD = ELEVATOR_GAINS.kD();
        config.Slot0.kS = ELEVATOR_GAINS.kS();
        config.Slot0.kV = ELEVATOR_GAINS.kV();
        config.Slot0.kG = ELEVATOR_GAINS.kG();

        config.MotionMagic.MotionMagicAcceleration = ELEVATOR_CONSTRAINTS.maxAcceleration();
        config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR_CONSTRAINTS.maxVelocity();

        config.CurrentLimits.StatorCurrentLimit = ELEVATOR_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = ELEVATOR_SUPPLY_CURRENT_LIMIT;

        config.MotorOutput.Inverted = LEADER_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (state == State.HOMING) {
            leaderMotor.setControl(voltageOut.withOutput(-0.5));
            if (currentTrigger().getAsBoolean()) {
                leaderMotor.setPosition(0);
                Commands.print("Homing complete");
                state = State.STOW;
            }
        } else {
            leaderMotor.setControl(motionMagicVoltage.withPosition(state.targetRotations.getAsDouble()));
        }

        displayInfo(true);
    }

    public Command setState(State state) {
        return runEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    public Trigger currentTrigger() {
        return new Trigger(() -> leaderMotor.getSupplyCurrent().getValue().gte(Amps.of(30)));
    }

    public Trigger atGoal() {
        return new Trigger(() -> leaderMotor.getPosition().getValue()
                .isNear(Rotations.of(state.targetRotations.getAsDouble()), Rotations.of(0.1)));
    }

    private void displayInfo(boolean debug) {
        if (!debug) {
            return;
        }

        DogLog.log("Elevator/State", state.toString());
        DogLog.log("Elevator/AtGoal", atGoal().getAsBoolean());

        DogLog.log("Elevator/Position", leaderMotor.getRotorPosition().getValueAsDouble());
        DogLog.log("Elevator/Velocity", leaderMotor.getVelocity().getValueAsDouble());
        DogLog.log("Elevator/StatorCurrent", leaderMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("Elevator/SupplyCurrent", leaderMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Elevator/MotorVoltage", leaderMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Elevator/SupplyVoltage", leaderMotor.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var motorSim = leaderMotor.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(motorSim.getMotorVoltage());
        sim.update(0.02);

        motorSim.setRawRotorPosition((sim.getPositionMeters() / PULLEY_CIRCUFERENCE.in(Meters)) * GEAR_RATIO);
        motorSim.setRotorVelocity((sim.getVelocityMetersPerSecond() / PULLEY_CIRCUFERENCE.in(Meters)) * GEAR_RATIO);
    }
}
