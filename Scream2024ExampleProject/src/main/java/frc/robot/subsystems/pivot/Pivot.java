package frc.robot.subsystems.pivot;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.pivot.PivotConstants.*;

public class Pivot extends SubsystemBase {
    private final TalonFX pivotMotor;

    private final TalonFXConfiguration config;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

    private State state = State.STOW;

    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), SIM_PIVOT_MOMENT_OF_INERTIA, PIVOT_GEAR_RATIO),
            DCMotor.getFalcon500(1));

    public enum State {
        STOW(() -> Degrees.of(0.0)),
        INTAKE_REGULAR(() -> Degrees.of(0.0)),
        INTAKE_TRAP(() -> Degrees.of(0.0)),
        AMP(() -> Degrees.of(0.0)),
        SHOOT(() -> Degrees.of(0.0)),
        FEED(() -> Degrees.of(0.0)),
        TRAP(() -> Degrees.of(0.0)),;

        private final Supplier<Angle> angleSupplier;

        private State(Supplier<Angle> angleSupplier) {
            this.angleSupplier = angleSupplier;
        }

        @Override
        public String toString() {
            return name().toLowerCase() + " (" + angleSupplier.get().in(Degrees) + ")";
        }
    }

    public Pivot() {
        pivotMotor = new TalonFX(PIVOT_MOTOR_ID, PIVOT_MOTOR_CANBUS);

        config = new TalonFXConfiguration();

        config.Slot0.kP = PIVOT_GAINS.kP();
        config.Slot0.kI = PIVOT_GAINS.kI();
        config.Slot0.kD = PIVOT_GAINS.kD();
        config.Slot0.kS = PIVOT_GAINS.kS();
        config.Slot0.kV = PIVOT_GAINS.kV();
        config.Slot0.kG = PIVOT_GAINS.kG();

        config.MotionMagic.MotionMagicAcceleration = PIVOT_CONSTRAINTS.maxAcceleration();
        config.MotionMagic.MotionMagicCruiseVelocity = PIVOT_CONSTRAINTS.maxVelocity();

        config.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = PIVOT_SUPPLY_CURRENT_LIMIT;

        config.MotorOutput.Inverted = PIVOT_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;

        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        pivotMotor.setControl(motionMagicVoltage.withPosition(state.angleSupplier.get()));

        displayInfo(true);
    }

    public Command setState(State state) {
        return runEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    public Trigger atGoal() {
        return new Trigger(
                () -> pivotMotor.getPosition().getValue().isNear(state.angleSupplier.get(), Degrees.of(3.0)));
    }

    private void displayInfo(boolean debug) {
        if (!debug) {
            return;
        }

        DogLog.log("ShooterPivot/State", state.toString());
        DogLog.log("ShooterPivot/AtGoal", atGoal().getAsBoolean());

        DogLog.log("ShooterPivot/Position", pivotMotor.getRotorPosition().getValueAsDouble());
        DogLog.log("ShooterPivot/Velocity", pivotMotor.getVelocity().getValueAsDouble());
        DogLog.log("ShooterPivot/StatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("ShooterPivot/SupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("ShooterPivot/MotorVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("ShooterPivot/SupplyVoltage", pivotMotor.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var pivotMotorSim = pivotMotor.getSimState();
        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInput(pivotMotorSim.getMotorVoltage());
        sim.update(0.02);

        pivotMotorSim.setRawRotorPosition(sim.getAngularPosition());
        pivotMotorSim.setRotorVelocity(sim.getAngularVelocity());
    }
}
