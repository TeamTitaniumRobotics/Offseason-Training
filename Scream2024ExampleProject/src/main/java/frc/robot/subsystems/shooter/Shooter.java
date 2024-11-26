package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMasterMotor, shooterSlaveMotor;

    private final TalonFXConfiguration config;

    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
    private final NeutralOut neutralOut = new NeutralOut();

    private State state = State.IDLE;

    private final FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.00599676919909 + 0.0001, SHOOTER_GEAR_RATIO),
            DCMotor.getFalcon500(1));

    public enum State {
        IDLE(() -> RotationsPerSecond.of(0)),
        SHOOTING(() -> RotationsPerSecond.of(5800));

        private final Supplier<AngularVelocity> velocitySupplier;

        private State(Supplier<AngularVelocity> velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
        }

        @Override
        public String toString() {
            return name().toLowerCase() + " (" + velocitySupplier.get().in(RotationsPerSecond) + ")";
        }
    }

    public Shooter() {
        shooterMasterMotor = new TalonFX(SHOOTER_MASTER_MOTOR_ID, SHOOTER_MASTER_MOTOR_CANBUS);
        shooterSlaveMotor = new TalonFX(SHOOTER_SLAVE_MOTOR_ID, SHOOTER_SLAVE_MOTOR_CANBUS);
        shooterSlaveMotor.setControl(new Follower(SHOOTER_MASTER_MOTOR_ID, SHOOTER_SLAVE_MOTOR_INVERTED));

        config = new TalonFXConfiguration();

        config.Slot0.kP = SHOOTER_GAINS.kP();
        config.Slot0.kI = SHOOTER_GAINS.kI();
        config.Slot0.kD = SHOOTER_GAINS.kD();
        config.Slot0.kS = SHOOTER_GAINS.kS();
        config.Slot0.kV = SHOOTER_GAINS.kV();
        config.Slot0.kG = SHOOTER_GAINS.kG();

        config.MotionMagic.MotionMagicAcceleration = SHOOTER_CONSTRAINTS.maxAcceleration();
        config.MotionMagic.MotionMagicCruiseVelocity = SHOOTER_CONSTRAINTS.maxVelocity();

        config.MotorOutput.Inverted = SHOOTER_MASTER_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        shooterMasterMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (state == State.IDLE) {
            shooterMasterMotor.setControl(neutralOut);
        } else if (state == State.SHOOTING) {
            shooterMasterMotor.setControl(motionMagicVelocityVoltage.withVelocity(state.velocitySupplier.get()));
        }

        displayInfo(true);
    }

    public Command setState(State state) {
        return runEnd(() -> this.state = state, () -> this.state = State.IDLE);
    }

    public Trigger atGoal() {
        return new Trigger(() -> shooterMasterMotor.getVelocity().getValue().isNear(state.velocitySupplier.get(),
                RotationsPerSecond.of(600)));
    }

    private void displayInfo(boolean debug) {
        if (!debug) {
            return;
        }

        DogLog.log("Shooter/State", state.toString());
        DogLog.log("Shooter/AtGoal", atGoal().getAsBoolean());

        DogLog.log("Shooter/Position", shooterMasterMotor.getRotorPosition().getValueAsDouble());
        DogLog.log("Shooter/Velocity", shooterMasterMotor.getVelocity().getValueAsDouble());
        DogLog.log("Shooter/StatorCurrent", shooterMasterMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("Shooter/SupplyCurrent", shooterMasterMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Shooter/MotorVoltage", shooterMasterMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Shooter/SupplyVoltage", shooterMasterMotor.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var shooterMotorSim = shooterMasterMotor.getSimState();
        shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInput(shooterMotorSim.getMotorVoltage());
        sim.update(0.02);

        shooterMotorSim.setRotorVelocity(sim.getAngularVelocity());
        shooterMotorSim.setRotorAcceleration(sim.getAngularAcceleration());
    }
}
