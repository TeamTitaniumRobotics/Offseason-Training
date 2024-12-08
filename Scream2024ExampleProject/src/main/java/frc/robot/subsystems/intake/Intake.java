package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    private final TalonFXConfiguration config;

    private final VoltageOut voltageOut = new VoltageOut(Volts.of(0.0));

    private State state = State.IDLE;

    private DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), SIM_INTAKE_MOMENT_OF_INERTIA,
                    INTAKE_GEAR_RATIO),
            DCMotor.getFalcon500(1));

    public enum State {
        IDLE(Volts.of(0.0)),
        INTAKING(Volts.of(9.5)),
        EJECTING(Volts.of(-6.0));

        private Voltage volts;

        private State(Voltage volts) {
            this.volts = volts;
        }

        @Override
        public String toString() {
            return name().toLowerCase() + " (" + volts.in(Volts) + ")";
        }
    }

    public Intake() {
        intakeMotor = new TalonFX(INTAKE_MOTOR_ID, INTAKE_MOTOR_CANBUS);

        config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = INTAKE_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = INTAKE_SUPPLY_CURRENT_LIMIT;

        config.MotorOutput.Inverted = INTAKE_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = INTAKE_GEAR_RATIO;

        intakeMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        intakeMotor.setControl(voltageOut.withOutput(state.volts));

        displayInfo(true);
    }

    public Command setState(State state) {
        return runEnd(() -> this.state = state, () -> this.state = State.IDLE);
    }

    public Trigger hasNote() {
        return new Trigger(() -> intakeMotor.getSupplyCurrent().getValue().isNear(Amps.of(25), Amps.of(5)))
                .debounce(0.25);
    }

    private void displayInfo(boolean debug) {
        if (!debug) {
            return;
        }

        DogLog.log("Intake/State", state.toString());

        DogLog.log("Intake/HasNote", hasNote().getAsBoolean());

        DogLog.log("Intake/Position", intakeMotor.getRotorPosition().getValueAsDouble());
        DogLog.log("Intake/Velocity", intakeMotor.getVelocity().getValueAsDouble());
        DogLog.log("Intake/StatorCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("Intake/SupplyCurrent", intakeMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Intake/MotorVoltage", intakeMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/SupplyVoltage", intakeMotor.getSupplyVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        var intakeMotorSim = intakeMotor.getSimState();
        intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInput(intakeMotorSim.getMotorVoltage());
        sim.update(0.02);

        intakeMotorSim.setRawRotorPosition(sim.getAngularPosition());
        intakeMotorSim.setRotorVelocity(sim.getAngularVelocity());
    }
}
