package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(25);

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VoltageOut voltageOut = new VoltageOut(0);

    private final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.03, 1),
            DCMotor.getFalcon500(1));

    public Command runIntake() {
        return runEnd(() -> intakeMotor.setControl(dutyCycleOut.withOutput(0.25)), () -> intakeMotor.stopMotor());
    }

    public Command runIntakeVolts(double volts) {
        return runEnd(() -> intakeMotor.setControl(voltageOut.withOutput(volts)), () -> intakeMotor.stopMotor());
    }

    public Trigger hasNote() {
        return new Trigger(() -> intakeMotor.getSupplyCurrent().getValueAsDouble() > 30).debounce(0.15);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = intakeMotor.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = talonFXSim.getMotorVoltage();

        sim.setInputVoltage(motorVoltage);
        sim.update(0.020);

        talonFXSim.setRawRotorPosition(sim.getAngularPosition());
        talonFXSim.setRotorVelocity(sim.getAngularVelocity());

        SmartDashboard.putNumber("Intake/Position", intakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Velocity", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    }
}
