package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(25);

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VoltageOut voltageOut = new VoltageOut(0);

    public Command runIntake() {
        return runEnd(() -> intakeMotor.setControl(dutyCycleOut.withOutput(0.25)), () -> intakeMotor.stopMotor());
    }

    public Command runIntakeVolts(double volts) {
        return runEnd(() -> intakeMotor.setControl(voltageOut.withOutput(volts)), () -> intakeMotor.stopMotor());
    }
}
