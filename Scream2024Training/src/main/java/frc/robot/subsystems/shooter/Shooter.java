package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public final TalonFX shooterMotor = new TalonFX(1);

    public final VoltageOut voltageOut = new VoltageOut(0.0);

    public final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    public Shooter() {
        
    }

    public Command runShooterCommand() {
        return runEnd(
            () -> shooterMotor.setControl(voltageOut.withOutput(2)),
            () -> shooterMotor.setControl(voltageOut.withOutput(0))
            );
    }

    public void startShooter(double voltage) {
        shooterMotor.setControl(voltageOut.withOutput(voltage));
    }

    public void stopShooter() {
        shooterMotor.setControl(voltageOut.withOutput(0));
    }

}
