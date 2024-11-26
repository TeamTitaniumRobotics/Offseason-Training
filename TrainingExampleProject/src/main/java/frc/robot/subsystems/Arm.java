package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(20);

    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0.0);

    public Arm() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.kP = 6;

        var foo = Units.Degree.of(1);

        armMotor.getConfigurator().apply(config);
    }

    public Command moveArm() {
        return runEnd(() -> armMotor.setControl(positionDutyCycle.withPosition(0.25)),
                () -> armMotor.setControl(positionDutyCycle.withPosition(0)));
    }
}
