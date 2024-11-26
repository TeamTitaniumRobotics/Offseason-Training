package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ExampleShooter {
    private final TalonFX shooterMotor = new TalonFX(10, "*");

    private final FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.04, 1), DCMotor.getKrakenX60(1));

    private final VoltageOut voltageOut = new VoltageOut(0);
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0);

    public ExampleShooter() {

    }

    public Command runShooter() {
        return Commands.runEnd(() -> shooterMotor.set(0.5), () -> shooterMotor.stopMotor());
    }

    
}
