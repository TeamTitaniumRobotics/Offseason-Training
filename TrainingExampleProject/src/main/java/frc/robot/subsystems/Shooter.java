package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(30);
    private final VoltageOut voltageOut = new VoltageOut(0);

    private final FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.04, 1), DCMotor.getKrakenX60(1));

    public Command runShooter(double volts) {
        return runEnd(() -> shooterMotor.setControl(voltageOut.withOutput(volts)),
                () -> shooterMotor.setControl(voltageOut.withOutput(0)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/SimVelocity", sim.getAngularVelocity().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/SimAcceleration",
                sim.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond));
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState shooterMotorSim = shooterMotor.getSimState();

        shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(shooterMotorSim.getMotorVoltage());
        sim.update(0.02);

        shooterMotorSim.setRotorVelocity(sim.getAngularVelocity());
        shooterMotorSim.setRotorAcceleration(sim.getAngularAcceleration());
    }
}
