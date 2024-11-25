// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.TunerConstants;

public class RobotContainer {
    private final double MAX_SPEED_METERS_PER_SEC = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 8.0; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED_METERS_PER_SEC * 0.1).withRotationalDeadband(MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final CommandXboxController driver = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MAX_SPEED_METERS_PER_SEC)
                    .withVelocityY(-driver.getLeftX() * MAX_SPEED_METERS_PER_SEC)
                    .withRotationalRate(-driver.getRightX() * MAX_ANGULAR_SPEED_RAD_PER_SEC)
            )
        );

        driver.start().whileTrue(drivetrain.applyRequest(() -> brake));

        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.leftTrigger().whileTrue(shooter.setState(Shooter.State.SHOOTING));

        drivetrain.registerTelemetry(RobotState::telemeterizeDrivetrain);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
