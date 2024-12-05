// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.util.AllianceFlipUtil;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoRoutines;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.TunerConstants;

public class RobotContainer {
    private final double MAX_SPEED_METERS_PER_SEC = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 8.0;

    // Swerve drive request objects
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED_METERS_PER_SEC * 0.1).withRotationalDeadband(MAX_ANGULAR_SPEED_RAD_PER_SEC * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);

    // Subsystems
    private final Swerve drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();

    // Auto-related objects
    private final AutoRoutines autoRoutines = new AutoRoutines(drivetrain);
    private final AutoFactory autoFactory;
    public final AutoChooser autoChooser;

    public RobotContainer() {
        autoFactory = Choreo.createAutoFactory(
                drivetrain,
                () -> drivetrain.getState().Pose,
                drivetrain::followPath,
                AllianceFlipUtil::shouldFlip,
                new AutoFactory.AutoBindings());

        autoChooser = new AutoChooser(autoFactory, "");

        autoChooser.addAutoRoutine("Five Piece Auto", autoRoutines::fivePieceAutoTriggerSeg);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MAX_SPEED_METERS_PER_SEC)
                        .withVelocityY(-driver.getLeftX() * MAX_SPEED_METERS_PER_SEC)
                        .withRotationalRate(-driver.getRightX() * MAX_ANGULAR_SPEED_RAD_PER_SEC)));

        driver.start().whileTrue(drivetrain.applyRequest(() -> brake));

        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.a().toggleOnTrue(shooter.setState(Shooter.State.SHOOTING));

        driver.leftTrigger().onTrue(intake.setState(Intake.State.INTAKING));

        driver.leftBumper().onTrue(intake.setState(Intake.State.IDLE));

        drivetrain.registerTelemetry(RobotState::telemeterizeDrivetrain);
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelectedAutoRoutine();
        return pivot.setState(Pivot.State.AMP);
    }
}
