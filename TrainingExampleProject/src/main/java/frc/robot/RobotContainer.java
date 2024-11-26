// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  private Shooter shooter = new Shooter();

  private CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    driverController.leftBumper().and(driverController.leftTrigger()).whileTrue(intake.runIntake());

    driverController.rightBumper().whileTrue(shooter.runShooter(6.0));

    driverController.a().whileTrue(shootNote());
  }

  public Command shootNote() {
    return Commands.sequence(intake.runIntake().until(intake.hasNote()), arm.moveArm(), shooter.runShooter(6.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("Autonomous Mode");
  }
}
