package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Swerve;

import static frc.robot.commands.AutoCommands.*;

public class AutoRoutines {
  private final Swerve swerve;

  public AutoRoutines(Swerve swerve) {
    this.swerve = swerve;
  }

  public Command fivePieceAutoTriggerSeg(AutoFactory factory) {
    final AutoLoop loop = factory.newLoop("fivePieceAuto");

    final AutoTrajectory ampToC1 = factory.trajectory("AMP-C1", loop);
    final AutoTrajectory c1ToM1 = factory.trajectory("C1-M1", loop);
    final AutoTrajectory m1ToS1 = factory.trajectory("M1-S1", loop);
    final AutoTrajectory m1ToM2 = factory.trajectory("M1-M2", loop);
    final AutoTrajectory m2ToS1 = factory.trajectory("M2-S1", loop);
    final AutoTrajectory s1ToC2 = factory.trajectory("S1-C2", loop);
    final AutoTrajectory c2ToC3 = factory.trajectory("C2-C3", loop);

    loop.enabled().onTrue(resetOdometry(ampToC1.getInitialPose().orElseGet(
        () -> {
          loop.kill();
          return new Pose2d();
        }), swerve)
        // .andThen(autoAimAndShoot(),
        // Commands.race(
        // intake(),
        // ampToC1.cmd(),
        // aimFor(ampToC1.getFinalPose().orElseGet(() -> new Pose2d()))))
        // .withName("fivePieceAuto entry point"));
        .andThen(ampToC1.cmd()));

    return loop.cmd();
  }
}
