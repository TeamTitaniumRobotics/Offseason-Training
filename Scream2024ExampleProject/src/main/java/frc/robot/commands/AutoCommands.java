package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoCommands {
    public static Command resetOdometry(Pose2d pose, Swerve swerve) {
        return Commands.runOnce(() -> swerve.resetPose(pose), swerve);
    }

    public static Command autoAimAndShoot() {
        return Commands.runOnce(() -> {
            // Aim and shoot
        });
    }

    public static Command intake() {
        return Commands.waitSeconds(50);
    }

    public static Command aimFor(Pose2d pose) {
        return Commands.runOnce(() -> {
            // Aim for pose
        });
    }
}
