package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;

public class RobotState {
    public static void telemeterizeDrivetrain(SwerveDriveState state) {
        DogLog.log("Drivetrain/RobotPose", state.Pose);
        DogLog.log("Drivetrain/MeasuredStates", state.ModuleStates);
        DogLog.log("Drivetrain/SetpointStates", state.ModuleTargets);
    }
}
