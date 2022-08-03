// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class RobotContainer {

    private Drivetrain drive = new Drivetrain();

    public Command getAutonomousCommand(){
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2),
            Units.feetToMeters(2));
    config.setKinematics(drive.getKinematics());


    Trajectory trajectory;
    trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), null, new Pose2d(), config);

    RamseteCommand command = new RamseteCommand(
        trajectory,
        drive :: getPose,
        new RamseteController(2.0, 0.7),
        drive.getFeedForward(),
        drive.getKinematics(),
        drive ::getSpeeds,
        drive.getLeftPIDController(),
        drive.getRightPIDController(),
        drive :: setOutput, 
        drive
    );

 return command;
    }
    
}
