// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/** Add your docs here. */
public class RobotContainer {

    private Drivetrain drive = new Drivetrain();

    public <drive, drive> Command getAutonomousCommand(){
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2),
            Units.feetToMeters(2));
    config.setKinematics(drive.getKinematics());


    Trajectory trajectory = new TrajectoryGenerator.generateTrajectory(
        Arrays.asList( new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
        config

    );
    

    RamseteCommand command = new RamseteCommand(
        trajectory,
        drive :: getPose,
        new RamseteController(2.0, 0.7),
        drive.getFeedforward(),
        drive.getKinematics(),
        drive ::getSpeeds,
        drive.getLeftPIDController(),
        drive.getRighController(),
        drive :: setOutput, 
        drive
    );

 return command;
}
}