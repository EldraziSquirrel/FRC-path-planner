// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  TalonSRX leftMaster = new TalonSRX(1);
  TalonSRX rightMaster = new TalonSRX(2);


  TalonSRX leftSlave = new TalonSRX(3);
  TalonSRX rightSlave = new TalonSRX(4);

  /**GYRO SCOPE GOES HERE */
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.incesToMeters(22.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics, getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(/** KS, KV, KA */);

  //PID
  PIDController leftPIDController = new PIDController(/**characterization P value */, 0, 0);
  PIDController rightPIDController = new PIDController(/**characterization P value */, 0, 0);
  
/**position */
  Pose2d pose;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInvtered(false);
    rightMaster.setInvtered(true);
  }

  public Rotation2d getHeading() {
  return Rotaion2d.fromDegrees(-gyro.getAngle());
}

public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getEncoder().getVelocity() / 6.61 * 2 * Math.PI * Units,inchesToMeters(3.0) / 60,
      rightMaster.getEncoder().getVelocity() / 6.61 * 2 * Math.PI * Units,inchesToMeters(3.0) / 60
    );
  }

    public SimpleMotorForward getFeedForward() {
      return feedforward; 
    }

    public DIfferentaiDriveKinematics getKinematics(){
      return kinematics;
    }

    public Pose2d getPose() {
      return pose;
    }

    public void setOutput(double leftVolts, double rightVolts){
      leftMaster.set(leftVolts / 12);
      rightMaster.set(rightVolts / 12);
        }

    public PIDController getLeftPIDController() {
      return leftPIDController;
    }

    public PIDController getRighController() {
      return rightPIDControllerl;
  }

  @Override
  public void periodic() {
  pose = odometry.update(getHeading(), getSpeeds());


  }
}
