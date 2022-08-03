// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  TalonFX leftMaster = new TalonFX(4);
  TalonFX rightMaster = new TalonFX(2);


  TalonFX leftSlave = new TalonFX(3);
  TalonFX rightSlave = new TalonFX(1);

  /**GYRO SCOPE GOES HERE */
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0/** KS, KV, KA */);

  //PID
  PIDController leftPIDController = new PIDController(/**characterization P value */0, 0, 0);
  PIDController rightPIDController = new PIDController(/**characterization P value */0, 0, 0);
  
/**position */
  Pose2d pose;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    leftSlave.setInverted(false);
    rightSlave.setInverted(true);
  }

  public Rotation2d getHeading() {
  return Rotation2d.fromDegrees(-gyro.getAngle());
}

public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity() / 6.61 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
      rightMaster.getSelectedSensorVelocity() / 6.61 * 2 * Math.PI * Units.inchesToMeters(3.0) /60
    );
  }


    public SimpleMotorFeedforward getFeedForward() {
      return feedforward; 
    }

    public DifferentialDriveKinematics getKinematics(){
      return kinematics;
    }

    public Pose2d getPose() {
      return pose;
    }

    public void setOutput(double leftVolts, double rightVolts){
      leftMaster.set(TalonFXControlMode.PercentOutput, leftVolts / 12);
      rightMaster.set(TalonFXControlMode.PercentOutput, rightVolts / 12);
        }

    public PIDController getLeftPIDController() {
      return leftPIDController;
    }

    public PIDController getRightPIDController() {
      return rightPIDController;
  }

  @Override
  public void periodic() {
  pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);


  }
}
