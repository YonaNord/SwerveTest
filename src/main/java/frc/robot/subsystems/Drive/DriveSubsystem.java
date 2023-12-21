// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//https://github.com/Team364/BaseFalconSwerve/tree/main
package frc.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Swerve.SwerveModule;


public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final SwerveDriveKinematics m_kinematics;
  private final AHRS m_navX;
  private final SwerveModulePosition[] m_modulePositions;
  private final SwerveDriveOdometry m_odometry;
  private ChassisSpeeds m_swerveSpeeds;
  private Pose2d m_currentPose;




  public DriveSubsystem() {
    System.out.println("we ran drive Subsis");
    this.m_frontLeftModule = new SwerveModule(
      Drive.Motors.kFrontLeftDriveFalconCANID, 
      Drive.Motors.kFrontLeftSteerFalconCANID, 
      Drive.Encoders.kFrontLeftSteerEncoderCANID, 
      Drive.Stats.kFrontLeftModuleOffsetInDegrees
    );
    this.m_frontRightModule = new SwerveModule(
      Drive.Motors.kFrontRightDriveFalconCANID, 
      Drive.Motors.kFrontRightSteerFalconCANID, 
      Drive.Encoders.kFrontRightSteerEncoderCANID,
      Drive.Stats.kFrontRightModuleOffsetInDegrees
      );
    this.m_backLeftModule = new SwerveModule(
      Drive.Motors.kBackLeftDriveFalconCANID, 
      Drive.Motors.kBackLeftSteerFalconCANID, 
      Drive.Encoders.kBackLeftSteerEncoderCANID,
      Drive.Stats.kBackLeftModuleOffsetInDegrees
    );
    this.m_backRightModule = new SwerveModule(
      Drive.Motors.kBackRightDriveFalconCANID, 
      Drive.Motors.kBackRightSteerFalconCANID, 
      Drive.Encoders.kBackRightSteerEncoderCANID,
      Drive.Stats.kBackRightModuleOffsetInDegrees
    );

    m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Drive.Stats.kTrackWidthMeters / 2.0, Drive.Stats.kWheelbaseMeters / 2.0),
            new Translation2d(Drive.Stats.kTrackWidthMeters / 2.0, -Drive.Stats.kWheelbaseMeters / 2.0),
            new Translation2d(-Drive.Stats.kTrackWidthMeters / 2.0, Drive.Stats.kWheelbaseMeters / 2.0),
            new Translation2d(-Drive.Stats.kTrackWidthMeters / 2.0, -Drive.Stats.kWheelbaseMeters / 2.0)
    );

    m_navX = new AHRS();

    m_modulePositions = new SwerveModulePosition[]{
      new SwerveModulePosition(m_frontLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_frontLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_frontRightModule.getDriveMotor().getPosition().asSupplier().get(), m_frontRightModule.getModuleState().angle),
      new SwerveModulePosition(m_backLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_backLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_backRightModule.getDriveMotor().getPosition().asSupplier().get(), m_backRightModule.getModuleState().angle)
    };

    m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(Double.valueOf(m_navX.getFusedHeading())), m_modulePositions);
  
    m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);

    m_currentPose = new Pose2d();
  }
/**
 * Sets the state of all of the swerve modules
 * @param moduleState
 * WPILib's SwerveModuleState library
 */
  public void setModulesStates(SwerveModuleState[] moduleState) {
    m_frontLeftModule.setModuleState(moduleState[0]);
    m_frontRightModule.setModuleState(moduleState[1]);
    m_backLeftModule.setModuleState(moduleState[2]);
    m_backRightModule.setModuleState(moduleState[3]);
  }

  /**
   * Sets the Speed / Angle / Stats of all of the modules
   * @param xVelocityMps
   * The X velocity (Meters Per Second)
   * @param yVelocityMps
   * The Y velocity (Meters Per Second)
   * @param rotationVelocityRps
   * Rotation velocity (Radians Per Second)
   */
  public void setModules(double xVelocityMps, double yVelocityMps, double rotationVelocityRps) {
      this.m_swerveSpeeds = new ChassisSpeeds(xVelocityMps, yVelocityMps, rotationVelocityRps);
      SwerveModuleState[] target_states = this.m_kinematics.toSwerveModuleStates(this.m_swerveSpeeds);
      setModulesStates(target_states);

  }

  public void resetOdometry() {
      m_odometry.resetPosition(m_navX.getRotation2d(), m_modulePositions, m_currentPose);
  }

  @Override
  public void periodic()
   {
    m_odometry.update(Rotation2d.fromDegrees(Double.valueOf(m_navX.getFusedHeading())), new SwerveModulePosition[]{
      new SwerveModulePosition(m_frontLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_frontLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_frontRightModule.getDriveMotor().getPosition().asSupplier().get(), m_frontRightModule.getModuleState().angle),
      new SwerveModulePosition(m_backLeftModule.getDriveMotor().getPosition().asSupplier().get(), m_backLeftModule.getModuleState().angle), 
      new SwerveModulePosition(m_backRightModule.getDriveMotor().getPosition().asSupplier().get(), m_backRightModule.getModuleState().angle)
    });
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_swerveSpeeds);

    states[0].speedMetersPerSecond = states[0].speedMetersPerSecond / Drive.Stats.kMaxVelocityMetersPerSecond * Swerve.Stats.kMaxVoltage;
    states[1].speedMetersPerSecond = states[1].speedMetersPerSecond / Drive.Stats.kMaxVelocityMetersPerSecond * Swerve.Stats.kMaxVoltage;
    states[2].speedMetersPerSecond = states[2].speedMetersPerSecond / Drive.Stats.kMaxVelocityMetersPerSecond * Swerve.Stats.kMaxVoltage;
    states[3].speedMetersPerSecond = states[3].speedMetersPerSecond / Drive.Stats.kMaxVelocityMetersPerSecond * Swerve.Stats.kMaxVoltage;

    m_frontLeftModule.setModuleState(states[0]);
    m_frontRightModule.setModuleState(states[0]);
    m_backLeftModule.setModuleState(states[0]);
    m_backRightModule.setModuleState(states[0]);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
