package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xDoubleSupplier;
    private final DoubleSupplier m_yDoubleSupplier;
    private final DoubleSupplier m_rotationDoubleSupplier;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xValue, DoubleSupplier yValue, DoubleSupplier rotationValue) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_xDoubleSupplier = xValue;
    this.m_yDoubleSupplier = yValue;
    this.m_rotationDoubleSupplier = rotationValue;

    addRequirements(driveSubsystem);
  }

  /**
   * Eliminates the drift from the joystick input
   */
  public double correctJoystickDrift(final double input) {
    return (Math.abs(input) > Constants.OI.kXboxcontrollerDrift) ? input : 0;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    m_driveSubsystem.setModules(
      correctJoystickDrift(m_xDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxVelocityMetersPerSecond,
      correctJoystickDrift(m_yDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxVelocityMetersPerSecond, 
      correctJoystickDrift(m_rotationDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxAngularVelocityRadiansPerSecond
    );
    // ! DEBUG
    // TODO CONVERT TO SHUFFLEBOARD
    // System.out.println("x: " + correctJoystickDrift(m_xDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxVelocityMetersPerSecond); 
    // System.out.println("y: " + correctJoystickDrift(m_yDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxVelocityMetersPerSecond);
    // System.out.println("rotation: " + correctJoystickDrift(m_rotationDoubleSupplier.getAsDouble()) * Drive.Stats.kMaxAngularVelocityRadiansPerSecond);
  }

  // public void setToZero() {
  //   m_driveSubsystem.setAllModulesToZero();
  // } 

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setModules(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
