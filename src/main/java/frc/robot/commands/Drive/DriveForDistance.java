package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class DriveForDistance extends CommandBase {
  private final DriveSubsystem m_differentialDrive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveForDistance(double inches, double speed, DriveSubsystem drive) {
    m_distance = inches;
    m_speed = speed;
    m_differentialDrive = drive;
  }

  @Override
  public void initialize() {
    m_differentialDrive.resetEncoders();
    m_differentialDrive.arcadeDrive(m_speed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_differentialDrive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_differentialDrive.getDistanceTraveled()) >= m_distance;
  }
}
