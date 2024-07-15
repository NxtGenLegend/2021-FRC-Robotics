package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class HalveDriveSpeed extends CommandBase {
  private final DriveSubsystem m_differentialDrive;

  public HalveDriveSpeed(DriveSubsystem drive) {
    m_differentialDrive = drive;
  }

  @Override
  public void initialize() {
    m_differentialDrive.setMaxOutput(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    m_differentialDrive.setMaxOutput(1);
  }
}