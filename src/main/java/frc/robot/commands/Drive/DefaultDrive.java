package frc.robot.commands.Drive;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem m_differentialDrive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;
    


 /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    m_differentialDrive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(m_differentialDrive);
  }
  @Override
  public void execute() {
    m_differentialDrive.arcadeDrive(m_forward.getAsDouble()*-1, m_rotation.getAsDouble());
  }
}


