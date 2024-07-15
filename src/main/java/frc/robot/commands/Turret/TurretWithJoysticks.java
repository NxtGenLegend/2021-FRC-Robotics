package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretWithJoysticks extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private DoubleSupplier m_speed;

    public TurretWithJoysticks(TurretSubsystem turretSubsystem, DoubleSupplier speed) {
        m_turretSubsystem = turretSubsystem;
        m_speed = speed;

        addRequirements(turretSubsystem);
    }

    public TurretWithJoysticks(TurretSubsystem m_turret, double z) {
	}

	@Override
    public void execute() {
        m_turretSubsystem.setVoltage(m_speed.getAsDouble() * TurretConstants.kMaxVoltage);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.stop();
    }
}