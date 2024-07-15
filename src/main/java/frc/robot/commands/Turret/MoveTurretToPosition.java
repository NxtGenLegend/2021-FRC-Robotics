package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class MoveTurretToPosition extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private double m_angle;

    public MoveTurretToPosition(TurretSubsystem turretSubsystem, double angle) {
        m_turretSubsystem = turretSubsystem;
        m_angle = angle;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        int sign = (m_angle - m_turretSubsystem.getPosition() * TurretConstants.kDistanceToDegrees >= 0) ? 1: -1;
        m_turretSubsystem.setVoltage(sign * TurretConstants.kMaxVoltage);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_turretSubsystem.getPosition() * TurretConstants.kDistanceToDegrees <= m_angle + TurretConstants.kTurretTolerance && m_turretSubsystem.getPosition() * TurretConstants.kDistanceToDegrees >= m_angle - 2;
    }

    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.stop();
    }
}
