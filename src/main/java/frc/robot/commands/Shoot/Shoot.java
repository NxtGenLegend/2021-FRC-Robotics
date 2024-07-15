package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_voltage;

    public Shoot(ShooterSubsystem shooterSubsystem, double voltage) {
        m_shooterSubsystem = shooterSubsystem;
        m_voltage = voltage;

        addRequirements(shooterSubsystem);
    }

    public Shoot(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        m_voltage = ShooterConstants.kMaxVoltage;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setVoltage(m_voltage);
        // 11.3 10ft

    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
    }
}
