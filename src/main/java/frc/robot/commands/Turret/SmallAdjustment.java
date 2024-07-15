package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class SmallAdjustment extends CommandBase {

    private TurretSubsystem m_turretSubsystem;

    private Timer m_timer = new Timer();

    private double m_speed;

    public SmallAdjustment(TurretSubsystem turretSubsystem, double speed) {
        m_turretSubsystem = turretSubsystem;
        m_speed = speed;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_turretSubsystem.setVoltage(m_speed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(TurretConstants.kAdjustmentDelay);
    }

    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.stop();
    }
}
