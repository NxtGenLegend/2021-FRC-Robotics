/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.commands.Drive.DefaultDrive;
import frc.robot.commands.Drive.DriveForDistance;

import frc.robot.commands.Drive.DriveForDistance;
import frc.robot.commands.Turret.MoveTurretToPosition;
import frc.robot.commands.Turret.SmallAdjustment;
import frc.robot.commands.Turret.TurretWithJoysticks;
import frc.robot.commands.Shoot.Shoot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final Command m_simpleAuto =
      new DriveForDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
                        m_robotDrive);  
 

  Joystick m_driverController = new Joystick(frc.robot.Constants.OIConstants.kDriverControllerPort);
  Joystick m_elevatorController = new Joystick(frc.robot.Constants.OIConstants.kElevatorControllerPort);

  public RobotContainer() {
    // Configure the button bindings
    m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive, () -> m_driverController.getY(), () -> m_driverController.getZ()));
    m_turret.setDefaultCommand(new TurretWithJoysticks(m_turret, m_elevatorController.getZ()));

    SmartDashboard.putBoolean("NavX Connected", m_robotDrive.getConnected());

    //double j = m_elevatorController.getX();


    configureButtonBindings();

     }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, 2)
       .whenPressed(() -> m_robotDrive.setMaxOutput(1))
       .whenReleased(() -> m_robotDrive.setMaxOutput(0.8));
    new JoystickButton(m_driverController, 1)
       .whenPressed(() -> m_robotDrive.setMaxOutput(0))
       .whenReleased(() -> m_robotDrive.setMaxOutput(0.8));
    new JoystickButton(m_driverController, 3)
       .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
       .whenReleased(() -> m_robotDrive.setMaxOutput(0.8));

    new JoystickButton(m_driverController, 8)
       .whenPressed(() -> m_feeder.FeederUp(0.6))
       .whenReleased(() -> m_feeder.FeederUp(0));  
        


    new JoystickButton(m_elevatorController, 3)
       .whenPressed(() -> m_intake.IntakeIn(0.6))
       .whenReleased(() -> m_intake.IntakeIn(0));
    new JoystickButton(m_elevatorController,4)
       .whenPressed(() -> m_intake.IntakeOut(-0.6))
       .whenReleased(() -> m_intake.IntakeOut(0)); 
    
    new JoystickButton(m_elevatorController, 5)
       .whenPressed(() -> m_intake.IntakeUP(0.6))
       .whenReleased(() -> m_intake.IntakeUP(0));
    new JoystickButton(m_elevatorController,6)
       .whenPressed(() -> m_intake.IntakeDown(-0.6))
       .whenReleased(() -> m_intake.IntakeDown(0)); 

    new JoystickButton(m_elevatorController, 7)
       .whenPressed(() -> m_feeder.LoaderUp(0.6))
       .whenReleased(() -> m_feeder.LoaderUp(0));
    new JoystickButton(m_elevatorController,8)
       .whenPressed(() -> m_feeder.LoaderDown(-0.6))
       .whenReleased(() -> m_feeder.LoaderDown(0)); 
   
    new JoystickButton(m_elevatorController, 9)
       .whileHeld(new Shoot(m_shooter));
   }
    
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_simpleAuto;

  }
}
