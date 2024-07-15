package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import edu.wpi.first.wpilibj.VictorSP;

public class IntakeSubsystem extends SubsystemBase {
    

    private VictorSP m_motor = new VictorSP(FeederConstants.KMotorPort3);
    private VictorSP m_motor1 = new VictorSP(FeederConstants.KMotorPort4);


      /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param IntakeINS the maximum output to which the drive will be constrained
   */

  public void IntakeIn(double IntakeINS){

    m_motor.set(IntakeINS);

  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param IntakeOUTS the maximum output to which the drive will be constrained
   */

  public void IntakeOut(double IntakeOUTS){
      m_motor.set(IntakeOUTS);
  }
     
  
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param IntakeUPS the maximum output to which the drive will be constrained
   */

  public void IntakeUP(double IntakeUPS){

    m_motor1.set(IntakeUPS);

  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param IntakeDOWNS the maximum output to which the drive will be constrained
   */

  public void IntakeDown(double IntakeDOWNS){
      m_motor1.set(IntakeDOWNS);
  }

}
