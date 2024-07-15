package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import edu.wpi.first.wpilibj.VictorSP;

public class FeederSubsystem extends SubsystemBase {

    private static FeederSubsystem m_instance;

    private VictorSP m_motor = new VictorSP(FeederConstants.kMotorPort);
    private VictorSP m_motor1 = new VictorSP(FeederConstants.kMotorPort2);



    private int powerCells = 0;

    public FeederSubsystem() {
        m_motor.setInverted(true);
        
    }
    public static FeederSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new FeederSubsystem();
        }

        return m_instance;
    }

    public void setNumPowerCells(int num) {
        powerCells = num;
    }

    public int getNumPowerCells() {
        return powerCells;
    }

   
    public void set(double speed) {
        m_motor.set(speed);
    }

    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public void stop() {
        set(0);
    }
    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param FeederUps the maximum output to which the drive will be constrained
   */

  public void FeederUp(double FeederUp){

    m_motor.set(FeederUp);

  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param FeederDowns the maximum output to which the drive will be constrained
   */

  public void FeederDown(double FeederDowns){
      m_motor.set(FeederDowns);
  }
   
  
  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param LoaderUPS the maximum output to which the drive will be constrained
   */

  public void LoaderUp(double LoaderUPS){

    m_motor1.set(LoaderUPS);

  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param LoaderDOWNS the maximum output to which the drive will be constrained
   */

  public void LoaderDown(double LoaderDOWNS){
      m_motor1.set(LoaderDOWNS);
  }


}
