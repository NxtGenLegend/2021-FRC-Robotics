package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

    private static TurretSubsystem m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(TurretConstants.kMotorPort);

   private Encoder m_encoder = new Encoder(TurretConstants.kEncoderPorts[0], TurretConstants.kEncoderPorts[1], TurretConstants.kEncoderReversed);


   private double offSet = 0;
   private double position = 0;

   public TurretSubsystem() {
       m_encoder.setDistancePerPulse(TurretConstants.kEncoderDistancePerPulse);
   }

    @Override
    public void periodic() {
        
        resetEncoder();
        offSet = 0;
    
        position = getEncoderDistance() + offSet;
    }

    public static TurretSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new TurretSubsystem();
        }

        return m_instance;
    }

    public double getPosition() {
       return position;
    }

    public void set(double speed) {
       if (position >= TurretConstants.kMaxDistance) {
           if (speed <= 0) {
               m_motor.set(speed);
           } else {
               stop();
           }
       } else if (position < 0) {
           if (speed >= 0) {
               m_motor.set(speed);
           } else {
               stop();
           }
       } else {
           m_motor.set(speed);
       }
    }

    public void setVoltage(double voltage) {
        if (position >= TurretConstants.kMaxDistance) {
            if (voltage <= 0) {
                m_motor.setVoltage(voltage);
            } else {
                stop();
            }
        } 
        else {
            m_motor.setVoltage(voltage);
        }
    }

    public void stop() {
        m_motor.setVoltage(0);
    }

    public int getEncoderValue() {
        return m_encoder.get();
    }

    public double getEncoderDistance() {
       return m_encoder.getDistance();
    }

    public double getEncoderRate() {
        return m_encoder.getRate();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }



}
