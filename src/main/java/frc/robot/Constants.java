/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* SI Units are used */

    public static final class OIConstants {

        //Stick Ports
        public static final int kDriverControllerPort = 0;
        public static final int kElevatorControllerPort = 1;
    }

    public static final class DriveConstants {

        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;
  
        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
  
        public static final int kEncoderCPR = 360;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
          // Assumes the encoders are directly mounted on the wheel shafts
          (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    	public static final double kP = 2;
    	public static final double kD = 2;
	    public static final double kI = 2;
    	public static double kTurnToleranceDeg = 5;
    	public static double kTurnRateToleranceDegPerS = 5;
       

    }

    public static final class AutoConstants {
    // TEST
        // Constraints
        public static final double kMaxVelocity = 2; // Meters per second
        public static final double kMaxAcceleration = 1; // Meters per second squared
        public static final double kMaxVoltage = 12;

        public static final double kTrackWidth = 0.69; // Meters
        public static final double kWheelBase = 0.94; // Meters
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

        // Feedforward
        public static final double kS = 1.48; // Volts
        public static final double kV = 2.11; // Volts seconds per meters
        public static final double kA = -0.00802; // Volts seconds per meters squared

        // Feedback
        public static final double kP = 4.14; // Volts seconds per meter
        public static final double kI = 0; // Volts seconds per meter
        public static final double kD = 0; // Volts per seconds per meter


        // Ramsete Controller
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Path Planning
        public static final Pose2d kStartingPosition = new Pose2d(new Translation2d(3.323, -0.867), new Rotation2d(0));

        public static final String RightStartToTrench = "/home/lvuser/deploy/output/RightStartToTrench.wpilib.json";
        public static final String TrenchToRightStart = "/home/lvuser/deploy/output/TrenchToRightStart.wpilib.json";

        public static final String[] RightTrenchGroup = {RightStartToTrench, TrenchToRightStart};
    // Current
        public static final double kAutoDriveDistanceInches = 60;
        public static final double kAutoBackupDistanceInches = 20;
        public static final double kAutoDriveSpeed = 0.5;
    }

    public static final class TurretConstants {

        // Ports
        public static final int kMotorPort = 7;
        public static final int[] kEncoderPorts = {8, 9};

        // Encoder Information
        public static final double kWheelDiameter = 0.15; // Meters
        public static final int kEncoderCPR = 7; // cycles/pulses per revolution
        public static final double gearRatio = 3.1875; // This is 1 if the encoder is directly mounted to the wheel shaft which it should to account for slip
        public static final double kEncoderDistancePerPulse = (kWheelDiameter * Math.PI) / (gearRatio * kEncoderCPR);
        public static final boolean kEncoderReversed = false;
        public static final double kDistanceToDegrees = 180 / 45.0; // Move turret 180 degrees then measure encoder distance and divide

        /* Characterization */

        // Feedforward
        public static final double kS = 0.451; // Volts
        public static final double kV = 1.57; // Volts seconds per meters
        public static final double kA = -0.00143; // Volts seconds per meters squared

        // Feedback
        public static final double kP = 3.08; // Volts seconds per meter
        public static final double kI = 0; // Volts seconds per meter
        public static final double kD = 0; // Volts per seconds per meter

        // Constraints
        public static final double kMaxVoltage = 12;

        // Operation Data
        public static final double kDefaultState = 0; // Distance from set point
        public static final double kMaxDistance = 50;
        public static final double kTurretTolerance = 2;
        public static final double kVisionTolerance = 4; // Degrees
        public static final double kVisionVoltage = 7;
        public static final double kAdjustmentVoltage = 6;
        public static final double kAdjustmentDelay = 0.2;


        public static final Translation2d kGoalPosition = new Translation2d(3.358, -2.358);

    }

    public static final class FeederConstants {

        // Ports
        public static final int kMotorPort = 8;
        public static final int kMotorPort2 = 9;
        public static final int KMotorPort3 = 10;
        public static final int KMotorPort4 = 11;


        // Operation Data
        public static final double kMaxVoltage = 9;
        public static final double kFeederDelay = 0.7;
        public static final double kBallDelay = 0.5;
        public static final double kAutoDelay = 3.7;

    }

    public static final class ShooterConstants {
        // Ports
        public static final int kMasterPort = 5;

        // Operation Data
        public static final double kMaxVoltage = 12;
        public static final double kAutoVoltage = 11.3;
        public static final double kVoltagePerDistance = 1.2 / 0.8; // Volts per meter
    }

    public static final class ElevatorConstants {
        // Ports
        public static final int RightElevator = 4;
        public static final int LeftElevator = 5;
    }
    public static final class VisionConstants {
        // Camera Information
        public static final double kCamera_width = 640; // in pixels
        public static final double kFieldOfView = 67; // in degrees
        public static final double kDegPerPixel = kFieldOfView / kCamera_width;

        // Camera Names
        public static final String TurretCamName = "TurretCam";
        public static final String DriveCamName = "DriveCam";

        // Object Data
        public static final double kTapeWidth = 0.2; // in meters

    }


    public static final class PhysicConstants {
        public static final double kGravity = 9.8;
    }
}
