package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private static VisionSubsystem m_instance;

    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = networkTableInstance.getTable("chameleon-vision").getSubTable("TurretCam");
    private NetworkTableEntry yawEntry = table.getEntry("targetYaw");
    private NetworkTableEntry widthEntry = table.getEntry("targetBoundingWidth");
    private double yaw = 0;
    private double width = 0;

    public static VisionSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new VisionSubsystem();
        }

        return m_instance;
    }

    @Override
    public void periodic() {
        setValues();
        System.out.println(yawEntry.getDouble(404));
    }

    public void setYaw() {
        double tempYaw = yawEntry.getDouble(Double.NaN);
        yaw = (Double.isNaN(tempYaw)) ? yaw: tempYaw; // if yaw entry is invalid then don't change yaw value
    }

    public void setWidth() {
        double tempWidth = widthEntry.getDouble(Double.NaN);
        width = (Double.isNaN(tempWidth)) ? width: tempWidth; // if yaw entry is invalid then don't change yaw value
    }

    public void setValues() {
        setYaw();
        setWidth();
    }

    public double getYaw() {
        return yaw;
    }

    public double getWidth() { return width; }

    public double getImageSizeInDeg() {
        return width * VisionConstants.kDegPerPixel;
    }

    public double getDistance() {
        double radians = Math.toRadians(getImageSizeInDeg());

        return (VisionConstants.kTapeWidth / 2) / Math.tan(radians);
    }
}
