package frc.robot.AprilTagTracking;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTag {
    int id;
    double yaw;
    double pitch;
    double area;
    
    /**
     * Builds the april tag target from PhotonVision data
     * @param target, photonvision object
     */
    AprilTag(PhotonTrackedTarget target)
    {
        id = target.getFiducialId();
        yaw = target.getYaw();
        pitch = target.getPitch();
        area = target.getArea();
    }


    /**
     * Accessor for target yaw angle.
     * @return The yaw angle of the target
     */
    public double GetYaw()
    {
        return yaw;
    }


    /**
     * Accessor for target pitch angle.
     * @return The pitch angle of the target
     */
    public double GetPitch()
    {
        return pitch;
    }

}
