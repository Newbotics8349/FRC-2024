package frc.robot.AprilTagTracking;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTag {
    int id;
    double yaw;
    double pitch;
    double area;
    
    AprilTag(PhotonTrackedTarget target)
    {
        id = target.getFiducialId();
        yaw = target.getYaw();
        pitch = target.getPitch();
        area = target.getArea();
    }
}
