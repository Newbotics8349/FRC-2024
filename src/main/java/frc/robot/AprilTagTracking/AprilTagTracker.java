package frc.robot.AprilTagTracking;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.AprilTagTracking.AprilTag;

public class AprilTagTracker {
    private PhotonCamera camera;
    private List<AprilTag> tags;
    
    public AprilTagTracker(String cameraNameString)
    {
        camera = new PhotonCamera(cameraNameString);
    }

    public void UpdateTracker()
    {
        tags.clear();
        PhotonPipelineResult result = camera.getLatestResult();
        for(PhotonTrackedTarget target : result.targets)
        {
            tags.add(new AprilTag(target));
        }
    }

    public int GetNumTargets()
    {
        return tags.size();
    }

    public List<AprilTag> GetTargets()
    {
        return tags;
    }

    public boolean HasTargetWithId(int id)
    {
        if (GetTargetWithId(id) != null) return true;
        return false;
    }

    public AprilTag GetTargetWithId(int id)
    {
        for (AprilTag tag : tags)
        {
            if (tag.id == id) return tag;
        }
        return null;
    }
}
