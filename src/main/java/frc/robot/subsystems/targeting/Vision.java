package frc.robot.subsystems.targeting;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    PhotonCamera camera;
    Transform3d targetData;
    
    public Vision(PhotonCamera camera) {
        this.camera = camera;
        camera.setPipelineIndex(0);
        
    }

    //returns whether a target (AprilTag) has been detected
    public boolean targetDetected() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return true;
        }
        return false;
    }

    public double getYaw() {
        if (targetDetected()) {
        PhotonPipelineResult result = camera.getLatestResult();

        PhotonTrackedTarget target = result.getBestTarget();
        
        if (target != null) {
            double yaw = target.getYaw();
            
            return yaw;
        }
        }
        return 0.0;
        
    }


    
    //gets target data such as x and y offset, rotational offset, and returns everything as a Transform3d 
    public Transform3d getTargetData() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (targetDetected()) {
            PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    return target.getBestCameraToTarget();
                }
            }
        return null;
        }


    
    
    //returns the current horizontal displacement with respect to the AprilTag (uses getY() because the Y offset in PhotonVision is the horizontal axis)
    public double getHorizontalDisplacement() {
        if (targetDetected()) {
            return targetData.getY();
        }
        else return 0;
    }

    public double getZAngle() {
        if (targetDetected()) {
            Rotation3d rot = targetData.getRotation();
            return Math.toDegrees(rot.getAngle());
        }
        else return 0.0;
    }



    @Override
    public void periodic() {
        //update targetData with current info
        targetData = getTargetData();

        //output values to SmartDashboard/Shuffleboard
        SmartDashboard.putBoolean("Target Detected", targetDetected());
        SmartDashboard.putNumber("Yaw Angle", getYaw());
        SmartDashboard.putNumber("Z Angle", getZAngle());

        SmartDashboard.putNumber("Horizontal Displacement", getHorizontalDisplacement());
    }

}
