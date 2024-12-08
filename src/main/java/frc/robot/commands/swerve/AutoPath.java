package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoPath extends SequentialCommandGroup{
    
    SwerveDrive swerve;
    boolean firstPath;
    Pose2d initialPose;

    public AutoPath(String pathName, SwerveDrive swerve, PIDConstants translational, PIDConstants rotational, boolean firstPath) {
    this.swerve = swerve;
    
    // Load path from 2024 PathPlannerLib
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      // Will automagically re-display the path every time teleop is started
      // Set field's trajectory to the trajectory of the path
      this.swerve.getField().getObject("traj").setPoses(poses);
    });

    var swerveAuto = AutoBuilder.followPath(path);

    // Setting voltage to 0 is necessary in order to stop robot
    addCommands(swerveAuto.beforeStarting(() -> {
      //Need to initialize the starting pose in here
      // Possible ways to get the start pose of the path
      // path.getPreviewStartingHolonomicPose()
      // path.getStartingDifferentialPose()
      if(firstPath == true){
        swerve.resetPose(path.getPreviewStartingHolonomicPose());
      }
      else {
        //we want to do nothing if it's not the first path that's being used
      }
    }).finallyDo(() -> {
      swerve.setModulesPositions(0, 0);
      swerve.setModuleVoltages(0, 0);
    }));
  }
  
}
