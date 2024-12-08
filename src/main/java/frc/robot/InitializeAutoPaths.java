package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.ExamplePath;
import frc.robot.subsystems.swerve.SwerveDrive;

public class InitializeAutoPaths {
    
    private final SwerveDrive swerve;

    private final ExamplePath ExamplePath;

    private final SendableChooser<Command> autoPathChooser = new SendableChooser<>();

    public InitializeAutoPaths(SwerveDrive swerve) {
        this.swerve = swerve;
         AutoBuilder.configureHolonomic(
        this.swerve::getPoseFromEstimator, // Robot pose supplier
        this.swerve::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this.swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this.swerve::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1,0,0), // Translation PID constants -> path independent
            new PIDConstants(1,0,0), // Rotation PID constants -> more or less path dependent
            Constants.SwerveConstants.maxChassisTranslationalSpeed, // Max module speed, in m/s
            Constants.SwerveConstants.trackWidthHypotenuse, // Drive base radius in meters. Distance from robot center
                                                            // to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this.swerve // Reference to this subsystem to set requirements
        );

        ExamplePath = new ExamplePath(swerve);

        autoPathChooser.addOption("Example Path", ExamplePath);

        SmartDashboard.putData(autoPathChooser);
    }

    public Command getAutonomousCommand() {
        return autoPathChooser.getSelected();
    }
}
