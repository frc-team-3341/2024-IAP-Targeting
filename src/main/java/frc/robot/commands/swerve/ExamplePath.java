package frc.robot.commands.swerve;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ExamplePath extends SequentialCommandGroup{
    
    SwerveDrive swerve;

    public ExamplePath(SwerveDrive swerve) {
        this.swerve = swerve;

        addCommands(
            new ParallelCommandGroup(new AutoPath("Example Path", this.swerve, new PIDConstants(1.0, 0 , 0), new PIDConstants(1.0, 0 , 0), true))
        );
    }
}
