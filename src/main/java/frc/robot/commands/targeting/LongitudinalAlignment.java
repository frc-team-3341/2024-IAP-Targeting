package frc.robot.commands.targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.targeting.Vision;

public class LongitudinalAlignment extends Command {
    SwerveDrive swerve;
    Vision vision;
    boolean isAligned = false;
    double[] toleranceArray = {3, 4};

    public LongitudinalAlignment(SwerveDrive swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        
        addRequirements(this.swerve, this.vision);
    }

    @Override
    public void initialize() {
        swerve.drive(new Translation2d(0, 0), 0, false, false);
        
        
    }

    @Override
    public void execute() { 
        if (vision.getLongitudinalDisplacement() <= toleranceArray[1] && 
        vision.getLongitudinalDisplacement() >= toleranceArray[0]) {
            isAligned = true;
        }

        if (vision.targetDetected() && isAligned == false) {
            swerve.drive(new Translation2d(1, 0), 0, false, false); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);

        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (isAligned) {
            swerve.drive(new Translation2d(0, 0), 0, false, false);

            swerve.stopMotors();
            
            return true;
        }
        return false;
    }
}
