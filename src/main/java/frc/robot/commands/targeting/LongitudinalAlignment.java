package frc.robot.commands.targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.targeting.Vision;

public class LongitudinalAlignment extends Command {
    SwerveDrive swerve;
    Vision vision;
    boolean isAligned;
    double[] toleranceArray = {0.5};

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
        if (vision.targetDetected() && vision.getLongitudinalDisplacement() > toleranceArray[0]) {
            swerve.drive(new Translation2d(0.5, 0), 0, false, false); 
        }

        else isAligned = true;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);

        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (vision.getLongitudinalDisplacement() <= toleranceArray[0]) {
            swerve.drive(new Translation2d(0, 0), 0, false, false);

            swerve.stopMotors();

            return true;
        }
        return false;
    }
}
