package frc.robot.commands.targeting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.targeting.Vision;

public class HorizontalAlignment extends Command {
    Vision vision;
    SwerveDrive swerve;
    boolean isAligned;
    int direction;

    double[] toleranceArray = {-0.05, 0.05};

    PIDController pid = new PIDController(0, 0, 0); //TODO find pid constants

    public HorizontalAlignment(SwerveDrive swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        
        addRequirements(this.swerve, this.vision);
    }
    @Override
    public void initialize() {
        isAligned = false;
    }

    @Override
    public void execute() {
        if (vision.targetDetected()) {
            if (vision.getHorizontalDisplacement() < toleranceArray[0]) {
                direction = -1;
            }
            else if (vision.getHorizontalDisplacement() > toleranceArray[1]) {
                direction = 1;
            }

            else isAligned = true;
          
            swerve.drive(new Translation2d(0, 0.3*direction), 0, false, false);
    }
        
    }

    //experimental pid code
    // @Override
    // public void execute() {
    //     if (vision.targetDetected()) {
    //         double output = pid.calculate(vision.getHorizontalDisplacement(), 0);

    //         swerve.drive(new Translation2d(output, 0), 0, false, false);
    //     }

    // }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);

        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (vision.getHorizontalDisplacement() >= toleranceArray[0] &&
        vision.getHorizontalDisplacement() <= toleranceArray[1] || isAligned) {
            swerve.drive(new Translation2d(0, 0), 0, false, false);

            swerve.stopMotors();

            return true;
        }

        return false;
    }

    //experimental pid code
    // @Override
    // public boolean isFinished() {
    //     return pid.atSetpoint();
    // }
}
