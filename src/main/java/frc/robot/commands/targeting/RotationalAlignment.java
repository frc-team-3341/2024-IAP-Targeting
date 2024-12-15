package frc.robot.commands.targeting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.targeting.Vision;


public class RotationalAlignment extends Command {
    Vision vision;
    SwerveDrive swerve;
    boolean isAligned;
    double direction;

    double[] toleranceArray = {178, 182}; //index 0 is bottom range, index 1 is highest possible value to be considered aligned
    

    PIDController pid = new PIDController(0, 0 , 0); //TODO figure out PID constants por favor

    public RotationalAlignment(SwerveDrive swerve, Vision vision) {
        // this.swerve = swerve;
        this.vision = vision;
        this.swerve = swerve;
        addRequirements(this.vision, this.swerve);
        // pid.setTolerance(0.5);
    }
    
    @Override
    public void initialize() {
        isAligned = false;
    }

    //TODO code with pid (still in progress, so do not use this code as it might be dangerous)

    // @Override
    // public void execute() {
    //     double output = pid.calculate(vision.getYaw(), 0);

    //     if (vision.targetDetected()) { //positive yaw means we need to rotate clockwise, and negative yaw means to rotate counterclockwise
    //         swerve.drive(new Translation2d(0, 0), output, false, false);
    //     }
      
    // }

    //THE FOLLOWING CODE IS WITHOUT PID, IF TESTING PID UNCOMMENT OUT THE OTHER EXECUTE METHOD (and comment this execute() method out)
    @Override
    public void execute() {
        if (vision.targetDetected()) {
            if (vision.getZAngle() > 180 && vision.getZAngle() > toleranceArray[1]) { //counterclockwise turn
                direction = -1;
            }
            else if (vision.getZAngle() < 180 && vision.getZAngle() < toleranceArray[0]) { //clockwise turn 
                direction = 1;
            }
            else isAligned = true;

            swerve.drive(new Translation2d(0, 0), 0.3*direction, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);

        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (vision.getZAngle() >= toleranceArray[0] && vision.getZAngle() <= toleranceArray[1] || isAligned) {

            swerve.drive(new Translation2d(0, 0), 0, false, false);

            swerve.stopMotors();

            return true;
        }
        return false;
    }

    //the following code is for the pid code (which is still in progress)

    // @Override
    // public boolean isFinished() {
    //     return pid.atSetpoint();
    // }
}
