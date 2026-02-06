package frc.robot.commands;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;
public class climberPositionCheck extends Command{
    private Drive drive;
    private final double goalXPosition = 0;
    private final double acceptableXError = 0;
    private final double goalYPosition = 0;
    private final double acceptableYError = 0;
    private final double goalRotation = 0;
    private final double acceptableRotationError = 0;
    private double currentXPosition = 0;
    private double currentYPosition = 0;
    private double currentRotation = 0;
    private boolean inPosition = false;


    public climberPositionCheck(Drive drive){
        this.drive = drive;
        currentXPosition = this.drive.getPose().getX();
        currentYPosition = this.drive.getPose().getY();
        currentRotation = this.drive.getRotation().getDegrees();
        inPosition = checkInPosition();
    }

    @Override
    public void initialize(){
        currentXPosition = this.drive.getPose().getX();
        currentYPosition = this.drive.getPose().getY();
        currentRotation = this.drive.getRotation().getDegrees();
        inPosition = checkInPosition();
    }

    @Override
    public void execute(){
        currentXPosition = this.drive.getPose().getX();
        currentYPosition = this.drive.getPose().getY();
        currentRotation = this.drive.getRotation().getDegrees();
        inPosition = checkInPosition();
    }

    @Override
    public void end(boolean interrupted){
        //Code to run when in position.
    }

    @Override
    public boolean isFinished(){
        return inPosition;
    }

    private boolean checkInPosition(){
        if(Math.abs(goalXPosition - currentXPosition) < acceptableXError && Math.abs(goalYPosition - currentYPosition) < acceptableYError && Math.abs(goalRotation - currentRotation) < acceptableRotationError){
            return true;
        }else if(false){
            //This if statement should check another potential climbable position, if applicable
            // I.E. if the robot is moved over and facing the opposite direction
            return true;
        }else{
            return false;
        }
    }
}
