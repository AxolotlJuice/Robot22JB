package Team4450.Robot22.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;

import static Team4450.Robot22.Constants.*;

import Team4450.Robot22.RobotContainer;
import Team4450.Robot22.subsystems.Chooter;
import Team4450.Robot22.subsystems.DriveBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonJB2 extends CommandBase{
    
    private SequentialCommandGroup	commands = null;
	private Command command = null;
    
    private Chooter chooter;
    
    private Pose2d startingPose;
    private final DriveBase driveBase;


    private double tempIntialX, tempIntialY;

    public AutonJB2(DriveBase driveBase, Pose2d startingPose){
        Util.consoleLog();
        this.driveBase = driveBase;
        this.startingPose = startingPose;
        addRequirements(driveBase);

        /*
        if(startingPose2d ==  ){
        }
        */ 
    }

    @Override
    public void initialize(){
        Util.consoleLog(); 
        driveBase.setMotorSafety(false);
        driveBase.resetEncodersWithDelay();
        RobotContainer.navx.resetYaw();
        RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees() + 90); //will needed to be changed
        RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees() + 90);
        driveBase.SetCANTalonRampRate(1.0);
        driveBase.resetOdometer(startingPose, startingPose.getRotation().getDegrees() + 90);
        commands = new SequentialCommandGroup();
     
        //run method in chooter
        //rotate
        //command = new AutoDrive(driveBase, -0.5, //-0.5 needs to be 3.5in equivalent
        //                        SRXMagneticEncoderRelative.getTicksForDistance( 3, DRIVE_WHEEL_DIAMETER),
        //                        AutoDrive.StopMotors.stop,
        //                        AutoDrive.Brakes.on,
        //                        AutoDrive.Pid.on,
        //                        AutoDrive.Heading.angle);
        //commands.addCommands(command);
        
        command = new InstantCommand(chooter::autonHighShot);
        commands.addCommands(command);

        command = new AutoDrive(driveBase, 0,
                                SRXMagneticEncoderRelative.getTicksForDistance(0, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        commands.addCommands(command);

        command = new AutoRotate(driveBase, 1.5, 1.0, AutoDrive.Pid.on, AutoDrive.Heading.angle);
        commands.addCommands(command);

        command = new AutoDrive(driveBase, 0,
                                SRXMagneticEncoderRelative.getTicksForDistance(0, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        commands.addCommands(command);
        
        command = new AutoRotate(driveBase, 1.5, -1.0, AutoDrive.Pid.on, AutoDrive.Heading.angle);
        commands.addCommands(command);

        command = new AutoDrive(driveBase, 0,
                                SRXMagneticEncoderRelative.getTicksForDistance(0, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        commands.addCommands(command);

        command = new InstantCommand(chooter::autonHighShot);
        commands.addCommands(command);

        command = new AutoDrive(driveBase, -1,
                                SRXMagneticEncoderRelative.getTicksForDistance(2, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        commands.schedule(); //this is used to signify the end of the command squence.
        
    } 
     
    @Override
	public void execute() 
	{
	}

    @Override
    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);

        driveBase.stop();

        //Add two things here not sure what they do yet though...
    }

    @Override
    public boolean isFinished(){

        return !commands.isScheduled();
    }
}
