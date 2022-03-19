package Team4450.Robot22.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;

import static Team4450.Robot22.Constants.*;

import Team4450.Robot22.RobotContainer;
import Team4450.Robot22.subsystems.Channel;
import Team4450.Robot22.subsystems.Chooter;
import Team4450.Robot22.subsystems.DriveBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class AutonJB1 extends CommandBase{
    
    private SequentialCommandGroup	commands = null;
	private Command command = null;
    
    private Chooter chooter;
    private Channel channel;
    private Pose2d startingPose;
    private final DriveBase driveBase;

    public AutonJB1(DriveBase driveBase, Pose2d startingPose){
        Util.consoleLog();
        this.driveBase = driveBase;
        this.startingPose = startingPose;
        addRequirements(driveBase);
    }

    @Override
    public void initialize(){
        Util.consoleLog(); 

        LCD.printLine(LCD_1, "Mode: Auto - ShootFirst - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);
        
        driveBase.setMotorSafety(false);
        
        driveBase.resetEncodersWithDelay();
        
        RobotContainer.navx.resetYaw();
        
        RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees() + 90); //will needed to be changed
        
        RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees() + 90);
        
        driveBase.SetCANTalonRampRate(1.0);
        
        driveBase.resetOdometer(startingPose, startingPose.getRotation().getDegrees() + 90);
        
        commands = new SequentialCommandGroup();

        command = new InstantCommand(chooter::enable);
        commands.addCommands(command);
        
        command = new WaitCommand(2.0);

        command = new InstantCommand(channel::feedBall);
        
        command = new InstantCommand(chooter::highShot);
        
        command = new InstantCommand(chooter::disable);

        command = new AutoRotate(driveBase, 1.0, 148.0, AutoDrive.Pid.on, AutoDrive.Heading.angle);
        commands.addCommands(command);
        
        command = new AutoDrive(driveBase, 1,
                                SRXMagneticEncoderRelative.getTicksForDistance(4.072792543, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        //move 4.072792543 ft.
        commands.addCommands(command);
        
        command = new AutoRotate(driveBase, 1.5, -61.0, AutoDrive.Pid.on, AutoDrive.Heading.angle);
        commands.addCommands(command);

        command = new AutoDrive(driveBase, 1,
                                SRXMagneticEncoderRelative.getTicksForDistance(3.975897565, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        //move 3.975897565 ft.
        commands.addCommands(command);

        command = new InstantCommand(chooter::enable);
        commands.addCommands(command);
        
        command = new WaitCommand(2.0);

        command = new InstantCommand(channel::feedBall);
        
        command = new InstantCommand(chooter::highShot);
        
        command = new InstantCommand(chooter::disable);
        commands.addCommands(command);

        

        command = new AutoDrive(driveBase, -1,
                                SRXMagneticEncoderRelative.getTicksForDistance(2.0, DRIVE_WHEEL_DIAMETER),
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