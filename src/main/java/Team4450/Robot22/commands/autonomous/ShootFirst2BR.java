package Team4450.Robot22.commands.autonomous;

import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;

import static Team4450.Robot22.Constants.*;
import Team4450.Robot22.RobotContainer;
import Team4450.Robot22.subsystems.Channel;
import Team4450.Robot22.subsystems.DriveBase;
import Team4450.Robot22.subsystems.Pickup;
import Team4450.Robot22.subsystems.Chooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Shoot loaded ball then drive out of starting area.
 */
public class ShootFirst2BR extends CommandBase
{
	private final DriveBase driveBase;

    private SequentialCommandGroup	commands = null;
	private Command					command = null;
	private	Pose2d					startingPose;
	private Chooter					chooter;
	private Channel					channel;
	private Pickup                  pickup;

	/**
	 * Creates a new ShootFirst_JBMod autonomous command.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ShootFirst2BR(DriveBase subsystem, Chooter chooter, Channel channel, Pose2d startingPose, Pickup pickup) 
	{
		Util.consoleLog("x=%.3f  y=%.3f  hdg=%.1f", startingPose.getX(), startingPose.getY(), 
						startingPose.getRotation().getDegrees());
		
		driveBase = subsystem;

		this.startingPose = startingPose;

		this.chooter = chooter;
		this.channel = channel;
		this.pickup = pickup;
			  
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.driveBase);
	}
	
	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();
		
		driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	LCD.printLine(LCD_1, "Mode: Auto - ShootFirst_JBMod - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				DriverStation.isFMSAttached(), gameMessage);
		
		// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees() + 90);
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees() + 90);
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking with initial x,y position and heading (passed at constructor). 
		// Robot must be placed in same starting location each time for pose tracking
		// to work. We add 90 degrees because robot will be facing the target instead of outward.
		driveBase.resetOdometer(startingPose, startingPose.getRotation().getDegrees() + 90);
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		chooter.initialize(true);

		commands = new SequentialCommandGroup();
		
        // First action is shoot the loaded ball.
		// We do this by starting the chooter motor PID at target speed.

		command = new InstantCommand(chooter::enable);
		
		commands.addCommands(command);

		// Now wait for chooter to spin up.

		command = new WaitCommand(2.0);
		
		commands.addCommands(command);

		// Now shoot the loaded ball.

		command = new InstantCommand(channel::feedBall);
		
		commands.addCommands(command);
		
		// Now wait for chooter to shoot.

		command = new WaitCommand(1.0);
		
		commands.addCommands(command);
		
		// Now disable chooter motor PID.

		command = new InstantCommand(chooter::disable);
		
		commands.addCommands(command);

		//start movement commands
		
		//rotate: 194 (clockwise) (192)
		command = new AutoRotate(driveBase, 0.3, -166, AutoDrive.Pid.on, AutoDrive.Heading.angle);
		commands.addCommands(command);

		//engage pickup
		command = new InstantCommand(pickup::toggleDeploy);

		//Drive: (x:9.288, y:5.443)     (old distance: 4.266713047441)
		//       (x:10.400, y:5.937) distance: 1.21679086124 m., 3.992096001443569 in.
		command = new AutoDrive(driveBase, 0.5, 
								SRXMagneticEncoderRelative.getTicksForDistance(3.99, DRIVE_WHEEL_DIAMETER),
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);
		commands.addCommands(command);
		
		//wait


		//retract pickup
		command = new InstantCommand(pickup::toggleDeploy);
		commands.addCommands(command);

		command = new AutoDrive(driveBase, -0.5, 
								SRXMagneticEncoderRelative.getTicksForDistance(3.99, DRIVE_WHEEL_DIAMETER),
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);
		commands.addCommands(command);

		//rotate 194 (counter-clockwise)
		command = new AutoRotate(driveBase, 0.3, 166, AutoDrive.Pid.on, AutoDrive.Heading.angle);
		commands.addCommands(command);
		
		//shoot command squence
		command = new InstantCommand(chooter::enable);
		commands.addCommands(command);

		command = new WaitCommand(2.0);
		commands.addCommands(command);

		command = new InstantCommand(channel::feedBall);
		commands.addCommands(command);
		
		command = new WaitCommand(1.0);
		commands.addCommands(command);
		
		command = new InstantCommand(chooter::disable);
		commands.addCommands(command);
		
		//drive out of stating area
		command = new AutoDrive(driveBase, -0.5, 
								SRXMagneticEncoderRelative.getTicksForDistance(4.0, DRIVE_WHEEL_DIAMETER),
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);

		commands.schedule();
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() 
	{
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
		
		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingR());
		Util.consoleLog("end ---------------------------------------------------------------");
	}
	
	/**
	 *  Returns true when this command should end. That should be when
	 *  all the commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() 
	{
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		if (commands == null)
			return true;
		else
			return !commands.isScheduled();
	}
}
