package Team4450.Robot22.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;

import static Team4450.Robot22.Constants.*;

import java.util.List;

import Team4450.Robot22.RobotContainer;
import Team4450.Robot22.subsystems.DriveBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonJB1 extends CommandBase{
    
    private SequentialCommandGroup	commands = null;
	private Command command = null;
    
    private Pose2d startingPose;
    private final DriveBase driveBase;

    private final AutonSelection autonSelection;

    private double tempIntialX, tempIntialY;

    public AutonJB1(DriveBase driveBase, AutonSelection autonSelection){
        Util.consoleLog();
        this.driveBase = driveBase;
        this.autonSelection = autonSelection;
        addRequirements(driveBase);
    }

    @Override
    public void initialize(){
        Util.consoleLog(); 
        driveBase.setMotorSafety(false);
        driveBase.resetEncodersWithDelay();
        RobotContainer.navx.resetYaw();
        RobotContainer.navx.setHeading(autonSelection.getSelected());
        RobotContainer.navx.setTargetHeading(autonSelection.getSelected());
        driveBase.setCANTalonRampRate(1.0);
        driveBase.resetOdometer(Pose2d(tempIntialX, tempIntialY, new Rotation2d(), RobotContainer.navx.getHeading());
        commands = new SequentialCommandGroup();
        command = new AutoDrive(driveBase, 0.0,
                                SRXMagneticEncoderRelative.getTicksForDistance( 0, DRIVE_WHEEL_DIAMETER),
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
        
        /**
         * backwards x amount
         * ajust to correct angle (if needed).
         * spin ball motors to y rpm.
         * move out of starting polygon (if needed).
         */
        commands.addCommands(command);
        //temp values (0.0, 0). More commands will need to be added like the one above.
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
