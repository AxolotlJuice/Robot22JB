/*
package Team4450.Robot22;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import Team4450.Robot22.commands.autonomous.DriveOut;
import Team4450.Robot22.commands.autonomous.ShootFirst;
import Team4450.Robot22.commands.autonomous.ShootFirst2BL;
import Team4450.Robot22.commands.autonomous.ShootFirst2BR;
import Team4450.Robot22.commands.autonomous.ShootFirst3BL;
import Team4450.Robot22.commands.autonomous.ShootFirst3BR;
import Team4450.Robot22.subsystems.Channel;
import Team4450.Robot22.subsystems.Chooter;
import Team4450.Robot22.subsystems.Climber;
import Team4450.Robot22.subsystems.DriveBase;
import Team4450.Robot22.subsystems.Pickup;

public class AutonSelect extends SubsystemBase
{
    private final DriveBase driveBase;
	private final Channel channel;
	private final Pickup pickup;
	private final Chooter chooter;
	private final Climber climber;

    public AutonSelect() //pull from Robot Container?
    {
        driveBase = new DriveBase();
        channel = new Channel();
        pickup = new Pickup();
        chooter = new Chooter(channel);
        climber = new Climber();
    }


    public static final Pose2d  LBL = new Pose2d(6.687, 5.406, new Rotation2d(Math.toRadians(243)));
    public static final Pose2d  LBR = new Pose2d(6.148, 4.022, new Rotation2d(Math.toRadians(160)));
    public static final Pose2d  RBL = new Pose2d(6.679, 2.818, new Rotation2d(Math.toRadians(153)));
    public static final Pose2d  RBR = new Pose2d(8.133, 2.251, new Rotation2d(Math.toRadians(70)));
    public static final Pose2d  LRL = new Pose2d(9.465, 2.812, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d  LRR = new Pose2d(9.866, 4.230, new Rotation2d(Math.toRadians(336)));
    public static final Pose2d  RRL = new Pose2d(9.311, 5.373, new Rotation2d(Math.toRadians(332)));
    public static final Pose2d  RRR = new Pose2d(7.902, 6.003, new Rotation2d(Math.toRadians(246)));

    //Selecting Auton
    private enum AutoProgram
    {
        NoProgram,
		DriveOut,
		ShootFirst,
		ShootFirst2BL,
		ShootFirst2BR,
		ShootFirst3BL,
		ShootFirst3BR     
    }

    private static SendableChooser<AutoProgram> autoChooser;
    private static SendableChooser<Pose2d>      startingPoseChooser;


    public Command getAutonomousCommand()
    {
        AutoProgram     program = AutoProgram.NoProgram;
        Pose2d          startingPose = LBL;
        Command         autoCommand = null;

        try
        {
            program = autoChooser.getSelected();

            startingPose = startingPoseChooser.getSelected();
        }
        catch (Exception e) { Util.logException(e); }
    
        switch (program)
		{
			case NoProgram:
				autoCommand = null;
				break;
 				
			case DriveOut:
				autoCommand = new DriveOut(driveBase, startingPose);
				break;
 				
            case ShootFirst:
                autoCommand = new ShootFirst(driveBase, chooter, channel, startingPose);
                break;
			case ShootFirst2BL:
				autoCommand = new ShootFirst2BL(driveBase, chooter, channel, startingPose, pickup);
				break;

			case ShootFirst2BR:
				autoCommand = new ShootFirst2BR(driveBase, chooter, channel, startingPose, pickup);
				break;

			case ShootFirst3BL:
				autoCommand = new ShootFirst3BL(driveBase, chooter, channel, startingPose, pickup);
				break;

			case ShootFirst3BR:
				autoCommand = new ShootFirst3BR(driveBase, chooter, channel, startingPose, pickup);
				break;

		}
        
        // Reset motor deadband for auto.
        driveBase.setPowerDeadBand(.02);

        setAutoChoices();
        setStartingPoses();

        return autoCommand;
    }

    public static void setAutoChoices()
    {       
        autoChooser = new SendableChooser<AutoProgram>();
            
        SendableRegistry.add(autoChooser, "Auto Program");
        autoChooser.setDefaultOption("No Program", AutoProgram.NoProgram);
		autoChooser.addOption("Drive Out", AutoProgram.DriveOut);
		autoChooser.addOption("ShootFrist", AutoProgram.ShootFirst);
		autoChooser.addOption("ShootFrist2BL", AutoProgram.ShootFirst2BL);
		autoChooser.addOption("ShootFrist2BR", AutoProgram.ShootFirst2BR);		
		autoChooser.addOption("ShootFrist3BL", AutoProgram.ShootFirst3BL);
		autoChooser.addOption("ShootFrist3BR", AutoProgram.ShootFirst3BR);       
                    
        SmartDashboard.putData(autoChooser);

        Util.consoleLog();
    }

    private static void setStartingPoses()
    {   
        startingPoseChooser = new SendableChooser<Pose2d>();
        
        SendableRegistry.add(startingPoseChooser, "Start Position");
        startingPoseChooser.setDefaultOption("Blue 1", LBL);
        startingPoseChooser.addOption("Blue 2", LBR);       
        startingPoseChooser.addOption("Blue 3", RBL);       
        startingPoseChooser.addOption("Blue 4", RBR);       

        startingPoseChooser.addOption("Red 1", LRL);        
        startingPoseChooser.addOption("Red 2", LRR);        
        startingPoseChooser.addOption("Red 3", RRL);        
        startingPoseChooser.addOption("Red 4", RRR);        
                
        SmartDashboard.putData(startingPoseChooser);

        Util.consoleLog();
    }
}
*/