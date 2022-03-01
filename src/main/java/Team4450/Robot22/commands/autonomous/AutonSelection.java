package Team4450.Robot22.commands.autonomous;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import Team4450.Robot22.subsystems.DriveBase;

public class AutonSelection extends CommandBase
{

    private final DriveBase driveBase;
    driveBase = new DriveBase();

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
        NoAuton,
        AutonJB1,
        AutonJB2    
    }

    private static SendableChooser<AutoProgram> autoChooser;
    private static SendableChooser<Pose2d>      startingPoseChooser;


    public Command getAutonomousCommand()
    {
        AutoProgram     program = AutoProgram.NoAuton;
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
            case NoAuton:
                autoCommand = null;
                break;
                
            case AutonJB1:
                autoCommand = new AutonJB1(driveBase, startingPose);
                break;
                
            case AutonJB2:
                autoCommand = new AutonJB2(driveBase, startingPose);
                break;
        }
        
        // Reset motor deadband for auto.
        driveBase.setPowerDeadBand(.02);

    return autoCommand;
    }

    private static void setAutoChoices()
    {
        Util.consoleLog();
            
        autoChooser = new SendableChooser<AutoProgram>();
            
        SendableRegistry.add(autoChooser, "Auto Program");
        autoChooser.setDefaultOption("No Program", AutoProgram.NoAuton);
        autoChooser.addOption("Auton JB1", AutoProgram.AutonJB1);       
        autoChooser.addOption("Auton JB2", AutoProgram.AutonJB2);       
                    
        SmartDashboard.putData(autoChooser);
    }

    private static void setStartingPoses()
    {
        Util.consoleLog();
        
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
    }
}