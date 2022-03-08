
package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.SHOOTER_TALON;
import static Team4450.Robot22.Constants.INDEXER_VICTOR;
import static Team4450.Robot22.Constants.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Chooter extends PIDSubsystem{
    
    private boolean			wheelRunning;
  	
    private WPI_TalonFX     shooterMotor = new WPI_TalonFX(SHOOTER_TALON);

    private FXEncoder       encoder = new FXEncoder(shooterMotor);

    public final double     defaultPower = .50, lowTargetRPM = 3000, highTargetRPM = 5000, maxRPM = 6000;
    private double          currentPower = defaultPower, targetRPM = highTargetRPM, toleranceRPM = 50;
    private static double   kP = .0002, kI = kP / 100, kD = 0;
    private boolean         startUp, highRPM = true;
    private double          startTime, kS = .498, kV = .108;

    private Channel         channel;
    // ks and kv determined by characterizing the shooter motor. See the shooter characterization
    // project.
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(kS, kV);
    
    public Chooter(Channel channel)
    {  
        super(new PIDController(kP, kI, kD));

        shooterMotor.setInverted(true);
    
        getController().setTolerance(toleranceRPM);
		  
        shooterMotor.setNeutralMode(NeutralMode.Coast);

        this.channel = channel;

        Util.consoleLog("Shooter created!");
    }

    public void enable(double rpm){
        targetRPM = rpm;
        enable();
    }

    public void enable(){
        Util.consoleLog("target rpm=%.0f", targetRPM);
        wheelRunning = true;
        startUp = true;
        Util.timeStamp();
        updateDS();
    }

    public void disable(){
        Util.consoleLog();
        stopWheel();
        updateDS();
    }
    
    public void highShot(){
        enable(5000); //temp value
    }
    
    public void lowShot(){
        enable(3000); //temp value
    }    

    public boolean toggleHighLowRPM(){
        Util.consoleLog();
        if (targetRPM == lowTargetRPM)
        {
            targetRPM = highTargetRPM;
            highRPM = true;
        }
        else
        {
            targetRPM = lowTargetRPM;
            highRPM = false;
        }
        updateDS();
        return highRPM;
    }

    public boolean isRunning(){
        return wheelRunning; 
    }

    private void backupIndexer(){ 
        Util.consoleLog();
        Timer.delay(0);
    }

    public void periodic()
    {
        if (robot.isEnabled())
        {
            
            super.periodic();

            Util.consoleLog("current=%.3f", shooterMotor.getStatorCurrent());

            if (isRunning() && startUp)
            {
                if (shooterMotor.getStatorCurrent() > 150)
                {
                    stopWheel();
                    backupIndexer();
                    startWheel();
                }
                if (Util.getElaspedTime(startTime) > 1.5) startUp = false;
            }
        }
        else
        {
            if (isRunning())
            {
                stopWheel();
            }}

            targetRPM = lowTargetRPM;
            highRPM = false;
        }
    public boolean toggleWheel(double power)
    {
        Util.consoleLog("%.2f", power);
        if (isRunning())
            stopWheel();
        else startWheel(power);
        return isRunning(); 
    }

    public boolean toggleWheel()
    {
        return toggleWheel(currentPower);
    }

    public double getRPM()
    {
        return encoder.getRPM();
    }

    private void startWheel()
    {
        startWheel(currentPower);
    }

    private void startWheel(double power)
    { 
        Util.consoleLog("%.2f", power);
        currentPower = power;
        if (isEnabled()) disable();
        shooterMotor.set(currentPower);
        wheelRunning = true;
        startUp = true;
        Util.timeStamp();
        updateDS();
    }

    public void stopWheel()
    {
        Util.consoleLog();
        if (isEnabled()) disable();
        shooterMotor.stopMotor();
        wheelRunning = false;
        updateDS();
    }

    public boolean togglePID()
    {
         Util.consoleLog();
         if (isEnabled())
               disable();
        else enable();
        return isRunning();
    }

    private void updateDS() 
    {
        SmartDashboard.putBoolean("Shooter", wheelRunning);
        SmartDashboard.putBoolean("ShooterHighRPM", highRPM);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        
        
    }

    @Override
    protected double getMeasurement() {
        return 0;
    }
}
