package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.SHOOTER_TALON;
import static Team4450.Robot22.Constants.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

    public void initialize(boolean high)
    {
        if (high)
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
    }

    public void enable(double rpm){
        targetRPM = rpm;
        enable();
    }

    public void enable(){
        Util.consoleLog("target rpm=%.0f", targetRPM);
        backupIndexer();
        setSetpoint(targetRPM);
        super.enable();
        wheelRunning = true;
        startUp = true;
        Util.timeStamp();
        updateDS();
    }

    public void disable(){
        Util.consoleLog();
        super.disable();
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

    public void periodic()
    {
        if (robot.isEnabled())
        {
            
            super.periodic();

            //Util.consoleLog("current=%.3f", shooterMotor.getStatorCurrent());

            if (isRunning() && startUp)
            {
                if (shooterMotor.getStatorCurrent() > 200)
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
            if (isRunning()) stopWheel();
            targetRPM = lowTargetRPM;
            highRPM = false;
        }
        
    }
    public boolean toggleWheel(double power)
    {
        Util.consoleLog("%.2f", power);
        if (isRunning())
            stopWheel();
        else 
            startWheel(power);
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

    @Override
    protected void useOutput(double output, double setpoint) 
    {
        double ff = m_shooterFeedforward.calculate(setpoint / 60);
        
        double volts = output + ff;

        //Util.consoleLog("rpm=%.0f  out=%.3f  set=%.3f  ff=%.3f  v=%.3f", getRPM(), output, setpoint, ff, volts);

        shooterMotor.setVoltage(volts);
    }

    private void startWheel()
    {
        startWheel(currentPower);
    }

    private void startWheel(double power)
    { 
        Util.consoleLog("%.2f", power);
        backupIndexer();
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
        if (isEnabled())super.disable();
        shooterMotor.stopMotor();
        wheelRunning = false;
        updateDS();
    }

    public boolean togglePID()
    {
         Util.consoleLog();
         if (isEnabled())
               disable();
        else 
            enable();
        return isRunning();
    }

    private void updateDS() 
    {
        SmartDashboard.putBoolean("Shooter", wheelRunning);
        SmartDashboard.putBoolean("ShooterHighRPM", highRPM);
    }

    private void backupIndexer()
    {
        Util.consoleLog();

        channel.toggleIndexerDown();

        Timer.delay(.5);   

        channel.stopIndexer();
    }

    @Override
    protected double getMeasurement() {
        return 0;
    }

    public double getMaxRPM()
    {
        return encoder.getMaxRPM();
    }
}
