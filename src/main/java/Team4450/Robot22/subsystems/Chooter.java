

package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.robot;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import Team4450.Lib.Util;

public class Chooter extends SubsystemBase{
    private PIDController getController() {
        return null; }
    public final double     defaultPower = .50, lowTargetRPM = 3000, highTargetRPM = 5000, maxRPM = 6000;
    private double  currentPower = defaultPower, tolerance = 0, targetRPM = lowTargetRPM;
    private boolean         startUp, wheelRunning, highRPM;
    private static double   kP = .0002, kI = kP / 100, kD = 0;
    private WPI_TalonFX     shooterMotor = new WPI_TalonFX(7);
    private FXEncoder       encoder = new FXEncoder(shooterMotor);
    
    public Chooter(Chooter channel){  
        new PIDController(kP, kI, kD); //why error :(
        getController().setTolerance((double) tolerance);
        shooterMotor.setNeutralMode(NeutralMode.Coast);}
    public void enable(double rpm){
        targetRPM = rpm;
        enable();}

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
        enable(1000); //temp value
    }
    
    public void lowShot(){
        enable(300); //temp value
    }    

    public boolean toggleHighLowRPM(){
        Util.consoleLog();
        if (targetRPM == lowTargetRPM){
            targetRPM = highTargetRPM;
            highRPM = true;}
        else{
            targetRPM = lowTargetRPM;
            highRPM = false;}
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

    public void periodic(){
        if (robot.isEnabled()) 
            if (isRunning() && startUp){
                if (shooterMotor.getStatorCurrent() > 150)
                    {stopWheel();
                    backupIndexer();
                    startWheel();
                }

        if (isRunning()) stopWheel();}}
    public boolean toggleWheel(double power){
        Util.consoleLog("%.2f", power);
        if (isRunning())
            stopWheel();
        else startWheel(power);
        return isRunning(); }

    public boolean toggleWheel(){
        return toggleWheel(currentPower);}

    public double getRPM(){
        return encoder.getRPM();}

    private void startWheel(){
        startWheel(currentPower);}

    private void startWheel(double power){ 
        Util.consoleLog("%.2f", power);
        currentPower = power;
        if (isEnabled()) disable();
        shooterMotor.set(currentPower);
        wheelRunning = true;
        startUp = true;
        Util.timeStamp();
        updateDS();}

    private void stopWheel(){
        Util.consoleLog();
        if (isEnabled()) disable();
        shooterMotor.stopMotor();
        wheelRunning = false;
        updateDS();}

    private boolean isEnabled() {
        return false;}

    public boolean togglePID(){
         Util.consoleLog();
         if (isEnabled())
               disable();
        else enable();
        return isRunning();}

    private void updateDS() {
        SmartDashboard.putBoolean("Shooter", wheelRunning);
        SmartDashboard.putBoolean("ShooterHighRPM", highRPM);
    }
}
