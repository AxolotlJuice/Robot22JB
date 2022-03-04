package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Robot22.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Channel extends SubsystemBase{
    private boolean indexerRunning;

    private WPI_VictorSPX   indexerMotor = new WPI_VictorSPX(INDEXER_VICTOR);

    private double defaultPower = 0.30;

    public Channel(){
        indexerMotor.setInverted(true);

        stopIndexer();

		Util.consoleLog("Channel created!");
    }

    @Override
    public void periodic() {

    }

    private void updateDS(){
        SmartDashboard.putBoolean("Indexer", indexerRunning);
    }

    public void stopIndexer(){
        Util.consoleLog();
        
        indexerMotor.stopMotor();
		
		indexerRunning = false;
		
		updateDS();
    }

    public void startIndexer(double power){
    Util.consoleLog("%.2f", power);
		
		indexerMotor.set(power);
		
		indexerRunning = true;
		
		updateDS();
    }

    public void startIndexer(){
        startIndexer(defaultPower);
    }

    public boolean toggleIndexer(double power){
        
        if (isRunning())
            stopIndexer();
        else
            startIndexer(power);

        return isRunning();
    }

    public boolean toggleIndexerUp(){
        return toggleIndexer(defaultPower);
    }

    public boolean toggleIndexerDown(){
        return toggleIndexer(-defaultPower);
    }

    public Runnable toggleTheIndexer(boolean up){
        Runnable aRunnable = new Runnable() {
            public void run(){
                if (up){
                    toggleIndexerUp();
                }
                else{
                    toggleIndexerDown();
                }
            }
        };
        return aRunnable;
    }

    public boolean isRunning(){
        return indexerRunning;
    }

    public void feedBall(){
        Util.consoleLog();

        // Can't feed a ball if shooter wheel is not running.
        if  (!RobotContainer.shooter.isRunning()) return;

        startIndexer();

        Timer.delay(.50);   // set time so one ball is fed.

        stopIndexer();
    }
}
