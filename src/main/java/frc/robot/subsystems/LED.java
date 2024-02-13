// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.RunElevator;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class LED extends SubsystemBase {
    private final CANdle candle;
    private final Timer timer, timer1, autoTimer, funeralTimer;
    private boolean green, orange, flag, flag1, flag2, animationFlag, funeralFlag = false; 
    private final CANdleConfiguration config = new CANdleConfiguration();
    //FIXME get led count
    private final ColorFlowAnimation flow = new ColorFlowAnimation(0, 0, 0, 0, 0.1, 50, ColorFlowAnimation.Direction.Backward, 0);
    private final RainbowAnimation rainbow = new RainbowAnimation(0.5, 0.1, 50, true, 0);


    public LED() {
        candle = new CANdle(Constants.CANDLE_ID, "CANivore");
        timer = new Timer();
        timer1 = new Timer();
        autoTimer = new Timer();
        funeralTimer = new Timer();
        timer.reset();
        timer.start();
        timer1.reset();
        autoTimer.reset();
        funeralTimer.reset();
        funeralTimer.start();
        candle.configAllSettings(new CANdleConfiguration());
        config.statusLedOffWhenActive = false;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 0.5;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config);
        candle.setLEDs(0, 0, 0);
    }

    public void green(){
        green = true;
        orange = false;
    }

    public void orange(){
        green = false;
        orange = true;
    }

    public void reset(){
        flag = false;
        candle.configBrightnessScalar(0.5);
    }

    public void resetColors(){
        green = false;
        orange = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(DriverStation.isDisabled() && false/*!Robot.robotContainer.s_Vision.isDead()*/){
            //TODO make pattern
            animationFlag = false;
        }
        else if(DriverStation.isAutonomousEnabled()){
            if(!animationFlag){
                candle.setLEDs(0, 0, 254);
                candle.animate(flow);
                animationFlag = true;
            }
        }
        else if(RunElevator.trapScored){
            if(!animationFlag){
                candle.animate(rainbow);
                animationFlag = true;
            }
        }
        else if(RunElevator.retracting && Robot.robotContainer.s_Elevator.isAtPosition()){
            candle.setLEDs(0, 255, 0);
        }
        else if(false/*Robot.robotContainer.s_Vision.isDead()*/){
            //FIXME get values for whole else if structure
            if(funeralTimer.get() > 0.2 && !funeralFlag){
                candle.setLEDs(12, 237, 54, 0, 0, 0);
                funeralTimer.reset();
                funeralFlag = true;
            }
            else if(funeralTimer.get() > 0.2 && funeralFlag){
                candle.setLEDs(179, 83, 97, 0, 0, 0);
                funeralTimer.reset();
                funeralFlag = false;
            }
            if(orange){
                if(!flag){
                    timer1.reset();
                    timer1.start();
                    flag = true;
                    flag1 = false;
                    flag2 = false;
                }
                else if(timer1.get() > 1){
                    flag1 = true;
                }
                if(!flag1){
                    if(timer.get() > 0.1 && !flag2){
                        candle.setLEDs(255, 165, 0, 0, 0, 0);
                        timer.reset();
                        flag2 = true;
                    }
                    else if(timer.get() > 0.1 && flag2){
                        candle.setLEDs(0, 0, 0, 0, 0, 0);
                        timer.reset();
                        flag2 = false;
                    }
                }
                else{
                    /*for(double i = 0.5; i > 0; i -= 0.001){
                        candle.setLEDs(255, 165, 0, 0, 0, 0);
                        candle.configBrightnessScalar(i);
                    }
                    for(double i = 0; i < 0.5; i += 0.001){
                        candle.setLEDs(255, 165, 0, 0, 0, 0);
                        candle.configBrightnessScalar(i);
                    }*/
                }
            }
            else if(!Robot.robotContainer.s_Intake.botFull()){
                candle.setLEDs(0, 0, 254, 0, 0, 0);
            }
            else if(green){
                if(!flag){
                    timer1.reset();
                    timer1.start();
                    flag = true;
                    flag1 = false;
                    flag2 = false;
                }
                else if(timer1.get() > 1){
                    flag1 = true;
                }
                if(!flag1){
                    if(timer.get() > 0.1 && !flag2){
                        candle.setLEDs(0, 255, 0, 0, 0, 0);
                        timer.reset();
                        flag2 = true;
                    }
                    else if(timer.get() > 0.1 && flag2){
                        candle.setLEDs(0, 0, 0, 0, 0, 0);
                        timer.reset();
                        flag2 = false;
                    }
                }
                else{
                    /*for(double i = 0.5; i > 0; i -= 0.001){
                        candle.setLEDs(0, 255, 0, 0, 0, 0);
                        candle.configBrightnessScalar(i);
                    }
                    for(double i = 0; i < 0.5; i += 0.001){
                        candle.setLEDs(0, 255, 0, 0, 0, 0);
                        candle.configBrightnessScalar(i);
                    }*/
                }
            }
            else if(Robot.robotContainer.s_Intake.botFull()){
                if(!flag){
                    timer1.reset();
                    timer1.start();
                    flag = true;
                    flag1 = false;
                    flag2 = false;
                }
                else if(timer1.get() > 1){
                    flag1 = true;
                }
                if(!flag1){
                    if(timer.get() > 0.1 && !flag2){
                        candle.setLEDs(0, 0, 254, 0, 0, 0);
                        timer.reset();
                        flag2 = true;
                    }
                    else if(timer.get() > 0.1 && flag2){
                        candle.setLEDs(0, 0, 0, 0, 0, 0);
                        timer.reset();
                        flag2 = false;
                    }
                }
                else{
                    /*for(double i = 0.5; i > 0; i -= 0.001){
                        candle.setLEDs(255, 0, 0, 0, 0, 0);
                        candle.configBrightnessScalar(i);
                    }
                    for(double i = 0; i < 0.5; i += 0.001){
                        candle.setLEDs(255, 0, 0, 0, 0, 0);
                        candle.configBrightnessScalar(i);
                    }*/
                }
            }
        }
        else if(true /*!Robot.robotContainer.s_Vision.isDead()*/){
            if(orange){
                if(!flag){
                    timer1.reset();
                    timer1.start();
                    flag = true;
                    flag1 = false;
                    flag2 = false;
                }
                else if(timer1.get() > 1){
                    flag1 = true;
                }
                if(!flag1){
                    if(timer.get() > 0.1 && !flag2){
                        candle.setLEDs(255, 165, 0);
                        timer.reset();
                        flag2 = true;
                    }
                    else if(timer.get() > 0.1 && flag2){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag2 = false;
                    }
                }
                else{
                    /*for(double i = 0.5; i > 0; i -= 0.001){
                        candle.setLEDs(255, 165, 0);
                        candle.configBrightnessScalar(i);
                    }
                    for(double i = 0; i < 0.5; i += 0.001){
                        candle.setLEDs(255, 165, 0);
                        candle.configBrightnessScalar(i);
                    }*/
                }
            }
            else if(!Robot.robotContainer.s_Intake.botFull()){
                candle.setLEDs(0, 0, 254);
            }
            else if(green){
                if(!flag){
                    timer1.reset();
                    timer1.start();
                    flag = true;
                    flag1 = false;
                    flag2 = false;
                }
                else if(timer1.get() > 1){
                    flag1 = true;
                }
                if(!flag1){
                    if(timer.get() > 0.1 && !flag2){
                        candle.setLEDs(0, 255, 0);
                        timer.reset();
                        flag2 = true;
                    }
                    else if(timer.get() > 0.1 && flag2){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag2 = false;
                    }
                }
                else{
                    /*for(double i = 0.5; i > 0; i -= 0.001){
                        candle.setLEDs(0, 255, 0);
                        candle.configBrightnessScalar(i);
                    }
                    for(double i = 0; i < 0.5; i += 0.001){
                        candle.setLEDs(0, 255, 0);
                        candle.configBrightnessScalar(i);
                    }*/
                }
            }
            else if(Robot.robotContainer.s_Intake.botFull()){
                if(!flag){
                    timer1.reset();
                    timer1.start();
                    flag = true;
                    flag1 = false;
                    flag2 = false;
                }
                else if(timer1.get() > 1){
                    flag1 = true;
                }
                if(!flag1){
                    if(timer.get() > 0.1 && !flag2){
                        candle.setLEDs(0, 0, 254);
                        timer.reset();
                        flag2 = true;
                    }
                    else if(timer.get() > 0.1 && flag2){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag2 = false;
                    }
                }
                else{
                    /*for(double i = 0.5; i > 0; i -= 0.001){
                        candle.setLEDs(255, 0, 0);
                        candle.configBrightnessScalar(i);
                    }
                    for(double i = 0; i < 0.5; i += 0.001){
                        candle.setLEDs(255, 0, 0);
                        candle.configBrightnessScalar(i);
                    }*/
                }
            }
        }
    }
}