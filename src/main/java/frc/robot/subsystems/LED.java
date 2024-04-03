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
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class LED extends SubsystemBase {
    private final CANdle candle;
    private final Timer timer;
    private boolean green, orange, flag, flag1 = false; 
    private final CANdleConfiguration config = new CANdleConfiguration();
    private final ColorFlowAnimation flow = new ColorFlowAnimation(0, 0, 0, 0, 0.1, Constants.NUM_LEDS, ColorFlowAnimation.Direction.Forward, 0);
    private final RainbowAnimation rainbow = new RainbowAnimation(1, 0.1, Constants.NUM_LEDS, false, 0);
    private final FireAnimation fire = new FireAnimation(1, 0.1, Constants.NUM_LEDS, 0.1, 0.1, false, 0);
    private final LarsonAnimation pocket = new LarsonAnimation(255, 0, 0, 0, 0.1, Constants.NUM_LEDS, LarsonAnimation.BounceMode.Center, 1);
    private final TwinkleOffAnimation twinkle = new TwinkleOffAnimation(0, 255, 0, 0, 0.1, Constants.NUM_LEDS, TwinkleOffPercent.Percent100, 0);
    private final SingleFadeAnimation fade = new SingleFadeAnimation(0, 0, 0, 0, 1, Constants.NUM_LEDS, 0);
    private final StrobeAnimation strobe = new StrobeAnimation(0, 0, 0, 0, 0.5, Constants.NUM_LEDS, 0);
    private final LarsonAnimation bad = new LarsonAnimation(179, 83, 97, 0, 0.1, Constants.NUM_LEDS, LarsonAnimation.BounceMode.Center, 1, 0);

    public LED() {
        candle = new CANdle(Constants.CANDLE_ID, Constants.CANIVORE_BUS);
        timer = new Timer();
        timer.reset();
        timer.start();
        candle.configAllSettings(new CANdleConfiguration());
        config.statusLedOffWhenActive = false;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGBW;
        config.brightnessScalar = 1;
        config.vBatOutputMode = VBatOutputMode.On;
        config.v5Enabled = false;
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
        green = false;
        orange = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(DriverStation.isFMSAttached()){
            candle.configBrightnessScalar(1);
        }
        else{
            candle.configBrightnessScalar(0.25);
        }
        if(DriverStation.isDisabled() && Robot.robotContainer.s_Vision.isDead()){
            bad.setNumLed(Constants.NUM_LEDS);
            candle.animate(bad);
        }
        else if(DriverStation.isDisabled() && !Robot.robotContainer.s_Vision.isDead()){
            if(timer.get() < 10){
                candle.animate(fire);
            }
            else if(timer.get() < 20){
                candle.animate(pocket);
            }
            else if(timer.get() < 30){
                candle.animate(twinkle);
            }
            else{
                timer.reset();
                timer.start();
                twinkle.setR((int) (Math.random() * 255));
                twinkle.setG((int) (Math.random() * 255));
                twinkle.setB((int) (Math.random() * 255));
                pocket.setR((int) (Math.random() * 255));
                pocket.setG((int) (Math.random() * 255));
                pocket.setB((int) (Math.random() * 255));
            }
        }
        else if(DriverStation.isAutonomousEnabled()){
            if(!Robot.getRedAlliance()){
                flow.setR(0);
                flow.setB(254);
            }
            else{
                flow.setR(255);
                flow.setB(0);
            }
            candle.animate(flow);
        }
        else if(RunElevator.trapScored){
            candle.animate(rainbow);
        }
        else if(RunElevator.retracting && Robot.robotContainer.s_Elevator.isAtPosition()){
            candle.clearAnimation(0);
            candle.setLEDs(0, 255, 0);
        }
        else if(Robot.robotContainer.s_Vision.isDead()){
            bad.setNumLed(Constants.NUM_LEDS - 9);
            candle.animate(bad, 0);
            if(orange){
                if(!flag){
                    timer.reset();
                    timer.start();
                    flag = true;
                    flag1 = false;
                    strobe.setLedOffset(18);
                    fade.setLedOffset(18);
                    strobe.setR(255);
                    strobe.setG(40);
                    strobe.setB(0);
                    fade.setR(255);
                    fade.setG(40);
                    fade.setB(0);
                }
                else if(timer.get() > 3){
                    flag1 = true;
                }
                if(!flag1){
                    candle.animate(strobe, 1);
                }
                else{
                    candle.animate(fade, 1);
                }
            }
            else if(!Robot.robotContainer.s_Intake.botFull()){
                candle.clearAnimation(1);
                candle.setLEDs(0, 0, 254, 0, 17, 8);
                flag = false;
            }
            else if(Shooter.shooterSad){
                candle.clearAnimation(1);
                candle.setLEDs(136, 0, 209, 0, 17, 8);
                flag = false;
            }
            else if(green){
                if(!flag){
                    timer.reset();
                    timer.start();
                    flag = true;
                    flag1 = false;
                    strobe.setLedOffset(18);
                    fade.setLedOffset(18);
                    strobe.setR(0);
                    strobe.setG(255);
                    strobe.setB(0);
                    fade.setR(0);
                    fade.setG(255);
                    fade.setB(0);
                }
                else if(timer.get() > 3){
                    flag1 = true;
                }
                if(!flag1){
                    candle.animate(strobe);
                }
                else{
                    candle.animate(fade);
                }
            }
            else if(Robot.robotContainer.s_Intake.botFull()){
                if(!flag){
                    timer.reset();
                    timer.start();
                    flag = true;
                    flag1 = false;
                    strobe.setLedOffset(18);
                    fade.setLedOffset(18);
                    strobe.setR(255);
                    strobe.setG(0);
                    strobe.setB(0);
                    fade.setR(255);
                    fade.setG(0);
                    fade.setB(0);
                }
                else if(timer.get() > 3){
                    flag1 = true;
                }
                if(!flag1){
                    candle.animate(strobe);
                }
                else{
                    candle.animate(fade);
                }
            }
        }
        else if(!Robot.robotContainer.s_Vision.isDead()){
            candle.clearAnimation(1);
            if(orange){
                if(!flag){
                    timer.reset();
                    timer.start();
                    flag = true;
                    flag1 = false;
                    strobe.setLedOffset(0);
                    fade.setLedOffset(0);
                    strobe.setR(255);
                    strobe.setG(40);
                    strobe.setB(0);
                    fade.setR(255);
                    fade.setG(40);
                    fade.setB(0);
                }
                else if(timer.get() > 3){
                    flag1 = true;
                }
                if(!flag1){
                    candle.animate(strobe);
                }
                else{
                    candle.animate(fade);
                }
            }
            else if(!Robot.robotContainer.s_Intake.botFull()){
                candle.clearAnimation(0);
                candle.setLEDs(0, 0, 254);
                flag = false;
            }
            else if(Shooter.shooterSad){
                candle.clearAnimation(0);
                candle.setLEDs(136, 0, 209);
                flag = false;
            }
            else if(green){
                if(!flag){
                    timer.reset();
                    timer.start();
                    flag = true;
                    flag1 = false;
                    strobe.setLedOffset(0);
                    fade.setLedOffset(0);
                    strobe.setR(0);
                    strobe.setG(255);
                    strobe.setB(0);
                    fade.setR(0);
                    fade.setG(255);
                    fade.setB(0);
                }
                else if(timer.get() > 3){
                    flag1 = true;
                }
                if(!flag1){
                    candle.animate(strobe);
                }
                else{
                    candle.animate(fade);
                }
            }
            else if(Robot.robotContainer.s_Intake.botFull()){
                if(!flag){
                    timer.reset();
                    timer.start();
                    flag = true;
                    flag1 = false;
                    strobe.setLedOffset(0);
                    fade.setLedOffset(0);
                    strobe.setR(255);
                    strobe.setG(0);
                    strobe.setB(0);
                    fade.setR(255);
                    fade.setG(0);
                    fade.setB(0);
                }
                else if(timer.get() > 3){
                    flag1 = true;
                }
                if(!flag1){
                    candle.animate(strobe);
                }
                else{
                    candle.animate(fade);
                }
            }
        }
    }
}