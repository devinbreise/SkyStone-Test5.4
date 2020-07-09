package org.firstinspires.ftc.teamcode.basicLibs;


import com.qualcomm.robotcore.hardware.Servo;

public class ServoControl {

    Servo servo;
    double maxAngularVelocity;

    public ServoControl(Servo servo, double maxAngularVelocity){
        this.servo = servo;
        this.maxAngularVelocity = maxAngularVelocity;
    }

    public void runToPosition(double angularVelocity, double targetPosition){
        if(angularVelocity > maxAngularVelocity){
            //log error for attempting to move too fast
            return;
        }

        double distance = targetPosition - servo.getPosition();
        double totalTime = distance/angularVelocity;
        double newPosition;
        long startTime = System.currentTimeMillis();
        long now = System.currentTimeMillis();

        while(now < startTime + totalTime){
            newPosition = angularVelocity * (now - startTime);
            servo.setPosition(newPosition);
            now = System.currentTimeMillis();
        }

    }

    public void runToPositionNoWait(final double angularVelocity, final double targetPosition){
        Thread thread = new Thread(new Runnable(){
            public void run(){
                runToPosition(angularVelocity, targetPosition);
            }
        });
        thread.start();
    }

}
