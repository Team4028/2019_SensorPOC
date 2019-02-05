/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IVisionSensor;
import frc.robot.util.LogDataBE;

/**
 * This class exposes the OnBoard IPhone Vision sensor
 * 
 * Lead Student: Izzy
 */
public class VisionIP implements IVisionSensor {
    private Socket _clientSocket;
    private PrintWriter _out;
    private BufferedReader _in;
    private boolean _inFov;
    private double _distanceInInches;
    private double _angle1InDegrees;
    private boolean _isSocketConnected;
    private double _time;
    private long _timeElapsed;
    private boolean _isVisionThreadRunning;
    private int i = 1;
    private int _restartThreadTimes = 30;
    private int _threadSleepingTimeInMillis = 2000;

    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static VisionIP _instance = new VisionIP();

    public static VisionIP getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private VisionIP() {
        startThread();
        System.out.println("is Socket Connected: " + get_isSocketConnected());
    }

    private void openConnection(String ipAddress, int port) {
        
        try {
            _clientSocket = new Socket();
            _clientSocket.connect(new InetSocketAddress(ipAddress, port), 500);
            _out = new PrintWriter(_clientSocket.getOutputStream(), true);
            _in = new BufferedReader(new InputStreamReader(_clientSocket.getInputStream()));
            _isSocketConnected = true;
        } catch (IOException e) {
            _isSocketConnected = false;
        }
    }

    private void startThread() {
        _isVisionThreadRunning = false;
        Thread t = new Thread(() -> {
            while (i <= _restartThreadTimes && !_isSocketConnected) {
                openConnection(RobotMap.SOCKET_CLIENT_CONNECTION_IPADRESS, RobotMap.SOCKET_CLIENT_CONNECTION_PORT);
                if (!_isSocketConnected) { 
                    i++;
                    try {
                        Thread.sleep(_threadSleepingTimeInMillis);
                        System.out .println("sleeping loop: " + i);
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }
            }
            while (!Thread.interrupted()) {
                long start = System.nanoTime();
                String resp = sendMessage("VISION");
                _isVisionThreadRunning = true;
                long finish = System.nanoTime();
                _timeElapsed = finish - start;
                String[] kvPairs = resp.split("\\|");

                for (String kvPair : kvPairs) {
                    String[] kv = kvPair.split("\\:");
                    String key = kv[0];
                    String value = kv[1];

                    if (key.equals("InFov")) {
                        _inFov = Boolean.parseBoolean(value);
                     
                    } else if (key.equals("angle1")) {
                        _angle1InDegrees = Double.parseDouble(value);
                    } else if (key.equals("distance")) {
                        _distanceInInches = Double.parseDouble(value);
                        
                    }else if (key.equals("time")){
                        _time = Double.parseDouble(value);
                    }
                }
            }
        });t.start();
    }

    private String sendMessage(String msg) {
        _out.println(msg);
        String _resp = "";
        try {
            _resp = _in.readLine();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println(e.toString());
        }
        return _resp;
    }

    public void stopConnection() {
        try {
            _in.close();
        } catch (IOException e1) {
        // TODO Auto-generated catch block
        e1.printStackTrace();
        }
        _out.close();
        try {
        _clientSocket.close();
        } catch (IOException e) {
      // TODO Auto-generated catch block
        e.printStackTrace();
        }
    }

    
    @Override
    public boolean get_isTargetInFOV() {
        return _inFov;
    }

    public double get_angle1InDegrees(){
        return _angle1InDegrees;
    }

    public boolean get_isSocketConnected(){
        return _isSocketConnected;
    }

    public double get_time(){
        return _time;
    }

    public boolean get_isVisionThreadRunning(){
        return _isVisionThreadRunning;
    }

   
    @Override
    public double get_distanceToTargetInInches() {
        return _distanceInInches;
    }

    @Override
    public void updateLogData(LogDataBE logData) {

    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Vision:CameraType", "IPhone");
        SmartDashboard.putBoolean("Vision:IsTargetInFOV", get_isTargetInFOV());
        SmartDashboard.putBoolean("isSocketConnected", get_isSocketConnected());
        SmartDashboard.putNumber("Socket:Message Time(msec)", _timeElapsed / 1000000);
        SmartDashboard.putBoolean("VisionIP:isInFovRunning", get_isTargetInFOV());
        SmartDashboard.putNumber("VisionIP:Angle1InDegrees", get_angle1InDegrees());
        SmartDashboard.putNumber("VisionIP:DistanceInInches", get_distanceToTargetInInches());
        SmartDashboard.putNumber("VisionIP:time", get_time());
        SmartDashboard.putBoolean("VisionIP:IsVisionThreadRunning", get_isVisionThreadRunning());
    }

    
}

