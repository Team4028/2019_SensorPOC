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
 * The IPHone exposes a TCP Socket Server that allows us to query for data
 * We poll the IPHONE asychronously to the main robot thread because of the 20 MSec limit
 * 
 * Lead Student: Izzy
 */
public class VisionIP implements IVisionSensor {

    private static final int RETRY_THREAD_LOOPS = 30;
    private static final int THREAD_SLEEP_TIME_IN_MSEC = 2000;

    private Socket _clientSocket;
    private PrintWriter _outBuffer;
    private BufferedReader _inBuffer;
    private boolean _isTargetInFOV;
    private double _distanceInInches;
    private double _angle1InDegrees;
    private boolean _isSocketConnected;
    private double _sensorTime;
    private long _timeElapsed;
    private boolean _isVisionThreadRunning;
    private int i = 1;

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

    private void startThread() {
        _isVisionThreadRunning = false;

        Thread t = new Thread(() -> {
            // try to open a connection to a remote socket server, retry if not available
            while (i <= RETRY_THREAD_LOOPS && !_isSocketConnected) {
                openConnection(RobotMap.SOCKET_CLIENT_CONNECTION_IPADRESS, RobotMap.SOCKET_CLIENT_CONNECTION_PORT);
                if (!_isSocketConnected) { 
                    i++;
                    try {
                        Thread.sleep(THREAD_SLEEP_TIME_IN_MSEC);
                        System.out .println("sleeping loop: " + i);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }

            // main loop to poll socket server
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
                        _isTargetInFOV = Boolean.parseBoolean(value);
                     
                    } else if (key.equals("angle1")) {
                        _angle1InDegrees = Double.parseDouble(value);
                    } else if (key.equals("distance")) {
                        _distanceInInches = Double.parseDouble(value);
                        
                    }else if (key.equals("time")){
                        _sensorTime = Double.parseDouble(value);
                    }
                }
            }
        });t.start();
    }

    private void openConnection(String ipAddress, int port) {
        try {
            _clientSocket = new Socket();
            _clientSocket.connect(new InetSocketAddress(ipAddress, port), 500);
            _outBuffer = new PrintWriter(_clientSocket.getOutputStream(), true);
            _inBuffer = new BufferedReader(new InputStreamReader(_clientSocket.getInputStream()));
            _isSocketConnected = true;
        } catch (IOException e) {
            _isSocketConnected = false;
        }
    }

    private String sendMessage(String msg) {
        _outBuffer.println(msg);
        String _resp = "";
        try {
            _resp = _inBuffer.readLine();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println(e.toString());
        }
        return _resp;
    }

    public void stopConnection() {
        try {
            _inBuffer.close();
        } catch (IOException e1) {
        // TODO Auto-generated catch block
        e1.printStackTrace();
        }
        _outBuffer.close();
        try {
        _clientSocket.close();
        } catch (IOException e) {
      // TODO Auto-generated catch block
        e.printStackTrace();
        }
    }

    // ====================================================================
    // Property Accessors
    // ====================================================================
    @Override
    public boolean get_isTargetInFOV() {
        return _isTargetInFOV;
    }

    @Override
    public double get_distanceToTargetInInches() {
        return _distanceInInches;
    }

    public double get_angle1InDegrees(){
        return _angle1InDegrees;
    }

    private boolean get_isSocketConnected(){
        return _isSocketConnected;
    }

    private double get_sensorTime(){
        return _sensorTime;
    }

    private boolean get_isVisionThreadRunning(){
        return _isVisionThreadRunning;
    }

    // ====================================================================
    // Logging Methods
    // ====================================================================

    @Override
    public void updateLogData(LogDataBE logData) {

    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Vision:CameraType", "IPhone");
        SmartDashboard.putBoolean("Vision:IsTargetInFOV", get_isTargetInFOV());
        SmartDashboard.putNumber("Vision:CycleTimeMSec", _timeElapsed / 1000000);
        SmartDashboard.putNumber("VisionIP:Angle1InDegrees", get_angle1InDegrees());
        SmartDashboard.putNumber("VisionIP:DistanceInInches", get_distanceToTargetInInches());
        SmartDashboard.putNumber("VisionIP:SensorTime", get_sensorTime());
        SmartDashboard.putBoolean("VisionIP:IsVisionThreadRunning", get_isVisionThreadRunning());
        SmartDashboard.putBoolean("VisionIP:isSocketConnected", get_isSocketConnected());
    }
}