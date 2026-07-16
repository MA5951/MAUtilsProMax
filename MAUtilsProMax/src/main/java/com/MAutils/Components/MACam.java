package com.MAutils.Components;

import java.util.Timer;
import java.util.TimerTask;

import com.MAutils.Components.MACam.MeasurementStatus;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;



public class MACam {

    public enum MeasurementStatus {
        
        VALID(0),
        FAIL_SigmaFail(1),
        FAIL_SignalFail(2),
        FAIL_RangeValidMinRangeClipped(3),
        FAIL_OutOfBoundsFail(4),
        FAIL_HardwareFail(5),
        FAIL_RangeValidNoWrapCheckFail(6),
        FAIL_WrapTargetFail(7),
        FAIL_XtalkSignalFail(9),
        FAIL_SynchronizationInt(10),
        FAIL_MinRangeFail(13),
        None(255);

        public final int id;

        private MeasurementStatus(int id) {
            this.id = id;
        }

        public static MeasurementStatus fromId(int id) {
            for (MeasurementStatus status : values()) {
                if (status.id == id) {
                    return status;
                }
            }
            return None;
        }
    }

    private static final int BASE_API_ID   = 0x0301; // Sensor 0
    private static final int CONFIG_API_ID = 0x0305;

    private final CAN can;

    private int distance;
    private int mode;
    private int roiCenter;
    private int roiX;
    private int roiY;
    private int status;

    // Desired configuration
    private int desiredMode      = 1;  
    private int desiredRoiCenter = 199;
    private int desiredRoiX      = 8;
    private int desiredRoiY      = 8;

    private long lastConfigSent = 0;

    private final Timer pollTimer;

    public MACam(int id) {
        this.can = new CAN(id); 

        pollTimer = new Timer("ToF-CAN-Poller " + id, true);

        pollTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                long now = System.currentTimeMillis();

                try {
                    CANData frame = new CANData();

                    if (can.readPacketLatest(BASE_API_ID, frame) && frame.length >= 8) {
                        byte[] data = frame.data;

                        distance      = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
                        mode          = data[2] & 0xFF;
                        roiCenter     = data[3] & 0xFF;
                        roiX          = data[4] & 0xFF;
                        roiY          = data[5] & 0xFF;
                        status  = data[6];
                    }

                    // Send config if mismatch
                    if ((mode != desiredMode ||
                         roiCenter != desiredRoiCenter ||
                         roiX != desiredRoiX ||
                         roiY != desiredRoiY) &&
                         (now - lastConfigSent > 200)) {

                        byte[] config = new byte[] {
                            (byte) desiredMode,
                            (byte) desiredRoiCenter,
                            (byte) desiredRoiX,
                            (byte) desiredRoiY
                        };

                        can.writePacket(config, CONFIG_API_ID);
                        lastConfigSent = now;

                        System.out.printf(
                            "[ToFCAN] Sent config | Mode: %d, ROI: %d %dx%d%n",
                            desiredMode, desiredRoiCenter, desiredRoiX, desiredRoiY
                        );
                    }

                } catch (Exception e) {
                    System.err.println("ToFCAN read/send error: " + e.getMessage());
                }
            }
        }, 0, 20); // 20 ms
    }

    // ---------------- Getters ----------------

    public int getDistance() {
        return distance;
    }

    public int getMode() {
        return mode;
    }

    public int getROICenter() {
        return roiCenter;
    }

    public int getROISizeX() {
        return roiX;
    }

    public int getROISizeY() {
        return roiY;
    }

    public MeasurementStatus getStatus() {
        return MeasurementStatus.fromId(status);
    }

    // ---------------- Config ----------------

    public void setDesiredConfig(int mode, int center, int x, int y) {
        this.desiredMode = mode;
        this.desiredRoiCenter = center;
        this.desiredRoiX = x;
        this.desiredRoiY = y;
    }

    public void stop() {
        pollTimer.cancel();
    }
}
