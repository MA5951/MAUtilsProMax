
package com.MAutils.Utils;


public class GainConfig {

    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;
    public double Kf = 0;
    public double Ks = 0;
    public double Kv = 0;
    public double Ka = 0; 

    public GainConfig() { //TODO change to private
        

    }
    //TODO add a factory func that init the kp and KF 

    public GainConfig withKP(double Kp) {
        this.Kp = Kp;
        return this;
    }
    
    public GainConfig withKI(double Ki) {
        this.Ki = Ki;
        return this;
    }

    public GainConfig withKD(double Kd) {
        this.Kd = Kd;
        return this;
    }

    public GainConfig withKF(double Kf) {
        this.Kf = Kf;
        return this;
    }

    public GainConfig withKS(double Ks) {
        this.Ks = Ks;
        return this;
    }

    public GainConfig withKV(double Kv) {
        this.Kv = Kv;
        return this;
    }

    public GainConfig withKA(double Ka) {
        this.Ka = Ka;
        return this;
    }
}
