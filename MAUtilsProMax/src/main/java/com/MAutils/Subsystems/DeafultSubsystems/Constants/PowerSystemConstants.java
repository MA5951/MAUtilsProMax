package com.MAutils.Subsystems.DeafultSubsystems.Constants;

import com.MAutils.Components.Motor;

public class PowerSystemConstants extends DeafultSystemConstants<PowerSystemConstants> {

    public PowerSystemConstants(DeafultSystemConstants.Builder b) {
        super(b);
    }

    // This is fine: same signature + same return type as base method.
    public static DeafultSystemConstants.Builder builder(String name, Motor master, Motor... motors) {
        return DeafultSystemConstants.builder(name, master, motors);
    }

    public static PowerSystemConstants build(String name, Motor master, Motor... motors) {
        return DeafultSystemConstants.builder(name, master, motors).build(PowerSystemConstants::new);
    }
}
