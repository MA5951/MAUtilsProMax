
package com.MAutils.Subsystems.SelfTests;

import java.util.function.BooleanSupplier;

/*
 * A time-based test for subsystems.
 */
public class TimeTest {

    public final BooleanSupplier testCondition;
    public final Runnable testAction;
    public final String testName;
    public final Double testTime;
    private static final String TAG = "TimedTest";

    public TimeTest(String testName, BooleanSupplier testCondition, Runnable testAction,  Double testTime) {
        this.testCondition = testCondition;
        this.testAction = testAction;
        this.testName = testName;
        this.testTime = testTime;
    }


}
