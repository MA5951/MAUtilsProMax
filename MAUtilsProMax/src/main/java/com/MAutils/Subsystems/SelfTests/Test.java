
package com.MAutils.Subsystems.SelfTests;

import java.util.function.BooleanSupplier;

/*
 * A general test for subsystems.
 */
public class Test {

    public final BooleanSupplier testCondition;
    public final Runnable testAction;
    public final String testName;
    public final Double testTimeCap;
    public final String TAG = "Test";

    public Test(String testName, BooleanSupplier testCondition, Runnable testAction,  Double testTimeCap) {
        this.testCondition = testCondition;
        this.testAction = testAction;
        this.testName = testName;
        this.testTimeCap = testTimeCap;
    }


}
