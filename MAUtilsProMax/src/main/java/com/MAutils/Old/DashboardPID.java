// package com.MAutils.Old;

// import java.util.function.Consumer;
// import java.util.function.Supplier;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;

// public class DashboardPID implements Sendable {
//     /**
//      * A function that accepts (p, i, d).
//      */
//     @FunctionalInterface
//     public interface TriConsumer {
//         void accept(double p, double i, double d);
//     }

//     // backing fields for the three gains
//     private double p = 0;
//     private double i = 0;
//     private double d = 0;

//     private final TriConsumer pidSetter;
//     private final Consumer<Double> targetSetter;
//     private final Supplier<Double> targetSupplier;
//     private final Supplier<Double> positionSupplier;

//     /**
//      * @param pidSetter        called whenever P, I, or D is changed (with the new triple)
//      * @param targetSetter     called whenever the dashboard Target value is edited
//      * @param targetSupplier   returns the current target (so the widget shows the live value)
//      * @param positionSupplier returns your live process variable (e.g. encoder::getDistance)
//      */
//     public DashboardPID(
//         TriConsumer pidSetter,
//         Consumer<Double> targetSetter,
//         Supplier<Double> targetSupplier,
//         Supplier<Double> positionSupplier
//     ) {
//         this.pidSetter       = pidSetter;
//         this.targetSetter    = targetSetter;
//         this.targetSupplier  = targetSupplier;
//         this.positionSupplier = positionSupplier;//TODO: fix this class
//     }

//     @Override
//     public void initSendable(SendableBuilder builder) {
//         builder.setSmartDashboardType("PIDController");

//         // P gain
//         builder.addDoubleProperty(
//             "P",
//             () -> p,
//             newP -> {
//                 p = newP;
//                 pidSetter.accept(p, i, d);
//             }
//         );

//         // I gain
//         builder.addDoubleProperty(
//             "I",
//             () -> i,
//             newI -> {
//                 i = newI;
//                 pidSetter.accept(p, i, d);
//             }
//         );

//         // D gain
//         builder.addDoubleProperty(
//             "D",
//             () -> d,
//             newD -> {
//                 d = newD;
//                 pidSetter.accept(p, i, d);
//             }
//         );

//         // Target position
//         builder.addDoubleProperty(
//             "Target",
//             targetSupplier::get,
//             targetSetter::accept
//         );

//         // Read-only current position
//         builder.addDoubleProperty(
//             "Position",
//             positionSupplier::get,
//             null
//         );
//     }
// }
