package com.MAutils.Auto.Planner;

import edu.wpi.first.math.geometry.Translation2d;

/** Geometry/Circle helpers (extracted from your original). */
final class Geometry {
  private Geometry() {}

  // ---------- Types ----------

  static final class IntersectResult { boolean hit; double tHit; }
  static final class TanPair { double plus, minus; TanPair(double p, double m){ plus=p; minus=m; } }

  // ---------- Segment-circle intersection ----------

  /** Segment-circle intersection on P->Q, returns nearest t in [0,1] if any. */
  public static IntersectResult segmentCircleIntersect(Translation2d P, Translation2d Q, Translation2d C, double R) {
    IntersectResult out = new IntersectResult();
    double dx = Q.getX() - P.getX();
    double dy = Q.getY() - P.getY();
    double fx = P.getX() - C.getX();
    double fy = P.getY() - C.getY();

    double a = dx*dx + dy*dy;
    double b = 2*(fx*dx + fy*dy);
    double c = fx*fx + fy*fy - R*R;

    double disc = b*b - 4*a*c;
    if (disc < 0) { out.hit = false; return out; }
    double sqrt = Math.sqrt(disc);
    double t1 = (-b - sqrt) / (2*a);
    double t2 = (-b + sqrt) / (2*a);

    double tHit = Double.POSITIVE_INFINITY;
    if (t1 >= 0 && t1 <= 1) tHit = Math.min(tHit, t1);
    if (t2 >= 0 && t2 <= 1) tHit = Math.min(tHit, t2);

    if (tHit != Double.POSITIVE_INFINITY) { out.hit = true; out.tHit = tHit; }
    else { out.hit = false; }
    return out;
  }

  // ---------- Straight helpers ----------

  public static Translation2d advanceAlongLine(Translation2d S, Translation2d T, double step) {
    double d = S.getDistance(T);
    if (d < 1e-9) return T;
    double t = Math.min(1.0, step / d);
    return new Translation2d(S.getX() + (T.getX() - S.getX()) * t,
                             S.getY() + (T.getY() - S.getY()) * t);
  }

  // ---------- Circle helpers ----------

  /** Tangent angles for P to circle centered at C with radius R. If inside, project radially. */
  public static TanPair tangentAngles(Translation2d P, Translation2d C, double R) {
    double dx = P.getX() - C.getX();
    double dy = P.getY() - C.getY();
    double d2 = dx*dx + dy*dy; double d = Math.sqrt(d2);
    double base = Math.atan2(dy, dx);
    if (d <= R + 1e-6) {
      return new TanPair(base, base);
    }
    double delta = Math.acos(R / d);
    return new TanPair(wrap(base + delta), wrap(base - delta));
  }

  public static double arcLen(double R, double a1, double a2, boolean ccw) {
    double d = angleDiff(a1, a2, ccw);
    return Math.abs(R) * d;
  }

  public static double angleDiff(double a1, double a2, boolean ccw) {
    double twoPi = Math.PI * 2.0;
    double d = (ccw) ? ((a2 - a1) % twoPi) : ((a1 - a2) % twoPi);
    if (d < 0) d += twoPi;
    return d;
  }

  public static Translation2d fromAngle(Translation2d C, double R, double ang) {
    return new Translation2d(C.getX() + R * Math.cos(ang), C.getY() + R * Math.sin(ang));
  }

  public static double wrap(double a) {
    double twoPi = Math.PI*2.0; a %= twoPi; if (a < 0) a += twoPi; return a;
  }
}
