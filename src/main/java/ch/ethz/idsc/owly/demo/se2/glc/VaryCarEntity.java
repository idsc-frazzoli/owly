// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.List;

import ch.ethz.idsc.owl.math.planar.SignedCurvature2D;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Round;

class VaryCarEntity extends CarEntity {
  VaryCarEntity(StateTime stateTime) {
    super(stateTime);
    timeStep = FIXEDSTATEINTEGRATOR.getTimeStepTrajectory();
  }

  private final Scalar timeStep;

  @Override
  public Tensor realisticControl(Tensor u) {
    StateTime currentPose = getStateTimeNow();
    System.out.println("pose = " + currentPose.toCompactString());
    System.out.println("ctrl = " + u.map(Round._4));
    //
    // use getFutureTrajectoryUntil to calculate upcoming path radius
    List<TrajectorySample> traj = getFutureTrajectoryUntil(timeStep);
    double scalingFactor = 0.2;
    if (traj.size() >= 3) {
      Tensor p1 = traj.get(0).stateTime().state().extract(0, 2);
      Tensor p2 = traj.get(1).stateTime().state().extract(0, 2);
      Tensor p3 = traj.get(2).stateTime().state().extract(0, 2);
      //
      Scalar invRad = SignedCurvature2D.of(p1, p2, p3).abs();
      Scalar radius = RealScalar.ONE.divide(invRad);
      System.out.println("radius = " + radius);
      scalingFactor = Math.min(0.5 * Math.sqrt(radius.number().doubleValue()), 1);
    }
    System.out.println("scaling = " + scalingFactor);
    return u.multiply(RealScalar.of(scalingFactor));
  }
}
