// code by ynager
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.List;

import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Round;

class VaryCarEntity extends CarEntity {
  VaryCarEntity(StateTime stateTime) {
    super(stateTime);
  }

  @Override
  public Tensor realisticControl(Tensor u) {
    StateTime currentPose = getStateTimeNow();
    System.out.println("pose = " + currentPose.toCompactString());
    System.out.println("ctrl = " + u.map(Round._4));
    Scalar px = currentPose.state().Get(0); // x coordinate of car
    double px_d = px.number().doubleValue();
    // currentPose.state().Get(1); // y coordinate of car
    // currentPose.state().Get(2); // theta heading of car
    //
    // use getFutureTrajectoryUntil to calculate upcoming path radius
    List<TrajectorySample> traj = getFutureTrajectoryUntil(RealScalar.of(.1));
    // estimate turning radius using finite difference
    // radius = d / (2*sin(dphi)), d = dx²+dy², dphi = abs(phi_1-phi_2)
    // TODO use SignedCurvature2D
    double scalingFactor = 0.1;
    if (traj.size() >= 2) {
      double dx = traj.get(0).stateTime().state().Get(0).subtract(traj.get(1) //
          .stateTime().state().Get(0)).number().doubleValue();
      double dy = traj.get(0).stateTime().state().Get(1).subtract(traj.get(1)//
          .stateTime().state().Get(1)).number().doubleValue();
      double d = Math.sqrt(dx * dx + dy * dy);
      double dphi = traj.get(0).stateTime().state().Get(2).subtract(traj.get(1)//
          .stateTime().state().Get(2)).abs().number().doubleValue();
      // FIXME potential division by zero
      double radius = d / (2 * Math.sin(dphi));
      System.out.println("radius = " + radius);
      // calculate velocity scaling factor by considering the centripetal force
      // F = mv²/radius -> v = sqrt(F*radius/m)
      // TODO select F/m to make physical sense, for now just a scale factor of 0.5 is used
      scalingFactor = Math.min(0.5 * Math.sqrt(radius), 1);
    }
    //
    System.out.println("scaling = " + scalingFactor);
    return u.multiply(RealScalar.of(scalingFactor));
  }
}
