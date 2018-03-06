// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owl.math.state.StateTime;
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
    // TODO change scaling factor of control to something that smoothly ...
    // ... depends on position and/or heading
    return u.multiply(RealScalar.of(.2));
  }
}
