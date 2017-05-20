// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

class DeltaStateSpaceModel implements StateSpaceModel {
  private final ImagePotentialRot rot;

  public DeltaStateSpaceModel(ImagePotentialRot rot) {
    this.rot = rot;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    return rot.at(x).add(u);
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE; // FIXME need max over gradient!
  }
}
