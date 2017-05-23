// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class DeltaStateSpaceModel implements StateSpaceModel {
  private final ImageGradient rot;

  public DeltaStateSpaceModel(ImageGradient rot) {
    this.rot = rot;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    return rot.rotate(x).add(u);
  }

  @Override
  public Scalar getLipschitz() {
    // TODO ???Lipschitz = max(||rotate.rot||) + max (||u||)
    return null; // null since lipschitz is not provided
  }
}
