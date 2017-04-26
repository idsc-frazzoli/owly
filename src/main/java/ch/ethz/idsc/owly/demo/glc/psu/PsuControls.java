// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Partition;
import ch.ethz.idsc.tensor.alg.Subdivide;

class PsuControls extends Controls {
  public PsuControls(double amplitude, int num) {
    StateSpaceModel stateSpaceModel = new PsuStateSpaceModel();
    for (Tensor u : Partition.of( //
        Subdivide.of(DoubleScalar.of(-amplitude), DoubleScalar.of(amplitude), num), 1))
      add(StateSpaceModels.createFlow(stateSpaceModel, u));
  }
}
