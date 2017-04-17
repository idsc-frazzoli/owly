// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.uni.adapter.PsuStateSpaceModel;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Partition;
import ch.ethz.idsc.tensor.alg.Subdivide;

enum PsuControls {
  ;
  public static Controls createControls(int num) {
    StateSpaceModel stateSpaceModel = new PsuStateSpaceModel();
    Controls controls = new Controls();
    for (Tensor u : Partition.of(Subdivide.of(DoubleScalar.of(-0.2), DoubleScalar.of(0.2), num), 1))
      controls.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    return controls;
  }
}
