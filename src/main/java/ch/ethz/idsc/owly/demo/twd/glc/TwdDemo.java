// code by jl
package ch.ethz.idsc.owly.demo.twd.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.twd.TwdControls;
import ch.ethz.idsc.owly.demo.twd.TwdStateSpaceModel;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum TwdDemo {
  ;
  public static void main(String[] args) {
    RationalScalar resolution = (RationalScalar) RealScalar.of(4);
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(5);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar wheelDistance = RealScalar.of(0.2);
    Scalar wheelRadius = RealScalar.of(0.05);
    Tensor wheelspeeds_max = Tensors.vector(3, 3).multiply(RealScalar.of(2 * Math.PI));
    TwdStateSpaceModel stateSpaceModel = new TwdStateSpaceModel(wheelRadius, wheelDistance, wheelspeeds_max);
    Parameters parameters = new TwdParameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    Collection<Flow> controls = TwdControls.createControls1(//
        stateSpaceModel, wheelspeeds_max, parameters.getResolutionInt());
    // --
    // TODO JONAS definiere hier schritt fuer schritt fuer eine demo des TWD
    // TODO JONAS experimentiere mit 2 kostenfunktionen
  }
}
