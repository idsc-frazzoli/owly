// code by jl
package ch.ethz.idsc.owly.demo.se2.twd;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.sca.N;

/** two wheel drive embedded as robot in se2 with capability to turn on the spot
 * Example: duckie bots
 * 
 * ^ +y
 * |
 * WL speedL
 * --
 * |
 * |
 * |------> +x
 * |
 * |
 * --
 * WR speedR */
public class TwdConfig {
  private final Scalar maxSpeed;
  private final Scalar halfWidth;

  /** @param maxSpeed [m*s^-1]
   * @param halfWidth [m*rad^-1] */
  public TwdConfig(Scalar maxSpeed, Scalar halfWidth) {
    this.maxSpeed = maxSpeed;
    this.halfWidth = halfWidth;
  }

  /** @param speedL
   * @param speedR
   * @return */
  public Flow singleton(Scalar speedL, Scalar speedR) {
    Scalar speed = speedL.add(speedR).multiply(maxSpeed).divide(RealScalar.of(2));
    Scalar rate = speedL.subtract(speedR).multiply(maxSpeed).divide(RealScalar.of(2)).divide(halfWidth);
    return StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, //
        N.DOUBLE.of(Tensors.of(speed, RealScalar.ZERO, rate)));
  }

  /** @param twdStateSpaceModel
   * @param resolution
   * @return collection of flows with size == 4 * resolution */
  public Collection<Flow> createControls(int resolution) {
    List<Flow> list = new ArrayList<>();
    Tensor range = Subdivide.of(-1, 1, resolution).extract(0, resolution); // [-1, ..., 1)
    for (Tensor _omega : range) {
      Scalar omega = _omega.Get();
      list.add(singleton(RealScalar.ONE, omega));
      list.add(singleton(omega.negate(), RealScalar.ONE));
      list.add(singleton(RealScalar.ONE.negate(), omega.negate()));
      list.add(singleton(omega, RealScalar.ONE.negate()));
    }
    return list;
  }

  // TODO document what this does
  public Collection<Flow> createControls2(int num) {
    int numSqr = num;
    Scalar wheelspeed_max = RealScalar.ONE;
    List<Flow> list = new ArrayList<>();
    Tensor wlList = Subdivide.of(wheelspeed_max.negate(), wheelspeed_max, numSqr);
    Scalar stepSize = wlList.Get(1).subtract(wlList.Get(0));
    for (Tensor _wl : wlList) {
      // |wl|+|wr|<=1
      Scalar wl = _wl.Get();
      Scalar wr = (wheelspeed_max.subtract(wl)).negate();
      while (Scalars.lessEquals(wr, wheelspeed_max.subtract(wl))) {
        list.add(singleton(wl, wr));
        wr = wr.add(stepSize);
      }
    }
    return list;
  }
}
