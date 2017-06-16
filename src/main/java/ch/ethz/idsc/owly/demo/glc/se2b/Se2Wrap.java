// code by jph
package ch.ethz.idsc.owly.demo.glc.se2b;

import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

/** identifies (x,y,theta) === (x,y,theta + 2 pi n) for all n */
public class Se2Wrap implements CoordinateWrap {
  private static Mod mod = Mod.function(RealScalar.of(Math.PI * 2));
  private static Mod mod_distance = Mod.function(RealScalar.of(Math.PI * 2), RealScalar.of(-Math.PI));
  // ---
  // angular error may need a different "weight" from error in x, y
  private final Tensor scale;

  /** one can choose scale == {1, 1, 1}
   * 
   * if more angular accuracy is required, one can choose, for instance scale == {1, 1, 2}
   * 
   * @param scale weighs the differences in (x,y,theta) */
  public Se2Wrap(Tensor scale) {
    this.scale = scale;
  }

  @Override
  public Tensor represent(Tensor x) {
    Tensor r = x.copy();
    r.set(mod, 2);
    return r;
  }

  @Override
  public Scalar distance(Tensor p, Tensor q) {
    Tensor d = p.subtract(q);
    d.set(mod_distance, 2);
    // TODO inf norm may be more canonic?
    return Norm._2.of(d.pmul(scale));
  }
}
