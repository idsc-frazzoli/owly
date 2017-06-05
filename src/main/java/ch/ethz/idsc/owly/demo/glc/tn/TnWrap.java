// code by jph
package ch.ethz.idsc.owly.demo.glc.tn;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

public class TnWrap implements CoordinateWrap {
  private static final Scalar NEGATIVE_HALF = RationalScalar.of(-1, 2);
  // ---
  private final Tensor extension;
  private final List<Mod> mod_distance = new ArrayList<>();

  /** @param extension of torus along each axis */
  public TnWrap(Tensor extension) {
    this.extension = extension;
    for (Tensor _n : extension) {
      Scalar n = (Scalar) _n;
      mod_distance.add(Mod.function(n, n.multiply(NEGATIVE_HALF)));
    }
  }

  @Override
  public Tensor represent(Tensor x) {
    return Tensors.vector(i -> Mod.function(extension.Get(i)).apply(x.Get(i)), x.length());
  }

  @Override
  public Scalar distance(Tensor p, Tensor q) {
    Tensor d = p.subtract(q);
    Tensor m = Tensors.vector(i -> mod_distance.get(i).apply(d.Get(i)), d.length());
    return Norm._2.of(m);
  }
}
