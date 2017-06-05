// code by jph
package ch.ethz.idsc.owly.demo.glc.tn;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

public class TnWrap {
  private static final Scalar NEGATIVE_HALF = RationalScalar.of(-1, 2);
  // ---
  private final Tensor wrap;
  private final List<Mod> mod_distance = new ArrayList<>();

  public TnWrap(Tensor wrap) {
    this.wrap = wrap;
    for (Tensor _n : wrap) {
      Scalar n = (Scalar) _n;
      mod_distance.add(Mod.function(n, n.multiply(NEGATIVE_HALF)));
    }
  }

  public Tensor represent(Tensor x) {
    return Tensors.vector(i -> Mod.function(wrap.Get(i)).apply(x.Get(i)), x.length());
  }

  public Scalar distance(Tensor p, Tensor q) {
    Tensor d = p.subtract(q);
    Tensor m = Tensors.vector(i -> mod_distance.get(i).apply(d.Get(i)), d.length());
    return Norm._2.of(m);
  }
}
