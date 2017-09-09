// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.io.Primitives;
import ch.ethz.idsc.tensor.sca.Decrement;

/** gives manhatten distance */
public class FloodFill2D {
  ;
  public static Tensor of(Set<Tensor> seeds, Scalar ttl, Tensor array, Scalar free) {
    FloodFill2D floodFill = new FloodFill2D(seeds, ttl, array, free);
    return floodFill.array;
  }

  private final List<Integer> dimensions;
  private final Tensor array;
  private final Tensor tensor;
  private final Scalar free;
  private Set<Tensor> next;

  private FloodFill2D(Set<Tensor> prev, Scalar ttl, Tensor tensor, Scalar free) {
    dimensions = Dimensions.of(tensor);
    this.array = Array.zeros(dimensions);
    this.tensor = tensor;
    this.free = free;
    {
      next = new HashSet<>();
      for (Tensor seed : prev)
        populate(seed, ttl);
      prev = next;
    }
    while (!prev.isEmpty()) {
      ttl = Decrement.ONE.apply(ttl);
      if (Scalars.isZero(ttl))
        break;
      next = new HashSet<>();
      for (Tensor seed : prev) {
        populate(seed.add(Tensors.vector(-1, 0)), ttl);
        populate(seed.add(Tensors.vector(+1, 0)), ttl);
        populate(seed.add(Tensors.vector(0, -1)), ttl);
        populate(seed.add(Tensors.vector(0, +1)), ttl);
      }
      prev = next;
    }
  }

  private void populate(Tensor point, Scalar ttl) {
    int[] index = Primitives.toArrayInt(point);
    int x = index[0];
    int y = index[1];
    if (0 <= x && x < dimensions.get(0) && //
        0 <= y && y < dimensions.get(1) && //
        Scalars.isZero(array.Get(x, y)) && //
        tensor.get(x, y).equals(free)) {
      array.set(ttl, x, y);
      next.add(point);
    }
  }
}
