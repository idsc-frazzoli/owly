// code by jph
package ch.ethz.idsc.owly.data.tree;

import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

public interface NodeCollection<T extends Node> {
  void insert(T node);

  Collection<T> nearTo(Tensor end, int k_nearest);

  Collection<T> nearFrom(Tensor start, int k_nearest);

  int size();
}
