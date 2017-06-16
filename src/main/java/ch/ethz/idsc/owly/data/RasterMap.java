// code by jph
package ch.ethz.idsc.owly.data;

import java.io.Serializable;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.tensor.Tensor;

public abstract class RasterMap<T> implements Serializable {
  private final Map<Tensor, T> map = new HashMap<>(); // actual storage

  public abstract Tensor toKey(Tensor tensor);

  public void put(Tensor tensor, T value) {
    putWithKey(toKey(tensor), value);
  }

  public synchronized void putWithKey(Tensor key, T value) {
    map.put(key, value);
  }

  // public T get(Tensor tensor) {
  // return getWithKey(toKey(tensor));
  // }
  public synchronized T getWithKey(Tensor key) {
    return map.get(key);
  }

  public Collection<T> values() {
    return map.values();
  }
}
