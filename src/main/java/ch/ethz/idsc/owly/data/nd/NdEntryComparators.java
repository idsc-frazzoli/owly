// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.util.Comparator;

import ch.ethz.idsc.tensor.Scalars;

enum NdEntryComparators {
  ;
  public static final Comparator<NdEntry<?>> INCREASING = new Comparator<NdEntry<?>>() {
    @Override
    public int compare(NdEntry<?> o1, NdEntry<?> o2) {
      return Scalars.compare(o1.distanceToCenter, o2.distanceToCenter);
    }
  };
  public static final Comparator<NdEntry<?>> DECREASING = new Comparator<NdEntry<?>>() {
    @Override
    public int compare(NdEntry<?> o1, NdEntry<?> o2) {
      return Scalars.compare(o2.distanceToCenter, o1.distanceToCenter);
    }
  };
}
