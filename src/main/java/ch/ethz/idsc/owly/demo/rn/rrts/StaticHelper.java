// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.rrts.adapter.SampledTransitionRegionQuery;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

/* package */ enum StaticHelper {
  ;
  public static TransitionRegionQuery polygon1() {
    return new SampledTransitionRegionQuery(new SimpleTrajectoryRegionQuery( //
        new TimeInvariantRegion( //
            new PolygonRegion(Tensors.matrix(new Number[][] { //
                { 3, 1 }, //
                { 4, 1 }, //
                { 4, 6 }, //
                { 1, 6 }, //
                { 1, 3 }, //
                { 3, 3 } //
            })))), RealScalar.of(.1));
  }

  public static TransitionRegionQuery noise1() {
    return new SampledTransitionRegionQuery(new SimpleTrajectoryRegionQuery( //
        new TimeInvariantRegion(new R2NoiseRegion(RealScalar.of(.4)))), RealScalar.of(.1));
  }
}
