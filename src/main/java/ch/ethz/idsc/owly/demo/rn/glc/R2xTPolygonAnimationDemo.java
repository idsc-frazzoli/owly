// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.ExamplePolygons;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.r2.CogPoints;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.se2.BijectionFamily;
import ch.ethz.idsc.owly.math.se2.Se2Family;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class R2xTPolygonAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    AbstractEntity abstractEntity = new R2xTEntity(Tensors.vector(1.2, 2.2), RealScalar.of(1.5));
    owlyAnimationFrame.set(abstractEntity);
    // ---
    BijectionFamily rigid1 = new Se2Family( //
        scalar -> Tensors.of( //
            Cos.FUNCTION.apply(scalar.multiply(RealScalar.of(0.1))).multiply(RealScalar.of(2.0)), //
            Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.1))).multiply(RealScalar.of(2.0)), //
            scalar.multiply(RealScalar.of(0.15))));
    Region<StateTime> region1 = new R2xTPolygonStateTimeRegion( //
        ExamplePolygons.CORNER_CENTERED, rigid1, () -> abstractEntity.getStateTimeNow().time());
    // ---
    BijectionFamily rigid2 = new Se2Family( //
        R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(5, 9, 4), 0.1, 2.0));
    Tensor polygon = CogPoints.of(4, RealScalar.of(1.5), RealScalar.of(0.5));
    Region<StateTime> region2 = new R2xTPolygonStateTimeRegion( //
        polygon, rigid2, () -> abstractEntity.getStateTimeNow().time());
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList(region1, region2))));
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.addBackground((RenderInterface) region2);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new R2xTPolygonAnimationDemo().start();
  }
}
