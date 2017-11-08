// code by jph
package ch.ethz.idsc.owly.demo.rice.glc;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.owly.demo.rice.Rice2Controls;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.glc.R2xTEllipsoidsAnimationDemo;
import ch.ethz.idsc.owly.demo.util.BijectionFamily;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.TranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

public class Rice2dxTImageDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Scalar mu = RealScalar.of(-.5);
    Collection<Flow> controls = Rice2Controls.create2d(mu, 1, 15);
    ImageRegion imageRegion = R2ImageRegions.inside_265b();
    // TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    AbstractEntity abstractEntity = new Rice2dEntity(mu, Tensors.vector(0, 0, 0, 0), controls);
    owlyAnimationFrame.set(abstractEntity);
    // owlyAnimationFrame.setObstacleQuery(EmptyTrajectoryRegionQuery.INSTANCE);
    // owlyAnimationFrame.setObstacleQuery(obstacleQuery);
    // owlyAnimationFrame.addRegionRender(imageRegion);
    // Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(.2, .1, 0, -.1), imageRegion);
    // ---
    ScalarTensorFunction stf1 = R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(4, 2), 0.03, 2.5);
    BijectionFamily noise1 = new TranslationFamily(stf1);
    Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(0.6, 0.8), noise1, () -> abstractEntity.getStateTimeNow().time());
    ScalarTensorFunction stf2 = R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(1, 3), 0.03, 2.5);
    BijectionFamily noise2 = new TranslationFamily(stf2);
    Region<StateTime> region2 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(0.8, 0.6), noise2, () -> abstractEntity.getStateTimeNow().time());
    TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList( //
            // new TimeInvariantRegion(imageRegion), //
            region1, region2 //
        )));
    // abstractEntity.obstacleQuery = trq;
    owlyAnimationFrame.setObstacleQuery(trq);
    // owlyAnimationFrame.addRegionRender(imageRegion);
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.addBackground((RenderInterface) region2);
    // ---
    owlyAnimationFrame.configCoordinateOffset(350, 350);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new Rice2dxTImageDemo().start();
  }
}
