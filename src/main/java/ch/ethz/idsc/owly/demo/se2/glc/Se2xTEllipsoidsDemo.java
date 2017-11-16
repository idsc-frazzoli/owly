// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;
import java.util.Arrays;

import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.glc.R2xTEllipsoidsAnimationDemo;
import ch.ethz.idsc.owly.demo.util.CameraEmulator;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.demo.util.SimpleTranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.se2.BijectionFamily;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class Se2xTEllipsoidsDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    CarxTEntity carxTEntity = new CarxTEntity(Tensors.vector(0, 0, 1));
    owlyAnimationFrame.set(carxTEntity);
    // ---
    ScalarTensorFunction stf1 = R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(0, 2), 0.05, 2.3);
    BijectionFamily noise1 = new SimpleTranslationFamily(stf1);
    Region<StateTime> region1 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(0.6, 0.8), noise1, () -> carxTEntity.getStateTimeNow().time());
    ScalarTensorFunction stf2 = R2xTEllipsoidsAnimationDemo.wrap1DTensor(SimplexContinuousNoise.FUNCTION, Tensors.vector(1, 3), 0.05, 2.3);
    BijectionFamily noise2 = new SimpleTranslationFamily(stf2);
    Region<StateTime> region2 = new R2xTEllipsoidStateTimeRegion( //
        Tensors.vector(0.8, 0.6), noise2, () -> carxTEntity.getStateTimeNow().time());
    TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery( //
        RegionUnion.wrap(Arrays.asList(region1, region2)));
    carxTEntity.obstacleQuery = trq;
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground((RenderInterface) region1);
    owlyAnimationFrame.addBackground((RenderInterface) region2);
    {
      RenderInterface renderInterface = new CameraEmulator( //
          48, RealScalar.of(10), () -> carxTEntity.getStateTimeNow(), trq);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      RenderInterface renderInterface = new LidarEmulator( //
          129, RealScalar.of(10), () -> carxTEntity.getStateTimeNow(), trq);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    // ---
    owlyAnimationFrame.configCoordinateOffset(350, 350);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) throws IOException {
    new Se2xTEllipsoidsDemo().start();
  }
}
