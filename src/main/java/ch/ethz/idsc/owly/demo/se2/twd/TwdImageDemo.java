// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.CameraEmulator;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class TwdImageDemo implements DemoInterface {
  @Override
  public void start() {
    ImageRegion imageRegion = R2ImageRegions.inside_0f5c_2182();
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    TwdEntity twdEntity = TwdEntity.createDefault(Tensors.vector(7, 5, 0));
    owlyAnimationFrame.set(twdEntity);
    owlyAnimationFrame.setObstacleQuery(obstacleQuery);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    {
      RenderInterface renderInterface = new CameraEmulator( //
          48, RealScalar.of(10), () -> twdEntity.getStateTimeNow(), obstacleQuery);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      RenderInterface renderInterface = new LidarEmulator( //
          129, RealScalar.of(10), () -> twdEntity.getStateTimeNow(), obstacleQuery);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new TwdImageDemo().start();
  }
}
