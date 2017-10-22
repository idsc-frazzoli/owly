// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;

import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensors;

public abstract class Se2CarDemo implements DemoInterface {
  static TrajectoryRegionQuery createCarQuery(Region region) {
    Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(.2, .1, 0, -.1), region);
    return new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(se2PointsVsRegion));
  }

  abstract void configure(OwlyAnimationFrame owlyAnimationFrame) throws IOException;

  @Override
  public final void start() {
    try {
      OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
      configure(owlyAnimationFrame);
      owlyAnimationFrame.configCoordinateOffset(50, 700);
      // owlyAnimationFrame.treeRender = null;
      owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
      owlyAnimationFrame.jFrame.setVisible(true);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }
}
