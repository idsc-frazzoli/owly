// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public abstract class Se2CarDemo implements DemoInterface {
  static TrajectoryRegionQuery createCarQuery(Region<Tensor> region) {
    Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(0.2, 0.1, 0, -0.1), region);
    return SimpleTrajectoryRegionQuery.timeInvariant(se2PointsVsRegion);
  }

  abstract void configure(OwlyAnimationFrame owlyAnimationFrame) throws IOException;

  @Override
  public final void start() {
    try {
      OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
      configure(owlyAnimationFrame);
      owlyAnimationFrame.configCoordinateOffset(50, 700);
      owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
      owlyAnimationFrame.jFrame.setVisible(true);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }
}
