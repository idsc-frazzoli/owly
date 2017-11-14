// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;

public class DeltaAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    // ---
    Scalar amp = RealScalar.of(-.05);
    Tensor range = Tensors.vector(12.6, 9.1).unmodifiable();
    ImageGradient imageGradient = ImageGradient.nearest(ResourceData.of("/io/delta_uxy.png"), range, amp);
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    ImageRegion imageRegion = new ImageRegion(obstacleImage, range, true);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(imageRegion);
    owlyAnimationFrame.set(new DeltaEntity(imageGradient, Tensors.vector(10, 3.5)));
    owlyAnimationFrame.setObstacleQuery(obstacleQuery);
    StateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(imageGradient);
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    owlyAnimationFrame.addBackground(DeltaHelper.vectorFieldRender(stateSpaceModel, range, imageRegion, RealScalar.of(0.5)));
    owlyAnimationFrame.jFrame.setVisible(true);
    owlyAnimationFrame.configCoordinateOffset(50, 600);
  }

  public static void main(String[] args) throws Exception {
    new DeltaAnimationDemo().start();
  }
}
