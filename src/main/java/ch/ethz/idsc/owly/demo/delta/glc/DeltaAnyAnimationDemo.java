// code by jph
//TODO MODIFY
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;

public class DeltaAnyAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    Tensor image = ResourceData.of("/io/delta_uxy.png");
    Tensor range = Tensors.vector(12.6, 9.1).unmodifiable(); // overall size of map
    Scalar amp = RealScalar.of(-.05); // direction and strength of river flow
    int resolution = 9;
    List<Tensor> obstacleInitList = new ArrayList<>();
    obstacleInitList.add(Tensors.vector(2.0, 1.5, 0.4));
    obstacleInitList.add(Tensors.vector(6.0, 6.0, 0.5));
    obstacleInitList.add(Tensors.vector(2.0, 7.0, 0.3));
    obstacleInitList.add(Tensors.vector(1.0, 8.0, 0.3));
    // ---
    DeltaAnyEntity entity = new DeltaAnyEntity(obstacleInitList, new StateTime(Tensors.vector(10, 3.5), RealScalar.ZERO), resolution);
    // ---
    ImageGradient imageGradient_quick = ImageGradient.nearest(image, range, amp);
    StateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(imageGradient_quick);
    // ---
    Tensor obstacleImage = DeltaAnyEntity.image;
    ImageRegion imageRegion = new ImageRegion(obstacleImage, range, true);
    // ---
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(entity);
    owlyAnimationFrame.setObstacleQuery(entity.getTrajectoryRegionQuery());
    owlyAnimationFrame.addBackground(RegionRenders.create(imageRegion));
    for (int i = 0; i < obstacleInitList.size(); i++) {
      owlyAnimationFrame.addBackground((RenderInterface) entity.getFloatingObstacle(i));
    }
    owlyAnimationFrame.addBackground(DeltaHelper.vectorFieldRender(stateSpaceModel, range, imageRegion, RealScalar.of(0.5)));
    owlyAnimationFrame.jFrame.setVisible(true);
    owlyAnimationFrame.configCoordinateOffset(50, 600);
  }

  public static void main(String[] args) throws Exception {
    new DeltaAnyAnimationDemo().start();
  }
}
