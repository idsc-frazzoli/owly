package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.model.car.CarState;
import ch.ethz.idsc.owly.model.car.CarStateSpaceModel;
import ch.ethz.idsc.owly.model.car.CarStatic;
import ch.ethz.idsc.owly.model.car.HomogenousTrack;
import ch.ethz.idsc.owly.model.car.VehicleModel;
import ch.ethz.idsc.owly.model.car.shop.RimoSinusIonModel;
import ch.ethz.idsc.tensor.RealScalar;

public class CarDemo {
  // @Override
  public void start() {
    VehicleModel vehicleModel = RimoSinusIonModel.standard();
    CarStateSpaceModel carStateSpaceModel = new CarStateSpaceModel(vehicleModel, HomogenousTrack.DRY_ROAD);
    CarState carState = CarStatic.x0_demo1(vehicleModel);
    CarEntity carEntity = new CarEntity(carStateSpaceModel, carState);
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    owlyAnimationFrame.set(carEntity);
    Region region = new R2NoiseRegion(RealScalar.of(0.2));
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region)));
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new CarDemo().start();
  }
}
