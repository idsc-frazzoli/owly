package ch.ethz.idsc.owl.mapping;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.image.BufferedImage;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.Timer;
import java.util.TimerTask;
import java.util.stream.Collectors;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.data.nd.NdCenterInterface;
import ch.ethz.idsc.owl.data.nd.NdCluster;
import ch.ethz.idsc.owl.data.nd.NdEntry;
import ch.ethz.idsc.owl.data.nd.NdTreeMap;
import ch.ethz.idsc.owl.gui.AffineTransforms;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Mod;

public class OccupancyMap2d implements RenderInterface {
  final static int maxDepth = 16;
  final static int maxDensity = 16;
  final private Tensor lbounds;
  final private Tensor ubounds;
  private Timer timer;
  private final int updateRate = 10; // [Hz]
  Scalar gridRes = DoubleScalar.POSITIVE_INFINITY;
  NdTreeMap<Scalar> ndTree;
  //
  BufferedImage bufferedImage;

  public OccupancyMap2d(Tensor lbounds, Tensor ubounds, Scalar gridRes) {
    ndTree = new NdTreeMap<>(lbounds, ubounds, 5, 20); // magic const
    GlobalAssert.that(Scalars.lessThan(RealScalar.ZERO, gridRes));
    this.gridRes = gridRes;
    this.ubounds = ubounds;
    this.lbounds = lbounds;
    //
    int xsize = ((ubounds.Get(0).subtract(lbounds.Get(0))).divide(gridRes)).number().intValue();
    int ysize = ((ubounds.Get(1).subtract(lbounds.Get(1))).divide(gridRes)).number().intValue();
    bufferedImage = new BufferedImage(xsize, ysize, BufferedImage.TYPE_INT_ARGB);
    Graphics graphics = bufferedImage.getGraphics();
    graphics.setColor(new Color(0f, 0.5f, 0.1f, 0.1f));
    graphics.fillRect(0, 0, bufferedImage.getWidth(), bufferedImage.getHeight());
  }

  private Scalar roundUp(Scalar i, Scalar n) {
    return i.add(n).subtract(Mod.function(n).apply(i));
  }

  public Tensor toTile(Tensor pos) {
    // get x index
    Scalar xShift = pos.Get(0).subtract(lbounds.get(0));
    Scalar xIdx = roundUp(xShift, gridRes).divide(gridRes).subtract(RealScalar.ONE);
    // get y index
    Scalar yShift = pos.Get(1).subtract(lbounds.get(1));
    Scalar yIdx = roundUp(yShift, gridRes).divide(gridRes).subtract(RealScalar.ONE);
    return Tensors.of(xIdx, yIdx);
  }

  public Tensor toTileCoordinates(Tensor tile) {
    Scalar s = gridRes.divide(RealScalar.of(2));
    return tile.multiply(gridRes).add(Tensors.of(s, s)).add(lbounds);
  }

  public boolean insert(Tensor pos) {
    // get tile corresponding to position
    Tensor tile = toTile(pos);
    // check if occupied
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(toTile(pos));
    NdCluster<Scalar> cluster = ndTree.buildCluster(distanceInterface, 1);
    List<Scalar> a = cluster.stream().map(NdEntry::distance).collect(Collectors.toList());
    if (!a.contains(RealScalar.ZERO)) {
      ndTree.add(tile, RealScalar.of(1));
      // rendering
      int xvalue = tile.Get(0).number().intValue();
      int yvalue = bufferedImage.getHeight() - tile.Get(1).number().intValue() - 1;
      bufferedImage.setRGB(xvalue, yvalue, 0x7FFF0000);
      return true;
    }
    return false;
  }

  public boolean remove(Tensor pos) {
    // TODO: build this method
    return false;
  }

  // calculates l2 distance between center of respective query tile and closest obstacle tile
  public Scalar getL2DistToClosest(Tensor pos) {
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(toTile(pos));
    NdCluster<Scalar> cluster = ndTree.buildCluster(distanceInterface, 1);
    Optional<NdEntry<Scalar>> closest = cluster.stream().findFirst();
    if (closest.isPresent()) {
      return closest.get().distance().multiply(gridRes);
    }
    return DoubleScalar.POSITIVE_INFINITY;
  }

  public Collection<Scalar> getKNearest(Tensor pos, int k_nearest) {
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(pos);
    NdCluster<Scalar> cluster = ndTree.buildCluster(distanceInterface, k_nearest);
    // System.out.println("considered " + cluster.considered() + " " + ndMap.size());
    return cluster.stream().map(NdEntry::value).collect(Collectors.toList());
  }

  public OccupancyMap2d getCopy() throws CloneNotSupportedException {
    return (OccupancyMap2d) this.clone();
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    final Tensor matrix = geometricLayer.getMatrix();
    AffineTransform aT = AffineTransforms.toAffineTransform(matrix);
    aT.scale(0.5, 0.5);
    graphics.drawImage(bufferedImage, aT, null);
  }
}
