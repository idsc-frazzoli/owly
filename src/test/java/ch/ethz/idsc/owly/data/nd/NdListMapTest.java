// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.BernoulliDistribution;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.pdf.UniformDistribution;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class NdListMapTest extends TestCase {
  public void testSimple() {
    NdListMap<String> m1 = new NdListMap<>();
    m1.add(Tensors.vector(1, 0), "p2");
    m1.add(Tensors.vector(1, 5), "p4");
    m1.add(Tensors.vector(0, 0), "p1");
    m1.add(Tensors.vector(1, 1), "p3");
    NdCluster<String> cl = m1.buildCluster(Tensors.vector(0, 0), 10, NdDistanceInterface.EUCLIDEAN);
    // for (cl.iterator()
    List<String> res = cl.stream().map(NdEntry::value).collect(Collectors.toList());
    assertEquals(res, Arrays.asList("p1", "p2", "p3", "p4"));
  }

  private static Scalar addDistances(NdCluster<String> cluster, Tensor center, NdDistanceInterface d) {
    Scalar sum = RealScalar.ZERO;
    for (NdEntry<String> entry : cluster) {
      Scalar dist = d.apply(center, entry.location());
      assertEquals(entry.distanceToCenter, dist);
      sum = sum.add(dist);
    }
    return sum;
  }

  private static void _checkCenter(Tensor center, int n) {
    NdListMap<String> m1 = new NdListMap<>();
    NdTreeMap<String> m2 = new NdTreeMap<>(Tensors.vector(-2, -1), Tensors.vector(2, 10), 3, 10);
    int index = 0;
    Distribution b = BernoulliDistribution.of(RealScalar.of(.25));
    Distribution ux = UniformDistribution.of(-2, 2);
    Distribution uy = UniformDistribution.of(-1, 10);
    for (int c = 0; c < 20; ++c) {
      Tensor location = Tensors.of(RandomVariate.of(ux), RandomVariate.of(uy));
      String value = "p" + (++index);
      m1.add(location, value);
      m2.add(location, value);
      while (Scalars.isZero(RandomVariate.of(b))) {
        value = "p" + (++index);
        m1.add(location, value);
        m2.add(location, value);
      }
    }
    // System.out.println(m1.size());
    assertEquals(m1.size(), m2.size());
    NdCluster<String> c1 = m1.buildCluster(center, n, NdDistanceInterface.EUCLIDEAN);
    NdCluster<String> c2 = m2.buildCluster(center, n, NdDistanceInterface.EUCLIDEAN);
    assertEquals(c1.size(), c2.size());
    assertTrue(c1.size() <= n);
    Scalar s1 = addDistances(c1, center, NdDistanceInterface.EUCLIDEAN);
    Scalar s2 = addDistances(c2, center, NdDistanceInterface.EUCLIDEAN);
    if (!Chop._11.close(s1, s2)) {
      System.out.println(s1);
      System.out.println(s2);
    }
    assertTrue(Chop._11.close(s1, s2));
  }

  public void testOne() {
    _checkCenter(Tensors.vector(.3, .3), 1);
    _checkCenter(Tensors.vector(.1, .3), 1);
    _checkCenter(Tensors.vector(5, 4.3), 1);
    _checkCenter(Tensors.vector(5, -3.3), 1);
  }

  public void testFew() {
    _checkCenter(Tensors.vector(.3, .3), 3);
    _checkCenter(Tensors.vector(.1, .3), 3);
    _checkCenter(Tensors.vector(5, 4.3), 3);
    _checkCenter(Tensors.vector(5, -3.3), 3);
  }

  public void testMany() {
    _checkCenter(Tensors.vector(.3, .3), 20);
    _checkCenter(Tensors.vector(.1, .3), 20);
    _checkCenter(Tensors.vector(5, 4.3), 20);
    _checkCenter(Tensors.vector(5, -3.3), 20);
  }

  public void testMost() {
    _checkCenter(Tensors.vector(.3, .3), 60);
    _checkCenter(Tensors.vector(.1, .3), 60);
    _checkCenter(Tensors.vector(5, 4.3), 60);
    _checkCenter(Tensors.vector(5, -3.3), 60);
  }

  public void testAll() {
    _checkCenter(Tensors.vector(.3, .3), 160);
    _checkCenter(Tensors.vector(.1, .3), 160);
    _checkCenter(Tensors.vector(5, 4.3), 160);
    _checkCenter(Tensors.vector(5, -3.3), 160);
  }
}
