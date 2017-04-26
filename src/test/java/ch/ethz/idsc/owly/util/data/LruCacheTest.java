// code by jph
package ch.ethz.idsc.owly.util.data;

import java.util.Map;

import ch.ethz.idsc.owly.data.LruCache;
import junit.framework.TestCase;

public class LruCacheTest extends TestCase {
  public void testLru1() {
    Map<Integer, String> map = LruCache.create(2);
    map.put(3, "1");
    map.put(4, "2");
    System.out.println(map);
    map.get(3);
    System.out.println(map);
    map.containsKey(4);
    System.out.println("containsKey 4");
    System.out.println(map);
    map.get(3);
    System.out.println(map);
    map.put(4, "0");
    System.out.println(map);
  }
}
