// code by stegu
package ch.ethz.idsc.owly.math.noise;

enum Floor {
  ;
  /** method is faster than (int)Math.floor(x)
   * 
   * @param x
   * @return */
  public static int of(double x) {
    int xi = (int) x;
    return x < xi ? xi - 1 : xi;
  }
}
