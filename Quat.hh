/*
 * This file is part on a WIP proprietary engine, by Ario Amin. All rights
 * reserved.
 *
 * A quaternion number is represented in the form a+bi+cj+dk, where a, b, c,
 * and d parts are real numbers, and i, j, and k are the basis elements,
 * satisfying the equation: i2 = j2 = k2 = ijk = −1.
 */
#ifndef QUATERNION_HH
#define QUATERNION_HH
#include "Vector.hh"

template<typename T>
class Quat {
  public:
  /**
   * Construct a Quaternion from at most 4 components of type T.
   * Specifying only a != 0 makes the Quaternion a real quat.
   */
  explicit Quat(T xx = 0, T yy = 0, T zz = 0, T ww = 0)
      : x{xx}, y{yy}, z{zz}, w{ww}
  {
  }

  /**
   * Construct a Quaternion from two 3d vectors.
   * @tparam    unitlength, a bool template  to tell if vectors are normed or not
   * @param     vectorA, vectorB
   */
  template<bool unitlength>
  Quat(Vector3 vector_a, Vector3 vector_b)
  {
    if constexpr (unitlength == false) {
      vector_a.normalize();
      vector_b.normalize();
    }

    Vector3 rot_axis = vector_a.cross(vector_b);
    float   rad_angle = vector_a.dot(vector_b);

    if ((rad_angle < 1e-6) && (rad_angle > -(1e-6))) {
      x = y = z = 0.0;
      w = 1.0;
      return;
    }
    Vector4 && temp{rot_axis.x, rot_axis.y, rot_axis.z, 1 + rad_angle};
    temp.normalize();

    x = temp.x;
    y = temp.y;
    z = temp.z;
    w = temp.w;
  }

  /**
   * Construct a Quaternion from two 3d vectors.
   * Does not assume the vector size, evaluates at runtime
   * @param     vectorA, vectorB
   */
  Quat(Vector3 vector_a, Vector3 vector_b)
  {
    if (!vector_a.isNorm())
      vector_a.normalize();
    if (!vector_b.isNorm())
      vector_b.normalize();

    Vector3 rot_axis = vector_b.cross(vector_a);
    float   rad_angle = vector_a.dot(vector_b);

    if ((rad_angle < 1e-6) && (rad_angle > -(1e-6))) {
      x = y = z = 0.0;
      w = 1.0;
      return;
    }

    Vector4 && temp{rot_axis.x, rot_axis.y, rot_axis.z, 1 + rad_angle};
    temp.normalize();

    x = temp.x;
    y = temp.y;
    z = temp.z;
    w = temp.w;
  }

  /**
   * Construct a Quaternion from rotation axis.
   * Assumed initial vectors were normalized
   * @param     rot/axis, rad_angle
   */
  Quat(Vector3 rot_axis, float rad_angle)
  {
    if ((rad_angle < 1e-6) && (rad_angle > -(1e-6))) {
      x = y = z = 0.0;
      w = 1.0;
      return;
    }

    Vector4 && temp{rot_axis.x, rot_axis.y, rot_axis.z, 1 + rad_angle};
    temp.normalize();

    x = temp.x;
    y = temp.y;
    z = temp.z;
    w = temp.w;
  }

  // Actual quat math
  /**
   * The inverse of a quaternion,
   * Qinverse = Qconj / Qmag^2
   */
  Quat
  invQuat()
  {
    auto sq_sum = squaredCompSums();
    return {conjugateQuat / sq_sum * sq_sum};
  }

  /**
   * q* = q4 ,-q1, -q2, -q3
   */
  Quat
  conjugateQuat()
  {
    return Quat{w, -x, -y, -z};
  }

  /*
    (Eq. 1)   v´ = q v q^-1 (where v = [0, v])
    (Eq. 2)   v' = v + 2 * r X (s * v + r X v) / m
    The result, a rotated vector v´, will always have a 0 scalar value for w
    (recall Eq. 1 earlier), so you can omit it from your computations

    You can "vectorize" the quaternion product i.e. rewriting in terms of vector
    operations. Doing that to get something like the Euler-Rodriguez Formula
    (Eq. 3) v' = v + 2 * r X (s * v + r X v) / m
  */
  Vector3
  rotateVector(Vector3 in_vec)
  {
#define MOV(A) std::move(A) // NOLINT
    Vector4 && quatvec4 = vectorizeSelf4d();
    Vector3 && quatvec3{MOV(quatvec4.x), MOV(quatvec4.y), MOV(quatvec4.z)};
    float &&   quatscalar = MOV(quatvec4.w);
    delete quatvec4; // nasty, I know

    float sum_quat = squaredCompSums();
    auto  v_rv = (in_vec * quatscalar) + (quatvec3.cross(in_vec));
    return (in_vec + 2) * (MOV(quatvec3).cross(v_rv) / sum_quat);
    // Returns rotated vector
#undef MOV
  }

  Vector4 &&
  vectorizeSelf4d()
  {
    Vector4 && vectorized{x, y, z, w};
    return std::move(vectorized);
  }

  Vector3 &&
  vectorizeSelf3d()
  {
    Vector3 && vectorized{x, y, z};
    return std::move(vectorized);
  }

  float
  squaredCompSums()
  {
    return (x * x) + (y * y) + (z * z) + (w * w);
  }

  // Quat multiplications, also implemented as operator overrides below
  Quat
  Mul(Quat & a, Quat & b)
  {
    auto a_as_vec3 = a.vectorizeSelf3d();
    auto b_as_vec3 = b.vectorizeSelf3d();
    auto mul3comp =
        ((b_as_vec3 * w) + (a_as_vec3 * b.w) + a_as_vec3.cross(b_as_vec3));
    return Quat{mul3comp.x,
                mul3comp.y,
                mul3comp.z,
                (w * b.w) - a_as_vec3.dot(b_as_vec3)};
  }

  // General static inits
  static Quat
  unitX()
  {
    return Quat<T>(1, 0, 0, 0);
  }
  static Quat
  unitY()
  {
    return Quat<T>(0, 1, 0, 0);
  }
  static Quat
  unitZ()
  {
    return Quat<T>(0, 0, 1, 0);
  }
  static Quat
  unitW()
  {
    return Quat<T>(0, 0, 0, 1);
  }

  // Some general setters
  void
  set(T xin, T yin, T zin, T win)
  {
    x = xin;
    y = yin;
    z = zin;
    w = win;
  }

  template<int SWITCH>
  void
  setUnit()
  {
    x = 0.0F;
    y = 0.0F;
    z = 0.0F;
    w = 0.0F;

    if constexpr (SWITCH == 0) {
      x = 1.0F;
    } else if constexpr (SWITCH == 1) {
      y = 1.0F;
    } else if constexpr (SWITCH == 2) {
      z = 1.0F;
    } else if constexpr (SWITCH == 3) {
      w = 1.0F;
    }
  }

  // Basic operation overrides
  Quat
  operator+(T b) const
  {
    return Quat(x + b, y + b, z + b, w + b);
  }

  Quat
  operator-(T b) const
  {
    return Quat(x - b, y - b, z - b, w - b);
  }

  Quat
  operator*(T b) const
  {
    return Quat(x * b, y * b, z * b, w * b);
  }

  Quat
  operator/(T b) const
  {
    return Quat(x / b, y / b, z / b, w / b);
  }

  Quat
  operator+(const Quat & b) const
  {
    return Quat((x + b.x), (y + b.y), (z + b.z), (w + b.w));
  }

  Quat
  operator-(const Quat & b) const
  {
    return Quat((x - b.x), (y - b.y), (z - b.z), (w - b.w));
  }

  // Multiplied = SaSb - vecA(dot)vecB; Sa*vecB  +  Sb*vecA  +  vecA x vecB;
  Quat
  operator*(const Quat & b) const
  {
    auto a_as_vec3 = vectorizeSelf3d();
    auto b_as_vec3 = b.vectorizeSelf3d();
    auto mul3comp =
        ((b_as_vec3 * w) + (a_as_vec3 * b.w) + a_as_vec3.cross(b_as_vec3));
    return Quat{mul3comp.x,
                mul3comp.y,
                mul3comp.z,
                (w * b.w) - a_as_vec3.dot(b_as_vec3)};
  }

  void
  operator+=(T b)
  {
    x += b;
    y += b;
    z += b;
    w += b;
  }

  void
  operator-=(T b)
  {
    x -= b;
    y -= b;
    z -= b;
    w -= b;
  }

  void
  operator*=(T b)
  {
    x *= b;
    y *= b;
    z *= b;
    w *= b;
  }

  void
  operator/=(T b)
  {
    x /= b;
    y /= b;
    z /= b;
    w /= b;
  }

  void
  operator+=(const Quat & b)
  {
    x += b.x;
    y += b.y;
    z += b.z;
    w += b.w;
  }

  void
  operator-=(const Quat & b)
  {
    x -= b.x;
    y -= b.y;
    z -= b.z;
    w -= b.w;
  }

  // Multiplied = SaSb - vecA(dot)vecB; Sa*vecB  +  Sb*vecA  +  vecA x vecB;
  void
  operator*=(const Quat & b)
  {
    auto a_as_vec3 = vectorizeSelf3d();
    auto b_as_vec3 = b.vectorizeSelf3d();
    auto mul3comp =
        ((b_as_vec3 * w) + (a_as_vec3 * b.w) + a_as_vec3.cross(b_as_vec3));
    *this = Quat{mul3comp.x,
                 mul3comp.y,
                 mul3comp.z,
                 (w * b.w) - a_as_vec3.dot(b_as_vec3)};
  }

  Quat
  operator-() const
  {
    return Quat(-x, -y, -z, -w);
  }

  T w, x, y, z;
};

#endif // QUATERNION_HH