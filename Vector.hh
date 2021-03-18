/*
 * This file is part on a WIP proprietary engine, by Ario Amin. All rights
 * reserved.
 *
 */
#ifndef VECTOR_HH
#define VECTOR_HH

#include <cassert>
#include <cfloat>
#include <cmath>

class Vector2 {
  public:
  // Components
  float x = 0;
  float y = 0;

  // Constructors
  Vector2() = default;
  explicit Vector2(float b) : x(b), y(b) {}
  explicit Vector2(float x, float y) : x(x), y(y) {}

   // Vector algebra
  /** Vector reflection, much simpler to create my own instead of bloating with
   * reflected = 2projv(u) - u;
   * Where:
   * projv(u) -> ((u| * v|) / (||v||^2)) * v|
   * v| * u|  -> (u1​∗v1​)+(u2​∗v2​)
   * ||v||^2  -> (_/ v.x^2 + v.y^2)^2 == v.x^2 + v.y^2
   * K        -> 2*(((u1​∗v1​)+(u2​∗v2​) / (v.x^2 + v.y^2))* v|);
   *
   * 2 * ((dotp(b,a) /  sqr(sqrt(v|))) * v|)
   * reflected = (K * u) - u
   **/
  [[nodiscard]] float
  dotp(Vector2 vector_b) const
  {
    return (x * vector_b.x) + (y * vector_b.y);
  }

  [[nodiscard]] Vector2
  iscalp(int b) // Int scalar product
  {
    return Vector2{x * static_cast<float>(b), y * static_cast<float>(b)};
  }

  [[nodiscard]] Vector2
  fscalp(float b) // Float scalar product
  {
    return Vector2{(x * b), (y * b)};
  }

  [[nodiscard]] Vector2
  vecSub(Vector2 & vector_b)
  {
    return Vector2{(x - vector_b.x), (y - vector_b.y)};
  }
  
  [[nodiscard]] Vector2
  reflected(Vector2 & reflect_against)
  {
    auto vdist_sqr = (x * x) + (y * y);
    auto projv_u = this->fscalp(this->dotp(reflect_against) / vdist_sqr);
    return projv_u.iscalp(0x2).vecsub(reflect_against);
  }

  [[nodiscard]] float
  magSq() const
  {
    return x * x + y * y;
  }
  [[nodiscard]] float
  mag() const
  {
    return std::sqrt(magSq());
  }

  // Normalization
  void 
  normalize() // Unsafe
  {
    (*this) /= mag();
  }
  [[nodiscard]] Vector2
  normalized() const // Unsafe
  {
    return (*this) / mag();
  }
  [[nodiscard]] bool
  isNorm() const
  {
    auto temp = mag() - 1;
    return (temp < 1e-6) && (temp > -(1e-6));
  }

  [[nodiscard]] float
  angle(const Vector2 & b) const
  {
    auto  s = *this * b;
    float p = s.x + s.y;
    float q = this->mag() * b.mag();

    return std::acos(p / q);
  }
  void
  clipMag(float clipm)
  {
    assert(clipm > 0.0f);
    const float rad = magSq() / (clipm * clipm);
    if (rad > 1.0f) {
      (*this) /= std::sqrt(rad);
    }
  }

  // General static inits.
  static Vector2
  zero()
  {
    return Vector2(0.0f);
  }
  static Vector2
  ones()
  {
    return Vector2(1.0f);
  }
  static Vector2
  unitX()
  {
    return Vector2(1, 0);
  }
  static Vector2
  unitY()
  {
    return Vector2(0, 1);
  }

  // General value-setters
  void
  set(float xin, float yin)
  {
    x = xin;
    y = yin;
  }
  void
  setZero()
  {
    x = 0.0f;
    y = 0.0f;
  }
  void
  setOnes()
  {
    x = 1.0f;
    y = 1.0f;
  }
  void
  setUnitX()
  {
    x = 1.0f;
    y = 0.0f;
  }
  void
  setUnitY()
  {
    x = 0.0f;
    y = 1.0f;
  }

  // Basic operations
  Vector2
  operator+(float b) const
  {
    return Vector2(x + b, y + b);
  }
  Vector2
  operator-(float b) const
  {
    return Vector2(x - b, y - b);
  }
  Vector2
  operator*(float b) const
  {
    return Vector2(x * b, y * b);
  }
  Vector2
  operator/(float b) const
  {
    return Vector2(x / b, y / b);
  }
  Vector2
  operator+(const Vector2 & b) const
  {
    return Vector2(x + b.x, y + b.y);
  }
  Vector2
  operator-(const Vector2 & b) const
  {
    return Vector2(x - b.x, y - b.y);
  }
  Vector2
  operator*(const Vector2 & b) const
  {
    return Vector2(x * b.x, y * b.y);
  }
  Vector2
  operator/(const Vector2 & b) const
  {
    return Vector2(x / b.x, y / b.y);
  }
  void
  operator+=(float b)
  {
    x += b;
    y += b;
  }
  void
  operator-=(float b)
  {
    x -= b;
    y -= b;
  }
  void
  operator*=(float b)
  {
    x *= b;
    y *= b;
  }
  void
  operator/=(float b)
  {
    x /= b;
    y /= b;
  }
  void
  operator+=(const Vector2 & b)
  {
    x += b.x;
    y += b.y;
  }
  void
  operator-=(const Vector2 & b)
  {
    x -= b.x;
    y -= b.y;
  }
  void
  operator*=(const Vector2 & b)
  {
    x *= b.x;
    y *= b.y;
  }
  void
  operator/=(const Vector2 & b)
  {
    x /= b.x;
    y /= b.y;
  }
  Vector2
  operator-() const
  {
    return Vector2(-x, -y);
  }
};

class Vector3 {
  public:
  // Components
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  // Constructors
  Vector3() = default;
  explicit Vector3(float b) : x(b), y(b), z(b) {}
  explicit Vector3(const Vector2 & xy, float z) : x(xy.x), y(xy.y), z(z) {}
  explicit Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

  // General
  // Get point on local XY plane
  [[nodiscard]] Vector2
  xy() const
  {
    return Vector2(x, y);
  }

  // Get point on local XZ plane
  [[nodiscard]] Vector2
  xz() const
  {
    return Vector2(x, z);
  }

  // Get point on local XZ plane
  [[nodiscard]] Vector2
  yz() const
  {
    return Vector2(y, z);
  }

  // Vector operations
  [[nodiscard]] float
  dotp(const Vector3 & b) const
  {
    return (x * b.x) + (y * b.y) + (z * b.z);
  }
  [[nodiscard]] Vector3
  cross(const Vector3 & b) const
  {
    return Vector3(
        (y * b.z - z * b.y), (z * b.x - x * b.z), (x * b.y - y * b.x));
  }
  [[nodiscard]] float
  magSq() const
  {
    return (x * x) + (y * y) + (z * z);
  }
  [[nodiscard]] float
  mag() const
  {
    return std::sqrt(magSq());
  }

  // Vectors normalization
  void
  normalize()
  {
    (*this) /= mag();
  }
  [[nodiscard]] Vector3
  normalized() const
  {
    return (*this) / mag();
  }

  [[nodiscard]] float
  angle(const Vector3 & b) const
  {
    return std::acos(normalized().dotp(b.normalized()));
  }

  void
  clipMag(float clipm)
  {
    assert(clipm > 0.0f);
    const float rad = magSq() / (clipm * clipm);
    if (rad > 1.0f) {
      (*this) /= std::sqrt(rad);
    }
  }

  [[nodiscard]] bool
  isNormDeviceCoords() const
  {
    return (x > -1.0f && x < 1.0f && y > -1.0f && y < 1.0f && z > -1.0f &&
            z < 1.0f);
  }

  [[nodiscard]] bool
  isNorm() const
  {
    auto temp = mag() - 1.0f;
    return (temp < 1e-6f) && (temp > -(1e-6f));
  }

  // General static inits
  // zero init Vector 3
  static Vector3
  zero()
  {
    return Vector3(0.0f);
  }

  // Fill-init vector3 with 1
  static Vector3
  ones()
  {
    return Vector3(1.0f);
  }

  // Init vector3 unitvector; x
  static Vector3
  unitX()
  {
    return Vector3(1.0f, 0.0f, 0.0f);
  }

  // Init vector3 unitvector; y
  static Vector3
  unitY()
  {
    return Vector3(0.0f, 1.0f, 0.0f);
  }

  // Init vector3 unitvector; z
  static Vector3
  unitZ()
  {
    return Vector3(0.0f, 0.0f, 1.0f);
  }

  // General setters
  void
  set(float xin, float yin, float zin)
  {
    x = xin;
    y = yin;
    z = zin;
  }
  void
  setZero()
  {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
  }
  void
  setOnes()
  {
    x = 1.0f;
    y = 1.0f;
    z = 1.0f;
  }
  void
  setUnitX()
  {
    x = 1.0f;
    y = 0.0f;
    z = 0.0f;
  }
  void
  setUnitY()
  {
    x = 0.0f;
    y = 1.0f;
    z = 0.0f;
  }
  void
  setUnitZ()
  {
    x = 0.0f;
    y = 0.0f;
    z = 1.0f;
  }

  // Operation overrides, Operations with a scalar
  Vector3
  operator+(float b) const
  {
    return Vector3((x + b), (y + b), (z + b));
  }
  Vector3
  operator-(float b) const
  {
    return Vector3((x - b), (y - b), (z - b));
  }
  Vector3
  operator*(float b) const
  {
    return Vector3((x * b), (y * b), (z * b));
  }
  Vector3
  operator/(float b) const
  {
    return Vector3((x / b), (y / b), (z / b));
  }

  void
  operator+=(float b)
  {
    x += b;
    y += b;
    z += b;
  }
  void
  operator-=(float b)
  {
    x -= b;
    y -= b;
    z -= b;
  }
  void
  operator*=(float b)
  {
    x *= b;
    y *= b;
    z *= b;
  }
  void
  operator/=(float b)
  {
    x /= b;
    y /= b;
    z /= b;
  }

  // Operation overrides, Operations with another vector
  Vector3
  operator+(const Vector3 & b) const
  {
    return Vector3((x + b.x), (y + b.y), (z + b.z));
  }
  Vector3
  operator-(const Vector3 & b) const
  {
    return Vector3((x - b.x), (y - b.y), (z - b.z));
  }
  Vector3
  operator*(const Vector3 & b) const
  {
    return Vector3((x * b.x), (y * b.y), (z * b.z));
  }
  Vector3
  operator/(const Vector3 & b) const
  {
    return Vector3((x / b.x), (y / b.y), (z / b.z));
  }

  void
  operator+=(const Vector3 & b)
  {
    x += b.x;
    y += b.y;
    z += b.z;
  }
  void
  operator-=(const Vector3 & b)
  {
    x -= b.x;
    y -= b.y;
    z -= b.z;
  }
  void
  operator*=(const Vector3 & b)
  {
    x *= b.x;
    y *= b.y;
    z *= b.z;
  }
  void
  operator/=(const Vector3 & b)
  {
    x /= b.x;
    y /= b.y;
    z /= b.z;
  }

  // Misc. operations
  Vector3
  operator-() const
  {
    return Vector3(-x, -y, -z);
  }

  Vector3
  operator>(const Vector3 & b) const
  {
    // Cross
    return Vector3(
        (y * b.z - z * b.y), (z * b.x - x * b.z), (x * b.y - y * b.x));
  }

  float
  operator<(const Vector3 & b) const
  {
    // Dot
    return (x * b.x) + (y * b.y) + (z * b.z);
  }
};

class Vector4 {
  public:
  // Components
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 0.0f;

  Vector4() = default;
  explicit Vector4(float b) : x(b), y(b), z(b), w(b) {}
  explicit Vector4(const Vector3 & xyz, float w)
      : x(xyz.x), y(xyz.y), z(xyz.z), w(w)
  {
  }
  explicit Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w)
  {
  }

  // Magnitude and Magnitude squared
  [[nodiscard]] float
  magSq() const
  {
    return (x * x) + (y * y) + (z * z) + (w * w);
  }
  [[nodiscard]] float
  mag() const
  {
    return std::sqrt(magSq());
  }

  // Normalization
  void
  normalize()
  {
    (*this) /= mag();
  }

  [[nodiscard]] Vector3
  xyz() const
  {
    return Vector3(x, y, z);
  }
  [[nodiscard]] Vector3
  xyzNormalized() const
  {
    return Vector3(x, y, z).normalized();
  }
  [[nodiscard]] Vector3
  homogenized() const
  {
    return Vector3((x / w), (y / w), (z / w));
  }

  // Operation overrides
  Vector4
  operator*(float b) const
  {
    return Vector4((x * b), (y * b), (z * b), (w * b));
  }
  Vector4
  operator/(float b) const
  {
    return Vector4((x / b), (y / b), (z / b), (w / b));
  }
  void
  operator*=(float b)
  {
    x *= b;
    y *= b;
    z *= b;
    w *= b;
  }
  void
  operator/=(float b)
  {
    x /= b;
    y /= b;
    z /= b;
    w /= b;
  }

  [[nodiscard]] float
  dotp(const Vector4 & b) const
  {
    return (x * b.x) + (y * b.y) + (z * b.z) + (w * b.w);
  }
};

#endif // VECTOR_HH
