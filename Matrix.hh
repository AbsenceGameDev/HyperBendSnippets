/*
 * This file is part on a WIP proprietary engine, by Ario Amin. All rights
 * reserved.
 *
 */
#ifndef MATRIX_HH
#define MATRIX_HH
#include "Vector.hh"

class Matrix4 {
  public:
  // Components (cells)
  float cells[16] = {0};

  // Constructors
  Matrix4() = default;
  explicit Matrix4(float b)
  {
    fillCells(b);
  }

  // Some general matrix operations
  void
  fillCells(float b)
  {
    std::fill(&cells[0], &cells[16], b);
  }
  void
  makeZero()
  {
    fillCells(0.0f);
  }
  void
  makeIdentity()
  {
    cells[0] = 1.0f;
    cells[1] = 0.0f;
    cells[2] = 0.0f;
    cells[3] = 0.0f;
    cells[4] = 0.0f;
    cells[5] = 1.0f;
    cells[6] = 0.0f;
    cells[7] = 0.0f;
    cells[8] = 0.0f;
    cells[9] = 0.0f;
    cells[10] = 1.0f;
    cells[11] = 0.0f;
    cells[12] = 0.0f;
    cells[13] = 0.0f;
    cells[14] = 0.0f;
    cells[15] = 1.0f;
  }
  void
  makeRotX(float a)
  {
    cells[0] = 1.0f;
    cells[1] = 0.0f;
    cells[2] = 0.0f;
    cells[3] = 0.0f;
    cells[4] = 0.0f;
    cells[5] = std::cos(a);
    cells[6] = -std::sin(a);
    cells[7] = 0.0f;
    cells[8] = 0.0f;
    cells[9] = std::sin(a);
    cells[10] = std::cos(a);
    cells[11] = 0.0f;
    cells[12] = 0.0f;
    cells[13] = 0.0f;
    cells[14] = 0.0f;
    cells[15] = 1.0f;
  }
  void
  makeRotY(float a)
  {
    cells[0] = std::cos(a);
    cells[1] = 0.0f;
    cells[2] = std::sin(a);
    cells[3] = 0.0f;
    cells[4] = 0.0f;
    cells[5] = 1.0f;
    cells[6] = 0.0f;
    cells[7] = 0.0f;
    cells[8] = -std::sin(a);
    cells[9] = 0.0f;
    cells[10] = std::cos(a);
    cells[11] = 0.0f;
    cells[12] = 0.0f;
    cells[13] = 0.0f;
    cells[14] = 0.0f;
    cells[15] = 1.0f;
  }
  void
  makeRotZ(float a)
  {
    cells[0] = std::cos(a);
    cells[1] = -std::sin(a);
    cells[2] = 0.0f;
    cells[3] = 0.0f;
    cells[4] = std::sin(a);
    cells[5] = std::cos(a);
    cells[6] = 0.0f;
    cells[7] = 0.0f;
    cells[8] = 0.0f;
    cells[9] = 0.0f;
    cells[10] = 1.0f;
    cells[11] = 0.0f;
    cells[12] = 0.0f;
    cells[13] = 0.0f;
    cells[14] = 0.0f;
    cells[15] = 1.0f;
  }
  void
  makeTrans(const Vector3 & t)
  {
    cells[0] = 1.0f;
    cells[1] = 0.0f;
    cells[2] = 0.0f;
    cells[3] = t.x;
    cells[4] = 0.0f;
    cells[5] = 1.0f;
    cells[6] = 0.0f;
    cells[7] = t.y;
    cells[8] = 0.0f;
    cells[9] = 0.0f;
    cells[10] = 1.0f;
    cells[11] = t.z;
    cells[12] = 0.0f;
    cells[13] = 0.0f;
    cells[14] = 0.0f;
    cells[15] = 1.0f;
  }
  void
  makeScale(const Vector3 & s)
  {
    cells[0] = s.x;
    cells[1] = 0.0f;
    cells[2] = 0.0f;
    cells[3] = 0.0f;
    cells[4] = 0.0f;
    cells[5] = s.y;
    cells[6] = 0.0f;
    cells[7] = 0.0f;
    cells[8] = 0.0f;
    cells[9] = 0.0f;
    cells[10] = s.z;
    cells[11] = 0.0f;
    cells[12] = 0.0f;
    cells[13] = 0.0f;
    cells[14] = 0.0f;
    cells[15] = 1.0f;
  }

  // SIdentities
  static Matrix4
  zero()
  {
    Matrix4 cells;
    cells.makeZero();
    return cells;
  }
  static Matrix4
  identity()
  {
    Matrix4 cells;
    cells.makeIdentity();
    return cells;
  }
  static Matrix4
  rotX(float a)
  {
    Matrix4 cells;
    cells.makeRotX(a);
    return cells;
  }
  static Matrix4
  rotY(float a)
  {
    Matrix4 cells;
    cells.makeRotY(a);
    return cells;
  }
  static Matrix4
  rotZ(float a)
  {
    Matrix4 cells;
    cells.makeRotZ(a);
    return cells;
  }
  static Matrix4
  trans(const Vector3 & t)
  {
    Matrix4 cells;
    cells.makeTrans(t);
    return cells;
  }
  static Matrix4
  scale(float s)
  {
    Matrix4 cells;
    cells.makeScale(Vector3(s));
    return cells;
  }
  static Matrix4
  scale(const Vector3 & s)
  {
    Matrix4 cells;
    cells.makeScale(s);
    return cells;
  }

  // Transformations
  [[nodiscard]] Matrix4
  transposed() const
  {
    Matrix4 out;
    out.cells[0] = cells[0];
    out.cells[1] = cells[4];
    out.cells[2] = cells[8];
    out.cells[3] = cells[12];
    out.cells[4] = cells[1];
    out.cells[5] = cells[5];
    out.cells[6] = cells[9];
    out.cells[7] = cells[13];
    out.cells[8] = cells[2];
    out.cells[9] = cells[6];
    out.cells[10] = cells[10];
    out.cells[11] = cells[14];
    out.cells[12] = cells[3];
    out.cells[13] = cells[7];
    out.cells[14] = cells[11];
    out.cells[15] = cells[15];
    return out;
  }
  void
  translate(const Vector3 & t)
  {
    cells[3] += t.x;
    cells[7] += t.y;
    cells[11] += t.z;
  }
  void
  stretch(const Vector3 & s)
  {
    cells[0] *= s.x;
    cells[5] *= s.y;
    cells[10] *= s.z;
  }

  [[nodiscard]] Vector3
  mulPoint(const Vector3 & b) const
  {
    const Vector3 p(cells[0] * b.x + cells[1] * b.y + cells[2] * b.z + cells[3],
                    cells[4] * b.x + cells[5] * b.y + cells[6] * b.z + cells[7],
                    cells[8] * b.x + cells[9] * b.y + cells[10] * b.z +
                        cells[11]);
    const float   w =
        cells[12] * b.x + cells[13] * b.y + cells[14] * b.z + cells[15];
    return p / w;
  }
  [[nodiscard]] Vector3
  mulDirection(const Vector3 & b) const
  {
    return Vector3(cells[0] * b.x + cells[1] * b.y + cells[2] * b.z,
                   cells[4] * b.x + cells[5] * b.y + cells[6] * b.z,
                   cells[8] * b.x + cells[9] * b.y + cells[10] * b.z);
  }

  // Inverse (^-1)
  [[nodiscard]] Matrix4
  inverse() const
  {
    Matrix4 inv;
    inv.cells[0] =
        cells[5] * cells[10] * cells[15] - cells[5] * cells[11] * cells[14] -
        cells[9] * cells[6] * cells[15] + cells[9] * cells[7] * cells[14] +
        cells[13] * cells[6] * cells[11] - cells[13] * cells[7] * cells[10];

    inv.cells[1] =
        -cells[1] * cells[10] * cells[15] + cells[1] * cells[11] * cells[14] +
        cells[9] * cells[2] * cells[15] - cells[9] * cells[3] * cells[14] -
        cells[13] * cells[2] * cells[11] + cells[13] * cells[3] * cells[10];

    inv.cells[2] =
        cells[1] * cells[6] * cells[15] - cells[1] * cells[7] * cells[14] -
        cells[5] * cells[2] * cells[15] + cells[5] * cells[3] * cells[14] +
        cells[13] * cells[2] * cells[7] - cells[13] * cells[3] * cells[6];

    inv.cells[3] =
        -cells[1] * cells[6] * cells[11] + cells[1] * cells[7] * cells[10] +
        cells[5] * cells[2] * cells[11] - cells[5] * cells[3] * cells[10] -
        cells[9] * cells[2] * cells[7] + cells[9] * cells[3] * cells[6];

    inv.cells[4] =
        -cells[4] * cells[10] * cells[15] + cells[4] * cells[11] * cells[14] +
        cells[8] * cells[6] * cells[15] - cells[8] * cells[7] * cells[14] -
        cells[12] * cells[6] * cells[11] + cells[12] * cells[7] * cells[10];

    inv.cells[5] =
        cells[0] * cells[10] * cells[15] - cells[0] * cells[11] * cells[14] -
        cells[8] * cells[2] * cells[15] + cells[8] * cells[3] * cells[14] +
        cells[12] * cells[2] * cells[11] - cells[12] * cells[3] * cells[10];

    inv.cells[6] =
        -cells[0] * cells[6] * cells[15] + cells[0] * cells[7] * cells[14] +
        cells[4] * cells[2] * cells[15] - cells[4] * cells[3] * cells[14] -
        cells[12] * cells[2] * cells[7] + cells[12] * cells[3] * cells[6];

    inv.cells[7] =
        cells[0] * cells[6] * cells[11] - cells[0] * cells[7] * cells[10] -
        cells[4] * cells[2] * cells[11] + cells[4] * cells[3] * cells[10] +
        cells[8] * cells[2] * cells[7] - cells[8] * cells[3] * cells[6];

    inv.cells[8] =
        cells[4] * cells[9] * cells[15] - cells[4] * cells[11] * cells[13] -
        cells[8] * cells[5] * cells[15] + cells[8] * cells[7] * cells[13] +
        cells[12] * cells[5] * cells[11] - cells[12] * cells[7] * cells[9];

    inv.cells[9] =
        -cells[0] * cells[9] * cells[15] + cells[0] * cells[11] * cells[13] +
        cells[8] * cells[1] * cells[15] - cells[8] * cells[3] * cells[13] -
        cells[12] * cells[1] * cells[11] + cells[12] * cells[3] * cells[9];

    inv.cells[10] =
        cells[0] * cells[5] * cells[15] - cells[0] * cells[7] * cells[13] -
        cells[4] * cells[1] * cells[15] + cells[4] * cells[3] * cells[13] +
        cells[12] * cells[1] * cells[7] - cells[12] * cells[3] * cells[5];

    inv.cells[11] =
        -cells[0] * cells[5] * cells[11] + cells[0] * cells[7] * cells[9] +
        cells[4] * cells[1] * cells[11] - cells[4] * cells[3] * cells[9] -
        cells[8] * cells[1] * cells[7] + cells[8] * cells[3] * cells[5];

    inv.cells[12] =
        -cells[4] * cells[9] * cells[14] + cells[4] * cells[10] * cells[13] +
        cells[8] * cells[5] * cells[14] - cells[8] * cells[6] * cells[13] -
        cells[12] * cells[5] * cells[10] + cells[12] * cells[6] * cells[9];

    inv.cells[13] =
        cells[0] * cells[9] * cells[14] - cells[0] * cells[10] * cells[13] -
        cells[8] * cells[1] * cells[14] + cells[8] * cells[2] * cells[13] +
        cells[12] * cells[1] * cells[10] - cells[12] * cells[2] * cells[9];

    inv.cells[14] =
        -cells[0] * cells[5] * cells[14] + cells[0] * cells[6] * cells[13] +
        cells[4] * cells[1] * cells[14] - cells[4] * cells[2] * cells[13] -
        cells[12] * cells[1] * cells[6] + cells[12] * cells[2] * cells[5];

    inv.cells[15] =
        cells[0] * cells[5] * cells[10] - cells[0] * cells[6] * cells[9] -
        cells[4] * cells[1] * cells[10] + cells[4] * cells[2] * cells[9] +
        cells[8] * cells[1] * cells[6] - cells[8] * cells[2] * cells[5];

    const float det = cells[0] * inv.cells[0] + cells[1] * inv.cells[4] +
                      cells[2] * inv.cells[8] + cells[3] * inv.cells[12];
    inv /= det;
    return inv;
  }

  // Some general getters
  [[nodiscard]] Vector3
  xAxis() const
  {
    return Vector3(cells[0], cells[4], cells[8]);
  }
  [[nodiscard]] Vector3
  yAxis() const
  {
    return Vector3(cells[1], cells[5], cells[9]);
  }
  [[nodiscard]] Vector3
  zAxis() const
  {
    return Vector3(cells[2], cells[6], cells[10]);
  }
  [[nodiscard]] Vector3
  translation() const
  {
    return Vector3(cells[3], cells[7], cells[11]);
  }
  [[nodiscard]] Vector3
  scale() const
  {
    return Vector3(cells[0], cells[5], cells[10]);
  }

  // Some general setters
  void
  setTranslation(const Vector3 & t)
  {
    cells[3] = t.x;
    cells[7] = t.y;
    cells[11] = t.z;
  }
  void
  setXAxis(const Vector3 & t)
  {
    cells[0] = t.x;
    cells[4] = t.y;
    cells[8] = t.z;
  }
  void
  setYAxis(const Vector3 & t)
  {
    cells[1] = t.x;
    cells[5] = t.y;
    cells[9] = t.z;
  }
  void
  setZAxis(const Vector3 & t)
  {
    cells[2] = t.x;
    cells[6] = t.y;
    cells[10] = t.z;
  }
  void
  setScale(const Vector3 & s)
  {
    cells[0] = s.x;
    cells[5] = s.y;
    cells[10] = s.z;
  }

  // Basic operation overrides
  Matrix4
  operator+(const Matrix4 & b) const
  {
    Matrix4 out;
    for (int i = 0; i < 16; ++i) {
      out.cells[i] = cells[i] + b.cells[i];
    }
    return out;
  }
  Matrix4
  operator-(const Matrix4 & b) const
  {
    Matrix4 out;
    for (int i = 0; i < 16; ++i) {
      out.cells[i] = cells[i] - b.cells[i];
    }
    return out;
  }
  void
  operator+=(const Matrix4 & b)
  {
    for (int i = 0; i < 16; ++i) {
      cells[i] += b.cells[i];
    }
  }
  void
  operator-=(const Matrix4 & b)
  {
    for (int i = 0; i < 16; ++i) {
      cells[i] -= b.cells[i];
    }
  }
  void
  operator*=(float b)
  {
    for (auto & cell : cells) {
      cell *= b;
    }
  }
  void
  operator/=(float b)
  {
    operator*=(1.0f / b);
  }

  // Multiplication
  Matrix4
  operator*(const Matrix4 & b) const
  {
    Matrix4 out;
    out.cells[0] = b.cells[0] * cells[0] + b.cells[4] * cells[1] +
                   b.cells[8] * cells[2] + b.cells[12] * cells[3];
    out.cells[1] = b.cells[1] * cells[0] + b.cells[5] * cells[1] +
                   b.cells[9] * cells[2] + b.cells[13] * cells[3];
    out.cells[2] = b.cells[2] * cells[0] + b.cells[6] * cells[1] +
                   b.cells[10] * cells[2] + b.cells[14] * cells[3];
    out.cells[3] = b.cells[3] * cells[0] + b.cells[7] * cells[1] +
                   b.cells[11] * cells[2] + b.cells[15] * cells[3];

    out.cells[4] = b.cells[0] * cells[4] + b.cells[4] * cells[5] +
                   b.cells[8] * cells[6] + b.cells[12] * cells[7];
    out.cells[5] = b.cells[1] * cells[4] + b.cells[5] * cells[5] +
                   b.cells[9] * cells[6] + b.cells[13] * cells[7];
    out.cells[6] = b.cells[2] * cells[4] + b.cells[6] * cells[5] +
                   b.cells[10] * cells[6] + b.cells[14] * cells[7];
    out.cells[7] = b.cells[3] * cells[4] + b.cells[7] * cells[5] +
                   b.cells[11] * cells[6] + b.cells[15] * cells[7];

    out.cells[8] = b.cells[0] * cells[8] + b.cells[4] * cells[9] +
                   b.cells[8] * cells[10] + b.cells[12] * cells[11];
    out.cells[9] = b.cells[1] * cells[8] + b.cells[5] * cells[9] +
                   b.cells[9] * cells[10] + b.cells[13] * cells[11];
    out.cells[10] = b.cells[2] * cells[8] + b.cells[6] * cells[9] +
                    b.cells[10] * cells[10] + b.cells[14] * cells[11];
    out.cells[11] = b.cells[3] * cells[8] + b.cells[7] * cells[9] +
                    b.cells[11] * cells[10] + b.cells[15] * cells[11];

    out.cells[12] = b.cells[0] * cells[12] + b.cells[4] * cells[13] +
                    b.cells[8] * cells[14] + b.cells[12] * cells[15];
    out.cells[13] = b.cells[1] * cells[12] + b.cells[5] * cells[13] +
                    b.cells[9] * cells[14] + b.cells[13] * cells[15];
    out.cells[14] = b.cells[2] * cells[12] + b.cells[6] * cells[13] +
                    b.cells[10] * cells[14] + b.cells[14] * cells[15];
    out.cells[15] = b.cells[3] * cells[12] + b.cells[7] * cells[13] +
                    b.cells[11] * cells[14] + b.cells[15] * cells[15];
    return out;
  }
  void
  operator*=(const Matrix4 & b)
  {
    (*this) = operator*(b);
  }
  Vector4
  operator*(const Vector4 & b) const
  {
    return Vector4(
        cells[0] * b.x + cells[1] * b.y + cells[2] * b.z + cells[3] * b.w,
        cells[4] * b.x + cells[5] * b.y + cells[6] * b.z + cells[7] * b.w,
        cells[8] * b.x + cells[9] * b.y + cells[10] * b.z + cells[11] * b.w,
        cells[12] * b.x + cells[13] * b.y + cells[14] * b.z + cells[15] * b.w);
  }
};
#endif // MATRIX_HH