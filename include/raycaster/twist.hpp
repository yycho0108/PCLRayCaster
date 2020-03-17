#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

template <class Scalar>
class Twist {
  using Vector3x = Eigen::Matrix<Scalar, 3, 1>;

 public:
  Twist() {}
  Twist(const Vector3x& rotation, const Vector3x& translation)
      : rotation_(rotation), translation_(translation) {}

  Twist(const Twist& twist)
      : rotation_(twist.rotation_), translation_(twist.translation_) {}

  const Vector3x& Translation() const { return translation_; }

  Vector3x& Translation() { return translation_; }

  const Vector3x& Rotation() const { return rotation_; }

  Vector3x& Rotation() { return rotation_; }

 private:
  Vector3x rotation_;
  Vector3x translation_;
};

template <class Scalar>
Twist<Scalar> operator*(const Twist<Scalar>& twist, const float scale) {
  return Twist<Scalar>{twist.Rotation() * scale, twist.Translation() * scale};
}

template <class Scalar>
Twist<Scalar> operator*(const float scale, const Twist<Scalar>& twist) {
  return Twist<Scalar>{twist.Rotation() * scale, twist.Translation() * scale};
}

/**
 * se3 -> SE3
 */
template <class Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry> Exp(const Twist<Scalar>& T) {
  const auto& w = T.Rotation();
  const auto& v = T.Translation();

  const float t2 = w.dot(w);
  const float t = std::sqrt(t2);
  const bool is_small_angle = (t <= std::numeric_limits<float>::epsilon());
  const Eigen::AngleAxis<Scalar> rotation(t, is_small_angle ? w : w / t);

  if (is_small_angle) {
    const Eigen::Matrix<Scalar, 3, 1> position = v + 0.5 * w.cross(v);
    return Eigen::Translation<Scalar, 3>{position} * rotation;
  }

  const float ct = std::cos(t);
  const float st = std::sin(t);
  const float awxv = (1.0 - ct) / t2;
  const float av = (st / t);
  const float aw = (1.0 - av) / t2;

  const Eigen::Matrix<Scalar, 3, 1> position =
      av * v + aw * w.dot(v) * w + awxv * w.cross(v);

  return Eigen::Translation<Scalar, 3>{position} * rotation;
}

/**
 * SE3 -> se3
 */
template <class Scalar>
Twist<Scalar> Log(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T) {
  Twist<Scalar> out;

  // Extract components.
  const Eigen::AngleAxis<Scalar> axa{T.linear()};
  const Eigen::Matrix<Scalar, 3, 1>& w = axa.axis() * axa.angle();
  const Eigen::Matrix<Scalar, 3, 1>& p = T.translation();

  // Set intermediate values.
  const float t = axa.angle();
  const float t2 = t * t;
  const bool is_small_angle = (t <= std::numeric_limits<float>::epsilon());

  if (is_small_angle) {
    const Eigen::Matrix<Scalar, 3, 1>& v = p + 0.5 * p.cross(w);
    return Twist<Scalar>{w, v};
  }

  const float st = std::sin(t);
  const float ct = std::cos(t);
  const float alpha = (t * st / (2.0 * (1.0 - ct)));
  const float beta = (1.0 / t2 - st / (2.0 * t * (1.0 - ct)));
  const Eigen::Matrix<Scalar, 3, 1>& v =
      (alpha * p - 0.5 * w.cross(p) + beta * w.dot(p) * w);
  return Twist<Scalar>{w, v};
}
