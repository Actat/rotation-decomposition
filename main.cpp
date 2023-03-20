#include <eigen3/Eigen/Geometry>
#include <iostream>

std::tuple<Eigen::Quaterniond, Eigen::Quaterniond> decompose_rotation(
    Eigen::Quaterniond r,
    Eigen::Vector3d e_q) {
  e_q.normalize();
  r.normalize();
  Eigen::Vector3d re_q = r * e_q;
  if (re_q.isApprox(e_q)) {
    return std::forward_as_tuple(Eigen::Quaterniond(1, 0, 0, 0), r);
  } else if (re_q.isApprox(-1 * e_q)) {
    return std::forward_as_tuple(r, Eigen::Quaterniond(1, 0, 0, 0));
  } else {
    Eigen::Vector3d e_p = (e_q.cross(re_q)).normalized();
    double theta_p      = std::acos(e_q.dot(re_q));
    auto p              = Eigen::Quaterniond(
        std::cos(theta_p / 2), std::sin(theta_p / 2) * e_p.x(),
        std::sin(theta_p / 2) * e_p.y(), std::sin(theta_p / 2) * e_p.z());
    Eigen::Quaterniond q = p.conjugate() * r;
    return std::forward_as_tuple(p, q);
  }
}

bool test(Eigen::Quaterniond r, Eigen::Vector3d e_q) {
  std::cout << "decomposition test" << std::endl;
  std::cout << "  input: " << std::endl;
  std::cout << "    quaternion r" << std::endl;
  std::cout << "      w: " << r.w() << std::endl;
  std::cout << "      x: " << r.x() << std::endl;
  std::cout << "      y: " << r.y() << std::endl;
  std::cout << "      z: " << r.z() << std::endl;
  std::cout << "    vector e_q" << std::endl;
  std::cout << "      x: " << e_q.x() << std::endl;
  std::cout << "      y: " << e_q.y() << std::endl;
  std::cout << "      z: " << e_q.z() << std::endl;

  Eigen::Quaterniond p, q;
  std::tie(p, q) = decompose_rotation(r, e_q);

  double theta_p = 2.0 * std::acos(p.w());
  double theta_q = 2.0 * std::acos(q.w());
  auto e_p       = Eigen::Vector3d(p.x(), p.y(), p.z());
  if (theta_p != 0) {
    e_p /= std::sin(theta_p / 2.0);
  }

  std::cout << "  output: " << std::endl;
  std::cout << "    quaternion p" << std::endl;
  std::cout << "      w: " << p.w() << std::endl;
  std::cout << "      x: " << p.x() << std::endl;
  std::cout << "      y: " << p.y() << std::endl;
  std::cout << "      z: " << p.z() << std::endl;
  std::cout << "      theta_p: " << theta_p << std::endl;
  std::cout << "    quaternion q" << std::endl;
  std::cout << "      w: " << q.w() << std::endl;
  std::cout << "      x: " << q.x() << std::endl;
  std::cout << "      y: " << q.y() << std::endl;
  std::cout << "      z: " << q.z() << std::endl;
  std::cout << "      theta_q: " << theta_q << std::endl;

  bool size_p  = std::abs(p.norm() - 1.0) < 1e-10;
  bool size_q  = std::abs(q.norm() - 1.0) < 1e-10;
  bool is_rpq  = r.isApprox(p * q);
  bool is_vert = e_p.dot(e_q) < 1e-10;
  std::cout << "  check: " << std::endl;
  std::cout << "    size of p is 1: " << size_p << std::endl;
  std::cout << "    size of q is 1: " << size_q << std::endl;
  std::cout << "    r = p * q: " << is_rpq << std::endl;
  std::cout << "    e_p and e_q are vertical: " << is_vert << std::endl;

  return size_p && size_q && is_rpq && is_vert;
}

int main(void) {
  std::cout << "re_q == e_q" << std::endl;
  if (test(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(1, 0, 0)) &&
      test(Eigen::Quaterniond(0, 1, 0, 0), Eigen::Vector3d(1, 0, 0))) {
    std::cout << "Good." << std::endl;
  } else {
    std::cout << "Something wrong." << std::endl;
    return 0;
  }
  std::cout << std::endl;

  std::cout << "re_q == -1 * e_q" << std::endl;
  if (test(Eigen::Quaterniond(0, 1, 0, 0), Eigen::Vector3d(0, 0, 1)) &&
      test(Eigen::Quaterniond(1 / std::sqrt(2), 1 / std::sqrt(2), 0, 0),
           Eigen::Vector3d(0, 0, 1))) {
    std::cout << "Good." << std::endl;
  } else {
    std::cout << "Something wrong." << std::endl;
    return 0;
  }
  std::cout << std::endl;

  std::cout << "Others" << std::endl;
  for (int i = 0; i < 1000; ++i) {
    auto r = Eigen::Quaterniond::UnitRandom();
    auto v = Eigen::Vector3d::Random().normalized();
    if (test(r, v)) {
      std::cout << "Good." << std::endl;
    } else {
      std::cout << "Something wrong." << std::endl;
      return 0;
    }
  }

  std::cout << std::endl << "All OK." << std::endl;

  return 0;
}
