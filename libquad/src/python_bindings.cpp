#include "Chassis.hpp"
#include "Leg.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(libquad, m, py::mod_gil_not_used()) {
  py::class_<Leg>(m, "Leg")
      .def(py::init([](Eigen::Vector<double, njoints + 1> &l,
                       Eigen::Matrix<double, 6, njoints> &Slist,
                       Eigen::Matrix4d &M,
                       std::array<Eigen::Matrix4d, njoints + 1> &Mlist,
                       std::array<Eigen::Matrix6d, njoints> &Glist,
                       Eigen::Matrix<double, njoints, 2> &thetaRange,
                       Eigen::Vector<double, njoints> &thetadMax,
                       Eigen::Vector<double, njoints> &thetaddMax,
                       Eigen::Vector<double, njoints> &tauMax) {
        Eigen::Isometry3d M_tf;
        std::array<Eigen::Isometry3d, njoints + 1> Mlist_tf;
        M_tf.matrix() = M;
        for (size_t i = 0; i < 4; i++) {
          Mlist_tf[i].matrix() = Mlist[i];
        }
        return std::make_unique<Leg>(l, Slist, M_tf, Mlist_tf, Glist,
                                     thetaRange, thetadMax, thetaddMax, tauMax);
      }))
      .def("fk",
           [](Leg &self, const Eigen::Vector3d &thetalist) {
             Eigen::Vector3d pf;
             bool success = self.fk(thetalist, pf);
             return success ? pf : Eigen::Vector3d::Constant(NAN);
           })
      .def("ik",
           [](Leg &self, const Eigen::Vector3d &pf) {
             Eigen::Vector3d thetalist;
             bool success = self.ik(pf, thetalist);
             return success ? thetalist : Eigen::Vector3d::Constant(NAN);
           })
      .def("ik_jacinv",
           [](Leg &self, const Eigen::Vector3d &pf) {
             Eigen::Vector3d thetalist;
             Eigen::Matrix3d jinv = Eigen::Matrix3d::Constant(NAN);
             bool success = self.ik(pf, thetalist, &jinv);
             return std::make_tuple(
                 success ? thetalist : Eigen::Vector3d::Constant(NAN), jinv);
           })
      .def("ivk",
           [](Leg &self, const Eigen::Vector3d &vf, Eigen::Matrix3d &jinv) {
             Eigen::Vector3d thetadlist;
             bool success = self.ivk(vf, jinv, thetadlist);
             return success ? thetadlist : Eigen::Vector3d::Constant(NAN);
           })
      .def("id",
           [](Leg &self) {
             Eigen::Vector3d taulist;
             bool success = self.id(taulist);
             return success ? taulist : Eigen::Vector3d::Constant(NAN);
           })
      .def_readwrite("Tf", &Leg::Tf)
      .def_readwrite("pf", &Leg::pf)
      .def_readwrite("vf", &Leg::vf)
      .def_readwrite("dvf", &Leg::dvf)
      .def_readwrite("ff", &Leg::ff)
      .def_readwrite("g", &Leg::g)
      .def_readwrite("thetalist", &Leg::thetalist)
      .def_readwrite("thetadlist", &Leg::thetadlist)
      .def_readwrite("thetaddlist", &Leg::thetaddlist)
      .def_readwrite("taulist", &Leg::taulist)
      .def_readwrite("T_stow", &Leg::T_stow)
      .def_readwrite("T_deploy", &Leg::T_deploy);

  py::class_<Chassis>(m, "Chassis")
      .def(py::init([](Eigen::Matrix6d &G, Eigen::Matrix4d &Ms,
                       std::array<Eigen::Matrix4d, 4> &M01) {
        Eigen::Isometry3d Ms_tf;
        std::array<Eigen::Isometry3d, 4> M01_tf;
        Ms_tf.matrix() = Ms;
        for (size_t i = 0; i < 4; i++) {
          M01_tf[i].matrix() = M01[i];
        }
        return std::make_unique<Chassis>(G, Ms_tf, M01_tf);
      }));

  //   .def("fk", &Leg::fk)
  // .def("ik", &Leg::ik);
}
