#include "drake/multibody/tree/universal_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

using std::cos;
using std::sin;

template <typename T>
Vector2<T> UniversalMobilizer<T>::get_angles(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q;
}

template <typename T>
const UniversalMobilizer<T>& UniversalMobilizer<T>::set_angles(
    systems::Context<T>* context, const Vector2<T>& angles) const {
  auto q = this->get_mutable_positions(&*context);
  DRAKE_ASSERT(q.size() == kNq);
  q = angles;
  return *this;
}

template <typename T>
Vector2<T> UniversalMobilizer<T>::get_angular_rates(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v;
}

template <typename T>
const UniversalMobilizer<T>& UniversalMobilizer<T>::set_angular_rates(
    systems::Context<T>* context, const Vector2<T>& angles_dot) const {
  auto v = this->get_mutable_velocities(&*context);
  DRAKE_ASSERT(v.size() == kNv);
  v = angles_dot;
  return *this;
}

template <typename T>
math::RigidTransform<T> UniversalMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  const math::RotationMatrix<T> R_FI =
      math::RotationMatrix<T>::MakeXRotation(q[0]);
  const math::RotationMatrix<T> R_IM =
      math::RotationMatrix<T>::MakeYRotation(q[1]);
  const math::RigidTransform<T> X_FM(R_FI * R_IM, Vector3<T>::Zero());
  return X_FM;
}

template <typename T>
Eigen::Matrix<T, 3, 2> UniversalMobilizer<T>::CalcHwMatrix(
    const systems::Context<T>& context) const {
  const Vector2<T>& q = this->get_positions(context);
  const math::RotationMatrix<T> R_FI =
      math::RotationMatrix<T>::MakeXRotation(q[0]);
  const Vector3<T> Fx_F = Vector3<T>::UnitX();
  const Vector3<T> My_F = R_FI.col(1);
  Eigen::Matrix<T, 3, 2> H;
  H << Fx_F, My_F;
  return H;
}

template <typename T>
SpatialVelocity<T> UniversalMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  const Eigen::Matrix<T, 3, 2>& Hw = this->CalcHwMatrix(context);
  return SpatialVelocity<T>(Hw * v, Vector3<T>::Zero());
}

template <typename T>
SpatialAcceleration<T>
UniversalMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  const Vector2<T>& q = this->get_positions(context);
  const Vector2<T>& v = this->get_velocities(context);
  DRAKE_ASSERT(vdot.size() == kNv);
  const Eigen::Matrix<T, 3, 2>& Hw = this->CalcHwMatrix(context);
  Eigen::Matrix<T, 3, 2> Hw_dot = Eigen::Matrix<T, 3, 2>::Zero();
  Hw_dot(1, 1) = -v[0] * sin(q[0]);
  Hw_dot(2, 1) = v[0] * cos(q[0]);
  return SpatialAcceleration<T>(Hw * vdot + Hw_dot * v, Vector3<T>::Zero());
}

template <typename T>
void UniversalMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>& context, const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  const Eigen::Matrix<T, 3, 2>& H = this->CalcHwMatrix(context);
  // Computes tau = H_FMᵀ * F_Mo_F where H_FM ∈ ℝ³ˣ² is:
  // H_FM = [axis_F_xᵀ; Rx(θ₁)*axis_M_yᵀ]ᵀ (see CalcHwMatrix()).
  tau = H.transpose() * F_Mo_F.rotational();
}

template <typename T>
void UniversalMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                          EigenPtr<MatrixX<T>> N) const {
  *N = Matrix2<T>::Identity();
}

template <typename T>
void UniversalMobilizer<T>::DoCalcNplusMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Matrix2<T>::Identity();
}

template <typename T>
void UniversalMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  *qdot = v;
}

template <typename T>
void UniversalMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  *v = qdot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
UniversalMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<UniversalMobilizer<ToScalar>>(inboard_frame_clone,
                                                        outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> UniversalMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> UniversalMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
UniversalMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::UniversalMobilizer)
