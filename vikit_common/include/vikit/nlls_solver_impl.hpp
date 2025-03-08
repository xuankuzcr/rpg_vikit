/*
 * Abstract Nonlinear Least-Squares Solver Class
 *
 * nlls_solver.h
 *
 *  Created on: Nov 5, 2012
 *      Author: cforster
 */

#ifndef VIKIT_NLLS_SOLVER_IMPL_HPP_
#define VIKIT_NLLS_SOLVER_IMPL_HPP_

#include <stdexcept>

namespace vk {

template <int D, typename T> void NLLSSolver<D, T>::optimize(ModelType &model) {
  if (method_ == GaussNewton)
    optimizeGaussNewton(model);
  else if (method_ == LevenbergMarquardt)
    optimizeLevenbergMarquardt(model);
}

template <int D, typename T>
void NLLSSolver<D, T>::optimizeGaussNewton(ModelType &model) {
  // Init variables
  ModelType new_model;
  double new_chi2 = 0.0;
  stop_ = false;
  iter_ = 0;

  // Compute Initial Error
  chi2_ = computeResiduals(model, false, true);

  // Optimization Loop
  while (!(stop_ || iter_ >= n_iter_)) {
    startIteration();
    H_.setZero();
    Jres_.setZero();

    // Compute Jacobian
    computeResiduals(model, true, false);

    // Solve the linear system
    if (solve()) {
      // Apply the update and compute new residual
      update(model, new_model);
      new_chi2 = computeResiduals(new_model, false, false);

      // Acceptance of the update
      if (new_chi2 < chi2_) {
        model = new_model;
        chi2_ = new_chi2;
      }
    }

    if (verbose_) {
      std::cout << "it=" << iter_ << "\t chi2=" << chi2_
                << "\t x_norm=" << vk::norm_max(x_) << std::endl;
    }

    stop_ = vk::norm_max(x_) <= eps_;
    ++iter_;
    finishIteration();
  }
}

template <int D, typename T>
void NLLSSolver<D, T>::optimizeLevenbergMarquardt(ModelType &model) {
  // Init variables
  ModelType new_model;
  double new_chi2 = 0.0;
  stop_ = false;
  iter_ = 0;
  n_trials_ = 0;

  // Compute Initial Error
  chi2_ = computeResiduals(model, false, true);

  // compute the initial rho and lambda
  rho_ = 0.0;
  double H_max_diag = 0;
  for (int j = 0; j < D; ++j)
    H_max_diag = std::max(H_max_diag, std::fabs(H_(j, j)));
  double lambda = 1e-4 * H_max_diag;

  // Optimization Loop
  while (!(stop_ || iter_ >= n_iter_)) {
    ++n_trials_;
    startIteration();

    // try to compute and apply update
    ModelType backup_model = model;
    double backup_chi2 = chi2_;

    // compute H and Jres
    H_.setZero();
    Jres_.setZero();
    computeResiduals(model, true, false);

    // add damping term:
    H_.diagonal() += Eigen::Matrix<double, D, 1>::Constant(lambda);

    // solve the linear system
    if (solve()) {
      // apply the update and compute new residual
      update(model, new_model);
      new_chi2 = computeResiduals(new_model, false, false);

      // compute ratio of residual improvement
      rho_ = (backup_chi2 - new_chi2);
      if (rho_ > 0.0) {
        // update decrased the error -> success
        double alpha = 1. - pow((2 * rho_ - 1), 3);
        lambda *= std::max(0.333, alpha);
        model = new_model;
        chi2_ = new_chi2;
        stop_ = vk::norm_max(x_) <= eps_;
        ++iter_;
        finishIteration();
      } else {
        // update increased the error -> fail
        lambda *= 10;
        model = backup_model;
        chi2_ = backup_chi2;
      }
    } else {
      // solving the linear system failed
      lambda *= 10;
      model = backup_model;
      chi2_ = backup_chi2;
    }

    if (verbose_) {
      std::cout << "it=" << iter_ << "\t trials=" << n_trials_
                << "\t chi2=" << chi2_ << "\t rho=" << rho_
                << "\t lambda=" << lambda << "\t x_norm=" << vk::norm_max(x_)
                << std::endl;
    }
  }
}

template <int D, typename T> void NLLSSolver<D, T>::reset() {
  iter_ = 0;
  n_trials_ = 0;
  chi2_ = 0.0;
  rho_ = 0.0;
  stop_ = false;
  have_prior_ = false;
}

template <int D, typename T>
void NLLSSolver<D, T>::setPrior(
    const ModelType &prior, const Eigen::Matrix<double, D, D> &Information) {
  I_prior_ = Information;
  ModelType::plus(prior, x_, prior_);
  have_prior_ = true;
}

template <int D, typename T>
const Eigen::Matrix<double, D, D> &
NLLSSolver<D, T>::getInformationMatrix() const {
  return H_;
}

} // end namespace vk

#endif /* VIKIT_NLLS_SOLVER_IMPL_HPP_ */
