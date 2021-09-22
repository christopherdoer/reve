// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <iostream>

#include <radar_ego_velocity_estimator/odr.h>

// inspired by scipy's odrpack c-wrapper https://github.com/scipy/scipy/blob/master/scipy/odr/__odrpack.c
// big thanks to Robert Kern!!

using namespace reve;

// odrpack fortan subroutines
extern "C" {
void F_FUNC(dodrc, DODRC)(void (*fcn)(int* n,
                                      int* m,
                                      int* np,
                                      int* nq,
                                      int* ldn,
                                      int* ldm,
                                      int* ldnp,
                                      double* beta,
                                      double* xplusd,
                                      int* ifixb,
                                      int* ifixx,
                                      int* ldifx,
                                      int* ideval,
                                      double* f,
                                      double* fjacb,
                                      double* fjacd,
                                      int* istop),
                          int* n,
                          int* m,
                          int* np,
                          int* nq,
                          double* beta,
                          double* y,
                          int* ldy,
                          double* x,
                          int* ldx,
                          double* we,
                          int* ldwe,
                          int* ld2we,
                          double* wd,
                          int* ldwd,
                          int* ld2wd,
                          int* ifixb,
                          int* ifixx,
                          int* ldifx,
                          int* job,
                          int* ndigit,
                          double* taufac,
                          double* sstol,
                          double* partol,
                          int* maxit,
                          int* iprint,
                          int* lunerr,
                          int* lunrpt,
                          double* stpb,
                          double* stpd,
                          int* ldstpd,
                          double* sclb,
                          double* scld,
                          int* ldscld,
                          double* work,
                          int* lwork,
                          int* iwork,
                          int* liwork,
                          int* info);

void F_FUNC(dwinf, DWINF)(int* n,
                          int* m,
                          int* np,
                          int* nq,
                          int* ldwe,
                          int* ld2we,
                          int* isodr,
                          int* delta,
                          int* eps,
                          int* xplus,
                          int* fn,
                          int* sd,
                          int* vcv,
                          int* rvar,
                          int* wss,
                          int* wssde,
                          int* wssep,
                          int* rcond,
                          int* eta,
                          int* olmav,
                          int* tau,
                          int* alpha,
                          int* actrs,
                          int* pnorm,
                          int* rnors,
                          int* prers,
                          int* partl,
                          int* sstol,
                          int* taufc,
                          int* apsma,
                          int* betao,
                          int* betac,
                          int* betas,
                          int* betan,
                          int* s,
                          int* ss,
                          int* ssf,
                          int* qraux,
                          int* u,
                          int* fs,
                          int* fjacb,
                          int* we1,
                          int* diff,
                          int* delts,
                          int* deltn,
                          int* t,
                          int* tt,
                          int* omega,
                          int* fjacd,
                          int* wrk1,
                          int* wrk2,
                          int* wrk3,
                          int* wrk4,
                          int* wrk5,
                          int* wrk6,
                          int* wrk7,
                          int* lwkmn);

void F_FUNC(dluno, DLUNO)(int* lun, char* fn, int fnlen);

void F_FUNC(dlunc, DLUNC)(int* lun);
}

/**
 * @brief Callback function called by odrpack --> see p.26 of src/odrpack/odrpack_manual.pdf
 * \note Assumes a linear model model y = f(x,beta) = x * beta whereas y is a vector
 */
void fcn_callback(int* n,
                  int* m,
                  int* np,
                  int* nq,
                  int* ldn,
                  int* ldm,
                  int* ldnp,
                  double* beta,
                  double* xplusd,
                  int* ifixb,
                  int* ifixx,
                  int* ldfix,
                  int* ideval,
                  double* f,
                  double* fjacb,
                  double* fjacd,
                  int* istop)
{
  if (*ideval > 9)
    std::cerr << "The odrpack callback requires the Jacobians, this function does not support jacobians!!" << std::endl;

  const Eigen::VectorXd beta_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(beta, *np, 1);
  const Eigen::MatrixXd x_eigen    = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned>(xplusd, *n, *m);
  const Eigen::VectorXd f_eigen    = x_eigen * beta_eigen;

  memcpy((void*)f, f_eigen.data(), static_cast<ulong>(*n) * sizeof(double));
}

bool reve::solveODR(const Eigen::VectorXd& y,
                   const Eigen::MatrixXd& x,
                   const Eigen::VectorXd& sigma_y,
                   const Eigen::MatrixXd& sigma_x,
                   Eigen::VectorXd& beta,
                   Eigen::VectorXd& sigma_beta,
                   Eigen::MatrixXd& covariance_sigma)
{
  // details see documentation of odrpack --> see p.26ff of odrpack_guide.pdf

  // check input
  assert(y.size() == sigma_y.size());
  assert(x.cols() == sigma_x.cols() && x.rows() == sigma_x.rows());
  assert(x.cols() == beta.size());
  assert(x.cols() == sigma_beta.size());
  assert(covariance_sigma.rows() == covariance_sigma.cols());
  assert(x.cols() == covariance_sigma.cols());

  int n, m, np, nq, ldy, ldx, ldwe, ld2we, ldwd, ld2wd, ldifx, job;

  // use default values
  int lunerr = -1, lunrpt = -1, ldstpd, ldscld, lwork, liwork, info = 0;
  int isodr  = 1;
  int ndigit = 0, maxit = -1, iprint = 0;
  double taufac = 0.0, sstol = -1.0, partol = -1.0;

  n = y.size();
  m = x.cols();

  np = beta.size();
  nq = 1;

  ldx = ldy = n;

  // weights y
  ldwe  = n;
  ld2we = 1;

  // weights x
  ldwd  = n;
  ld2wd = 1;

  // fix variables -> no
  int ifixb[1] = {-1};  // do not fix y
  int ifixx[1] = {-1};  // do not fix x
  ldifx        = 1;

  // explicit orthogonal distance regression; forward finite differences; Vβ and
  // σβ calculated using derivatives recomputed at solution; initialized to zero
  // by ODRPACK; fit is not a restart
  job = 0;

  double stpb[np];
  stpb[0] = -1.0;

  double stpd[m];
  stpd[0] = -1.0;
  ldstpd  = 1;

  double pstpd[m];
  pstpd[0] = -1.0;
  ldstpd   = 1;

  double pstpb[np];
  pstpb[0] = -1.0;

  double sclb[np];
  sclb[0] = -1.0;

  double scld[n];
  scld[0] = -1.0;
  ldscld  = 1;

  lwork = 18 + 11 * np + np * np + m + m * m + 4 * n * nq + 6 * n * m + 2 * n * nq * np + 2 * n * nq * m + nq * nq +
          5 * nq + nq * (np + m) + ldwe * ld2we * nq;

  isodr  = 1;
  liwork = 20 + np + nq * (np + m);

  double* work[lwork];
  double* iwork[liwork];

  // fortran: column major; Eigen default: column major --> simple use underlying data :) !

  Eigen::MatrixXd weights_model       = 1.0 / sigma_x.array().pow(2);
  Eigen::MatrixXd weights_observation = 1.0 / sigma_y.array().pow(2);

  // clang-format off
  F_FUNC(dodrc, DODRC)
  (fcn_callback,
   &n, &m, &np, &nq,
   (double *)(beta.data()),
   (double *)(y.data()), &ldy,
   (double *)(x.data()), &ldx,
   (double *)(weights_observation.data()), &ldwe, &ld2we, (double *)(weights_model.data()), &ldwd, &ld2wd,
   (int *)(ifixb), (int *)((ifixx)), &ldifx,
   &job, &ndigit, &taufac,
   &sstol, &partol, &maxit, &iprint, &lunerr, &lunrpt,
   (double *)((stpb)), (double *)((stpd)), &ldstpd,
   (double *)((sclb)), (double *)((scld)), &ldscld,
   (double *)((work)), &lwork, (int *)((iwork)), &liwork,
   &info);
  // clang-format on

  // catch bad or "questionable" result --> see p.35 of odrpack_guide.pdf
  if (info >= 5)
    return false;

  {
    // retrieve estimated sigma and covariance via convenience wrapper provided by odrpack
    // quite some overhead :(
    int delta, eps, xplus, fn, sd, vcv, rvar, wss, wssde, wssep, rcond;
    int eta, olmav, tau, alpha, actrs, pnorm, rnors, prers, partl, sstol;
    int taufc, apsma, betao, betac, betas, betan, s, ss, ssf, qraux, u;
    int fs, fjacb, we1, diff, delts, deltn, t, tt, omega, fjacd;
    int wrk1, wrk2, wrk3, wrk4, wrk5, wrk6, wrk7;

    F_FUNC(dwinf, DWINF)
    (&n,
     &m,
     &np,
     &nq,
     &ldwe,
     &ld2we,
     &isodr,
     &delta,
     &eps,
     &xplus,
     &fn,
     &sd,
     &vcv,
     &rvar,
     &wss,
     &wssde,
     &wssep,
     &rcond,
     &eta,
     &olmav,
     &tau,
     &alpha,
     &actrs,
     &pnorm,
     &rnors,
     &prers,
     &partl,
     &sstol,
     &taufc,
     &apsma,
     &betao,
     &betac,
     &betas,
     &betan,
     &s,
     &ss,
     &ssf,
     &qraux,
     &u,
     &fs,
     &fjacb,
     &we1,
     &diff,
     &delts,
     &deltn,
     &t,
     &tt,
     &omega,
     &fjacd,
     &wrk1,
     &wrk2,
     &wrk3,
     &wrk4,
     &wrk5,
     &wrk6,
     &wrk7,
     (int*)&work);

    //     convert FORTRAN indices to C indices
    sd--;
    vcv--;
    rvar--;

    sigma_beta = Eigen::Vector3d((double*)(work + sd));

    double covariance_scale = *((double*)(work + rvar));
    Eigen::Matrix3d beta_covariance_unscaled((double*)(work + vcv));
    covariance_sigma = Eigen::Matrix3d(beta_covariance_unscaled * covariance_scale);
  }

  return true;
}
