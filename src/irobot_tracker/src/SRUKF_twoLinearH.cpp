/**
  ******************************************************************************
  * @file    SRUKF.cpp
  * @author  YANG Shuo
  * @version V1.0.0
  * @date    29-May-2014
  * @brief   This file provides SRUKF functions initialization, predict and correct 
  *          It has two measure functions, each of which are linear, matrix form
  *           
  ******************************************************************************  
  */ 
#include "irobot_tracker/SRUKF_twoLinearH.h"
/*
SRUKF::SRUKF(int _n, int _m, 
             float _q, float  _r, 
             void (*_f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt), 
             void (*_h_func)(VectorXf&, const VectorXf&), 
             float _w_m0, float _w_c0)
{
    n = _n;
    m = _m;
    f_func = _f_func;
    h_func = _h_func;

    R = _r * MatrixXf::Identity(n, n);
    Q = _q * MatrixXf::Identity(m, m);

    w_m0 = _w_m0;
    w_c0 = _w_c0;

    w_mi = 0.5 * (1-w_m0)/(float)n;
    w_ci = 0.5 * (1-w_c0)/(float)n;

    gamma = sqrt((float)n/(1-w_m0));

    state_pre.resize(n);
    state_post.resize(n);

    S_pre = MatrixXf::Identity(n,n);
    S_post = MatrixXf::Identity(n,n);

    //cout << w_mi << " - " << w_ci << " - " << gamma << endl << endl;
}
*/
SRUKF::SRUKF(int _n, int _m, 
			 float posStd_R, float ortStd_R, float imuStd_R,
			 float posStd_Q, float ortStd_Q, float imuStd_Q,
             void (*_f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt), 
             MatrixXf _H1,
             MatrixXf _H2, 
             float _w_m0, float _w_c0)
{
    n = _n;
    m = _m;
    f_func = _f_func;
    H1 = _H1;
    H2 = _H2;

    w_m0 = _w_m0;
    w_c0 = _w_c0;

    w_mi = 0.5 * (1-w_m0)/(float)n;
    w_ci = 0.5 * (1-w_c0)/(float)n;

    gamma = sqrt((float)n/(1-w_m0));

    R = MatrixXf::Identity(n, n);
    Q = MatrixXf::Identity(m, m);
	for (int i = 0; i < 3; i++)
    	R(i,i) = posStd_R;
	for (int i = 3; i < 7; i++)
    	R(i,i) = ortStd_R;
	for (int i = 7; i < 13; i++)
    	R(i,i) = imuStd_R;
	for (int i = 0; i < 3; i++)
    	Q(i,i) = posStd_Q;
	for (int i = 3; i < 7; i++)
    	Q(i,i) = ortStd_Q;
	for (int i = 7; i < 13; i++)
    	Q(i,i) = imuStd_Q;

	// give additional noise on z direction
	R(2,2) = posStd_R;
	Q(2,2) = posStd_Q;

    state_pre.resize(n);
    state_post.resize(n);

    S_pre = MatrixXf::Identity(n,n);
    S_post = MatrixXf::Identity(n,n);
}
void SRUKF::setR(float _posStd_R, float _ortStd_R, float _imuStd_R)
{
	for (int i = 0; i < 3; i++)
    	R(i,i) = _posStd_R;
	for (int i = 3; i < 7; i++)
    	R(i,i) = _ortStd_R;
	for (int i = 7; i < 13; i++)
    	R(i,i) = _imuStd_R;
}

void SRUKF::setQ(float _posStd_Q, float _ortStd_Q, float _imuStd_Q)
{
	for (int i = 0; i < 3; i++)
    	Q(i,i) = _posStd_Q;
	for (int i = 3; i < 7; i++)
    	Q(i,i) = _ortStd_Q;
	for (int i = 7; i < 13; i++)
    	Q(i,i) = _imuStd_Q;
}

void SRUKF::predict(const vector3f a, const float dt)
{
    //TODO: check if state_post is empty or not, 
    //      currently just assume state_post is ready to use
    //cout << "predict start ===========================" << endl;
    sigmaPoints.resize(0,0);
    sigmaPoints = state_post.replicate(1,2*n+1);
    sigmaPoints.block(0,1,n,n) += gamma * S_post;
    sigmaPoints.block(0,n+1,n,n) -= gamma * S_post;

    x_tmp.resize(n);
    for (int i = 0; i < 2*n+1; i++)
    {
        f_func(x_tmp, sigmaPoints.col(i), a, dt);
        sigmaPoints.col(i) = x_tmp;
    }

    state_pre = w_m0* sigmaPoints.col(0);
    for (int i = 1; i < 2*n+1; i++)
        state_pre += w_mi * sigmaPoints.col(i);

    OS.resize(n, 3*n);

    OS.block(0,0,n,2*n) = sqrt(w_ci)*(sigmaPoints.block(0,1,n,2*n)
                          - state_pre.replicate(1,2*n));
    OS.block(0,2*n,n,n) = R; 

    //cout << "OS:\n" << OS << endl << endl;
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    qrSolver.compute(OS.transpose());
    //m_q = qrSolver.householderQ();
    m_r = qrSolver.matrixQR().triangularView<Upper>();

    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    //cout << "QR error: " << (m_q*m_r - OS.transpose()).norm() << endl << endl;

    S_pre = m_r.block(0,0,n,n).transpose();
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    //cout << "S_pre before update:\n" << S_pre << endl << endl;
    //cout << "sigma point 0" << sigmaPoints.col(0).transpose() << endl << endl; 
    //cout << "state_pre:\n" << state_pre.transpose() << endl << endl; 

    if (w_c0 < 0)
        internal::llt_inplace<float,Upper>::rankUpdate(S_pre, 
            sqrt(-w_c0)*(sigmaPoints.col(0) - state_pre), 
            -1);
    else
        internal::llt_inplace<float,Upper>::rankUpdate(S_pre, 
            sqrt(w_c0)*(sigmaPoints.col(0) - state_pre), 
            1);
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    //for (int i = 1; i < n; i++)
    //    for (int j = 0; j < i; j++)
    //        S_pre(i,j) = 0;
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    /*
    if (w_c0 < 0)
        internal::ldlt_inplace<Lower>::updateInPlace(S_pre, sqrt(-w_c0)*(sigmaPoints.col(0) - state_pre), -1);
    else
        internal::ldlt_inplace<Lower>::updateInPlace(S_pre, sqrt(w_c0)*(sigmaPoints.col(0) - state_pre), 1);
    */
    //cout << "S_pre after update: \n" << S_pre << endl << endl; 
    //cout << "predict end ===========================" << endl;
}

void SRUKF::correct(const MatrixXf& H, const VectorXf& z_t)
{
    /*
    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;

    // temp3 = temp2*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);

    // temp4 = inv(temp3)*temp2 = Kt(k)
    solve(temp3, temp2, temp4, DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurementMatrix*statePre;

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain*temp5;

    // P(k) = P'(k) - K(k)*temp2
    errorCovPost = errorCovPre - gain*temp2;

    return statePost;

     */
    P = S_pre*S_pre.transpose();
    tmp1.noalias() = H*P; 
    S.noalias() = tmp1*H.transpose() + Q;

    K = S.colPivHouseholderQr().solve(tmp1);
    K.transposeInPlace();

    z_tmp = z_t - H*state_pre;

    state_post = state_pre + K*z_tmp;
    P.noalias() = P - K*tmp1;
    
    S_post = P.llt().matrixL();
}


