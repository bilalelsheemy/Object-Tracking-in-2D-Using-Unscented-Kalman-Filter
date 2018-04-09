#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

  double px_sqrd_err;
	double py_sqrd_err;
	double vx_sqrd_err;
	double vy_sqrd_err;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0)
			{
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
			}
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); i++){
        // ... your code here
        //cout<<px_sqrd_err + pow((estimations[i][0] - ground_truth[i][0]),2)<<endl;
        px_sqrd_err = px_sqrd_err + pow((estimations[i][0] - ground_truth[i][0]),2);
				py_sqrd_err = py_sqrd_err + pow((estimations[i][1] - ground_truth[i][1]),2);
				vx_sqrd_err = vx_sqrd_err + pow((estimations[i][2] - ground_truth[i][2]),2);
				vy_sqrd_err = vy_sqrd_err + pow((estimations[i][3] - ground_truth[i][3]),2);
	}

	//calculate the mean
	// ... your code here
    rmse <<  px_sqrd_err / estimations.size(),
						py_sqrd_err / estimations.size(),
						vx_sqrd_err / estimations.size(),
						vy_sqrd_err / estimations.size();


	//calculate the squared root
	// ... your code here
	for(int n=0; n < rmse.size(); n++){

	    rmse[n] = sqrt(rmse[n]);

	}
	/*for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];
		//cout<<"Estimation"<<estimations[i]<<endl;
		//cout<<"G Truth"<<ground_truth[i]<<endl;
		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();
	//cout <<"RMSE Size"<<estimations.size()<<endl;
	//calculate the squared root
	rmse = rmse.array().sqrt();*/

	//return the result
	return rmse;
}
