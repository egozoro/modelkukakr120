#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#define SEUIL 0.000001

	Eigen::MatrixXd pseudo_inverse(Eigen::MatrixXd J){
		Eigen::MatrixXd J_t = J.transpose();
		Eigen::MatrixXd JJ_t_inv = (J*J_t).inverse();
		Eigen::MatrixXd J_pseudo = J_t * JJ_t_inv ;
		std::cout<<"J_pseudo = \n"<<J_pseudo<<std::endl;

	return J_pseudo;
	}

	Eigen::MatrixXd Jacobian(Eigen::VectorXd q){

		Eigen::MatrixXd Jacobienne(6,7);
		float A = 0.240;      
		float B = 0.350;      
		float C = 1.150;      
		float D = 0.041;      
		float E = 1.2;
		float F = 0.215;

		Eigen::MatrixXd T01(4,4);
		T01 << 	1, 0, 0 ,0,
				0, 0, 1, q[0],
				0, -1, 0, 0,
				0, 0, 0, 1;

		Eigen::MatrixXd T12(4,4);
		T12 << 	cos(q[1]), -sin(q[1]), 0, 0,
				0, 0, -1, -A,
				sin(q[1]), cos(q[1]), 0, 0,
				0, 0, 0, 1;

		Eigen::MatrixXd T23(4,4);
	   	T23 <<  cos(q[2]), -sin(q[2]), 0, B,
				0, 0, -1, 0,
				sin(q[2]), cos(q[2]), 0, 0,
		 		0, 0, 0, 1;

	   	Eigen::MatrixXd T34(4,4);
	    T34 << 	cos(q[3]), -sin(q[3]), 0, C,
				sin(q[3]), cos(q[3]), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

		Eigen::MatrixXd T45(4,4);
	    T45 << 	cos(q[4]), -sin(q[4]), 0, -D,
				0, 0, -1, -E,
				sin(q[4]), cos(q[4]), 0, 0,
		 		0, 0, 0, 1;

		Eigen::MatrixXd T56(4,4);
	    T56 << 	cos(q[5]), -sin(q[5]), 0, 0,
				0, 0, 1, 0,
				-sin(q[5]), -cos(q[5]), 0, 0,
				0, 0, 0, 1;

		Eigen::MatrixXd T67(4,4);
	    T67 << 	cos(q[6]), -sin(q[6]), 0, 0,
				0, 0, -1, -F,
				sin(q[6]), cos(q[6]), 0, 0,
				0, 0, 0, 1;

	    // std::cout<< "T01 = \n"<< T01 <<std::endl;	
	    // std::cout<< "T12 = \n"<< T12 <<std::endl;	
	    // std::cout<< "T23 = \n"<< T23 <<std::endl;
	    // std::cout<< "T34 = \n"<< T34 <<std::endl;	
	    // std::cout<< "T45 = \n"<< T45 <<std::endl;	
	    // std::cout<< "T56 = \n"<< T56 <<std::endl;	
	    // std::cout<< "T67 = \n"<< T67 <<std::endl;

	    Eigen::MatrixXd T02(4,4);
	    Eigen::MatrixXd T03(4,4);
	    Eigen::MatrixXd T04(4,4);
	    Eigen::MatrixXd T05(4,4);
	    Eigen::MatrixXd T06(4,4);
	    Eigen::MatrixXd T07(4,4);

	    T02 = T01*T12;
	    T03 = T01*T12*T23;
	    T04 = T01*T12*T23*T34;
	    T05 = T01*T12*T23*T34*T45;
	    T06 = T01*T12*T23*T34*T45*T56;
	    T07 = T01*T12*T23*T34*T45*T56*T67;

	   	// std::cout<< "T02 = \n"<< T02 <<std::endl;
	   	// std::cout<< "T03 = \n"<< T03 <<std::endl;	
	   	// std::cout<< "T04 = \n"<< T04 <<std::endl;	
	   	// std::cout<< "T05 = \n"<< T05 <<std::endl;	
	   	// std::cout<< "T07 = \n"<< T07 <<std::endl;	

	//ecriture des vecteurs OiOn(R0) pour le calcul de la matrice jacobienne

	    Eigen::Vector3d O7O7;
	    O7O7 <<	0, 0, 0; 

	    Eigen::Vector3d O6O7;
	    Eigen::Vector3d vecttemp;
	    vecttemp<< 0,-F, 0;
	    O6O7 =	T06.block<3,3>(0,0) * vecttemp;

	    Eigen::Vector3d O5O7;
	    vecttemp<< 0, 0, 0;
	    O5O7 =	T05.block<3,3>(0,0) * vecttemp + O6O7;
	
		Eigen::Vector3d O4O7;
	    vecttemp<< -D, -E, 0;
	    O4O7 =	T04.block<3,3>(0,0) * vecttemp + O5O7;

	    Eigen::Vector3d O3O7;
	    vecttemp<< C, 0, 0;
	    O3O7 =	T03.block<3,3>(0,0) * vecttemp + O4O7;

	    Eigen::Vector3d O2O7;
	    vecttemp<< B, 0, 0;
	    O2O7 =	T02.block<3,3>(0,0) * vecttemp + O3O7;

	    // std::cout << "O7O7 = \n"<<O7O7<<std::endl;
	   	// std::cout << "O6O7 = \n"<<O6O7<<std::endl;
	    // std::cout << "O5O7 = \n"<<O5O7<<std::endl;
	    // std::cout << "O4O7 = \n"<<O4O7<<std::endl;
	    // std::cout << "O3O7 = \n"<<O3O7<<std::endl;
   	 // 	std::cout << "O2O7 = \n"<<O2O7<<std::endl;


	// ecriture des vecteurs zi(R0) pournle calcul de la matrice jacobienne
	   	Eigen::Vector3d z1;
	   	z1 << 0, 1, 0;

	   	Eigen::Vector3d z2;
		z2 << 0, 0, 1;

		Eigen::Vector3d z3;
		z3 << sin(q[1]), -cos(q[1]), 0;

		Eigen::Vector3d z4;
		z4 <<sin(q[1]), -cos(q[1]), 0;

		Eigen::Vector3d z5;
		z5 <<sin(q[3])*cos(q[2])*cos(q[1]) + cos(q[3])*sin(q[2])*cos(q[1]), sin(q[3])*cos(q[2])*sin(q[1]) + cos(q[3])*sin(q[2])*sin(q[1]), sin(q[3])*sin(q[2]) - cos(q[3])*cos(q[2]);

		Eigen::Vector3d z6;
		z6 <<-sin(q[4])*cos(q[3])*cos(q[2])*cos(q[1]) + sin(q[4])*sin(q[3])*sin(q[2])*cos(q[1]) + cos(q[4])*sin(q[1]), -sin(q[4])*cos(q[3])*cos(q[2])*sin(q[1]) + sin(q[4])*sin(q[3])*sin(q[2])*sin(q[1]) + cos(q[4])*cos(q[1]), -sin(q[4])*cos(q[3])*sin(q[2]) - sin(q[4])*sin(q[3])*cos(q[2]);

		Eigen::Vector3d z7;
		z7 <<sin(q[5])*cos(q[4])*cos(q[3])*cos(q[2])*cos(q[1]) - sin(q[5])*cos(q[4])*sin(q[3])*sin(q[2])*cos(q[1]) + cos(q[5])*sin(q[3])*cos(q[2])*cos(q[1]) + cos(q[5])*cos(q[3])*sin(q[2])*cos(q[1]) + sin(q[5])*sin(q[4])*sin(q[1]), -(-sin(q[5])*cos(q[4])*sin(q[3])*sin(q[2])*sin(q[1]) + sin(q[5])*cos(q[4])*cos(q[3])*cos(q[2])*sin(q[1]) + cos(q[5])*sin(q[3])*cos(q[2])*sin(q[1]) + cos(q[5])*cos(q[3])*sin(q[2])*sin(q[1]) + sin(q[5])*sin(q[4])*cos(q[1])), sin(q[5])*cos(q[4])*cos(q[3])*sin(q[2]) + sin(q[5])*cos(q[4])*sin(q[3])*cos(q[2]) + cos(q[5])*sin(q[3])*sin(q[2]) - cos(q[5])*cos(q[3])*cos(q[2]);

		std::cout << "z1 = \n"<<z1<<std::endl;
		std::cout << "z2 = \n"<<z2<<std::endl;
		std::cout << "z3 = \n"<<z3<<std::endl;
		std::cout << "z4 = \n"<<z4<<std::endl;
		std::cout << "z5 = \n"<<z5<<std::endl;
		std::cout << "z6 = \n"<<z6<<std::endl;
		std::cout << "z7 = \n"<<z7<<std::endl;


   	 	Eigen::Vector3d vecttemp2;
   	 	vecttemp2 << 0,0,0;
   	 	Jacobienne << z1, z2.cross(O2O7), z3.cross(O3O7), z4.cross(O4O7), z5.cross(O5O7), z6.cross(O6O7), z7.cross(O7O7),
        			vecttemp2, z2, z3, z4, z5, z6, z7;

    // allow to set to zero values that are under a certain threshold 
	// for (int i = 0; i< Jacobienne.rows(); i++){
	// 	for(int j=0; j < Jacobienne.cols(); j++){
	// 		if (   (Jacobienne(i,j) > -SEUIL) && (Jacobienne(i,j) < SEUIL )){
	// 			Jacobienne(i,j) = 0;
	// 			}
	//  	}
	// }


   	 	return Jacobienne;	
	}


Eigen::VectorXd inverse_kinematic(Eigen::VectorXd xDot, Eigen::MatrixXd pseudo_jacobian){

	return pseudo_jacobian * xDot;
}



void timeBetweenPoses(Eigen::VectorXd current_tcp, Eigen::VectorXd goal_tcp, double velocity, double &time){
    time = ((goal_tcp - current_tcp).norm())/velocity;
}


Eigen::VectorXd calculate_xDot(Eigen::VectorXd current_tcp, Eigen::VectorXd goal_tcp, double velocity){
	double time; 
	Eigen::VectorXd xDot(6);
	Eigen::Vector3d deltaPos;
	Eigen::Vector3d deltaRot;

	deltaPos = goal_tcp.head(3) - current_tcp.head(3);
	deltaRot = goal_tcp.tail(3) - current_tcp.tail(3);

	std::cout << "deltaPos = \n" << deltaPos<< std::endl;

	std::cout << "deltaRot = \n" << deltaRot<< std::endl;

	timeBetweenPoses(current_tcp, goal_tcp, velocity, time);
	xDot.head(3) = deltaPos/time;
	xDot.tail(3) = deltaRot/time;

return xDot;
}



int main(void){

 	Eigen::VectorXd current_q(7);
    current_q << 0, M_PI/2, M_PI/2, 0, 0, M_PI/4, 0;

	std::cout<< "q = \n"<< current_q <<std::endl;
	Eigen::MatrixXd Jacobi = Jacobian(current_q);

	std::cout<< "Jacobienne = \n"<< Jacobi << std::endl;

	Eigen::MatrixXd pseudo_inv = pseudo_inverse(Jacobi);
	std::cout<< "pseudo_inv = \n"<< pseudo_inv << std::endl;

	Eigen::VectorXd current_tcp(6);
	Eigen::VectorXd goal_tcp(6);

	current_tcp << -0.552383238, 0.406372763, 0.111879657, 3.1416, 0, 0;
	goal_tcp << -0.556596652, 0.402568498, 0.111946183, 3.1416, 0, 0;


	double velocity = 0.01;

	Eigen::VectorXd xDot = calculate_xDot(current_tcp, goal_tcp, velocity);
	std::cout << "xDot = \n"<<xDot<<std::endl;

	Eigen::VectorXd qDot = inverse_kinematic(xDot,pseudo_inv);
	std::cout << "qDot = \n" << qDot<< std::endl;

	// Eigen::MatrixXd J_pseudo_inv = Jacobi.completeOrthogonalDecomposition().pseudoInverse();
	// std::cout<< "pseudo_invcpp = \n"<< J_pseudo_inv << std::endl;

}
