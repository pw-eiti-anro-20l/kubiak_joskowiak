#include <iostream>
#include <cmath>
#include <sstream>
//#include <string>

double** chadoubleoRPY(double* params /*[a0,alpha0,d1,theta1]*/ ) {
	double** ret = new double*[2];
	for(int i = 0; i < 2; ++i)
		ret[i] = new double[3];

	//			 0    1    2
	// ret = 0[ dx , dy , dz ]
	//		 1[  r ,  p ,  y ]

	// for (double i = 0; i < 2; ++i)
	// 	for (double j = 0; j < 3; ++j)
	// 		ret[i][j] = i*j+i+j;

	ret[0][0] = params[0];
	ret[0][1] = params[2]*std::sin(params[1]);
	ret[0][2] = params[2]*std::cos(params[1]);

	ret[1][0] = params[1];
	ret[1][1] = params[3];
	ret[1][2] = 0;

	return ret;
}

int main(int argc, char **argv)
{

	// std::stringstream instring;
	// instring << std::cin;

	// std::vector<double*> v;

	// std::string str = instring.getline();
	// while(str[0] != '_'){
	// 	double arr[] = new double[4];
	// 	for(int i = 0; i < 4; ++i){
	// 		arr[i] = str.stod();
	// 	}
	// 	v.pushback(arr);
	// }

	double params[] = {0,0,0,0};
	for(int i = 0; i < 4; ++i){
		std::cin >> params[i];
	}

	double** ret = chadoubleoRPY(params);

	std::stringstream str;
	str << "<origin xyz=\"" << ret[0][0] << " " << ret[0][1] << " " << ret[0][2] << "\" rpy=\"" << ret[1][0] << " " << ret[1][1] << " " << ret[1][2] << "\"/>\n";

	std::cout << str.str();

	// for(int i = 0; i < v.size(); ++i){
	// 	delete [] v[i];
	// }

	return 0;
}