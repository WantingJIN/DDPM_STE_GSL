#include <math.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include "nvwa/debug_new.h"
#include "PlumeModel.h"
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <stdexcept>

/********************************************************/
//						PlumeModel 						//
/********************************************************/

// PlumeModel::PlumeModel(const PlumeModel & another_model){
// 	nbr_param = another_model.nbr_param;

// 	param_list = new parameter[nbr_param];
// 	expected_value = new double[nbr_param];
// 	max_a_post_value = new double[nbr_param];
// 	covar_sig_ii = new double[nbr_param];

// 	for(int i=0; i<nbr_param; i++){
// 		param_list[i] = another_model.param_list[i];
// 		expected_value[i] = another_model.expected_value[i];
// 		max_a_post_value[i] = another_model.max_a_post_value[i];
// 		covar_sig_ii[i] = another_model.covar_sig_ii[i];
// 	}
// }

//destructor
// PlumeModel::~PlumeModel(){
// 	delete[] param_list;
// 	delete[] expected_value;
// 	delete[] max_a_post_value;
// 	delete[] covar_sig_ii;
// }

void PlumeModel::random_params(double * rnd_params){
	for(int i = 0; i < nbr_param; i++)
		rnd_params[i] = ((double)rand() / (double)RAND_MAX) * (param_list[i].upper_limit - param_list[i].lower_limit) + param_list[i].lower_limit;
}

void PlumeModel::get_expected_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		container[i] = expected_value[i];
	}
}

void PlumeModel::get_max_a_post_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		container[i] = max_a_post_value[i];
	}
}

void PlumeModel::set_expected_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		expected_value[i] = container[i];
	}
}

void PlumeModel::set_max_a_post_value(double* container){
	int i = 0;
	for(i = 0; i < nbr_param; i++){
		max_a_post_value[i] = container[i];
	}
}


/********************************************************/
//					GaussianPlumeModel2D 				//
/********************************************************/

//constructor : default values
GaussianPlumeModel2D::GaussianPlumeModel2D(){
	//number of parameters to estimate
	nbr_param = 5;
	
	//allocate memory for the parameters
	param_list = new parameter[nbr_param];
	
	//hard-coded parameters {name, lower_limit, upper_limit, resolution, nbins, estimated, max_a_post}
	//source_x
	param_list[0] = parameter {"source_x", 		2.75, 	12.75, 	0.5, 	20, 	0, 0};
	//source_y
	param_list[1] = parameter {"source_y", 		0,  	4, 		0.2, 	20, 	0, 0};
	//Q
	param_list[2] = parameter {"Q", 			1, 		20, 	1.9, 	10, 	0, 0};
	//sigma_a
	param_list[3] = parameter {"sigma_y_a", 	0.01,  	0.05, 	0.004, 	10, 	0, 0};
	//sigma_b
	param_list[4] = parameter {"sigma_y_b",  	0.1,   	0.5,  	0.04, 	10, 	0, 0};

	//initialize expected and max a posteriori values to zero
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	for (int i = 0; i < nbr_param; i++){
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	//MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new double[nbr_param];
	covar_sig_ii[0] = 4;	//x
	covar_sig_ii[1] = 0.1;	//y
	covar_sig_ii[2] = 4;	//Q
	covar_sig_ii[3] = 0.00001;	//sigma_y_a
	covar_sig_ii[4] = 0.0005;	//sigma_y_n
}

//destructor
GaussianPlumeModel2D::~GaussianPlumeModel2D(){
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

int GaussianPlumeModel2D::get_source_position_params_nbr(){
	return 2;
}

void GaussianPlumeModel2D::get_source_position_params(parameter * source_position_param_list){
	source_position_param_list[0] = param_list[0]; //x
	source_position_param_list[1] = param_list[1]; //y
}

//returns concentration for a set of parameters and a position
double GaussianPlumeModel2D::concentration(Position robot_position, double wind_intensity, double param_list[]){
	//ASSUMPTION : X is towards the wind speed
	//notation simplification (axis rotation, hence the signs)
	double x = param_list[0] - robot_position.x;
	double y = param_list[1] - robot_position.y;
	double z = 0;//robot_position[2] - parameters.source_position[2];

	// verify if the point is not before the source
	if(x < 0)
		return 0.0;
	else{
		//std on y and z
		double sigma_y = param_list[3] * x + param_list[4];
		double sigma_z = 1;//parameters.a_z * x + parameters.b_z;
		//double sigma_y = 0.000803  * x*x - 0.004406 * x + 0.1726;
    	//double sigma_z = 0.0006798 * x*x - 0.00244  * x + 0.1562;
		//double sigma_y = 0.01 * x + 0.1;
        //double sigma_z = 0.01 * x + 0.1;

		//model : C(x,y,z) = (Q/2pi.u.sigmaY.sigmaZ) * exp (-y^2/2sigmaY^2 - z^2/2sigmaZ^2)
		double model_value = (param_list[2]/(2*PI*wind_intensity*sigma_y*sigma_z)) * exp( -((y*y)/(2*sigma_y*sigma_y)) - ((z*z)/(2*sigma_z*sigma_z)) );
		
		return model_value;
	}
}

//returns a list of concentrations for ONE set of parameters and A LIST of positions
void GaussianPlumeModel2D::concentration(Position robot_position[], int nbr, double wind_intensity, double param_list[], vector<vector<float>> &c_avg){
	//ASSUMPTION : enough memory is already is already allocated for c_avg
	for(int i = 0; i < nbr; i++){
		c_avg[i][0] = concentration(robot_position[i], wind_intensity, param_list);
	}
}

//returns a list of concentrations for A LIST of parameters sets and A LIST of positions
// void GaussianPlumeModel2D::concentration(Position robot_position[], int nbr, double wind_intensity, vector<vector<float>> &c_avg){
// 	//for all source positions
	
// }


// void GaussianPlumeModel2D::concentration(Position robot_position[], int nbr, double wind_intensity, double param_list[], double * c_avg){
// 	//ASSUMPTION : enough memory is already is already allocated for c_avg
// 	for(int i = 0; i < nbr; i++){
// 		c_avg[i] = concentration(robot_position[i], wind_intensity, param_list);
// 	}
// }

double GaussianPlumeModel2D::approx_concentration(Position robot_position, double wind_intensity){
	return concentration(robot_position, wind_intensity, expected_value);
}

void GaussianPlumeModel2D::print_expected_val(){
	printf("Expected value: Q=%f @(%f %f) sigma y: %f %f  \n", 
		expected_value[2], expected_value[0], expected_value[1], expected_value[3], expected_value[4]);
}

void GaussianPlumeModel2D::print_max_a_post_val(){
	printf("MaxApost value: Q=%f @(%f %f) sigma y: %f %f  \n", 
		max_a_post_value[2], max_a_post_value[0], max_a_post_value[1], max_a_post_value[3], max_a_post_value[4]);
}


/********************************************************/
//					NNGaussianPlumeModel2D 				//
/********************************************************/

//constructor : default values
NNGaussianPlumeModel2D::NNGaussianPlumeModel2D(){
	//number of parameters to estimate
	nbr_param = 3;
	
	//allocate memory for the parameters
	param_list = new parameter[nbr_param];
	
	//hard-coded parameters {name, lower_limit, upper_limit, resolution, nbins, estimated, max_a_post}
	//source_x
	param_list[0] = parameter {"source_x", 		0, 		20, 	0.5, 	40, 	0, 0};
	//source_y
	param_list[1] = parameter {"source_y", 		0,  	4, 		0.2, 	20, 	0, 0};
	//Q
	param_list[2] = parameter {"Q", 			1, 		20, 	1.9, 	10, 	0, 0};

	//initialize expected and max a posteriori values to zero
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	for (int i = 0; i < nbr_param; i++){
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	//MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new double[nbr_param];
	covar_sig_ii[0] = 4;	//x
	covar_sig_ii[1] = 0.1;	//y
	covar_sig_ii[2] = 4;	//Q
}

//destructor
NNGaussianPlumeModel2D::~NNGaussianPlumeModel2D(){
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

int NNGaussianPlumeModel2D::get_source_position_params_nbr(){
	return 2;
}

void NNGaussianPlumeModel2D::get_source_position_params(parameter * source_position_param_list){
	source_position_param_list[0] = param_list[0]; //x
	source_position_param_list[1] = param_list[1]; //y
}

double NNGaussianPlumeModel2D::myNeuralNetworkFunction(const double x1[3]){
  double xp1_idx_0;
  double xp1_idx_1;
  double xp1_idx_2;
  double a;
  int k;
  static const double b_a[20] = { 23.411220186580607, 23.770728218738348,
    -60.252899750856905, 33.419520110665474, -61.556295464636804,
    51.809272455693105, -56.06596616244498, -69.85949095165671,
    -5.9488404001597575, 42.860264496631828, -0.15922146485825844,
    -15.583409690409814, -108.87830603878412, 0.01043238141446471,
    17.778806814772445, -83.586824253113662, 50.104042772017237,
    182.37477569259715, -232.30930950300865, -168.12276746340558 };

  static const double c_a[20] = { 8.6797081203720339, -10.301540349481531,
    24.73735018485916, 14.026487416391676, 8.75192314332444, 10.722929996141488,
    35.297213270821516, -9.0490768885350921, 13.061467996350604,
    -11.556081947055658, 8.6547726334454413, -12.283884024778052,
    29.611760003115503, -14.837726474975147, 10.058181148070378,
    -29.040581832952231, -9.5987802285056265, 31.313254619324145,
    31.107557196477579, -32.715222634196344 };

  static const double d_a[60] = { -0.16798480688738329, 0.57403246620838311,
    9.6632185836148015, 5.71665036493623, -0.29419821637974014,
    -0.51377152505146162, 10.8458849334019, 0.48685746344225095,
    5.595701012651654, 0.46281233355203139, -0.32702443459511582,
    0.44302028331186644, 14.755773934461548, -3.8881337951081614,
    -0.82376684633887864, -13.650172431077458, 0.68813758915220313,
    13.658845122762155, 11.950347468449007, -10.908371231331143,
    9.0722451873911236, -10.749722935157697, 14.96126715426038,
    8.1465393516362461, 9.079875168588412, 11.245126000484026,
    23.785065240875465, -9.277801077029757, 8.14347475185832,
    -12.190386682808992, 8.6271444636618764, -13.004006590716719,
    14.870799905120125, -11.689012933456102, 10.130450860078113,
    -15.545636650733067, -9.7323145015758481, 17.351721310114044,
    18.726845664962351, -21.239509783579148, -0.69205477689227823,
    0.0439602388946877, -1.3486173743862258, -0.74200085256496018,
    -0.7722429778322305, 0.032596006380215425, -2.7871473641402762,
    0.95267661741982423, -0.73360078954319186, -0.15279702047121993,
    -1.8370324868069403, -0.23916291828896682, -1.340730652109551,
    -2.4403330378255417, -1.3165404138000798, 1.3926473907035617,
    1.166368016190247, -1.6985890403016244, -1.9981548340043365,
    2.4147201157992773 };

  /* MYNEURALNETWORKFUNCTION neural network simulation function. */
  /*  */
  /*  Auto-generated by MATLAB, 18-Mar-2020 11:14:51. */
  /*  */
  /*  [y1] = myNeuralNetworkFunction(x1) takes these arguments: */
  /*    x = Qx3 matrix, input #1 */
  /*  and returns: */
  /*    y = Qx1 matrix, output #1 */
  /*  where Q is the number of samples. */
  /*  ===== NEURAL NETWORK CONSTANTS ===== */
  /*  Input 1 */
  /*  Layer 1 */
  /*  Layer 2 */
  /*  Output 1 */
  /*  ===== SIMULATION ======== */
  /*  Dimensions */
  /*  samples */
  /*  Input 1 */
  /*  ===== MODULE FUNCTIONS ======== */
  /*  Map Minimum and Maximum Input Processing Function */
  xp1_idx_0 = (x1[0] - -0.99999) * 0.100000375001406 + -1.0;
  xp1_idx_1 = x1[1] * 0.571860407730752 + -1.0;
  xp1_idx_2 = (x1[2] - 4.54545454545454) * 0.00814814814814815 + -1.0;

  /*  Layer 1 */
  /*  Sigmoid Symmetric Transfer Function */
  /*  Layer 2 */
  /*  Output 1 */
  /*  Map Minimum and Maximum Output Reverse-Processing Function */
  a = 0.0;
  for (k = 0; k < 20; k++) {
    a += b_a[k] * (2.0 / (std::exp(-2.0 * (c_a[k] + ((d_a[k] * xp1_idx_0 + d_a[k
      + 20] * xp1_idx_1) + d_a[k + 40] * xp1_idx_2))) + 1.0) - 1.0);
  }

  return ((a + -5.0298082711685232) - -1.0) / 0.000715946427197975;
}

//returns concentration for a set of parameters and a position
double NNGaussianPlumeModel2D::concentration(Position robot_position, double wind_intensity, double param_list[]){
	//ASSUMPTION : X is towards the wind speed
	double input[3] = {0};

	input[0] = param_list[0] - robot_position.x; 	//X
	input[1] = abs(param_list[1] - robot_position.y);	//Y
	input[2] = param_list[2] / wind_intensity; 		//Q/U

	// call the neural net function
	return myNeuralNetworkFunction(input);
}

double NNGaussianPlumeModel2D::approx_concentration(Position robot_position, double wind_intensity){
	return concentration(robot_position, wind_intensity, expected_value);
}

void NNGaussianPlumeModel2D::print_expected_val(){
	printf("Expected value: Q=%f @(%f %f) \n", 
		expected_value[2], expected_value[0], expected_value[1]);
}

void NNGaussianPlumeModel2D::print_max_a_post_val(){
	printf("MaxApost value: Q=%f @(%f %f)  \n", 
		max_a_post_value[2], max_a_post_value[0], max_a_post_value[1]);
}


/********************************************************/
//					DeepCFDPlumeModel2D 				//
/********************************************************/

//constructor : default values
DeepCFDPlumeModel2D::DeepCFDPlumeModel2D(){
	//number of parameters to estimate
	nbr_param = 2;
	
	//allocate memory for the parameters
	param_list = new parameter[nbr_param];
	
	//hard-coded parameters {name, lower_limit, upper_limit, resolution, nbins, estimated, max_a_post}
	//source_x
	param_list[0] = parameter {"source_x", 		2.75, 	12.75, 	0.5, 	20, 	0, 0};
	//source_y
	param_list[1] = parameter {"source_y", 		0,  	4, 		0.2, 	20, 	0, 0};

	//initialize expected and max a posteriori values to zero
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	for (int i = 0; i < nbr_param; i++){
		expected_value[i] = 0;
		max_a_post_value[i] = 0;
	}

	//MCMC proposal distribution's covariance sigma_i,i
	covar_sig_ii = new double[nbr_param];
	covar_sig_ii[0] = 2;	//x
	covar_sig_ii[1] = 0.1;	//y
}

DeepCFDPlumeModel2D::DeepCFDPlumeModel2D(const DeepCFDPlumeModel2D & another_model){
	nbr_param = another_model.nbr_param;

	param_list = new parameter[nbr_param];
	expected_value = new double[nbr_param];
	max_a_post_value = new double[nbr_param];
	covar_sig_ii = new double[nbr_param];

	for(int i=0; i<nbr_param; i++){
		param_list[i] = another_model.param_list[i];
		expected_value[i] = another_model.expected_value[i];
		max_a_post_value[i] = another_model.max_a_post_value[i];
		covar_sig_ii[i] = another_model.covar_sig_ii[i];
	}
}

//destructor
DeepCFDPlumeModel2D::~DeepCFDPlumeModel2D(){
	delete[] param_list;
	delete[] expected_value;
	delete[] max_a_post_value;
	delete[] covar_sig_ii;
}

int DeepCFDPlumeModel2D::get_source_position_params_nbr(){
	return 2;
}

void DeepCFDPlumeModel2D::get_source_position_params(parameter * source_position_param_list){
	source_position_param_list[0] = param_list[0]; //x
	source_position_param_list[1] = param_list[1]; //y
}

// void DeepCFDPlumeModel2D::random_params(double * rnd_params){
// 	for(int i = 0; i < nbr_param; i++)
// 		rnd_params[i] = ((double)rand() / (double)RAND_MAX) * (param_list[i].upper_limit - param_list[i].lower_limit) + param_list[i].lower_limit;
// }

//returns concentration for ONE set of parameters and ONE position
double DeepCFDPlumeModel2D::concentration(Position robot_position, double wind_intensity, double param_list[]){
	// std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	float sx = param_list[0];
	float sy = param_list[1];
	float px = robot_position.x;
	float py = robot_position.y;
	double c_avg = 0;

	// the name of the python file that will be used
	CPyObject pName = PyUnicode_FromString("Myapply_DeepCFD");
	// import that file as a module
	CPyObject pModule = PyImport_Import(pName);
	// make the argument list that will be fed to the function
	CPyObject args = Py_BuildValue("(ffff)", sx, sy, px, py);

	// if the module is well imported
	if(pModule){
		// find the function
		CPyObject pFunc = PyObject_GetAttrString(pModule, "DeepCFD_output_eval");
		// in the function is found and is callable
		if(pFunc && PyCallable_Check(pFunc)){
			// call the function with the argument list and get the output
			CPyObject pValue = PyObject_CallObject(pFunc, args);
			// printf("Model values C_avg = %f, C_std = %f\n", PyFloat_AsDouble(PyTuple_GetItem(pValue,0)), PyFloat_AsDouble(PyTuple_GetItem(pValue,1)));
			// convert the output to c++-readable values and set in the pointers
			c_avg = PyFloat_AsDouble(PyTuple_GetItem(pValue,0));
			//c_std = PyFloat_AsDouble(PyTuple_GetItem(pValue,1));
		}
		else
			printf("ERROR: function DeepCFD_output()\n");
	}else
		printf("ERROR: apply_DeepCFD Module not imported\n");
	
	// return the average concentration predicted by the model
	// std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
	return c_avg;
}

// helper function for the one bellow
vector<float> listTupleToVector_Int(PyObject* incoming) {
	vector<float> data;
	if (PyTuple_Check(incoming)) {
		for(Py_ssize_t i = 0; i < PyTuple_Size(incoming); i++) {
			PyObject *value = PyTuple_GetItem(incoming, i);
			data.push_back( PyFloat_AsDouble(value) );
		}
	} else {
		if (PyList_Check(incoming)) {
			for(Py_ssize_t i = 0; i < PyList_Size(incoming); i++) {
				PyObject *value = PyList_GetItem(incoming, i);
				data.push_back( PyFloat_AsDouble(value) );
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list or tuple!");
		}
	}
	return data;
}

//returns a list of concentrations for ONE set of parameters and A LIST of positions
void DeepCFDPlumeModel2D::concentration(Position robot_position[], int nbr, double wind_intensity, double param_list[], vector<vector<float>> &c_avg, vector<vector<float>> &c_std){
	// nbr here stands for the number of gathered measurements instead of the number of robots

	float sx = param_list[0];
	float sy = param_list[1];
	float px[nbr];
	float py[nbr];

	for(int i=0; i<nbr; i++){
		px[i] = robot_position[i].x;
		py[i] = robot_position[i].y;
	}

	// import the python file as a module
	CPyObject pModule = PyImport_Import(PyUnicode_FromString("Myapply_DeepCFD"));
	// make the arguments with the provided values
	npy_intp dims[1] = {nbr};
    import_array(); 
	CPyObject pArgs = PyTuple_New(4);
    PyTuple_SetItem(pArgs, 0, PyFloat_FromDouble(sx));
    PyTuple_SetItem(pArgs, 1, PyFloat_FromDouble(sy));
    PyTuple_SetItem(pArgs, 2, PyArray_SimpleNewFromData(1, dims, NPY_FLOAT, px));
    PyTuple_SetItem(pArgs, 3, PyArray_SimpleNewFromData(1, dims, NPY_FLOAT, py));

	// if the module is well imported
	if(pModule){
		// find the function
		CPyObject pFunc = PyObject_GetAttrString(pModule, "DeepCFD_output_eval");
		// in the function is found and is callable
		if(pFunc && PyCallable_Check(pFunc)){
			// call the function with the argument list and get the output
			CPyObject pValue = PyObject_CallObject(pFunc, pArgs);
			// convert the output to c++-readable values and set in the pointers
			for(int i = 0; i < nbr; i++){
				//printf("We are in here %f \n", PyFloat_AsDouble(PyList_GetItem(PyTuple_GetItem(pValue,0),i)));
				c_avg[i] = listTupleToVector_Int(PyList_GetItem(PyTuple_GetItem(pValue,0),i));
				c_std[i] = listTupleToVector_Int(PyList_GetItem(PyTuple_GetItem(pValue,1),i));
				// printf("Model values C_avg[i]=");
				// for (auto c_avg_iter : c_avg[i])
                // {
                //   printf("%f", c_avg_iter);
                // }
				// printf("\n");
			}
		}else
			printf("ERROR: function DeepCFD_output()\n");
	}else
		printf("ERROR: apply_DeepCFD Module not imported\n");
}


double DeepCFDPlumeModel2D::approx_concentration(Position robot_position, double wind_intensity){
	return concentration(robot_position, wind_intensity, expected_value);
}

void DeepCFDPlumeModel2D::print_expected_val(){
	printf("Expected value: @(%f %f)\n", 
		expected_value[0], expected_value[1]);
}

void DeepCFDPlumeModel2D::print_max_a_post_val(){
	printf("MaxApost value: @(%f %f)\n", 
		max_a_post_value[0], max_a_post_value[1]);
}
