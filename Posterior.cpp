#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <vector>
#include "nvwa/debug_new.h"
#include "Posterior.h"

struct normal_random_variable{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](double x) { return dist(gen); });
    }
};

//constructor : uniform distribution
Posterior::Posterior(PlumeModel *input_model){
	int i=0;
	nbr_element = 1;

	//model
	model = input_model;
	nbr_dimension = model->nbr_param;
	for(i = 0; i < nbr_dimension; i++)
		nbr_element *= model->param_list[i].nbins;

	//stats
    sum = 1;
    max = 1/(float)nbr_element;
    dirac_flag = 0;
    entropy = 0;

    //pdf
    //pdf = (double*) malloc (nbr_element*sizeof(double));
	pdf = new double[nbr_element];
	for(i = 0; i < nbr_element; i++){
		pdf[i] = 1/(float)nbr_element;
		entropy += pdf[i] * log(pdf[i]);
	}
    entropy = (-1)*entropy;

    //max a post and expected values
    for(i = 0; i < nbr_dimension; i++){
    	model->max_a_post_value[i] = 	(float)(model->param_list[i].upper_limit - model->param_list[i].lower_limit) / (float)(2*model->param_list[i].nbins) + model->param_list[i].lower_limit;
    	model->expected_value[i] = 		(float)(model->param_list[i].upper_limit - model->param_list[i].lower_limit) / 2.0 + model->param_list[i].lower_limit;
    }

    //marginal
	nbr_element_marginal = 1;
	int nbr_source_position_param = model->get_source_position_params_nbr();
	for(i = 0; i < nbr_source_position_param; i++)
		nbr_element_marginal *= model->param_list[i].nbins;
	source_marginal_pdf = new double[nbr_element_marginal];
	for(i = 0; i < nbr_element_marginal; i++)
		source_marginal_pdf[i] = 1/(float)nbr_element_marginal;

    //printf("expected_value: %f %f %f %f %f\n", 
	//	model->expected_value[0], model->expected_value[1], model->expected_value[2], model->expected_value[3], model->expected_value[4]);
}

//constructor : empty distribution (all elements 0)
Posterior::Posterior(PlumeModel *input_model, int zero){
	int i=0;
	nbr_element = 1;

    //model
	model = input_model;
	nbr_dimension = model->nbr_param;
	for(i = 0; i < nbr_dimension; i++)
		nbr_element *= model->param_list[i].nbins;

	//stats
    sum = 0;
    max = 0;
    dirac_flag = 0;
    entropy = 1;

    //pdf
    //pdf = (double*) malloc (nbr_element*sizeof(double));
	pdf = new double[nbr_element];
	for(i = 0; i < nbr_element; i++)
		pdf[i] = 0;

	//max a post and expected values
    for(i = 0; i < nbr_dimension; i++){
    	model->max_a_post_value[i] = (float)(model->param_list[i].upper_limit - model->param_list[i].lower_limit) / (float)(2*model->param_list[i].nbins) + model->param_list[i].lower_limit;
    	model->expected_value[i] = (float)(model->param_list[i].upper_limit - model->param_list[i].lower_limit) / 2.0 + model->param_list[i].lower_limit;
    }

    //marginal
	nbr_element_marginal = 1;
	int nbr_source_position_param = model->get_source_position_params_nbr();
	for(i = 0; i < nbr_source_position_param; i++)
		nbr_element_marginal *= model->param_list[i].nbins;
	source_marginal_pdf = new double[nbr_element_marginal];
	for(i = 0; i < nbr_element_marginal; i++)
		source_marginal_pdf[i] = 0;
}

//copy constructor
Posterior::Posterior(const Posterior & to_copy_from){
	int i=0;
	sum = to_copy_from.sum; 
    max = to_copy_from.max;
    entropy = to_copy_from.entropy;
    dirac_flag = to_copy_from.dirac_flag;
    nbr_dimension = to_copy_from.nbr_dimension;
    nbr_element = to_copy_from.nbr_element;
    nbr_element_marginal = to_copy_from.nbr_element_marginal;
    //pdf
    pdf = new double[nbr_element];
    for(i = 0; i < nbr_element; i++)
    	pdf[i] = to_copy_from.pdf[i];
    //marginal
    source_marginal_pdf = new double[nbr_element_marginal];
    for(i = 0; i < nbr_element_marginal; i++)
    	source_marginal_pdf[i] = to_copy_from.source_marginal_pdf[i];
    //model
    model = to_copy_from.model;
}

//destructor
Posterior::~Posterior(){
	delete[] pdf;
	delete[] source_marginal_pdf;
}

//copy the pdf of a posterior in this one
void Posterior::pdf_reset(){
	int i=0;
	sum = 0; 
    max = 0;
    entropy = 0;
    dirac_flag = 0;
    for(i = 0; i < nbr_element; i++)
    	pdf[i] = 0;
    for(i = 0; i < nbr_element_marginal; i++)
    	source_marginal_pdf[i] = 0;
}

//this function takes an array of indices for a multi-dimensional matrix and generates a single index for an array of the same number of elements
int Posterior::indices_to_index(int *indices, int a_nbr_dimension, int a_nbr_element){
	int i=0, j=0;
	int index = indices[0];
	int factor = 1;

	for(i = 1; i < a_nbr_dimension; i++){
		for(j = i-1; j >= 0; j--)
			factor *= model->param_list[j].nbins;
		index += indices[i]*factor;
		factor = 1;
	}
	//error detection
	if(index < 0 || index >= a_nbr_element){ 
		printf("ERROR if indices_to_index : index = %d\n", index);
		index = 0;
	}
	return index;
}

//this function takes an array of indices for a multi-dimensional matrix and generates a single index for an array of the same number of elements
int Posterior::indices_to_index(int *indices){
	//overloaded with Posterior values
	return indices_to_index(indices, nbr_dimension, nbr_element);
}

//this function takes a single index in a multi-dimensional matrix and generates the indices for all the dimensions
void Posterior::index_to_indices(int index, int *indices){
	int i=0;
	int factor = 1;
	int residu = 0;

	//first element
	indices[0] = index % model->param_list[0].nbins;
	factor = model->param_list[0].nbins;
	residu = indices[0];
	//the rest
	for(i = 1; i < nbr_dimension; i++){
		indices[i] = ((index - residu)/factor) % model->param_list[i].nbins;
		factor *= model->param_list[i].nbins;
		residu += indices[i]*model->param_list[i-1].nbins;
		//error detection
		if(indices[i] < 0 || indices[i] >= model->param_list[i].nbins){
			printf("ERROR in index_to_indices: indices[%d] = %d\n", i , indices[i]);
			indices[i] = 0;
		}
	}
}

//this function takes a single index in a multi-dimensional matrix and returns the values in the matrix corresponding to that index
void Posterior::index_to_param_values(int index, double *bin_values){
	int j = 0;
	int indices[nbr_dimension];

	//convert the single index to a set of indices for different dimensions
	index_to_indices(index, indices);

	//calculate the bin center value based on the index of each dimension
	for(j = 0; j < nbr_dimension; j++){
		//bin_values[j] = (((float)(model->param_list[j].upper_limit - model->param_list[j].lower_limit) / (float)(2*model->param_list[j].nbins)) * (2*indices[j]+1) + model->param_list[j].lower_limit);
		bin_values[j] = (model->param_list[j].resolution/2.0) * (2*indices[j]+1) + model->param_list[j].lower_limit;
		//error detection
		if(bin_values[j] < model->param_list[j].lower_limit || bin_values[j] > model->param_list[j].upper_limit){
			printf("ERROR in index_to_param_values: bin_values[%d] = %f \n", j, bin_values[j]);
			bin_values[j] = 0;
		}
	}
}

//update the posterior pdf with the value given by sample parameter set
void Posterior::update(double value, double *sample){ 
	int i=0;
	int indices[nbr_dimension];

	//convert sample values to indices
	for(i = 0; i < nbr_dimension; i++){
		// make sure sample has proper values
		if(std::isnan(sample[i])){
			printf("ERROR in update: sample is NaN  \n");
			value = 0;
		}
		//indices[i] = (int)((sample[i]-model->param_list[i].lower_limit) / ((float)(model->param_list[i].upper_limit-model->param_list[i].lower_limit)/(float)(model->param_list[i].nbins)));
		indices[i] = (int)( (sample[i] - model->param_list[i].lower_limit) / (float)model->param_list[i].resolution);
		//error detection
		if(indices[i] < 0 || indices[i] >= model->param_list[i].nbins){
			printf("ERROR in update: indices[%d] = %d - sample: %f - min: %f - res: %f\n", i, indices[i], sample[i], model->param_list[i].lower_limit,model->param_list[i].resolution);
			indices[i] = 0;
		}
	}
	pdf[indices_to_index(indices)] += value;
	sum += value;
}

//normalization + entropy + expected value + max a posterior (value and parameter set) using only the pdf
void Posterior::process(){ 
	int i=0, j=0;
	int indices[nbr_dimension];
	double bin_values[nbr_dimension];
	max = 0;
	entropy = 0; 
	for(j = 0; j < nbr_dimension; j++)
		model->expected_value[j] = 0;
	
	//for every value in the pdf
	for(i = 0; i < nbr_element; i++){
		//normalize
		//printf("%f / %f =  %f ", pdf[i],sum, pdf[i]/sum);
		if (sum == 0){
			printf("!!!!!!!!!!!!!sum is equal to zero!\n");
		}
		pdf[i] /= sum;
		
		//entropy
		if(pdf[i] > 0) 
			entropy += pdf[i] * log(pdf[i]);
		// find the set of param corresponding to index i
		index_to_param_values(i, bin_values);
		//max value and max a post value
		if(pdf[i] > max){
			max = pdf[i];
			memcpy(model->max_a_post_value, bin_values, sizeof(double)*nbr_dimension); 
		}
		//expected value
		index_to_indices(i, indices);
		for(j = 0; j < nbr_dimension; j++)
			model->expected_value[j] += bin_values[j] * pdf[i];
		//marginal
		source_marginal_pdf[indices_to_index(indices, model->get_source_position_params_nbr(), nbr_element_marginal)] += pdf[i];
	}
	entropy = -1 * entropy;
	//printf("\n");
    printf("here4\n");
	//log
	//const char* str = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/pdf.csv";
	//log_pdf(str, 'a');
	// const char* str2 = "/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/pdf_marginal.csv";
	// log_marginal_pdf(str2, 'a');
	printf("here5\n");
	//print on screen
	if(VERBOSE) model->print_expected_val();
	if(VERBOSE) model->print_max_a_post_val();
}

//efficient sampling through MCMC
void Posterior::MCMC_evaluation(struct history history){
	int i=0, j=0, count = simulation_parameters.MCMC_iteration_num; //number of iterations
	double p_acceptance = 0, posterior_value = 0;
	int acceptance_rate = 0;
	double sample[nbr_dimension];
	double last_sample[nbr_dimension];
	double candidate[nbr_dimension];
	
	//initial values (maybe randomize later)
	model->get_expected_value(sample); //copy expected value in last sample
	
	//proposal distribution : 5D Gaussian
	Eigen::VectorXd mean(nbr_dimension);
	Eigen::MatrixXd covar(nbr_dimension,nbr_dimension);

	//covariance of the proposal distribution given by the plume model
	for(i = 0; i < nbr_dimension; i++){
		for(j = 0; j < nbr_dimension; j++){
			if(i == j)
				covar(i,j) = model->covar_sig_ii[i];
			else
				covar(i,j) = 0;
		}
	}
	
	// //error detection
	// double sample_likelihood = likelihood(history, sample, model);
	// // if the expected value is not a valid sample to start with
	// if(prior(sample, nbr_dimension, model) == 0 || sample_likelihood == 0){
	// 	// we start with the max a posteriori value
	// 	model->get_max_a_post_value(sample);
	// 	sample_likelihood = likelihood(history, sample, model);
	// }
	// // if max a post value is also not a valid sample
	// while(prior(sample, nbr_dimension, model) == 0 || sample_likelihood == 0){
	// 	if(VERBOSE) printf("done ---- prior = %d, likelihood = %f, sample: %f %f %f %f %f\n", 
	// 		prior(sample, nbr_dimension, model), sample_likelihood, sample[0], sample[1], sample[2], sample[3], sample[4]);
	// 	// we choose a sample randomly
	// 	model->random_params(sample);
	// 	sample_likelihood = likelihood(history, sample, model);
	// 	printf("Started from random sample\n");
	// }

	// check if expected value is in obstacle and relocate if so
	// Position tmp_p(sample[0], sample[1], 0);
	// goal_outside_obstacles(&tmp_p);
	// sample[0] = tmp_p.x; 
	// sample[1] = tmp_p.y;

	//error detection
	double sample_likelihood = likelihood(history, sample, model);
	if(prior(sample, nbr_dimension, model) == 0 || sample_likelihood == 0){
		if(VERBOSE) printf("done ---- prior = %d, likelihood = %f, sample: %f %f %f %f %f\n", 
			prior(sample, nbr_dimension, model), sample_likelihood, sample[0], sample[1], sample[2], sample[3], sample[4]);
		dirac_flag = 1;
		return;
	}

	posterior_value = prior(sample, nbr_dimension, model) * sample_likelihood;
	update(posterior_value, sample);


	//get model concentrations
	// if(history.size==1){
	// 	vector<vector<float>> model_c_avg(history.size, vector<float> (model->param_list[0].nbins * model->param_list[1].nbins)); 
	// 	model->concentration(history.position, history.size, 0, model_c_avg);
	// 	// FILE* model_values = fopen("/data/FaezehRahbar/Projects/Data-driven_plume_model/webots/model_values.csv","w");
	// 	// std::ofstream output_file("/data/FaezehRahbar/Projects/Data-driven_plume_model/webots/model_values.csv");
 //  //   	std::ostream_iterator<float> output_iterator(output_file, "\n");
 //  //   	std::copy(model_c_avg.begin(), model_c_avg.end(), output_iterator);
	// 	for(const auto& row : model_c_avg) {
 //    		std::copy(row.cbegin(), row.cend(), std::ostream_iterator<float>(std::cout, " "));
 //  			std::cout << '\n';
 //  		}
 //    }
	
	// copy sample and last_sample before starting
	memcpy(last_sample, sample, sizeof(double)*nbr_dimension);
	//sampling
	for(i=0; i<count; i++){
		//////////draw a point to jump to from a normal dist centered on the previous value
		
		// Set the mean of the proposal distribution to the last sample
		for(j = 0; j < nbr_dimension; j++)
			mean(j) = sample[j];
		
		//draw a candidate sample
		normal_random_variable random_sample{mean, covar};
		for(j = 0; j < nbr_dimension; j++)
			candidate[j] = random_sample()(j);
		
		//calculate the acceptance chance
		double candidate_likelihood = likelihood(history, candidate, model);
		p_acceptance = (prior(candidate, nbr_dimension, model) * candidate_likelihood) /
					   (prior(sample, nbr_dimension, model) * sample_likelihood);
		if(p_acceptance > 1) p_acceptance = 1;
		
		//accept if a random value higher than the acceptance chance
		if(randu() < p_acceptance){
			//set the next value
			memcpy(sample, candidate, sizeof(double)*nbr_dimension);
			sample_likelihood = candidate_likelihood;
			acceptance_rate++;
		}else
			continue;
		
		//use the new sample to evaluate the posterior pdf
		posterior_value = prior(sample, nbr_dimension, model) * sample_likelihood;
		update(posterior_value, sample);
	}
	memcpy(last_sample, sample, sizeof(double)*nbr_dimension);
	
	//print of screen
	if(acceptance_rate == 0 && VERBOSE)
		printf("prior : %d, likelihood : %f, p_acceptance: %f \n", prior(sample, nbr_dimension, model), sample_likelihood, p_acceptance);
	if(FSM.state == STATE_ESTIMATING && VERBOSE)
		printf("acceptance rate : %f  \n", (float)acceptance_rate/count);
}

//exhaustive evaluation of all the values of the posterior pdf
void Posterior::exhaustive_evaluation(struct history history){
	int i=0;
	double sample[nbr_dimension];

	//for each element of the pdf
	for(i = 0; i < nbr_element; i++){
		//convert index 'i' to set of parameter values 'sample'
		index_to_param_values(i, sample);
		//calculate the posterior of 'sample'
		pdf[i] = prior(sample, nbr_dimension, model) * likelihood(history, sample, model);
		//printf("%f, ", pdf[i]);
		//add the new value to the sum
		sum += pdf[i];
	}
	//printf("\n");
}

//relative entropy calculation between one posterior pdf and another P
double Posterior::Kullback_Leibler_divergence(Posterior P){
	double DKL = 0;
	int i=0;
	//for each element of the pdf
	for(i = 0; i < nbr_element; i++){
		if(pdf[i] > 0 && P.pdf[i] > 0){
			//DKL += ((P.pdf[i]+0.00000000000001)/P.sum) * log(((P.pdf[i]+0.00000000000001)/P.sum) / (pdf[i]+0.00000000000001));
			DKL += (P.pdf[i]/P.sum) * log((P.pdf[i]/P.sum) / pdf[i]);
			// P is not normalized, but its sum was calculated and updated
		}
	}
	return DKL;
}

//calculates the marginal posterior pdf for source position on X and Y, multiplied by the distance inverse pdf 
//and then generates the best goal point based on the two pdfs 
void Posterior::XYmarginal_distance_combined(Position current_position, double ratio, double *best_point){
	int i=0,j=0;
	double marginal = 0, value = 0;
	double tmp_position[2] = {0};
	double inv_dist [model->param_list[0].nbins][model->param_list[1].nbins];
	double sum_inv_distance = 0;
	int indices[nbr_dimension];
	
	//find the current cell number based on the current position
	int current_cell[2] = {	(int)( (current_position.x-model->param_list[0].lower_limit) / ((float)model->param_list[0].resolution) ), 
							(int)( (current_position.y-model->param_list[1].lower_limit) / ((float)model->param_list[1].resolution) )};
	
	//calculate distance inverse 'inv_dist' based on current cell 'current_cell'
	for(i = 0; i < model->param_list[0].nbins; i++){
		for(j = 0; j < model->param_list[1].nbins; j++){
			// check if we are in the current cell
			if(i == current_cell[0] && j == current_cell[1]){
				inv_dist[i][j] = 0;
				continue;
			}
			//calculate position (cell center) based on i and j
			tmp_position[0] = (model->param_list[0].resolution)/2.0 * (2*i+1) + model->param_list[0].lower_limit;
			tmp_position[1] = (model->param_list[1].resolution)/2.0 * (2*j+1) + model->param_list[1].lower_limit;
			//calculate distance inverse and its sum
			inv_dist[i][j] = 1 / (sqrt(pow(current_position.x-tmp_position[0], 2) + pow(current_position.y-tmp_position[1], 2)));
			sum_inv_distance += inv_dist[i][j];
		}
	}
	
	//calculate the marginal
	for(i = 0; i < model->param_list[0].nbins; i++){
		for(j = 0; j < model->param_list[1].nbins; j++){
			//calculate position (cell center) based on i and j
			tmp_position[0] = (model->param_list[0].resolution)/2.0 * (2*i+1) + model->param_list[0].lower_limit;
			tmp_position[1] = (model->param_list[1].resolution)/2.0 * (2*j+1) + model->param_list[1].lower_limit;
			
			//marginal for each cell
			indices[0] = i; indices[1] = j;
			marginal = source_marginal_pdf[indices_to_index(indices, model->get_source_position_params_nbr(), nbr_element_marginal)];

			//combined value
			double tmp_value = (ratio * marginal) + ((1-ratio) * (inv_dist[i][j]/sum_inv_distance));//marginal - ratio*(distance/MAX_DISTANCE);
			
			//if the cell(i,j) is the best so far, its center is copied in best_point
			if(tmp_value > value){
				value = tmp_value;
				best_point[0] = tmp_position[0]; 
				best_point[1] = tmp_position[1]; 
			}
		}
	}
}

//log the posterior pdf
void Posterior::log_pdf(const char * file_name, char writing_mode){
	int i=0;
	FILE * posterior_pdf_3D = fopen(file_name, &writing_mode);

	for(i = 0; i < nbr_element-1; i++)
		fprintf(posterior_pdf_3D, "%.9f, ", pdf[i]);

	fprintf(posterior_pdf_3D, "%.9f\n", pdf[nbr_element-1]);

	fclose(posterior_pdf_3D);
}

void Posterior::log_marginal_pdf(const char * file_name, char writing_mode){
	int i=0;
	FILE * posterior_pdf_marginal = fopen(file_name, &writing_mode);
	
	for(i = 0; i < nbr_element_marginal-1; i++)
		fprintf(posterior_pdf_marginal, "%f ", source_marginal_pdf[i]);
	
	fprintf(posterior_pdf_marginal, "%f\n", source_marginal_pdf[i]);

	fclose(posterior_pdf_marginal);
}