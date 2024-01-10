#ifndef POSTERIOR_H
#define POSTERIOR_H

#include "controller_STE_multi-robot.h"
#include "PlumeModel.h"
#include "Position.h"

class Posterior{ 
    public: 
    
    double sum; 
    double max;
    double entropy;
    int dirac_flag;
    int nbr_dimension;
    int nbr_element;
    double * pdf; 
    int nbr_element_marginal;
    double * source_marginal_pdf;
    PlumeModel * model;


    //constructor : uniform distribution
    Posterior(PlumeModel * input_model);
    //constructor : empty distribution (all elements 0)
    Posterior(PlumeModel * input_model, int zero);
    //copy constructor
    Posterior(const Posterior & to_copy_from);
    //destructor
    ~Posterior();

    //reset the pdfs and their stats to 0
    void pdf_reset();

    //this function takes an array of indices for a multi-dimensional matrix and generates a single index for an array of the same number of elements
    int indices_to_index(int *indices, int a_nbr_dimension, int a_nbr_element);
    int indices_to_index(int *indices);
    //this function takes a single index in a multi-dimensional matrix and generates the indices for all the dimensions
    void index_to_indices(int index, int *indices);
    //this function takes a single index in a multi-dimensional matrix and returns the values in the matrix corresponding to that index
    void index_to_param_values(int index, double *bin_values);

    //update the posterior pdf with the value given by sample paramter set
    void update(double value, double *sample);
    //normalization + entropy + expected value + max a posterior (value and parameter set)
    void process();

    //efficient sampling through MCMC
    void MCMC_evaluation(struct history history);
    //exhaustive evaluation of all the values of the posterior pdf
    void exhaustive_evaluation(struct history history);

    //relative entropy calculation between one posterior pdf and another P
    double Kullback_Leibler_divergence(Posterior P);
    //calculates the marginal posterior pdf for source position on X and Y, multiplied by the distance inverse pdf 
    //and then generates the best goal point based on the two pdfs 
    void XYmarginal_distance_combined(Position current_position, double ratio, double * best_point);

    //log the posterior pdf
    void log_pdf(const char * file_name, char writing_mode);
    void log_marginal_pdf(const char * file_name, char writing_mode);
}; 

#endif