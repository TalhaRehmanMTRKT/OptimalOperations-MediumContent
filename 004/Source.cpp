#include "Header.h"
#include <ilcplex/ilocplex.h>

int main() {
    
    int T = 24;
    double* c_buy = new double[T] {  90, 90, 90, 90, 90, 90, 110, 110, 110, 110, 110, 125, 125, 125, 125, 125, 125, 125, 110, 110, 110, 110, 110, 110 };
    double* c_sell = new double[T] { 70, 70, 70, 70, 70, 70, 90, 90, 90, 90, 90, 105, 105, 105, 105, 105, 105, 105, 90, 90, 90, 90, 90, 90};

    int id1 = 1;
    string mg1_parameters_path = "./mg1_params.txt";
    string mg1_rdg_path = "./mg1_rdg.csv";
    string mg1_demand_path = "./mg1_demand.csv";
    Microgrid mg1(id1, T, c_buy, c_sell, mg1_parameters_path, mg1_rdg_path, mg1_demand_path);

    int id2 = 2;
    string mg2_parameters_path = "./mg2_params.txt";
    string mg2_rdg_path = "./mg2_rdg.csv";
    string mg2_demand_path = "./mg2_demand.csv";
    Microgrid mg2(id2, T,  c_buy, c_sell, mg2_parameters_path, mg2_rdg_path, mg2_demand_path);
    
    opt_res res1 = mg1.get_results();
    opt_res res2 = mg2.get_results();


    vector<opt_res> results = {res1, res2};


    CentralEMS cems(T, c_buy, c_sell, results);

    cems.run_optimization();



    return 0;
}
