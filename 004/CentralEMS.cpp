#include "Header.h"



CentralEMS::CentralEMS(int T, double* c_buy, double* c_sell, vector<opt_res> mgs_results)
{

	this->T = T;
	this->c_buy = c_buy;
	this->c_sell = c_sell;
	this->mgs_results = mgs_results;
	this->num_mgs = mgs_results.size();

	cout << "CentralEMS initialized with " << num_mgs << " microgrids." << endl;
}

void CentralEMS::run_optimization()
{

	IloEnv env;
	IloModel model(env);

	// Decision variables
	NumVar2D p_buy(env, num_mgs);
	NumVar2D p_sell(env, num_mgs);

	for (int i = 0; i < num_mgs; i++)
	{
		p_buy[i] = IloNumVarArray(env, T, 0, IloInfinity, ILOFLOAT);
		p_sell[i] = IloNumVarArray(env, T, 0, IloInfinity, ILOFLOAT);

	}


	// Objective function
	IloExpr obj(env);
	for (int t = 0; t < T; t++)
	{
		for (int i = 0; i < num_mgs; i++)
		{
			obj += c_buy[t] * p_buy[i][t] - c_sell[t] * p_sell[i][t];
		}
	}
	model.add(IloMinimize(env, obj));

	// Constraints
	for (int t = 0; t < T; t++)
	{
		for (int i = 0; i < num_mgs; i++)
		{
			model.add(p_buy[i][t] + mgs_results[i].p_sur[t] == p_sell[i][t] + mgs_results[i].p_short[t]);
		}
	}

	// Solve the model
	IloCplex cplex(model);
	cplex.setOut(env.getNullStream());
	cplex.solve();

	// Print the objective value
	cout << "Objective value: " << cplex.getObjValue() << endl;

	// Save the results w.r.t each MG
	ofstream file("results_cems.csv");

	for (int i = 0; i < num_mgs; i++)
	{
		file << "MG" << i + 1 << " p_buy,p_sell" << endl;
		for (int t = 0; t < T; t++)
		{
			file << cplex.getValue(p_buy[i][t]) << "," << cplex.getValue(p_sell[i][t]) << endl;
		}
		file << endl;
	}

	file.close();


	env.end();

}



