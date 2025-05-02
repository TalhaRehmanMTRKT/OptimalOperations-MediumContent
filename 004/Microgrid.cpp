#include "Header.h"

Microgrid::Microgrid(int id, int T, double* c_buy, double* c_sell, string parameters_path, string rdg_path, string demand_path)
{
	this->T = T;
	this->c_buy = c_buy;
	this->c_sell = c_sell;
	this->id = id;

	ifstream file(parameters_path);

	if (file.is_open())
	{
		string line;
		while (getline(file, line))
		{
			istringstream iss(line);
			string parameter;
			iss >> parameter;
			if (parameter == "battery_cap")
			{
				iss >> battery_cap;
			}
			else if (parameter == "battery_chg_max")
			{
				iss >> battery_chg_max;
			}
			else if (parameter == "battery_dch_max")
			{
				iss >> battery_dch_max;
			}
			else if (parameter == "battery_eff")
			{
				iss >> battery_eff;
			}
			else if (parameter == "battery_soc_min")
			{
				iss >> battery_soc_min;
			}
			else if (parameter == "battery_soc_max")
			{
				iss >> battery_soc_max;
			}
			else if (parameter == "dg_max")
			{
				iss >> dg_max;
			}
			else if (parameter == "dg_min")
			{
				iss >> dg_min;
			}
			else if (parameter == "dg_cost")
			{
				iss >> dg_cost;
			}
		}
	}
	else
	{
		cout << "Error: Unable to open file " << parameters_path << endl;
	}

	file.close();


	// Read the csv file
	ifstream file2(rdg_path);

	std::string rdg_str;

	while (getline(file2, rdg_str)) {
		std::stringstream ss(rdg_str);

		double rdg_value;
		ss >> rdg_value;
		// Store in vector
		rdg.push_back(rdg_value);
	}

	file2.close();


	// Read the csv file
	ifstream file3(demand_path);

	std::string demand_str;

	while (getline(file3, demand_str)) {
		std::stringstream ss(demand_str);

		double demand_value;
		ss >> demand_value;
		// Store in vector
		demand.push_back(demand_value);
	}

	file3.close();

	print_info();
}


void Microgrid::print_info()
{
	cout << "_____ Printing Microgrid " << id << " info _____" << endl;
	cout << "Microgrid ID: " << id << endl;
	cout << "battery_cap: " << battery_cap << endl;
	cout << "battery_chg_max: " << battery_chg_max << endl;
	cout << "battery_dch_max: " << battery_dch_max << endl;
	cout << "battery_eff: " << battery_eff << endl;
	cout << "battery_soc_min: " << battery_soc_min << endl;
	cout << "battery_soc_max: " << battery_soc_max << endl;
	cout << "dg_max: " << dg_max << endl;
	cout << "dg_min: " << dg_min << endl;
	cout << "dg_cost: " << dg_cost << endl;

	cout << "rdg: ";
	for (int i = 0; i < rdg.size(); i++)
	{
		cout << rdg[i] << " ";
	}

	cout << endl;

	cout << "demand: ";
	for (int i = 0; i < demand.size(); i++)
	{
		cout << demand[i] << " ";
	}

	cout << endl<<endl;
}



opt_res Microgrid::get_results()
{
	return run_optimization();
}


opt_res Microgrid::run_optimization()
{
	print_info();

	// Constants
	double M = 1000000;
	
	// Create the model
	IloEnv env;
	IloModel model(env);

	// Variables
	IloNumVarArray p_sur(env, T, 0, IloInfinity, ILOFLOAT);
	IloNumVarArray p_short(env, T, 0, IloInfinity, ILOFLOAT);
	IloNumVarArray p_battery_chg(env, T, 0, IloInfinity, ILOFLOAT);
	IloNumVarArray p_battery_dch(env, T, 0, IloInfinity, ILOFLOAT);
	IloNumVarArray battery_soc(env, T, 0, 1, ILOFLOAT);
	IloNumVarArray p_dg(env, T, 0, IloInfinity, ILOFLOAT);

	//binary variable
	IloNumVarArray omega(env, T, 0, 1, ILOINT);
	IloNumVarArray nu(env, T, 0, 1, ILOINT);


	// Objective function
	IloExpr obj(env);
	for (int t = 0; t < T; t++)
	{
		obj += c_buy[t] * p_short[t] - c_sell[t] * p_sur[t] + dg_cost * p_dg[t];
	}
	model.add(IloMinimize(env, obj));

	// Constraints
	for (int t = 0; t < T; t++)
	{

		// limiting and binary constraintss
		model.add(p_dg[t] >= dg_min);
		model.add(p_dg[t] <= dg_max);
		model.add(p_battery_chg[t] <= battery_chg_max * omega[t]);
		model.add(p_battery_dch[t] <= battery_dch_max * (1-omega[t]));
		model.add(p_sur[t] <= M * nu[t]);
		model.add(p_short[t] <= M* (1-nu[t]));

		model.add(battery_soc[t] >= battery_soc_min);
		model.add(battery_soc[t] <= battery_soc_max);
		
		// non-negativity constraints
		model.add(p_battery_chg[t] >= 0);
		model.add(p_battery_dch[t] >= 0);
		model.add(p_sur[t] >= 0);
		model.add(p_short[t] >= 0);
		model.add(p_dg[t] >= 0);

		// Energy balance constraints
		model.add(p_short[t] + rdg[t] + p_dg[t] + p_battery_dch[t] == demand[t] + p_sur[t] + p_battery_chg[t] );

		// battery state of charge constraints
		if (t == 0)
		{
			model.add(battery_soc[t] == battery_soc[T-1] + (battery_eff * p_battery_chg[t] - p_battery_dch[t]/battery_eff)/battery_cap);
			model.add(p_battery_chg[t] <= (1-battery_soc[T - 1]) * battery_cap / battery_eff);
			model.add(p_battery_dch[t] <= battery_soc[T - 1] * battery_cap * battery_eff);
		}
		else
		{
			model.add(battery_soc[t] == battery_soc[t-1] + (battery_eff * p_battery_chg[t] - p_battery_dch[t]/battery_eff)/battery_cap);
			model.add(p_battery_chg[t] <= (1-battery_soc[t-1]) * battery_cap / battery_eff);
			model.add(p_battery_dch[t] <= battery_soc[t-1] * battery_cap * battery_eff);
		}

	}

	// Solve the model
	IloCplex cplex(model);
	cplex.setOut(env.getNullStream());

	cplex.solve();

	// Print the objective value
	cout << "Objective value: " << cplex.getObjValue() << endl;

	// Print the cost of DGs
	double dg_cost_total = 0;
	for (int t = 0; t < T; t++)
	{
		dg_cost_total += cplex.getValue(p_dg[t]) * dg_cost;
	}
	cout << "DG cost: " << dg_cost_total << endl;


	// Get the results
	opt_res results;
	for (int t = 0; t < T; t++)
	{
		results.p_sur.push_back(cplex.getValue(p_sur[t]));
		results.p_short.push_back(cplex.getValue(p_short[t]));
	}

	// Save the local operation results to csv file
	ofstream file("results_microgrid_" + to_string(id) + ".csv");
	file << "p_sur,p_short,p_battery_chg,p_battery_dch,battery_soc,p_dg,demand,rdg,c_buy,c_sell" << endl;
	for (int t = 0; t < T; t++)
	{
		file << results.p_sur[t] << "," << results.p_short[t] << "," << cplex.getValue(p_battery_chg[t]) << "," << cplex.getValue(p_battery_dch[t]) << "," << cplex.getValue(battery_soc[t]) << "," << cplex.getValue(p_dg[t]) << "," << demand[t] << "," << rdg[t] << "," << c_buy[t] << "," << c_sell[t] << endl;
	}
	file.close();


	// End the environment
	env.end();

	// Return the results
	return results;
}