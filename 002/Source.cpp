#include <ilcplex/ilocplex.h>
#include <iostream>
#include <fstream>
ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVar2D;




int
main(int, char**)
{

    
    IloEnv env;
    IloModel model(env);


#pragma region Input Data
    int T = 24; // Number of time intervals
    
    //int numMgs = 3; // Number of microgrids increased to 3

    int numMgs = 2; // Number of microgrids

    float chargeEff = 0.95; // Charging efficiency
    float dischargeEff = 0.95; // Discharging efficiency
    int chgMax = 50; // Maximum charging rate of battery
    int dchMax = 50; // Maximum discharging rate of battery


    int capDg = 150; // Capacity of diesel generator
    int dgMin = 50; // Minimum power output of diesel generator
    int dgMax = 150; // Maximum power output of diesel generator

    int* capBatt = new int[numMgs] { 100, 100 }; // Capacity of battery in each microgrid for 2 microgrid

    //int* capBatt = new int[numMgs] { 100, 100, 100}; // Capacity of battery in each microgrid for 3 microgrid


    double* Cbuy = new double[T] { 90, 90, 90, 90, 90, 90, 110, 110, 110, 110, 110, 125, 125, 125, 125, 125, 125, 125, 110, 110, 110, 110, 110, 110 }; // Cost of buying electricity
    double* Csell = new double[T] { 70, 70, 70, 70, 70, 70, 90, 90, 90, 90, 90, 105, 105, 105, 105, 105, 105, 105, 90, 90, 90, 90, 90, 90}; //selling price to grid w.r.t time
    int costdg = 80; // Cost of diesel generator

    double** load = new double* [numMgs]; // Load of each microgrid
    load[0] = new double[T] {169, 175, 179, 171, 181, 190, 270, 264, 273, 281, 300, 320, 280, 260, 250, 200, 180, 190, 240, 280, 325, 350, 300, 250};
    load[1] = new double[T] {130, 125, 120, 120, 125, 135, 150, 160, 175, 190, 195, 200, 195, 195, 180, 170, 185, 190, 195, 200, 195, 190, 180, 175};


    //// ADDED LINE
    //load[2] = new double[T] {130, 125, 120, 120, 125, 135, 150, 160, 175, 190, 195, 200, 195, 195, 180, 170, 185, 190, 195, 200, 195, 190, 180, 175};

    double** Pwt = new double* [numMgs]; // Wind turbine power generation
    Pwt[0] = new double[T] {10, 15, 20, 23, 28, 33, 35, 34, 70, 80, 90, 100, 100, 100, 100, 40, 50, 60, 70, 80, 90, 100, 100, 100};
    Pwt[1] = new double[T] { 70, 80, 90, 100, 100, 35, 34, 70, 80, 90, 100, 100, 100, 100, 40, 50, 60, 70, 80, 90, 100, 100, 100,100};


    //// ADDED LINE
    //Pwt[2] = new double[T] { 70, 80, 90, 100, 100, 35, 34, 70, 80, 90, 100, 100, 100, 100, 40, 50, 60, 70, 80, 90, 100, 100, 100,100};


#pragma endregion



#pragma region Decision Variables

    NumVar2D Pdg(env, numMgs); // Power output of diesel generator
    NumVar2D Pchg(env, numMgs); // Power charging the battery
    NumVar2D Pdch(env, numMgs); // Power discharging the battery
    NumVar2D Pbuy(env, numMgs); // Power bought from the grid
    NumVar2D Psell(env, numMgs); // Power sold to the grid
    NumVar2D SOC(env, numMgs); // State of charge of the battery

    NumVar2D Bin(env, numMgs); // State of charge of the battery


    for (int i = 0; i < numMgs; i++)
    {
        Pdg[i] = IloNumVarArray(env, T, 0, dgMax, ILOFLOAT);
		Pchg[i] = IloNumVarArray(env, T, 0, chgMax, ILOFLOAT);
		Pdch[i] = IloNumVarArray(env, T, 0, dchMax, ILOFLOAT);
		Pbuy[i] = IloNumVarArray(env, T, 0, IloInfinity, ILOFLOAT);
		Psell[i] = IloNumVarArray(env, T, 0, IloInfinity, ILOFLOAT);
		SOC[i] = IloNumVarArray(env, T, 0, 1, ILOFLOAT);
        Bin[i] = IloNumVarArray(env, T, 0, 1, ILOINT); // Binary variable for battery charging/discharging
    }

#pragma endregion


#pragma region Objective Function
	IloExpr obj(env);
    for (int i = 0; i < numMgs; i++)
    {
        for (int t = 0; t < T; t++)
        {
			obj += costdg * Pdg[i][t] +  Cbuy[t] * Pbuy[i][t] - Csell[t] * Psell[i][t] ;
		}
	}
	model.add(IloMinimize(env, obj));
#pragma endregion



#pragma region Constraints


    // Limiting Constraints
    for (int i = 0; i < numMgs; i++)
    {
        for (int t = 0; t < T; t++)
        {
			model.add(Pdg[i][t] >= dgMin);
			model.add(Pdg[i][t] <= dgMax);
			model.add(Pchg[i][t] <= chgMax);
			model.add(Pdch[i][t] <= dchMax);
            model.add(Pchg[i][t] >= 0);
            model.add(Pdch[i][t] >= 0);
            model.add(Pbuy[i][t] >= 0);
			model.add(Psell[i][t] >= 0);
			model.add(SOC[i][t] <= 1);
			model.add(SOC[i][t] >= 0);
		}
	}


    // Power balance
    for (int i = 0; i < numMgs; i++)
    {
        for (int t = 0; t < T; t++)
        {
			model.add(Pwt[i][t]+ Pdg[i][t] + Pbuy[i][t] + Pdch[i][t] == Psell[i][t] + load[i][t] + Pchg[i][t]);
		}
	}


    // Battery  Operating constraints Constratints
    for (int i = 0; i < numMgs; i++)
    {
        for (int t = 0; t < T; t++)
        {
            if (t == 0)
            {
				model.add(SOC[i][t] == SOC[i][T-1] + (chargeEff * Pchg[i][t] - (1 / dischargeEff) * Pdch[i][t]) / capBatt[i]);
                model.add(Pchg[i][t] <= (1 - SOC[i][T-1]) * capBatt[i] / chargeEff);
                model.add(Pdch[i][t] <= SOC[i][T-1] * capBatt[i] * dischargeEff);
			}
            else
            {
				model.add(SOC[i][t] == SOC[i][t - 1] + (chargeEff * Pchg[i][t] - (1 / dischargeEff) * Pdch[i][t])/ capBatt[i]);
                model.add(Pchg[i][t] <= (1 - SOC[i][t - 1]) * capBatt[i] / chargeEff);
				model.add(Pdch[i][t] <= SOC[i][t - 1] * capBatt[i] * dischargeEff);
			}

            // Binary variable constraints
			model.add(Pchg[i][t] <= chgMax * Bin[i][t]);
			model.add(Pdch[i][t] <= dchMax * (1 - Bin[i][t]));

        }
    } 

#pragma endregion


    #pragma region Solving the Model
	IloCplex cplex(model);
	cplex.solve();
	std::cout << "Objective Function Value: " << cplex.getObjValue() << std::endl;

    cout<< "Power output of diesel generator at time [0][0]: " << cplex.getValue(Pdg[0][0]) << endl;


    for (int t = 0; t < T; t++)
    {
        cout << "Power output of diesel generator in MG 0 at time " << t << " : " << cplex.getValue(Pdg[0][t]) << endl;
    }

#pragma endregion


#pragma region Storing data in CSV file

    std::ofstream outputFile("output.csv");

    if (outputFile.is_open()) {


        for (int mg = 0; mg < numMgs; mg++)
        {
            outputFile << "Time, Load, Wind Power, Diesel Power, Battery Charge, Battery Discharge, Buy, Sell, SOC" << std::endl;

            for (int t = 0; t < T; t++)
            {

                outputFile << t << ", " << load[mg][t] << ", " << Pwt[mg][t] << ", " << cplex.getValue(Pdg[mg][t]) << ", " << cplex.getValue(Pchg[mg][t]) << ", " << cplex.getValue(Pdch[mg][t]) << ", " << cplex.getValue(Pbuy[mg][t]) << ", " << cplex.getValue(Psell[mg][t]) << ", " << cplex.getValue(SOC[mg][t]) << std::endl;

            }
        }
    }

    else {
		std::cout << "Error opening file";
	}

    outputFile.close();

#pragma endregion



}