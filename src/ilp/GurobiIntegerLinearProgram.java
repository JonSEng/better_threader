package ilp;

import indexer.HashMapIndexer;
import indexer.Indexer;
import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;

public class GurobiIntegerLinearProgram extends IntegerLinearProgram {
	
	double maxTime;
	Indexer<GRBVar> varIndexer;
	Indexer<String> constraintIndexer;
	GRBLinExpr objectiveExpr;
	GRBEnv env;
	GRBModel model;
	boolean maximize;
    
    public GurobiIntegerLinearProgram(double maxTime) {
    	this.maxTime = maxTime;
    	clear();
    }
	
    public GurobiIntegerLinearProgram() {
    	this.maxTime = Double.POSITIVE_INFINITY;
    	clear();
    }

	public int addBoundedIntVar(double lower, double upper) {
		if (varIndexer.locked()) throw new RuntimeException("Variables are locked.");
		GRBVar var = null;
		try {
			if (lower == 0.0 && upper == 1.0) {
				var = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "var"+varIndexer.size());
			} else {
				var = model.addVar(lower, upper, 0.0, GRB.INTEGER, "var"+varIndexer.size());
			}
		} catch (GRBException e) {
			e.printStackTrace();
		}
		try {
			model.update();
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return varIndexer.getIndex(var);
	}

	public int addBoundedVar(double lower, double upper) {
		if (varIndexer.locked()) throw new RuntimeException("Variables are locked.");
		GRBVar var = null;
		try {
			var = model.addVar(lower, upper, 0.0, GRB.CONTINUOUS, "var"+varIndexer.size());
		} catch (GRBException e) {
			e.printStackTrace();
		}
		try {
			model.update();
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return varIndexer.getIndex(var);
	}

	public void addLessThanConstraint(int[] indices, double[] weights, double rhs) {
		String constrName = "constr"+constraintIndexer.size();
		constraintIndexer.getIndex(constrName);
		GRBLinExpr lhs = new GRBLinExpr();
		for (int i=0; i<indices.length; ++i) {
			lhs.addTerm(weights[i], varIndexer.getObject(indices[i]));
		}
		try {
			model.addConstr(lhs, GRB.LESS_EQUAL, rhs, constrName);
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}

	public void addObjectiveWeight(int pos, double val) {
		objectiveExpr.addTerm(val, varIndexer.getObject(pos));
	}

	public void setToMaximize() {
		maximize = true;
	}

	public void lockVariableCount() {
		varIndexer.lock();
	}

	public void optimize() {
		try {
			model.setObjective(objectiveExpr, (maximize ? GRB.MAXIMIZE : GRB.MINIMIZE));
			model.optimize();
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public double objectiveValue() {
		try {
			if (model.get(GRB.IntAttr.Status) == 9) return 0.0;
		} catch (GRBException e1) {
			e1.printStackTrace();
		}

		double objVal = 0.0;
		try {
			objVal = model.get(GRB.DoubleAttr.ObjVal);
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return objVal;
	}

	public double[] solution() {
		try {
			if (model.get(GRB.IntAttr.Status) == 9) return null;
		} catch (GRBException e1) {
			e1.printStackTrace();
		}
		
		double[] solution = new double[varIndexer.size()];
		for (int i=0; i<varIndexer.size(); ++i) {
			GRBVar var = varIndexer.getObject(i);
			try {
				solution[i] = var.get(GRB.DoubleAttr.X);
			} catch (GRBException e) {
				e.printStackTrace();
			}
		}
		return solution;
	}
	
	public void clear() {
		try {
			if (env != null) env.dispose();
			if (model != null) model.dispose();
		} catch (GRBException e1) {
			e1.printStackTrace();
		}
		
		objectiveExpr = new GRBLinExpr();
		maximize = false;
		varIndexer = new HashMapIndexer<GRBVar>();
		constraintIndexer = new HashMapIndexer<String>();
    	try {
			env   = new GRBEnv();
			env.set(GRB.IntParam.Threads, 1);
			env.set(GRB.IntParam.LogToConsole, 0);
			env.set(GRB.DoubleParam.TimeLimit, maxTime);
			model = new GRBModel(env);
		} catch (GRBException e) {
			e.printStackTrace();
		}
	}

	public void finalize() {
		try {
			if (env != null) env.dispose();
			if (model != null) model.dispose();
		} catch (GRBException e1) {
			e1.printStackTrace();
		}
	}
	
}
