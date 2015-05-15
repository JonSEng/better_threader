package structpred;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import opt.AdaGradL1Minimizer;
import opt.AdaGradL2Minimizer;
import opt.DifferentiableFunction;
import opt.OnlineMinimizer;
import tuple.Pair;
import counter.CounterInterface;
import counter.IntCounter;

public class PrimalSubgradientSVMLearner<D> implements LossAugmentedLearner<D> {
	
	int numFeatures;
	double C;
	double delta;
	double stepSize;
	boolean L1reg;
	
	public PrimalSubgradientSVMLearner(double C, double delta, double stepSize, int numFeatures, boolean L1reg) {
		this.C = C;
		this.delta = delta;
		this.stepSize = stepSize;
		this.numFeatures = numFeatures;
		this.L1reg = L1reg;
	}
	
	public CounterInterface<Integer> train(CounterInterface<Integer> initWeights, final LossAugmentedLinearModel<D> model, List<D> data, int iters) {
		double[] denseInitWeights = dense(initWeights, numFeatures);
		List<DifferentiableFunction> objs = new ArrayList<DifferentiableFunction>();

		for (final D datum : data) {
			objs.add(new DifferentiableFunction() {
				public Pair<Double, double[]> calculate(double[] x) {
					CounterInterface<Integer> weights = sparse(x);
					model.setWeights(weights);
					UpdateBundle ub = model.getLossAugmentedUpdateBundle(datum, 1.0);
					CounterInterface<Integer> delta = new IntCounter();
					delta.incrementAll(ub.gold, -1.0);
					delta.incrementAll(ub.guess, 1.0);
					double val = ub.loss + delta.dotProduct(weights);
					if (val <= 0.0) {
						return Pair.makePair(0.0, new double[numFeatures]);
					} else {
						return Pair.makePair(val, dense(delta, numFeatures));
					}
					
				}
			});
		}
		
		OnlineMinimizer minimizer = (L1reg? new AdaGradL1Minimizer(stepSize, delta, C, iters) : new AdaGradL2Minimizer(stepSize, delta, C, iters));
		return sparse(minimizer.minimize(objs, denseInitWeights, true, null));
	}
	
	public static double[] dense(CounterInterface<Integer> vect, int dim) {
		double[] result = new double[dim];
		for(Map.Entry<Integer,Double> entry : vect.entries()) {
			result[entry.getKey()] = entry.getValue();
		}
		return result;
	}
	
	public static CounterInterface<Integer> sparse(double[] vect) {
		return IntCounter.wrapArray(vect, vect.length);
	}
	
}
