#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

int main( ){
        USING_NAMESPACE_ACADO
        Parameter y1,y2,y3;
        NLP nlp;
        nlp.minimize(0,y1);
        nlp.minimize(1,y2);
        nlp.minimize(2,y3);
        nlp.subjectTo(-5.0<=y1<=5.0);
        nlp.subjectTo(-5.0<=y2<=5.0);
        nlp.subjectTo(-5.0<=y3<=5.0);
        nlp.subjectTo(y1*y1+y2*y2+y3*y3-4<=0);
       MultiObjectiveAlgorithm algorithm(nlp);
        algorithm.set(PARETO_FRONT_GENERATION,PFG_WEIGHTED_SUM);
        algorithm.set(PARETO_FRONT_DISCRETIZATION,11);
        algorithm.solve();
        algorithm.getWeights("scalar3_ws_weights.txt");
        VariablesGrid paretoFront;
        algorithm.getParetoFront(paretoFront);
        paretoFront.print();
      GnuplotWindow window;
        window.addSubplot3D(paretoFront,"Pareto Front y1 vs y2 vs y3","y1","y2",PM_POINTS);
        window.plot();
        algorithm.printInfo();
   return 0;
}
