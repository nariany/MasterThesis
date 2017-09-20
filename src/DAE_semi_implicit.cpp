#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( ){
         USING_NAMESPACE_ACADO
         DifferentialState x,l;
         AlgebraicState z;
         Control u;
         DifferentialEquation f;
         const double t_start=0.0;
         const double t_end=10.0;
         f<<dot(x)==-x+0.5*x*x+u+0.5*z;
         f<<dot(l)==x*x+3.0*u*u;
         f<<  0==z+exp(z)-1.0+x;
         OCP ocp(t_start,t_end,10);
         ocp.minimizeMayerTerm(l);
         ocp.subjectTo(f);
         ocp.subjectTo(AT_START,x==1.0);
         ocp.subjectTo(AT_START,l==0.0);
         GnuplotWindow window;
          window.addSubplot(x,"Diferencijalno stanje x");
          window.addSubplot(z,"Algebarsko stanje z");
          window.addSubplot(u,"Upravljanje u");
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set(ABSOLUTE_TOLERANCE,1e-7);
        algorithm.set(INTEGRATOR_TOLERANCE,1e-7);
        algorithm.set(HESSIAN_APPROXIMATION,EXACT_HESSIAN);
        algorithm<<window;
        algorithm.solve();
     return 0;
}
