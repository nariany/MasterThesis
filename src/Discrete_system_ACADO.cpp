#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( ){
         USING_NAMESPACE_ACADO
         DifferentialState s,v,m;
         Control u;
         const double t_start=0.0;
         const double t_end=10.0;
         const double h=0.01;
         DiscretizedDifferentialEquation f(h);
         f<<next(s)==s+h*v;
         f<<next(v)==v+h*(u-0.02*v*v)/m;
         f<<next(m)==m-h*(u*u)/100;
         OCP ocp(t_start,t_end,50);
         ocp.minimizeLagrangeTerm(u*u);
         ocp.subjectTo(f);
         ocp.subjectTo(AT_START, s==0.0);
         ocp.subjectTo(AT_START, v==0.0);
         ocp.subjectTo(AT_START, m==1.0);
         ocp.subjectTo(AT_END,s==10.0);
         ocp.subjectTo(AT_END,v==0.0);
         ocp.subjectTo(-0.01<=v<=1.3);
         GnuplotWindow window;
          window.addSubplot(s,"Stanje s");
          window.addSubplot(v,"Stanje v");
          window.addSubplot(m,"Stanje m");
          window.addSubplot(u,"Upravljanje");
          window.addSubplot(PLOT_KKT_TOLERANCE,"KKT_Toleracija");
          window.addSubplot(0.5*m*v*v,"Kinetička energija");
        OptimizationAlgorithm algorithm(ocp);
          algorithm.set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);
          algorithm.set(KKT_TOLERANCE, 1e-10);
          algorithm<<window;
          algorithm.solve();
     return 0;
}
