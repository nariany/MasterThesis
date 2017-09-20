#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

int main( ){
         USING_NAMESPACE_ACADO
         DifferentialState phi,omega;
         Parameter l,alpha;
         const double g=9.81;
         DifferentialEquation f;
         Function h;
         VariablesGrid measurements;
         measurements=readFromFile("data.txt");
         OCP ocp(measurements.getTimePoints());
         h<<phi;
         ocp.minimizeLSQ(h,measurements);
         f<<dot(phi)==omega;
         f<<dot(omega)==-(g/l)*sin(phi)-alpha*omega;
         GnuplotWindow window;
          window.addSubplot(phi,"The angle phi","time [s]","angle [rad]");
          window.addSubplot(omega,"The angular velocity dphi");
          window.addData(0,measurements(0));
         ParameterEstimationAlgorithm algorithm(ocp);
          algorithm<<window;
          algorithm.solve();
       return 0;
}
