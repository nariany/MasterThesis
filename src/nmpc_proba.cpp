#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
int main( ){
        USING_NAMESPACE_ACADO
        //VARIJABLLE
        DifferentialState x,y,theta;
        Control v,omega;
        const double Ts=0.01;
        const double t_start=0.0;
        const double N=30.0;
        
        DiscretizedDifferentialEquation f(Ts);
        //Define a discrete-time system
        f<<next(x)==x+Ts*v*cos(theta);
        f<<next(y)==y+Ts*v*sin(theta);
        f<<next(theta)==theta+Ts*omega;
        //Reference function an weighting matrices
        Function h;
        h<<x;
        h<<y;
        h<<theta;
        DMatrix P(3,3);
        P(0,0)=0.1;
        P(1,1)=0.1;
        P(2,2)=0.1;
        DMatrix Q(3,3);
        Q(0,0)=0.1;
        Q(1,1)=0.1;
        Q(2,2)=0.1;
        DVector r(3);
        r(0)=5;
        r(1)=5;
        r(2)=5;
        //r.setAll(0.0);
        //Define an optimal Control Problem
        OCP ocp(t_start,N*Ts,N);
        ocp.minimizeLSQ(Q,h,r);
        ocp.minimizeLSQEndTerm(P,h,r);
        ocp.subjectTo(f);
        ocp.subjectTo(AT_START,x==3.0);
        ocp.subjectTo(AT_START,y==3.0);
        ocp.subjectTo(AT_START,theta==0.0);
        ocp.subjectTo(AT_START,v==0.3);
        ocp.subjectTo(AT_START,omega==2.2);
       // ocp.subjectTo(AT_END,x==4);
       // ocp.subjectTo(AT_END,y==4);
       // ocp.subjectTo(AT_END,theta==3.62);
        ocp.subjectTo(-0.7<=v<=0.7);
        ocp.subjectTo(-4.2<=omega<=4.2);
        //
        GnuplotWindow window;
        window.addSubplot(x,"x koordinata pozicije Mobilnog robota");
        window.addSubplot(y,"y koordinata pozicije Mobilnog robota");
        window.addSubplot(theta,"Zakret mobilnog robota");
        window.addSubplot(v,"Upravljanje v");
        window.addSubplot(omega,"Upravljanje omega");
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set(HESSIAN_APPROXIMATION,EXACT_HESSIAN);
        algorithm.set(KKT_TOLERANCE, 1e-10);
        algorithm<<window;
        algorithm.solve();
        VariablesGrid control;
        algorithm.getControls(control);
	std::cout << "Printam contols...\n";
        std::cout<<"Printam zadnje controls..\n";
        std::cout<<control(1,0)<<"  "<<control(1,1)<<"  "<<control(1,2)<<"\n";
        control.print();
      return 0;
}
