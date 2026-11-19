#include <gcopter/gcopter.hpp>
#include <cassert>
#include <iostream>

namespace gcopter {
struct QuadratureRegression {
  static double constantVelocityCost(int intervals) {
    Eigen::VectorXd times(1);times<<1.;
    Eigen::MatrixX3d coefficients=Eigen::MatrixX3d::Zero(8,3);
    coefficients(1,0)=3.; // x(t)=3t: a constant speed violation above2.5m/s.
    Eigen::VectorXi indices(1);indices<<0;
    Eigen::MatrixX4d box(6,4);
    box<<1.,0.,0.,-100., -1.,0.,0.,-100., 0.,1.,0.,-100.,
         0.,-1.,0.,-100., 0.,0.,1.,-100., 0.,0.,-1.,-100.;
    GCOPTER_PolytopeSFC::PolyhedraH corridor={box};
    Eigen::VectorXd bounds(9),weights=Eigen::VectorXd::Zero(9);
    bounds<<2.5,10000.,1.5,0.,100.,0.,0.,0.,0.;weights(1)=1.;
    flatness::FlatnessMap flatMap;flatMap.reset(1.,9.81,0.,0.,0.,1.e-4);
    double cost=0.,dilation=.6;
    Eigen::VectorXd gradientTimes=Eigen::VectorXd::Zero(1);
    Eigen::MatrixX3d gradientCoefficients=Eigen::MatrixX3d::Zero(8,3);
    GCOPTER_PolytopeSFC::attachPenaltyFunctional(times,coefficients,indices,
        corridor,.01,intervals,bounds,weights,flatMap,cost,gradientTimes,
        gradientCoefficients,dilation);
    // The analytic integral of the constant smoothed-L1 violation is2.745.
    assert(std::isfinite(cost) && std::abs(cost-2.745)<1.e-9);
    assert(std::abs(gradientCoefficients(1,0)-6.)<1.e-9);
    assert(std::abs(gradientTimes(0)-cost)<1.e-9);
    return cost;
  }
  static double turningCorridorViolation(double length) {
    // Recorded rejected warehouse turn: unchanged PVAJ, corridor and limits.
    GCOPTER_PolytopeSFC o;Eigen::Matrix<double,3,4> head,tail;
    head<<10.123685263688451,0.057872433427572506,1.1385652170594529,-0.083651625676043562,12.955808523965265,0.15638755534251525,-1.48202823798814,-0.15812665919550861,1.7144348954010646,-0.038652528690112986,0.0041024887087165937,0.15982341499102146;
    tail<<7.4000000953674316,0,0,0,15.800000190734863,0,0,0,1.7000000476837158,0,0,0;
    GCOPTER_PolytopeSFC::PolyhedraH polys;
    {Eigen::MatrixX4d h(18,4);h<<0.99999999999999978,-2.0816681711721685e-17,8.3266726846886741e-17,-12.550001525878905,8.3266726846886741e-17,-7.6327832942979512e-17,0.99999999999999956,-2.1499999999999995,-0.047534673542120764,0.77872865289725135,-0.52975739947323064,-9.2545117061908773,0.32436526742132082,0.039670416205572143,-0.61603125660549229,-3.1285034582658811,0.1127480404627364,0.2086996157229275,-1.1435942332908373,-2.0774791306703202,0.33352451238620651,-0.21202667278639811,-0.74144967658320915,0.38403368020760287,0.087535449208852062,-0.85176441081415588,-0.65828895742191751,11.154186357064191,-0.092247855518179822,-0.66852066547348954,0.76250305071407865,7.8743212359447714,-0.26168079140798495,0.44648572425153976,1.0603846372524073,-5.5934712328773513,-0.20579601733584532,0.83430523281287228,0.65710186866604903,-10.564285419419409,0.022563040444674713,-0.68507164595764058,0.68995930934823113,7.0628957228689604,0.22249309782157731,0.5357524535057433,-0.17824768226616253,-9.5975097228733102,-8.3266726846886741e-17,7.6327832942979512e-17,-0.99999999999999956,1.1999999046325671,-0.99999999999999978,2.0816681711721685e-17,-8.3266726846886741e-17,5.6499992370605447,2.7755575615628914e-17,-1.0000000000000004,4.8572257327350599e-17,10.323511695861821,-2.7755575615628914e-17,1.0000000000000004,-4.8572257327350599e-17,-16.350000762939459,0,0,1,-2.1299999999999999,0,0,-1,1.3700000000000001;polys.push_back(h);}
    {Eigen::MatrixX4d h(11,4);h<<-1.3877787807814457e-17,1.0000000000000002,-3.2526065174565133e-19,-18.199999237060549,-0.99999999999999989,1.3877787807814457e-17,-3.4694469519536142e-18,5.0000000953674304,3.4694469519536142e-18,-3.2526065174565133e-19,0.99999999999999989,-2.1500000000000004,-3.4694469519536142e-18,3.2526065174565133e-19,-0.99999999999999989,1.199999904632568,1.3877787807814457e-17,-1.0000000000000002,3.2526065174565133e-19,12.400000190734866,0.5283820581751475,-0.017158532317166679,-0.061587760313628809,-3.6401410397560618,0.51216436681676636,0.058286115998239026,-0.0034179910532507671,-4.8361273633576083,0.50469213498168253,0.023336442913153115,0.23406093026198416,-4.6288164943284,0.99999999999999989,-1.3877787807814457e-17,3.4694469519536142e-18,-10.800000095367432,0,0,1,-2.1299999999999999,0,0,-1,1.3700000000000001;polys.push_back(h);}
    Eigen::VectorXd b(9),w(9),phys(6);b<<2.5,10000,0.65000000000000002,10,28,2.5,5,0.25,0.25;w<<1100000,1500000,11000,110000,1100000,100000,100000,100000,100000;phys<<1.99,9.8066499999999994,0.69999999999999996,0.80000000000000004,0.01,0.0001;
    assert(o.setup(100.,.6,head,tail,polys,length,.01,32,b,w,phys));
    Trajectory<7> traj;assert(std::isfinite(o.optimize(traj,3.e-7,2.,0.,1.)));
    assert((traj.getPos(0.)-head.col(0)).norm()<1.e-6);
    assert((traj.getVel(0.)-head.col(1)).norm()<1.e-6);
    assert((traj.getAcc(0.)-head.col(2)).norm()<1.e-6);
    assert((traj.getJer(0.)-head.col(3)).norm()<1.e-6);
    assert((traj.getPos(traj.getTotalDuration())-tail.col(0)).norm()<1.e-6);
    double violation=-1.e9;
    for(int i=0;i<traj.getPieceNum();++i)for(double t=0.;t<=traj[i].getDuration();t+=.005) {
      const auto p=traj[i].getPos(t);
      for(int j=0;j<o.hPolytopes[o.hPolyIdx(i)].rows();++j) {
        const auto h=o.hPolytopes[o.hPolyIdx(i)].row(j);
        violation=std::max(violation,h.head<3>().dot(p)+h(3));
      }
    }
    return violation;
  }
  static void run() {
    const double one=constantVelocityCost(1),many=constantVelocityCost(32);
    assert(std::abs(one-many)<1.e-9);
    assert(turningCorridorViolation(INFINITY)>.2);
    assert(turningCorridorViolation(1.)<.002);
    std::cout<<"PASS: piece length restores recorded turn feasibility without changing boundary PVAJ\n";
    std::cout<<"PASS: complete trapezoidal endpoint weights, analytic constant-velocity cost and gradients\n";
  }
};
}
int main() {gcopter::QuadratureRegression::run();}
