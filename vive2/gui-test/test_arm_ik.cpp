#include "kida_ik.h"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>

using namespace avatar;

int main(int argc,char** argv) {
    std::array<kida::Arm,2> arms{{kida::Arm(0),kida::Arm(1)}};
    if (argc==2&&(std::string(argv[1])=="--fk"||std::string(argv[1])=="--hand-frames")) {
        bool hands=std::string(argv[1])=="--hand-frames";
        std::cout<<std::setprecision(17);
        for (;;) {
            std::array<kida::Vector7,2> q;
            if (!(std::cin>>q[0][0])) return 0;
            for (int i=1;i<14;++i) if (!(std::cin>>q[i/7][i%7])) return 2;
            for (int side=0;side<2;++side) {
                auto t=arms[side].forward(q[side]).tcp;
                if (hands) t.linear()=kida::matrix(kida::hand_orientation(kida::pose(t).rotation,side));
                for (int row=0;row<4;++row) for (int col=0;col<4;++col)
                    std::cout<<t.matrix()(row,col)<<' ';
            }
            std::cout<<'\n';
        }
    }
    std::mt19937 rng(19);
    std::uniform_real_distribution<double> random(-1,1);
    for (auto& a:arms) {
        // With every joint zero, fingers point down and the dorsum points
        // laterally out of the body. At Q4=90 degrees, thumbs point upward.
        kida::Vector7 zero=kida::Vector7::Zero();
        auto zero_tcp=kida::pose(a.forward(zero).tcp);
        auto hand=kida::hand_orientation(zero_tcp.rotation,a.side());
        assert(length(hand.rotate({1,0,0})-Vec3{0,0,-1})<1e-10);
        assert(length(hand.rotate({0,0,1})-Vec3{0,a.side()==0?1.:-1.,0})<1e-10);
        zero[3]=pi/2;
        hand=kida::hand_orientation(kida::pose(a.forward(zero).tcp).rotation,a.side());
        assert(length(hand.rotate(kida::hand_thumb_direction(a.side()))-Vec3{0,0,1})<1e-10);
        assert(length(hand.rotate({1,0,0})-Vec3{1,0,0})<1e-10);
        assert(a.position_error<.002&&a.angle_error<.02);
        auto home=a.reference;
        assert(std::abs(home.position.x-.30)<.002);
        assert(std::abs(home.position.y-(a.side()==0?.23:-.23))<.002);
        assert(std::abs(home.position.z+.40)<.002);
        // Finite difference every joint, including coincident wrist axes.
        for (int sample=0;sample<80;++sample) {
            kida::Vector7 q;
            for (int i=0;i<7;++i) q[i]=(a.low[i]+a.high[i])/2+random(rng)*(a.high[i]-a.low[i])*.45;
            auto f=a.forward(q); auto j=a.jacobian(f);
            assert(std::abs(f.tcp.linear().determinant()-1)<1e-10);
            assert((f.joints[5].translation()-f.joints[6].translation()).norm()<1e-12);
            assert(std::abs(f.joints[5].linear().col(2).dot(f.joints[6].linear().col(2)))<1e-12);
            for (int i=0;i<7;++i) {
                auto perturbed=q; perturbed[i]+=1e-7;
                auto moved=a.forward(perturbed);
                auto dp=(moved.tcp.translation()-f.tcp.translation())/1e-7;
                Eigen::Vector3d dr=kida::rotation_error(kida::pose(moved.tcp).rotation,kida::pose(f.tcp).rotation)/1e-7;
                assert((dp-j.col(i).head<3>()).norm()<1e-6);
                assert((dr-j.col(i).tail<3>()).norm()<1e-6);
            }
        }
        // Reachable poses with independent position and orientation changes.
        for (int sample=0;sample<120;++sample) {
            a.reset(); auto q=a.home;
            for (int i=0;i<7;++i) q[i]+=random(rng)*.3;
            q=q.cwiseMax(a.low).cwiseMin(a.high);
            a.solve(kida::pose(a.forward(q).tcp),100);
            assert(a.position_error<.002&&a.angle_error<.02);
        }
        a.reset();
        auto previous=a.q;
        for (int sample=0;sample<1000;++sample) {
            auto q=a.home;
            for (int i=0;i<7;++i) q[i]+=.3*std::sin(sample*.009*(1+i*.1));
            q=q.cwiseMax(a.low).cwiseMin(a.high);
            a.solve(kida::pose(a.forward(q).tcp));
            assert(a.position_error<.002&&a.angle_error<.02);
            assert((a.q-previous).cwiseAbs().maxCoeff()<.10);
            previous=a.q;
        }
        for (auto target : {Vec3{3,3,3},Vec3{0,0,0},Vec3{0,0,-.8}}) {
            a.solve({target,Quat::yaw(pi)},100);
            assert(a.q.allFinite()&&a.fk.tcp.matrix().allFinite());
            assert((a.q.array()>=a.low.array()-1e-12).all());
            assert((a.q.array()<=a.high.array()+1e-12).all());
        }
        bool rejected=false;
        try { a.solve({{std::numeric_limits<double>::quiet_NaN(),0,0},{}}); }
        catch (const std::runtime_error&) { rejected=true; }
        assert(rejected);
    }
    std::array<Pose,2> references{{arms[0].reference,arms[1].reference}};
    Quat world=Quat::yaw(.87);
    Vec3 origin{2,-3,.7};
    Pose left{world.rotate({.30,.23,1.2})+origin,Quat::yaw(.4)};
    Pose right{world.rotate({.30,-.23,1.2})+origin,Quat::yaw(-.6)};
    kida::Calibration c; c.set(left,right,references,.8);
    for (int side=0;side<2;++side) {
        Pose start=side==0?left:right;
        auto mapped=c.map(start,side);
        assert(length(mapped.position-references[side].position)<1e-10);
        assert(kida::rotation_error(mapped.rotation,references[side].rotation).norm()<1e-10);
        Pose moved=start; moved.position=moved.position+world.rotate({.1,0,.05});
        Quat roll{std::sin(.2),0,0,std::cos(.2)};
        moved.rotation=world*roll*world.inverse()*start.rotation;
        mapped=c.map(moved,side);
        assert(length(mapped.position-references[side].position-Vec3{.08,0,.04})<1e-10);
        assert(kida::rotation_error(mapped.rotation,roll*references[side].rotation).norm()<1e-10);
    }
    assert(length(rpy_degrees(Quat::yaw(pi/2))-Vec3{0,0,90})<1e-9);
    std::cout<<"PASS: KIDA 7-DOF FK/Jacobians, 240 pose IKs, 2000 continuous steps, bounds and calibration.\n";
}
