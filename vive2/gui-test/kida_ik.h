#pragma once
#include "arm_ik.h"
#include "kida_model_data.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace kida {
using avatar::Vec3;
using avatar::Quat;
using avatar::Pose;
using Vector7 = Eigen::Matrix<double,7,1>;
using Vector6 = Eigen::Matrix<double,6,1>;
using Jacobian = Eigen::Matrix<double,6,7>;
using Transform = Eigen::Isometry3d;
inline Eigen::Vector3d eigen(Vec3 v) { return {v.x,v.y,v.z}; }
inline Vec3 vector(const Eigen::Vector3d& v) { return {v.x(),v.y(),v.z()}; }
inline Quat quaternion(const Eigen::Matrix3d& r) {
    Eigen::Quaterniond q(r); return {q.x(),q.y(),q.z(),q.w()};
}
inline Eigen::Matrix3d matrix(Quat q) {
    q=q.normalized(); return Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
}
inline Pose pose(const Transform& t) { return {vector(t.translation()),quaternion(t.linear())}; }
inline Eigen::Matrix3d rz(double angle) {
    return Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
inline Transform transform(const model::Offset& o) {
    Transform t=Transform::Identity();
    t.translation()=Eigen::Vector3d(o.position.data());
    auto e=o.euler_degrees;
    // Tact's YAML defaults to intrinsic XYZ: Rx * Ry * Rz.
    t.linear()=(Eigen::AngleAxisd(e[0]*avatar::pi/180,Eigen::Vector3d::UnitX())*
                Eigen::AngleAxisd(e[1]*avatar::pi/180,Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(e[2]*avatar::pi/180,Eigen::Vector3d::UnitZ())).toRotationMatrix();
    return t;
}
inline Eigen::Vector3d rotation_error(Quat desired,Quat actual) {
    Eigen::AngleAxisd angle(matrix(desired)*matrix(actual).transpose());
    return angle.axis()*angle.angle();
}

// The procedural hand uses +X=fingers, +Z=dorsum. KIDA's hand YAML
// uses +Z=fingers, -X=dorsum. kida-run mounts that frame with intrinsic
// XYZ [0,90,-90] on the left and [0,90,+90] on the right. Composing the
// two transforms gives a TCP-local X rotation of -90 / +90 degrees.
inline Quat hand_orientation(Quat tcp,int arm_index) {
    double half_angle=(arm_index==0?-1:1)*avatar::pi/4;
    return (tcp*Quat{std::sin(half_angle),0,0,std::cos(half_angle)}).normalized();
}
// Local thumb side in the procedural frame: left=-Y, right=+Y.
inline Vec3 hand_thumb_direction(int arm_index) {
    return {0,arm_index==0?-1.:1.,0};
}

struct Forward {
    std::array<Transform,7> joints;
    Transform tcp;
};

class Arm {
public:
    explicit Arm(int side,Vec3 root={}) : side_(side), root_(root) {
        if (side<0||side>1) throw std::runtime_error("Invalid KIDA arm index");
        const auto& specs=side==0?model::joints0:model::joints1;
        for (int i=0;i<7;++i) {
            offsets_[i]=transform(specs[i].offset);
            low[i]=specs[i].low; high[i]=specs[i].high;
        }
        tool_=transform(side==0?model::tcp0:model::tcp1);
        q=Eigen::Map<const Vector7>((side==0?model::seed0:model::seed1).data());
        home=q;
        const auto& h=side==0?model::home0:model::home1;
        Quat r=quaternion((Eigen::AngleAxisd(h[5],Eigen::Vector3d::UnitZ())*
                          Eigen::AngleAxisd(h[4],Eigen::Vector3d::UnitY())*
                          Eigen::AngleAxisd(h[3],Eigen::Vector3d::UnitX())).toRotationMatrix());
        solve({root+Vec3{h[0],h[1],h[2]},r},400);
        if (position_error>.002||angle_error>.02)
            throw std::runtime_error("KIDA home IK did not converge within controller bounds");
        home=q;
        reference=pose(fk.tcp);
    }

    Forward forward(const Vector7& angles) const {
        Forward out;
        Transform t=Transform::Identity(); t.translation()=eigen(root_);
        for (int i=0;i<7;++i) {
            t=t*offsets_[i];
            t.linear()=t.linear()*rz(angles[i]);
            out.joints[i]=t;
        }
        out.tcp=t*tool_;
        return out;
    }
    Jacobian jacobian(const Forward& f) const {
        Jacobian j;
        for (int i=0;i<7;++i) {
            Eigen::Vector3d axis=f.joints[i].linear().col(2);
            j.col(i).head<3>()=axis.cross(f.tcp.translation()-f.joints[i].translation());
            j.col(i).tail<3>()=axis;
        }
        return j;
    }
    Vector6 error(const Forward& f,Pose target) const {
        Vector6 e;
        e.head<3>()=eigen(target.position)-f.tcp.translation();
        e.tail<3>()=rotation_error(target.rotation,quaternion(f.tcp.linear()))*.2;
        return e;
    }
    void reset() { q=home; fk=forward(q); position_error=angle_error=0; }

    void solve(Pose target,int iterations=24) {
        const auto& r=target.rotation;
        if (!avatar::finite(target.position)||
            !std::isfinite(r.x*r.x+r.y*r.y+r.z*r.z+r.w*r.w)||
            r.x*r.x+r.y*r.y+r.z*r.z+r.w*r.w<1e-12)
            throw std::runtime_error("Non-finite or zero KIDA target pose");
        q=q.cwiseMax(low).cwiseMin(high);
        for (int step=0;step<iterations;++step) {
            Forward f=forward(q); Vector6 e=error(f,target);
            if (e.head<3>().norm()<.0003&&e.tail<3>().norm()<.0006) break;
            Jacobian j=jacobian(f); j.bottomRows<3>()*=.2;
            Vector7 gradient=j.transpose()*e;
            // Remove coordinates that would push farther through a bound.
            for (int i=0;i<7;++i)
                if ((q[i]<low[i]+1e-8&&gradient[i]<0)||
                    (q[i]>high[i]-1e-8&&gradient[i]>0)) j.col(i).setZero();
            const double cost=e.squaredNorm();
            bool accepted=false;
            for (double damping : {.003,.015,.06}) {
                Eigen::Matrix<double,6,6> system=j*j.transpose();
                system.diagonal().array()+=damping*damping;
                auto factor=system.ldlt();
                Eigen::Matrix<double,7,6> pinv=j.transpose()*factor.solve(Eigen::Matrix<double,6,6>::Identity());
                // The unobserved seventh DoF prefers the home posture. Warm
                // starts preserve elbow continuity; tracking two wrists cannot
                // measure the user's elbow swivel.
                Vector7 dq=pinv*e+(Vector7(home-q)-pinv*(j*(home-q)))*.015;
                dq*=std::min(1.,.16/std::max(1e-12,dq.cwiseAbs().maxCoeff()));
                for (double fraction : {1.,.5,.25,.125}) {
                    Vector7 candidate=(q+fraction*dq).cwiseMax(low).cwiseMin(high);
                    if (error(forward(candidate),target).squaredNorm()<cost-1e-14) {
                        q=candidate; accepted=true; break;
                    }
                }
                if (accepted) break;
            }
            if (!accepted) break;
        }
        fk=forward(q);
        Vector6 residual=error(fk,target);
        position_error=residual.head<3>().norm(); angle_error=residual.tail<3>().norm()/.2;
    }
    bool limited() const { return position_error>.008||angle_error>.08; }
    int side() const { return side_; }
    Vector7 q,home,low,high;
    Pose reference;
    Forward fk;
    double position_error=0,angle_error=0;
private:
    int side_;
    Vec3 root_;
    std::array<Transform,7> offsets_;
    Transform tool_;
};

// Capture level wrists in the robot's bent-elbow ready pose. Each wrist then
// controls a relative displacement and rotation about the corresponding TCP.
struct Calibration {
    bool ready=false;
    double scale=1;
    Quat heading;
    std::array<Pose,2> sensors,reference;
    void set(Pose left,Pose right,const std::array<Pose,2>& robot,double gain=1) {
        Vec3 span=left.position-right.position;
        double horizontal=std::hypot(span.x,span.y);
        if (!avatar::finite(left.position)||!avatar::finite(right.position)||horizontal<.20)
            throw std::runtime_error("KEEP WRISTS AT LEAST 20 CM APART.");
        if (std::abs(span.z)>.1)
            throw std::runtime_error("KEEP BOTH WRISTS LEVEL IN THE READY POSE.");
        if (!(gain>0&&std::isfinite(gain))) throw std::runtime_error("Invalid motion scale");
        for (auto p : {left,right}) {
            auto r=p.rotation;
            double n=r.x*r.x+r.y*r.y+r.z*r.z+r.w*r.w;
            if (!(n>1e-12&&std::isfinite(n))) throw std::runtime_error("Invalid tracker orientation");
        }
        heading=Quat::yaw(avatar::pi/2-std::atan2(span.y,span.x));
        sensors={left,right}; reference=robot; scale=gain; ready=true;
    }
    Pose map(Pose raw,int index) const {
        Pose start=sensors[index],base=reference[index];
        return {base.position+heading.rotate(raw.position-start.position)*scale,
            (heading*raw.rotation*start.rotation.inverse()*heading.inverse()*base.rotation).normalized()};
    }
};
} // namespace kida
