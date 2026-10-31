// Native arm kinematics and damped least-squares control. All buffers are row-major.
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Mat4 = Eigen::Matrix<double,4,4,Eigen::RowMajor>;
using Matrix = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using Vec = Eigen::VectorXd;
thread_local std::string error;
struct Element { int movable; Mat4 before, after; Eigen::Vector3d axis; };
struct Chain { int n; std::vector<Element> elements; Vec lo,hi; };
struct Options {
    double posture, rotation, damping, step, pos_tol, rot_tol;
    int iterations;
};
struct Result { Vec q; double position, rotation; int iterations; };

void require(bool ok, const char* message) { if (!ok) throw std::invalid_argument(message); }
Chain& chain(void* pointer) { require(pointer != nullptr, "null arm handle"); return *static_cast<Chain*>(pointer); }
Vec vector(const double* values, int n) {
    require(values != nullptr, "null vector buffer");
    Vec v = Eigen::Map<const Vec>(values,n);
    require(v.allFinite(), "non-finite arm vector");
    return v;
}
Mat4 pose(const double* data) {
    require(data != nullptr, "null pose buffer");
    Mat4 t = Eigen::Map<const Mat4>(data);
    require(t.allFinite(), "non-finite arm pose");
    require((t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm() < 1e-8, "invalid homogeneous pose");
    auto rotation = t.topLeftCorner<3,3>();
    require((rotation.transpose()*rotation-Eigen::Matrix3d::Identity()).norm()<1e-5 && std::abs(rotation.determinant()-1.0)<1e-5,
            "pose rotation must be orthonormal and right-handed");
    return t;
}
Vec clamp(const Chain& c, const Vec& q) { return q.cwiseMax(c.lo).cwiseMin(c.hi); }
Options options(const double* p, int iterations) {
    require(p != nullptr, "null solver options");
    for(int i=0;i<6;++i) require(std::isfinite(p[i]),"non-finite solver option");
    require(iterations>0,"max_iters must be positive");
    require(p[0]>=0 && p[1]>=0 && p[2]>0 && p[3]>0 && p[4]>0 && p[5]>0,"invalid solver weights or tolerances");
    return {p[0],p[1],p[2],p[3],p[4],p[5],iterations};
}
Mat4 forward(const Chain& c, const Vec& q, const Mat4& base, Matrix* axes=nullptr, Matrix* origins=nullptr) {
    Mat4 t=base;
    if(axes) axes->resize(c.n,3);
    if(origins) origins->resize(c.n,3);
    int k=0;
    for(const Element& e:c.elements) {
        t=t*e.before;
        if(e.movable) {
            if(axes) axes->row(k)=(t.topLeftCorner<3,3>()*e.axis).transpose();
            if(origins) origins->row(k)=t.topRightCorner<3,1>().transpose();
            Mat4 rotation=Mat4::Identity();
            rotation.topLeftCorner<3,3>()=Eigen::AngleAxisd(q[k],e.axis).toRotationMatrix();
            t=t*rotation*e.after;
            ++k;
        }
    }
    return t;
}
Mat4 jacobian(const Chain& c,const Vec& q,const Mat4& base,Matrix& j) {
    Matrix axes,origins;
    Mat4 t=forward(c,q,base,&axes,&origins);
    j.resize(6,c.n);
    for(int k=0;k<c.n;++k) {
        Eigen::Vector3d axis=axes.row(k).transpose();
        Eigen::Vector3d delta=t.topRightCorner<3,1>()-origins.row(k).transpose();
        j.col(k).head<3>()=axis.cross(delta);
        j.col(k).tail<3>()=axis;
    }
    return t;
}
Eigen::Vector3d rot_error(const Mat4& target,const Mat4& current) {
    Eigen::AngleAxisd angle(target.topLeftCorner<3,3>()*current.topLeftCorner<3,3>().transpose());
    return angle.angle()*angle.axis();
}
Result solve(const Chain& c,const Mat4& target,const Vec& initial,const Mat4& base,const Vec* posture,const Options& o) {
    Vec q=clamp(c,initial);
    const Vec reference=posture?*posture:q;
    int iterations=0;
    for(int it=0;it<o.iterations;++it) {
        iterations=it+1;
        Matrix j;
        Mat4 t=jacobian(c,q,base,j);
        Eigen::Vector3d ep=target.topRightCorner<3,1>()-t.topRightCorner<3,1>();
        Eigen::Vector3d er=rot_error(target,t);
        if(ep.norm()<o.pos_tol && er.norm()<o.rot_tol) break;
        Eigen::Matrix<double,6,1> e;
        e.head<3>()=ep;
        e.tail<3>()=o.rotation*er;
        j.bottomRows(3)*=o.rotation;
        Eigen::MatrixXd a=j.transpose()*j;
        a.diagonal().array()+=o.damping*o.damping+o.posture;
        Vec b=j.transpose()*e+o.posture*(reference-q);
        Vec dq=a.ldlt().solve(b);
        require(dq.allFinite(),"non-finite IK update");
        const double largest=dq.cwiseAbs().maxCoeff();
        if(largest>o.step) dq*=o.step/largest;
        q=clamp(c,q+dq);
    }
    Mat4 t=forward(c,q,base);
    return {q,(target.topRightCorner<3,1>()-t.topRightCorner<3,1>()).norm(),rot_error(target,t).norm(),iterations};
}
Result multistart(const Chain& c,const Mat4& target,const Vec& initial,const Mat4& base,const Vec* posture,
                  const Options& o,int starts,unsigned int seed) {
    require(starts>=0,"n_starts must be nonnegative");
    Result best=solve(c,target,initial,base,posture,o);
    double score=best.position+0.05*o.rotation*best.rotation;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    for(int s=0;s<starts;++s) {
        Vec initial_random(c.n);
        for(int i=0;i<c.n;++i) initial_random[i]=c.lo[i]+uniform(rng)*(c.hi[i]-c.lo[i]);
        Result candidate=solve(c,target,initial_random,base,posture,o);
        const double next=candidate.position+0.05*o.rotation*candidate.rotation;
        if(next<score) { best=std::move(candidate); score=next; }
    }
    return best;
}
template<typename Fn> int guarded(Fn operation) {
    try { operation(); error.clear(); return 0; }
    catch(const std::exception& e) { error=e.what(); return -1; }
    catch(...) { error="unknown native arm error"; return -1; }
}
}
extern "C" {
const char* ar_arm_last_error() { return error.c_str(); }
void* ar_arm_create(int count,int n,const int* movable,const double* before,const double* after,
                    const double* axes,const double* lower,const double* upper) {
    void* result=nullptr;
    guarded([&] {
        require(count>0 && n>0,"arm chain must contain joints");
        require(movable && before && after && axes,"null arm chain buffers");
        auto c=std::make_unique<Chain>();
        c->n=n; c->lo=vector(lower,n); c->hi=vector(upper,n);
        require((c->lo.array()<=c->hi.array()).all(),"invalid joint limits");
        int joints=0;
        for(int i=0;i<count;++i) {
            require(movable[i]==0 || movable[i]==1,"invalid joint kind");
            Element e{movable[i],pose(before+16*i),pose(after+16*i),Eigen::Map<const Eigen::Vector3d>(axes+3*i)};
            require(e.axis.allFinite(),"non-finite arm axis");
            if(e.movable) { require(std::abs(e.axis.norm()-1.0)<1e-6,"joint axis must have unit length"); ++joints; }
            c->elements.push_back(e);
        }
        require(joints==n,"arm chain joint count mismatch");
        result=c.release();
    });
    return result;
}
void ar_arm_destroy(void* pointer) { delete static_cast<Chain*>(pointer); }
int ar_arm_fk(void* pointer,const double* q,const double* base,double* out,double* axes,double* origins) {
    return guarded([&] {
        Chain& c=chain(pointer);
        require(out && axes && origins,"null FK output buffer");
        Matrix a,p; Mat4 t=forward(c,vector(q,c.n),pose(base),&a,&p);
        Eigen::Map<Mat4> output_pose(out);
        output_pose=t;
        Eigen::Map<Matrix>(axes,c.n,3)=a;
        Eigen::Map<Matrix>(origins,c.n,3)=p;
    });
}
int ar_arm_jacobian(void* pointer,const double* q,const double* base,double* out,double* out_j) {
    return guarded([&] {
        Chain& c=chain(pointer);
        require(out && out_j,"null Jacobian output buffer");
        Matrix j; Mat4 t=jacobian(c,vector(q,c.n),pose(base),j);
        Eigen::Map<Mat4> output_pose(out);
        output_pose=t;
        Eigen::Map<Matrix>(out_j,6,c.n)=j;
    });
}
int ar_arm_solve(void* pointer,const double* target,const double* initial,const double* base,const double* posture,
                 const double* params,int iterations,int starts,unsigned int seed,double* out,double* errors) {
    return guarded([&] {
        Chain& c=chain(pointer);
        require(out && errors,"null IK output buffer");
        Vec reference=posture?vector(posture,c.n):Vec();
        Result r=multistart(c,pose(target),vector(initial,c.n),pose(base),posture?&reference:nullptr,
                            options(params,iterations),starts,seed);
        Eigen::Map<Vec>(out,c.n)=r.q;
        errors[0]=r.position; errors[1]=r.rotation; errors[2]=r.iterations;
    });
}
int ar_arm_trajectory(void* pointer,int frames,const double* targets,const double* initial,const double* base,
                      const double* posture,const double* params,int iterations,int starts,unsigned int seed,
                      double* out,double* position,double* rotation) {
    return guarded([&] {
        Chain& c=chain(pointer);
        require(frames>=0 && starts>=0,"invalid trajectory length or multistart count");
        require(frames==0 || (targets && out && position && rotation),"null trajectory buffer");
        Vec q=vector(initial,c.n);
        Vec reference=posture?vector(posture,c.n):Vec();
        Mat4 b=pose(base);
        Options o=options(params,iterations);
        for(int i=0;i<frames;++i) {
            Result r=multistart(c,pose(targets+16*i),q,b,posture?&reference:nullptr,o,i==0?starts:0,seed);
            q=r.q;
            Eigen::Map<Vec>(out+i*c.n,c.n)=q;
            position[i]=r.position; rotation[i]=r.rotation;
        }
    });
}
}
