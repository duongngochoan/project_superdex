// Native hand FK, analytical vector/adaptive objectives and NLopt SLSQP.
/*
Native ports of wuji-retargeting vector and adaptive analytical objectives.
MIT License

Copyright (c) 2025 WujiHand Technologies

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using V = Eigen::VectorXd;
using M = Eigen::MatrixXd;
using T4 = Eigen::Matrix4d;
using V3 = Eigen::Vector3d;
using Row4 = Eigen::Matrix<double,4,4,Eigen::RowMajor>;
thread_local std::string hand_error;
using Opt = void*;
using Objective = double (*)(unsigned, const double*, double*, void*);
struct Nlopt {
    void* lib = nullptr;
    Opt opt = nullptr;
    Opt (*create)(int,unsigned) = nullptr;
    void (*destroy)(Opt) = nullptr;
    int (*set_lower)(Opt,const double*) = nullptr;
    int (*set_upper)(Opt,const double*) = nullptr;
    int (*set_obj)(Opt,Objective,void*) = nullptr;
    int (*maxeval)(Opt,int) = nullptr;
    int (*ftol)(Opt,double) = nullptr;
    int (*optimize)(Opt,double*,double*) = nullptr;
    template<class F> F symbol(const char* name) {
        void* ptr = dlsym(lib,name);
        if (!ptr) throw std::runtime_error(std::string("NLopt symbol unavailable: ")+name);
        return reinterpret_cast<F>(ptr);
    }
    Nlopt(const char* path, int algorithm, int n, const double* lo, const double* hi) {
        lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
        if (!lib) throw std::runtime_error(std::string("Cannot load native NLopt: ")+dlerror());
        try {
        create=symbol<decltype(create)>("nlopt_create"); destroy=symbol<decltype(destroy)>("nlopt_destroy");
        set_lower=symbol<decltype(set_lower)>("nlopt_set_lower_bounds"); set_upper=symbol<decltype(set_upper)>("nlopt_set_upper_bounds");
        set_obj=symbol<decltype(set_obj)>("nlopt_set_min_objective"); maxeval=symbol<decltype(maxeval)>("nlopt_set_maxeval");
        ftol=symbol<decltype(ftol)>("nlopt_set_ftol_abs"); optimize=symbol<decltype(optimize)>("nlopt_optimize");
        opt=create(algorithm,n);
        if (!opt) throw std::runtime_error("Cannot construct NLopt optimizer");
        if (set_lower(opt,lo)<0 || set_upper(opt,hi)<0 || maxeval(opt,50)<0 || ftol(opt,1e-4)<0)
            throw std::runtime_error("Invalid NLopt settings");
        } catch (...) {
            if (opt && destroy) destroy(opt);
            dlclose(lib);
            opt = nullptr; lib = nullptr;
            throw;
        }
    }
    ~Nlopt() { if (opt) destroy(opt); if (lib) dlclose(lib); }
};
struct Hand {
    int n,nj,nf,mode=0,wrist=0;
    std::vector<int> parent,qindex,frame_parent,roles,pip,dip,vec;
    std::vector<T4> origin,frame_offset;
    std::vector<V3> axis;
    V lo,hi;
    std::vector<double> scale,p;
    std::unique_ptr<Nlopt> solver;
    Hand(int n_,int nj_,int nf_,const int* pa,const int* qi,const double* org,const double* ax,
         const int* fp,const double* fo,const double* l,const double* h,const char* nlpath,int algorithm)
      : n(n_),nj(nj_),nf(nf_),parent(pa,pa+nj),qindex(qi,qi+nj),frame_parent(fp,fp+nf),
        lo(Eigen::Map<const V>(l,n)),hi(Eigen::Map<const V>(h,n)) {
        if(n<=0 || nj<=0 || nf<=0 || !lo.allFinite() || !hi.allFinite() || (lo.array()>hi.array()).any())
            throw std::invalid_argument("Invalid hand model dimensions or limits");
        std::vector<bool> observed(n,false);
        for(int j=0;j<nj;++j) {
            if(parent[j]<0 || parent[j]>=nj || (j>0 && parent[j]>=j) || qindex[j]<-1 || qindex[j]>=n)
                throw std::invalid_argument("Hand joints must be topologically ordered");
            if(qindex[j]>=0) {
                if(observed[qindex[j]]) throw std::invalid_argument("Duplicate hand qpos index");
                observed[qindex[j]]=true;
            }
            origin.push_back(Eigen::Map<const Row4>(org+16*j));
            axis.push_back(Eigen::Map<const V3>(ax+3*j));
            if(!origin.back().allFinite() || !axis.back().allFinite()) throw std::invalid_argument("Invalid joint geometry");
            if(qindex[j]>=0 && std::abs(axis.back().norm()-1)>1e-8) throw std::invalid_argument("Hand joint axis must be unit length");
        }
        if(qindex[0]!=-1 || !std::all_of(observed.begin(),observed.end(),[](bool value){return value;}))
            throw std::invalid_argument("Hand qpos indices must cover every actuated joint");
        for(int f=0;f<nf;++f) {
            if(fp[f]<0 || fp[f]>=nj) throw std::invalid_argument("Invalid frame parent");
            frame_offset.push_back(Eigen::Map<const Row4>(fo+16*f));
            if(!frame_offset.back().allFinite()) throw std::invalid_argument("Invalid hand frame placement");
        }
        solver=std::make_unique<Nlopt>(nlpath,algorithm,n,l,h);
    }
    void fk(const V& q,std::vector<T4>& frames,std::vector<M>* jac=nullptr) const {
        std::vector<T4> world(nj,T4::Identity());
        std::vector<V3> axes(nj,V3::Zero()),centers(nj,V3::Zero());
        for(int j=1;j<nj;++j) {
            world[j]=world[parent[j]]*origin[j];
            if(qindex[j]>=0) {
                axes[j]=world[j].block<3,3>(0,0)*axis[j]; centers[j]=world[j].block<3,1>(0,3);
                T4 rot=T4::Identity(); rot.block<3,3>(0,0)=Eigen::AngleAxisd(q[qindex[j]],axis[j]).toRotationMatrix();
                world[j]=world[j]*rot;
            }
        }
        frames.resize(nf); if(jac) jac->assign(nf,M::Zero(3,n));
        for(int f=0;f<nf;++f) {
            frames[f]=world[frame_parent[f]]*frame_offset[f];
            if(jac) for(int j=frame_parent[f];j>0;j=parent[j]) if(qindex[j]>=0)
                (*jac)[f].col(qindex[j])=axes[j].cross(V3(frames[f].block<3,1>(0,3))-centers[j]);
        }
    }
    double loss(const V& q,const double* kp,const V* previous,V& grad) const {
        std::vector<T4> frames; std::vector<M> jac; fk(q,frames,&jac);
        grad=V::Zero(n); double total=0;
        auto point=[&](int f)->V3 { return frames[f].block<3,1>(0,3)*100.; };
        auto key=[&](int i)->V3 { return Eigen::Map<const V3>(kp+3*i); };
        auto add=[&](const V3& diff,const M& J,double delta,double weight) {
            const double d=diff.norm();
            total+=weight*(d<=delta ? .5*d*d : delta*(d-.5*delta));
            grad+=weight*std::min(d,delta)/(d+1e-8)*J.transpose()*diff;
        };
        if(mode==0) {
            int nv=static_cast<int>(scale.size());
            for(int i=0;i<nv;++i) {
                int o=vec[4*i],t=vec[4*i+1],ko=vec[4*i+2],kt=vec[4*i+3];
                add(point(t)-point(o)-(key(kt)-key(ko))*100.*scale[i],(jac[t]-jac[o])*100.,p[0],1./nv);
            }
        } else {
            double alpha[5]={0,0,0,0,0};
            for(int f=1;f<5;++f) {
                double dist=(key(4+f*4)-key(4)).norm()*100.;
                alpha[f]=std::clamp((p[32+f-1]-dist)/(p[32+f-1]-p[28+f-1]+1e-8),0.,.7);
                alpha[0]=std::max(alpha[0],alpha[f]);
            }
            for(int f=0;f<5;++f) {
                int pp=roles[f],dd=roles[5+f],tt=roles[10+f],ki=4*f;
                add(point(tt)-point(wrist)-(key(ki+4)-key(0))*100.*p[5],(jac[tt]-jac[wrist])*100.,p[0],alpha[f]*p[3]);
                V3 v=point(tt)-point(dd); double norm=v.norm(); V3 u=v/(norm+1e-8);
                V3 target=key(ki+4)-key(ki+3); target/=target.norm()+1e-8;
                M Jnorm=(Eigen::Matrix3d::Identity()-u*u.transpose())/(norm+1e-8);
                add(u-target,Jnorm*(jac[tt]-jac[dd])*100.,p[2],alpha[f]*p[4]);
                bool skip=(f==0 && p[7]!=0); double weight=(1-alpha[f])*p[6]/(skip ? 2. : 3.);
                int fs[3]={pp,dd,tt};
                for(int seg=skip?1:0;seg<3;++seg)
                    add(point(fs[seg])-point(wrist)-(key(ki+2+seg)-key(0))*100.*p[13+f*3+seg],
                        (jac[fs[seg]]-jac[wrist])*100.,p[0],weight);
                for(int index:{pip[f],dip[f]}) {
                    double penalty=std::max(p[9]-q[index],0.);
                    total+=p[8]*penalty*penalty; grad[index]-=2*p[8]*penalty;
                }
                double diff=q[dip[f]]-p[11]*q[pip[f]];
                total+=p[10]*diff*diff; grad[dip[f]]+=2*p[10]*diff; grad[pip[f]]-=2*p[10]*p[11]*diff;
            }
        }
        if(previous) {total+=p[1]*(q-*previous).squaredNorm();grad+=2*p[1]*(q-*previous);}
        return total;
    }
};
struct Context {const Hand* hand;const double* kp;const V* previous;};
double objective(unsigned n,const double* q,double* gradient,void* data) {
    auto& c=*static_cast<Context*>(data); V g;
    double value=c.hand->loss(Eigen::Map<const V>(q,n),c.kp,c.previous,g);
    if(gradient) std::copy(g.data(),g.data()+n,gradient);
    return value;
}
Hand& checked(void* handle) { if(!handle) throw std::invalid_argument("Null hand handle"); return *static_cast<Hand*>(handle); }
void finite(const double* p,int n) { if(n>0 && !p) throw std::invalid_argument("Null hand input buffer"); for(int i=0;i<n;++i) if(!std::isfinite(p[i])) throw std::invalid_argument("Hand inputs must be finite"); }
}
extern "C" {
const char* rt_hand_last_error() { return hand_error.c_str(); }
void* rt_hand_create(int n,int nj,int nf,const int* pa,const int* qi,const double* org,const double* ax,
                    const int* fp,const double* fo,const double* lo,const double* hi,const char* nlpath,int algorithm) {
    try {
        hand_error.clear();
        if(n<=0 || nj<=0 || nf<=0 || !pa || !qi || !org || !ax || !fp || !fo || !lo || !hi || !nlpath)
            throw std::invalid_argument("Invalid hand dimensions or null model buffer");
        return new Hand(n,nj,nf,pa,qi,org,ax,fp,fo,lo,hi,nlpath,algorithm);
    }
    catch(const std::exception& e) {hand_error=e.what();return nullptr;}
}
void rt_hand_destroy(void* handle) { delete static_cast<Hand*>(handle); }
int rt_hand_configure(void* handle,int mode,int wrist,const int* roles,const int* pip,const int* dip,
                      int nv,const int* vec,const double* scales,const double* params) {
    try {
        auto& h=checked(handle); if(!roles || !pip || !dip || !vec || !scales || !params) throw std::invalid_argument("Null hand objective buffer"); if(mode<0 || mode>1 || wrist<0 || wrist>=h.nf || nv<=0) throw std::invalid_argument("Invalid hand objective configuration");
        h.mode=mode;h.wrist=wrist;h.roles.assign(roles,roles+15);h.pip.assign(pip,pip+5);h.dip.assign(dip,dip+5);
        h.vec.assign(vec,vec+4*nv);h.scale.assign(scales,scales+nv);h.p.assign(params,params+36);
        finite(params,36);finite(scales,nv);
        if(h.p[0]<=0 || h.p[2]<=0 || h.p[12]<=0 || h.p[12]>1) throw std::invalid_argument("Huber deltas must be positive; lp_alpha must be in (0,1]");
        for(int i:h.roles) if(i<0 || i>=h.nf) throw std::invalid_argument("Invalid objective link index");
        for(int i:h.pip) if(i<0 || i>=h.n) throw std::invalid_argument("Invalid PIP joint index");
        for(int i:h.dip) if(i<0 || i>=h.n) throw std::invalid_argument("Invalid DIP joint index");
        std::vector<int> flex=h.pip;flex.insert(flex.end(),h.dip.begin(),h.dip.end());
        std::sort(flex.begin(),flex.end());
        if(std::adjacent_find(flex.begin(),flex.end())!=flex.end()) throw std::invalid_argument("PIP/DIP joint indices must be distinct");
        for(int k=0;k<nv;++k) if(vec[4*k]<0 || vec[4*k]>=h.nf || vec[4*k+1]<0 || vec[4*k+1]>=h.nf || vec[4*k+2]<0 || vec[4*k+2]>20 || vec[4*k+3]<0 || vec[4*k+3]>20) throw std::invalid_argument("Invalid key vector");
        for(int i=0;i<4;++i) if(h.p[32+i]<=h.p[28+i]) throw std::invalid_argument("Pinch d2 must exceed d1");
        return 0;
    } catch(const std::exception& e){hand_error=e.what();return -1;}
}
int rt_hand_sequence(void* handle,int count,const double* keypoints,int filter,double* output,double* cost,int* statuses) {
    try {
        auto& h=checked(handle);if(count<=0 || h.p.empty() || !output || !cost || !statuses) throw std::invalid_argument("Empty hand sequence, unconfigured controller or null output");
        finite(keypoints,count*63);V previous,filtered,q=(h.lo+h.hi)*.5,gradient;
        for(int frame=0;frame<count;++frame) {
            Context c{&h,keypoints+frame*63,frame?&previous:nullptr}; V initial=q;
            if(h.solver->set_obj(h.solver->opt,objective,&c)<0) throw std::runtime_error("Cannot set NLopt objective");
            double value=0; int status=h.solver->optimize(h.solver->opt,q.data(),&value);
            statuses[frame]=status;
            // Match the existing wrapper's failed-solve behavior without changing the previous command.
            if(status<0) q=initial;
            for(int j=0;j<h.n;++j) q[j]=std::clamp(static_cast<double>(static_cast<float>(q[j])),h.lo[j],h.hi[j]);
            previous=q;
            cost[frame]=h.loss(q,c.kp,nullptr,gradient);
            if(filter) { if(frame==0) filtered=q;else filtered+=h.p[12]*(q-filtered); }
            const V& result=filter?filtered:q;std::copy(result.data(),result.data()+h.n,output+frame*h.n);
        }
        return 0;
    } catch(const std::exception& e){hand_error=e.what();return -1;}
}
int rt_hand_fk_points(void* handle,const double* q,int count,const int* indices,int wrist,double* output) {
    try {
        auto& h=checked(handle);finite(q,h.n);if(count<0 || wrist<0 || wrist>=h.nf || (count>0 && (!indices || !output))) throw std::invalid_argument("Invalid FK request");
        std::vector<T4> frames;h.fk(Eigen::Map<const V>(q,h.n),frames);T4 inv=frames[wrist].inverse();
        for(int i=0;i<count;++i){if(indices[i]<0 || indices[i]>=h.nf)throw std::invalid_argument("Invalid FK frame");
            V3 point=(inv*frames[indices[i]]).block<3,1>(0,3);std::copy(point.data(),point.data()+3,output+3*i);}
        return 0;
    }catch(const std::exception& e){hand_error=e.what();return -1;}
}
int rt_hand_cost_grad(void* handle,const double* q,const double* kp,const double* prev,double* loss,double* grad) {
    try {auto& h=checked(handle);if(h.p.empty() || !loss || !grad) throw std::invalid_argument("Unconfigured hand objective or null output");finite(q,h.n);finite(kp,63);V g,previous;
        if(prev){finite(prev,h.n);previous=Eigen::Map<const V>(prev,h.n);}
        *loss=h.loss(Eigen::Map<const V>(q,h.n),kp,prev?&previous:nullptr,g);
        std::copy(g.data(),g.data()+h.n,grad);return 0;
    }catch(const std::exception& e){hand_error=e.what();return -1;}
}
int rt_hand_squeeze(void* handle,int count,const double* q,const int* mask,const double* weight,double squeeze,double* output) {
    try {auto& h=checked(handle);if(count<=0 || !std::isfinite(squeeze) || !mask || !output)throw std::invalid_argument("Invalid squeeze request");
        finite(q,count*h.n);finite(weight,count);
        for(int i=0;i<count;++i)for(int j=0;j<h.n;++j) output[i*h.n+j]=std::clamp(q[i*h.n+j]+(mask[j]?squeeze*weight[i]:0.),h.lo[j],h.hi[j]);return 0;
    }catch(const std::exception& e){hand_error=e.what();return -1;}
}
}
