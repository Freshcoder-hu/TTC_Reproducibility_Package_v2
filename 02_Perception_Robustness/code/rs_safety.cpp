// Observation-only finite-candidate safety supervisor. Forecasts use the same
// joint robot limits as limits_v2.py; animal velocities are held constant.
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
static double cl(double x,double a,double b){return std::max(a,std::min(b,x));}
static double wr(double x){double y=std::fmod(x+M_PI,2*M_PI);if(y<0)y+=2*M_PI;return y-M_PI;}
static void step(double cx,double cy,double&v,double&h,double&w,const double*p){
 double dt=p[0],vmax=p[1],wmax=p[2],a=p[3],aa=p[4],alat=p[5];
 double sd=std::min(std::hypot(cx,cy),vmax),td=0,des=0;
 if(sd>1e-3){des=std::atan2(cy,cx);td=cl(wr(des-h),-wmax*dt,wmax*dt);}
 double wd=td/dt,wn=cl(wd,w-aa*dt,w+aa*dt);wn=cl(wn,-wmax,wmax);
 double turn=wn==wd?td:wn*dt,hn=h+turn;
 double vd=sd>1e-3?sd*std::max(0.,std::cos(wr(des-hn))):0.;
 double vn=cl(v+cl(vd-v,-a*dt,a*dt),0,vmax);
 if(vn*std::abs(wn)>alat*(1+1e-15)){
  double vlo=std::max(0.,v-a*dt),wm=std::max(0.,std::abs(w)-aa*dt),vc=INFINITY;
  double sg=wn>=0?1:-1;
  for(int z=0;z<3;z++){
   if(vn<=0){wn=0;break;}
   double wnew=alat/vn;
   if(wnew>=wm)wn=sg*wnew;
   else{wn=sg*wm;if(wm>0)vc=std::min(vc,alat/wm);}
   hn=h+wn*dt;vd=sd>1e-3?sd*std::max(0.,std::cos(wr(des-hn))):0.;
   vn=std::min(cl(v+cl(vd-v,-a*dt,a*dt),0,vmax),vc);
   vn=std::max(vn,std::min(vlo,vc));
   if(vn*std::abs(wn)<=alat*(1+1e-15))break;
   if(wn!=0)vn=std::min(vn,alat/std::abs(wn));
  }
 }
 v=vn;h=hn;w=wn;
}
extern "C" void test_rs_step(double cx,double cy,double*v,double*h,double*w,const double*p){step(cx,cy,*v,*h,*w,p);}
static double segdist(double ax,double ay,double bx,double by){
 double dx=bx-ax,dy=by-ay,s=dx*dx+dy*dy;
 double t=s>1e-30?cl(-(ax*dx+ay*dy)/s,0,1):0;
 return std::hypot(ax+t*dx,ay+t*dy);
}
extern "C" void rs_select(int M,int N,int K,const double*pos,const double*head,
 const double*speed,const double*omega,const double*cmd,const double*sp,
 const double*sv,const double*sa,const double*op,const double*ov,const double*oa,
 const double*p,double*out,double*stats){
 // p: dt, vmax,wmax,a,aa,alat,H,rs_radius,rr_radius,ro_radius,
 // margin,animal_sigma,obstacle_sigma,sigma_scale,growth
 int L=std::max(1,(int)std::round(p[6]/p[0]));double dt=p[0];
 const double angles[]={0,.35,-.35,.70,-.70,1.1,-1.1,1.57,-1.57,2.2,-2.2,M_PI};
 for(int i=0;i<M;i++){
  double bx=cmd[2*i],by=cmd[2*i+1],bn=std::hypot(bx,by);
  double bh=bn>1e-3?std::atan2(by,bx):head[i];
  double bestscore=INFINITY,bestx=0,besty=0,bestw=-INFINITY;bool found=false;
  int bestidx=-1;
  for(int c=0;c<25;c++){
   double cn=c==24?0:bn*(c<12?1:.45);
   double ca=bh+angles[c%12],cx=cn*std::cos(ca),cy=cn*std::sin(ca);
   double x=pos[2*i],y=pos[2*i+1],v=speed[i],h=head[i],w=omega[i];
   double worst=INFINITY,lastmin=INFINITY;
   for(int l=1;l<=L;l++){
    double oldx=x,oldy=y;step(cx,cy,v,h,w,p);x+=v*std::cos(h)*dt;y+=v*std::sin(h)*dt;
    double t=l*dt,prev=t-dt;lastmin=INFINITY;
    for(int q=0;q<N;q++){
     double age=sa[q],x0=sp[2*q]+sv[2*q]*(age+prev),y0=sp[2*q+1]+sv[2*q+1]*(age+prev);
     double x1=x0+sv[2*q]*dt,y1=y0+sv[2*q+1]*dt;
     double buffer=p[10]+p[13]*p[11]+p[14]*(age+t);
     double clr=segdist(oldx-x0,oldy-y0,x-x1,y-y1)-p[7]-buffer;
     worst=std::min(worst,clr);lastmin=std::min(lastmin,std::hypot(x-x1,y-y1)-p[7]-buffer);
    }
    for(int q=0;q<M;q++)if(q!=i){
     double ux=speed[q]*std::cos(head[q]),uy=speed[q]*std::sin(head[q]);
     double clr=segdist(oldx-pos[2*q]-ux*prev,oldy-pos[2*q+1]-uy*prev,
                        x-pos[2*q]-ux*t,y-pos[2*q+1]-uy*t)-p[8]-.10;
     worst=std::min(worst,clr);lastmin=std::min(lastmin,clr);
    }
    for(int q=0;q<K;q++){
     double x0=op[2*q]+ov[2*q]*(oa[q]+prev),y0=op[2*q+1]+ov[2*q+1]*(oa[q]+prev);
     double clr=segdist(oldx-x0,oldy-y0,x-x0-ov[2*q]*dt,y-y0-ov[2*q+1]*dt)
                    -p[9]-p[10]-p[13]*p[12];
     worst=std::min(worst,clr);lastmin=std::min(lastmin,clr);
    }
   }
   bool safe=worst>=0;
   double dev=((cx-bx)*(cx-bx)+(cy-by)*(cy-by))/(1+bn*bn);
   double score=safe?dev:-worst-.05*cl(lastmin,-5,5)+.01*dev;
   bool choose=(safe&&!found)||(safe==found&&score<bestscore);
   if(choose){found=safe;bestscore=score;bestx=cx;besty=cy;bestw=worst;bestidx=c;}
   // Unchanged command is feasible and is the unique minimum deviation.
   if(c==0&&safe)break;
  }
  out[2*i]=bestx;out[2*i+1]=besty;
  stats[3*i]=bestidx;stats[3*i+1]=found?1:0;stats[3*i+2]=bestw;
 }
}
