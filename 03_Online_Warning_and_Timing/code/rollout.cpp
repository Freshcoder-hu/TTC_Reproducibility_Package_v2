// Coupled finite-horizon predictor only. This does not replace the evaluator.
// Same Couzin interaction, slot geometry, safety filter and unicycle equations;
// a coarser integration step is an explicit prediction approximation.
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
using std::vector;
struct V { double x,y; V operator+(V b)const{return{x+b.x,y+b.y};}
 V operator-(V b)const{return{x-b.x,y-b.y};}
 V operator*(double a)const{return{x*a,y*a};}
 V operator/(double a)const{return{x/a,y/a};} };
double dot(V a,V b){return a.x*b.x+a.y*b.y;}
double norm(V a){return std::sqrt(dot(a,a));}
V unit(V a){double n=norm(a);return n>1e-9?a/n:V{0,0};}
double clip(double a,double l,double h){return std::max(l,std::min(h,a));}
double wrap(double a){while(a>M_PI)a-=2*M_PI;while(a<-M_PI)a+=2*M_PI;return a;}
V heading(double a){return{std::cos(a),std::sin(a)};}
struct Path {
 vector<V> p,tang,normal; vector<double> len,cum,width;
 Path(const double* x,const double*w,int k){
  cum.push_back(0);
  for(int i=0;i<k;i++){p.push_back({x[2*i],x[2*i+1]});width.push_back(w[i]);}
  for(int i=0;i<k-1;i++){V d=p[i+1]-p[i];double l=norm(d);len.push_back(l);tang.push_back(d/l);normal.push_back({-d.y/l,d.x/l});cum.push_back(cum.back()+l);}
 }
 int segment(double s)const{int i=0;while(i+1<(int)len.size()&&cum[i+1]<=s)i++;return i;}
 void closest(V a,double&s,double&lat,double&w)const{
  double best=1e300;s=lat=0;w=width[0];
  for(int i=0;i<(int)len.size();i++){
   double u=clip(dot(a-p[i],tang[i])/len[i],0,1);V diff=a-(p[i]+tang[i]*(u*len[i]));double q=dot(diff,diff);
   if(q<best){best=q;s=cum[i]+u*len[i];lat=dot(diff,normal[i]);w=width[i]*(1-u)+width[i+1]*u;}
  }
 }
 V point(double s)const{s=clip(s,0,cum.back());int i=segment(s);return p[i]+tang[i]*(s-cum[i]);}
 V tangent(double s)const{return tang[segment(clip(s,0,cum.back()))];}
};
V centroid(const vector<V>&p){V c{0,0};for(V v:p)c=c+v;return c/(double)p.size();}
double radius95(const vector<V>&p,V c){vector<double> r;for(V v:p)r.push_back(norm(v-c));std::sort(r.begin(),r.end());double k=.95*(r.size()-1);int i=(int)k;return std::max(1.6,r[i]*(1-(k-i))+r[std::min(i+1,(int)r.size()-1)]*(k-i));}

extern "C" void evaluate_rollouts(
 int N,int M,int K,int P,int A,const double* sheep_pos,const double* sheep_vel,
 const double* robot_pos,const double* robot_head,const double* robot_speed,
 const double* obs_pos,const double* obs_vel,const double* obs_radius,
 const double* path_pos,const double* path_width,const double* cfg,
 const double* offsets,double dt,int steps,double* out){
 // cfg: vbase,vflee,turn,rrep,rori,ratt,ravo,avok,goalbias,vmax,omega,alin,
 //      kp,drr,dro,gap,rsheep,lookahead
 Path path(path_pos,path_width,P);
 for(int action=0;action<A;action++){
  vector<V> p(N),rob(M),op(K);vector<double> head(N),spd(N),rh(M),rv(M);
  for(int i=0;i<N;i++){p[i]={sheep_pos[2*i],sheep_pos[2*i+1]};head[i]=std::atan2(sheep_vel[2*i+1],sheep_vel[2*i]);spd[i]=std::hypot(sheep_vel[2*i],sheep_vel[2*i+1]);}
  for(int j=0;j<M;j++){rob[j]={robot_pos[2*j],robot_pos[2*j+1]};rh[j]=robot_head[j];rv[j]=robot_speed[j];}
  for(int k=0;k<K;k++)op[k]={obs_pos[2*k],obs_pos[2*k+1]};
  vector<double> margin(N,1e100);double outside=0,centerpen=0,spreadpen=0,progress=0;
  double init_s,lat,w;path.closest(centroid(p),init_s,lat,w);
  for(int tick=0;tick<steps;tick++){
   V c=centroid(p);double R=radius95(p,c),s;path.closest(c,s,lat,w);
   double ahead=std::abs(offsets[action])<1e-8?6:cfg[17];
   V tan=path.tangent(s+ahead);V normal{-tan.y,tan.x};
   V direction=unit(path.point(s+ahead)+normal*offsets[action]-c);
   vector<V> slot(M),cmd(M);
   for(int j=0;j<M;j++){
    double phi=M_PI+(-.95+1.9*j/std::max(1,M-1));V perp{-direction.y,direction.x};
    slot[j]=c+(direction*std::cos(phi)+perp*std::sin(phi))*(R+.62*cfg[6]);
   }
   for(int it=0;it<6;it++){
    bool moved=false;
    for(int i=0;i<M;i++)for(int j=i+1;j<M;j++){
     V d=slot[i]-slot[j];double dist=norm(d),limit=cfg[13]+cfg[15];
     if(dist<limit){if(dist<1e-6){d={-direction.y,direction.x};dist=1;}V corr=d*((limit-dist)/(2*dist));slot[i]=slot[i]+corr;slot[j]=slot[j]-corr;moved=true;}
    }
    if(!moved)break;
   }
   for(int j=0;j<M;j++){cmd[j]=(slot[j]-rob[j])*cfg[12];double n=norm(cmd[j]);if(n>cfg[9])cmd[j]=cmd[j]*(cfg[9]/n);}
   for(int it=0;it<4;it++){
    vector<V> next(M);for(int j=0;j<M;j++)next[j]=rob[j]+cmd[j]*dt;
    for(int i=0;i<M;i++)for(int j=i+1;j<M;j++){
     V d=next[i]-next[j];double dist=norm(d);
     if(dist<cfg[13]){if(dist<1e-6){d={1,0};dist=1;}V corr=unit(d)*((cfg[13]-dist)/(2*dt));cmd[i]=cmd[i]+corr;cmd[j]=cmd[j]-corr;}
    }
   }
   for(int i=0;i<M;i++)for(int j=i+1;j<M;j++){
    V d=rob[i]-rob[j];double dist=norm(d);if(dist<1e-6)continue;
    V n=d/dist;double close=dot(cmd[i]-cmd[j],n);
    if(dist<cfg[13]&&close<0){V corr=n*(.5*close);cmd[i]=cmd[i]-corr;cmd[j]=cmd[j]+corr;}
   }
   for(int j=0;j<M;j++){
    double nearest=1e100;V d{0,0};
    for(int k=0;k<K;k++){V z=rob[j]-op[k];double dist=norm(z);if(dist<nearest){nearest=dist;d=z;}}
    if(nearest<cfg[14])cmd[j]=cmd[j]+unit(d)*((cfg[14]-nearest)*2);
    double n=norm(cmd[j]);if(n>cfg[9])cmd[j]=cmd[j]*(cfg[9]/n);
    double desired=std::min(norm(cmd[j]),cfg[9]),vd=0;
    if(desired>1e-3){double target=std::atan2(cmd[j].y,cmd[j].x);rh[j]+=clip(wrap(target-rh[j]),-cfg[10]*dt,cfg[10]*dt);vd=desired*std::max(0.,std::cos(wrap(target-rh[j])));}
    rv[j]=clip(rv[j]+clip(vd-rv[j],-cfg[11]*dt,cfg[11]*dt),0,cfg[9]);rob[j]=rob[j]+heading(rh[j])*(rv[j]*dt);
   }
   vector<V> updated(N);vector<double> updatedhead(N),updatedspeed(N);
   for(int i=0;i<N;i++){
    V rep{0,0},ali{0,0},att{0,0},rep_robot{0,0};int nr=0,na=0,robot_n=0;
    for(int j=0;j<N;j++)if(i!=j){
     V d=p[i]-p[j];double dist=norm(d);
     if(dist<cfg[3]){rep=rep+d/std::max(dist,1e-6);nr++;}
     else if(dist<cfg[4])ali=ali+heading(head[j]);
     else if(dist<cfg[5]){att=att+p[j];na++;}
    }
    for(int j=0;j<M;j++){V d=p[i]-rob[j];double dist=norm(d);if(dist<cfg[6]){rep_robot=rep_robot+d*(cfg[7]*(1-dist/cfg[6])/std::max(dist,1e-6));robot_n++;}}
    V want,rep_total=rep+rep_robot;
    if((nr>0||robot_n>0)&&norm(rep_total)>1e-6)want=unit(rep_total);
    else{
     V steer{0,0};
     if(robot_n==0){steer=ali*.6+(att/(double)std::max(na,1)-p[i])*.4;double si,li,wi;path.closest(p[i],si,li,wi);steer=steer+path.tangent(si)*cfg[8];}
     want=norm(steer)>1e-6?unit(steer):heading(head[i]);
    }
    updatedhead[i]=head[i]+clip(wrap(std::atan2(want.y,want.x)-head[i]),-cfg[2]*dt,cfg[2]*dt);
    updatedspeed[i]=robot_n>0?cfg[1]:cfg[0];updated[i]=p[i]+heading(updatedhead[i])*(updatedspeed[i]*dt);
   }
   p.swap(updated);head.swap(updatedhead);spd.swap(updatedspeed);
   for(int k=0;k<K;k++)op[k]=op[k]+V{obs_vel[2*k],obs_vel[2*k+1]}*dt;
   c=centroid(p);double cs,cl,cw;path.closest(c,cs,cl,cw);centerpen+=cl*cl/(cw*cw);
   for(int i=0;i<N;i++){
    double si,li,wi;path.closest(p[i],si,li,wi);double exceed=std::max(0.,std::abs(li)-wi+cfg[16]);outside+=exceed*exceed;
    spreadpen+=std::max(0.,norm(p[i]-c)-12)*std::max(0.,norm(p[i]-c)-12);
    for(int k=0;k<K;k++)margin[i]=std::min(margin[i],norm(p[i]-op[k])-cfg[16]-obs_radius[k]);
   }
   progress=cs-init_s;
  }
  double count=0,pressure=0,worst=1e100;
  for(double m:margin){count+=(m<0);pressure+=std::pow(std::max(0.,.8-m),2);worst=std::min(worst,m);}
  out[action*7]=count/N;out[action*7+1]=pressure/N;
  out[action*7+2]=outside/(N*steps);out[action*7+3]=centerpen/steps;
  out[action*7+4]=spreadpen/(N*steps);out[action*7+5]=progress;out[action*7+6]=worst;
 }
}
