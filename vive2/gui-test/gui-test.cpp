#include "ultimate_tracker.h"
#include "kida_ik.h"
#include <GLFW/glfw3.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "third_party/stb_easy_font.h"
#pragma GCC diagnostic pop

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace avatar;
using Clock=std::chrono::steady_clock;
namespace {
volatile std::sig_atomic_t interrupted=0;
void interrupt_handler(int) { interrupted=1; }

struct Device {
    std::string serial;
    vive::TrackerData data{};
    Clock::time_point received{};
    bool connected=false, has_pose=false;
    unsigned long long count=0;
};

class Tracker final : public vive::ITrackerSubscriber {
public:
    Tracker():devices_(vive::GetNumTrackers()) {}
    ~Tracker() override { if (started_) vive::StopTrackerServer(); }
    bool start() { started_=vive::StartTrackerServer(); return started_; }
    std::vector<Device> snapshot() {
        std::lock_guard<std::mutex> lock(mutex_); return devices_;
    }
    void ServerStart() override {}
    void ServerStop() override { stopped.store(true); }
    void Add(uint32_t id,const char* serial) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id<devices_.size()) {
            devices_[id]={}; devices_[id].connected=true;
            devices_[id].serial=serial?serial:"";
        }
    }
    void Update(const vive::TrackerData& p) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (p.Id<devices_.size()) {
            auto& d=devices_[p.Id]; d.data=p; d.received=Clock::now();
            d.has_pose=true; ++d.count;
        }
    }
    void Remove(uint32_t id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (id<devices_.size()) devices_[id]={};
    }
    void Click(uint32_t,int,int clicks) override { if (clicks>=3) stopped.store(true); }
    void CommandResponse(const vive::TrackerCommandResponse&) override {}
    std::atomic<bool> stopped{false};
private:
    bool started_=false;
    std::mutex mutex_;
    std::vector<Device> devices_;
};

Pose pose_of(const Device& d) {
    const auto& p=d.data;
    return {{p.Location[0],p.Location[1],p.Location[2]},
            {p.Rotation[0],p.Rotation[1],p.Rotation[2],p.Rotation[3]}};
}
std::string health(const Device& d) {
    if (!d.connected||!d.has_pose) return "NOT CONNECTED";
    if (std::chrono::duration<double>(Clock::now()-d.received).count()>.3) return "STALE DATA";
    const auto& p=d.data;
    if (p.State==vive::Syncing) return "SYNCING MAP";
    if (p.State==vive::BuildMap) return "BUILDING MAP";
    if (p.Flags&vive::TrackerFlag_SetupRequired) return "MAP SETUP REQUIRED";
    if (p.Flags&vive::TrackerFlag_RotationOnly) return "ROTATION ONLY";
    // SDK viewer/debug labels: low pose bits 2=Good, 3=Calibrating.
    // Ready describes the map lifecycle; it does not imply a usable position.
    if (p.State!=vive::Ready) return "WAITING FOR MAP";
    unsigned pose_state=p.Flags&vive::TrackerFlag_PoseStateMask;
    if (pose_state==3) return "LOCALIZING / CALIBRATING";
    if (pose_state!=2) return "POSE STATE "+std::to_string(pose_state)+" / NOT TRACKING";
    auto pose=pose_of(d);
    const auto& q=pose.rotation;
    if (!finite(pose.position)||!std::isfinite(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)||
            q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<.1) return "INVALID POSE";
    return "TRACKING";
}
bool usable(const Device& d) { return health(d)=="TRACKING"; }

struct Color { float r,g,b; };
constexpr Color blue{.35f,.59f,.76f}, left_color{.26f,.86f,.68f},
    right_color{.72f,.55f,1.f}, white{.89f,.93f,.97f}, muted{.55f,.64f,.72f},
    amber{1.f,.71f,.34f};
void color(Color c) { glColor3f(c.r,c.g,c.b); }
void vertex(Vec3 p) { glVertex3d(p.x,p.y,p.z); }
void normal(Vec3 p) { p=unit(p); glNormal3d(p.x,p.y,p.z); }

void rotate(Quat q) {
    q=q.normalized(); double x=q.x,y=q.y,z=q.z,w=q.w;
    double m[16]={1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w),0,
        2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w),0,
        2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y),0, 0,0,0,1};
    glMultMatrixd(m);
}
void ellipsoid(Vec3 p,Vec3 scale,Color c,Quat q={}) {
    glPushMatrix(); glTranslated(p.x,p.y,p.z); rotate(q);
    glScaled(scale.x,scale.y,scale.z); color(c);
    for (int j=0;j<14;++j) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int i=0;i<=24;++i) for (int k : {1,0}) {
            double latitude=-pi/2+pi*(j+k)/14,longitude=2*pi*i/24;
            Vec3 v{std::cos(latitude)*std::cos(longitude),
                std::cos(latitude)*std::sin(longitude),std::sin(latitude)};
            normal(v); vertex(v);
        }
        glEnd();
    }
    glPopMatrix();
}
void segment(Vec3 a,Vec3 b,double ra,double rb,Color c) {
    Vec3 delta=b-a,direction=unit(delta);
    if (length(delta)<1e-8) return;
    Vec3 u=unit(cross(direction,std::abs(direction.z)<.9?Vec3{0,0,1}:Vec3{1,0,0}));
    Vec3 v=cross(direction,u); color(c);
    glBegin(GL_TRIANGLE_STRIP);
    for (int i=0;i<=24;++i) {
        double angle=2*pi*i/24;
        Vec3 radial=u*std::cos(angle)+v*std::sin(angle);
        normal(radial+direction*((ra-rb)/length(delta)));
        vertex(a+radial*ra); vertex(b+radial*rb);
    }
    glEnd();
    ellipsoid(a,{ra,ra,ra},c); ellipsoid(b,{rb,rb,rb},c);
}
void line(Vec3 a,Vec3 b,Color c,float width=2) {
    glDisable(GL_LIGHTING); color(c); glLineWidth(width);
    glBegin(GL_LINES); vertex(a); vertex(b); glEnd();
    glEnable(GL_LIGHTING);
}
void hand(Vec3 wrist,Quat q,int arm_index,double h,Color c) {
    auto point=[&](Vec3 v){ return wrist+q.rotate(v*h); };
    ellipsoid(point({.034,0,0}),{.045*h,.029*h,.013*h},c,q);
    for (int i=0;i<4;++i) {
        double y=(i-1.5)*.013;
        double extent=.105-(i==0||i==3?.009:0);
        segment(point({.058,y,0}),point({extent,y,0}),h*.0065,h*.005,c);
    }
    double thumb=kida::hand_thumb_direction(arm_index).y;
    segment(point({.025,thumb*.022,0}),point({.06,thumb*.047,-.006}),h*.008,h*.006,c);
    // A small dorsal patch makes the back of the hand unambiguous.
    ellipsoid(point({.035,0,.012}),{.018*h,.014*h,.003*h},{.18f,.35f,.49f},q);
}

void body_mesh(const Body& b) {
    double h=b.height;
    // Procedural mannequin: no licensed SMPL/SMPL-H asset required.
    struct Ring { double z,x,y; };
    // Narrow abdomen/pelvis so the visual body does not hide the robot arms.
    Ring rings[]={{.48,.055,.065},{.53,.054,.060},{.58,.045,.051},
        {.65,.060,.070},{.73,.085,.100},{.79,.075,.115},{.83,.045,.09}};
    color(blue);
    for (unsigned j=0;j+1<sizeof(rings)/sizeof(rings[0]);++j) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int i=0;i<=32;++i) for (int k : {0,1}) {
            auto r=rings[j+k]; double a=2*pi*i/32;
            normal({std::cos(a)/r.x,std::sin(a)/r.y,0});
            vertex({h*r.x*std::cos(a),h*r.y*std::sin(a),h*r.z});
        }
        glEnd();
    }
    ellipsoid({0,0,h*.505},{h*.060,h*.068,h*.065},blue);
    segment({0,0,h*.80},{0,0,h*.88},h*.033,h*.032,blue);
    ellipsoid({0,0,h*.935},{h*.073,h*.062,h*.09},blue);
    ellipsoid({h*.071,0,h*.935},{h*.017,h*.014,h*.025},blue);
    for (int side : {-1,1}) {
        ellipsoid({h*.067,side*h*.025,h*.956},{h*.009,h*.008,h*.006},{.1f,.18f,.24f});
        ellipsoid({0,side*h*.063,h*.935},{h*.017,h*.013,h*.028},blue);
        Vec3 hip{0,side*h*.058,h*.50},knee{h*.01,side*h*.065,h*.285},
            ankle{0,side*h*.065,h*.055};
        segment(hip,knee,h*.055,h*.034,blue);
        segment(knee,ankle,h*.034,h*.023,blue);
        ellipsoid({h*.035,side*h*.065,h*.033},{h*.075,h*.033,h*.032},blue);
    }
}
void cylinder(const kida::Transform& t,double radius,double half,Color c) {
    glPushMatrix();
    auto p=kida::pose(t); glTranslated(p.position.x,p.position.y,p.position.z); rotate(p.rotation);
    color(c); glBegin(GL_QUAD_STRIP);
    for (int i=0;i<=32;++i) {
        double angle=2*pi*i/32,x=std::cos(angle),y=std::sin(angle);
        glNormal3d(x,y,0); glVertex3d(radius*x,radius*y,-half); glVertex3d(radius*x,radius*y,half);
    }
    glEnd();
    for (int sign : {-1,1}) {
        glBegin(GL_TRIANGLE_FAN); glNormal3d(0,0,sign); glVertex3d(0,0,sign*half);
        for (int i=0;i<=32;++i) {
            double angle=sign*2*pi*i/32;
            glVertex3d(radius*std::cos(angle),radius*std::sin(angle),sign*half);
        }
        glEnd();
    }
    glPopMatrix();
}
void draw_arm(const kida::Arm& a,Pose target,Color accent,bool axes) {
    const auto& shapes=a.side()==0?kida::model::shapes0:kida::model::shapes1;
    for (auto shape : shapes) {
        auto t=(shape.body==7?a.fk.tcp:a.fk.joints[shape.body])*kida::transform(shape.offset);
        cylinder(t,shape.radius,shape.half_length,shape.body==7?accent:blue);
    }
    for (int j=0;j<7;++j) {
        auto p=kida::pose(a.fk.joints[j]);
        // This disk rotates about the actual local joint Z axis. J6/J7 share
        // an origin, but their axes and rotations remain independent.
        cylinder(a.fk.joints[j],.024,.006,accent);
        if (j<6) segment(p.position,kida::pose(a.fk.joints[j+1]).position,.012,.012,blue);
        if (axes) line(p.position,p.position+p.rotation.rotate({0,0,.065}),accent,3);
    }
    auto tcp=kida::pose(a.fk.tcp);
    hand(tcp.position,kida::hand_orientation(tcp.rotation,a.side()),a.side(),1,blue);
    if (a.limited()) {
        ellipsoid(target.position,{.012,.012,.012},amber);
        line(tcp.position,target.position,amber);
    }
    if (axes) {
        line(tcp.position,tcp.position+tcp.rotation.rotate({.12,0,0}),{1,.3f,.3f});
        line(tcp.position,tcp.position+tcp.rotation.rotate({0,.12,0}),{.3f,1,.3f});
        line(tcp.position,tcp.position+tcp.rotation.rotate({0,0,.12}),{.4f,.6f,1});
    }
}
void text(double x,double y,const std::string& value,Color c=white,double scale=1.5) {
    alignas(float) std::array<char,32768> buffer{};
    int quads=stb_easy_font_print(0,0,const_cast<char*>(value.c_str()),nullptr,buffer.data(),int(buffer.size()));
    glPushMatrix(); glTranslated(x,y,0); glScaled(scale,scale,1); color(c);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2,GL_FLOAT,16,buffer.data());
    glDrawArrays(GL_QUADS,0,quads*4);
    glDisableClientState(GL_VERTEX_ARRAY); glPopMatrix();
}
void rect(double x,double y,double w,double h,Color c) {
    color(c); glBegin(GL_QUADS);
    glVertex2d(x,y); glVertex2d(x+w,y); glVertex2d(x+w,y+h); glVertex2d(x,y+h); glEnd();
}
void look_at(Vec3 eye,Vec3 target) {
    Vec3 f=unit(target-eye),r=unit(cross(f,{0,0,1})),u=cross(r,f);
    double m[16]={r.x,u.x,-f.x,0,r.y,u.y,-f.y,0,r.z,u.z,-f.z,0,0,0,0,1};
    glMultMatrixd(m); glTranslated(-eye.x,-eye.y,-eye.z);
}

struct Options {
    unsigned left=0,right=1;
    double height=1.75,scale=1,seconds=0;
    bool demo=false,zero_pose=false;
    bool simulated() const { return demo||zero_pose; }
    std::string screenshot;
};
Options options(int argc,char** argv) {
    Options o;
    for (int i=1;i<argc;++i) {
        std::string key=argv[i];
        if (key=="--help") {
            std::puts("Usage: gui-test [--left-id N] [--right-id N] [--height METERS] [--scale GAIN]\n"
                      "            [--demo | --zero-pose] [--seconds N] [--screenshot frame.ppm]\n"
                      "C: calibrate ready pose (elbows bent, thumbs up, palms facing each other);\n"
                      "S: swap wrists; L/R: choose IDs; 1/2: identify tracker;\n"
                      "A: joint/TCP axes; ESC: quit. Robot dimensions are fixed.\n"
                      "Mouse drag: orbit; scroll: zoom; HOME: reset camera.");
            std::exit(0);
        }
        if (key=="--demo") { o.demo=true; continue; }
        if (key=="--zero-pose") { o.zero_pose=true; continue; }
        if (i+1>=argc) throw std::runtime_error("Missing value for "+key);
        std::string value=argv[++i];
        if (key=="--screenshot") { o.screenshot=value; continue; }
        char* end=nullptr; double n=std::strtod(value.c_str(),&end);
        if (!end||end==value.c_str()||*end||!std::isfinite(n))
            throw std::runtime_error("Invalid number for "+key);
        if (key=="--left-id"||key=="--right-id") {
            if (n<0||n>=vive::GetNumTrackers()||std::floor(n)!=n)
                throw std::runtime_error("Tracker ID is outside available slots.");
            if (key=="--left-id") o.left=unsigned(n); else o.right=unsigned(n);
        } else if (key=="--height") {
            if (n<.8||n>2.5) throw std::runtime_error("Height must be 0.8 to 2.5 meters.");
            o.height=n;
        } else if (key=="--scale") {
            if (n<.1||n>3) throw std::runtime_error("Motion scale must be 0.1 to 3.");
            o.scale=n;
        } else if (key=="--seconds") {
            if (n<=0) throw std::runtime_error("Seconds must be positive.");
            o.seconds=n;
        } else throw std::runtime_error("Unknown option: "+key);
    }
    if (o.left==o.right) throw std::runtime_error("Left/right tracker IDs must differ.");
    if (o.demo&&o.zero_pose) throw std::runtime_error("Choose --demo or --zero-pose.");
    return o;
}
struct Input {
    bool calibrate=false,swap=false,left=false,right=false,echo_left=false,echo_right=false;
    bool axes=true,drag=false;
    double azimuth=.30,elevation=.15,distance=3.2,mx=0,my=0;
};
void install_input(GLFWwindow* w,Input& input) {
    glfwSetWindowUserPointer(w,&input);
    glfwSetKeyCallback(w,[](GLFWwindow* win,int key,int,int action,int){
        if (action!=GLFW_PRESS) return;
        auto& in=*static_cast<Input*>(glfwGetWindowUserPointer(win));
        switch (key) {
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(win,GLFW_TRUE); break;
        case GLFW_KEY_C: in.calibrate=true; break;
        case GLFW_KEY_S: in.swap=true; break;
        case GLFW_KEY_L: in.left=true; break;
        case GLFW_KEY_R: in.right=true; break;
        case GLFW_KEY_1: in.echo_left=true; break;
        case GLFW_KEY_2: in.echo_right=true; break;
        case GLFW_KEY_A: in.axes=!in.axes; break;
        case GLFW_KEY_HOME: in.azimuth=.30; in.elevation=.15; in.distance=3.2; break;
        }
    });
    glfwSetMouseButtonCallback(w,[](GLFWwindow* win,int button,int action,int){
        if (button!=GLFW_MOUSE_BUTTON_LEFT) return;
        auto& in=*static_cast<Input*>(glfwGetWindowUserPointer(win));
        in.drag=action==GLFW_PRESS; glfwGetCursorPos(win,&in.mx,&in.my);
    });
    glfwSetCursorPosCallback(w,[](GLFWwindow* win,double x,double y){
        auto& in=*static_cast<Input*>(glfwGetWindowUserPointer(win));
        if (in.drag) {
            in.azimuth-=(x-in.mx)*.007;
            in.elevation=std::clamp(in.elevation+(y-in.my)*.007,-.25,1.25);
        }
        in.mx=x; in.my=y;
    });
    glfwSetScrollCallback(w,[](GLFWwindow* win,double,double y){
        auto& in=*static_cast<Input*>(glfwGetWindowUserPointer(win));
        in.distance=std::clamp(in.distance*std::exp(-y*.1),1.2,8.);
    });
}
void screenshot(const std::string& path,int w,int h) {
    std::vector<unsigned char> pixels(size_t(w)*h*3);
    glPixelStorei(GL_PACK_ALIGNMENT,1); glReadBuffer(GL_BACK);
    glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
    FILE* file=std::fopen(path.c_str(),"wb");
    if (!file) throw std::runtime_error("Cannot write screenshot: "+path);
    std::fprintf(file,"P6\n%d %d\n255\n",w,h);
    bool ok=true;
    for (int y=h-1;y>=0;--y)
        if (std::fwrite(pixels.data()+size_t(y)*w*3,1,size_t(w)*3,file)!=size_t(w)*3) ok=false;
    if (std::fclose(file)!=0||!ok) throw std::runtime_error("Screenshot write failed.");
}

void scene(int width,int height,const Body& body,const std::array<Pose,2>& wrists,
           const Input& in,const std::array<kida::Arm,2>& arms) {
    int view_width=std::max(1,width-340);
    glViewport(0,0,width,height); glClearColor(.76f,.79f,.81f,1);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,view_width,height);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    double top=.05*std::tan(38*pi/360),aspect=double(view_width)/std::max(1,height);
    glFrustum(-top*aspect,top*aspect,-top,top,.05,50);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    Vec3 target{0,0,body.height*.54};
    Vec3 eye=target+Vec3{in.distance*std::cos(in.elevation)*std::cos(in.azimuth),
        in.distance*std::cos(in.elevation)*std::sin(in.azimuth),in.distance*std::sin(in.elevation)};
    look_at(eye,target);
    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING); glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL); glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE); glShadeModel(GL_SMOOTH);
    float light[]={3,-4,8,1},ambient[]={.45f,.45f,.45f,1},diffuse[]={.75f,.75f,.75f,1};
    glLightfv(GL_LIGHT0,GL_POSITION,light); glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
    color({.67f,.71f,.73f}); glBegin(GL_QUADS); glNormal3d(0,0,1);
    glVertex3d(-4,-4,-.015); glVertex3d(4,-4,-.015);
    glVertex3d(4,4,-.015); glVertex3d(-4,4,-.015); glEnd();
    for (int i=-12;i<=12;++i) {
        line({i*.25,-3,-.01},{i*.25,3,-.01},{.59f,.64f,.67f},1);
        line({-3,i*.25,-.01},{3,i*.25,-.01},{.59f,.64f,.67f},1);
    }
    body_mesh(body);
    for (int i=0;i<2;++i) {
        draw_arm(arms[i],wrists[i],i==0?left_color:right_color,in.axes);
    }
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST);
    glViewport(0,0,width,height); glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glOrtho(0,width,height,0,-1,1); glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

void panel(int width,int height,const Options& o,const std::array<Device,2>& devices,
           const kida::Calibration& calibration,const std::array<kida::Arm,2>& arms,
           const std::array<Pose,2>& wrists,const std::string& message) {
    int x=width-340;
    rect(x,0,340,height,{.055f,.083f,.11f});
    text(x+24,25,"VIVE / KIDA",white,2.4);
    text(x+25,61,o.simulated()?"SIMULATED INPUT":"TWO WRIST TRACKERS",muted,1.35);
    bool tracking=o.simulated()||(usable(devices[0])&&usable(devices[1]));
    text(x+24,98,o.zero_pose?"ZERO POSE / ALL Q = 0":o.demo?"DEMO":(!calibration.ready?"READY POSE + PRESS C":
        (tracking?"LIVE / CALIBRATED":"TRACKING LOST / HOLDING")),tracking?left_color:amber,1.4);
    for (int i=0;i<2;++i) {
        int y=153+i*230; Color c=i==0?left_color:right_color;
        text(x+24,y,i==0?"LEFT WRIST":"RIGHT WRIST",c,1.8);
        char label[160]; std::snprintf(label,sizeof(label),"TRACKER ID %u",i==0?o.left:o.right);
        text(x+24,y+28,label,white,1.35);
        text(x+24,y+48,o.zero_pose?"STATIC KIDA MODEL":o.demo?"SYNTHETIC 6DOF":(devices[i].serial.empty()?"--":devices[i].serial),muted,1.25);
        text(x+24,y+72,o.simulated()?"MODEL PREVIEW":health(devices[i]),o.simulated()||usable(devices[i])?c:amber,1.25);
        if (o.simulated()||devices[i].has_pose) {
            auto pose=o.simulated()?wrists[i]:pose_of(devices[i]);
            auto p=pose.position;
            std::snprintf(label,sizeof(label),"POS %.2f / %.2f / %.2f m",p.x,p.y,p.z);
            text(x+24,y+93,label,muted,1.15);
            auto rpy=rpy_degrees(pose.rotation);
            std::snprintf(label,sizeof(label),"ORI R/P/Y %.1f / %.1f / %.1f deg",rpy.x,rpy.y,rpy.z);
            text(x+24,y+114,label,muted,1.1);
        }
        auto q=arms[i].q*(180/pi);
        std::snprintf(label,sizeof(label),"Q1-4 %5.1f %5.1f %5.1f %5.1f",q[0],q[1],q[2],q[3]);
        text(x+24,y+139,label,c,1.15);
        std::snprintf(label,sizeof(label),"Q5-7 %5.1f %5.1f %5.1f deg",q[4],q[5],q[6]);
        text(x+24,y+160,label,c,1.15);
        std::snprintf(label,sizeof(label),"IK ERROR %.1f mm / %.1f deg",arms[i].position_error*1000,arms[i].angle_error*180/pi);
        text(x+24,y+182,label,muted,1.1);
        if ((calibration.ready||o.demo)&&arms[i].limited())
            text(x+24,y+202,"IK LIMIT / TARGET NOT REACHED",amber,1.15);
    }
    int y=630;
    text(x+24,y,"CONTROLS",white,1.6);
    const char* controls[]={"C    CALIBRATE READY POSE","S    SWAP LEFT / RIGHT","L/R  SELECT TRACKER IDS",
        "1/2  FLASH LEFT / RIGHT","A    TOGGLE JOINT / TCP AXES","DRAG TO ORBIT / SCROLL TO ZOOM","ESC  QUIT"};
    for (auto control:controls) { y+=25; text(x+24,y,control,muted,1.2); }
    text(24,24,"KIDA / 7 + 7 DOF",{.12f,.20f,.25f},2.0);
    text(24,55,"YAML GEOMETRY / POSITION + ORIENTATION IK",{.25f,.33f,.38f},1.25);
    std::string note=message;
    if (note.empty()) note=o.zero_pose?"ALL JOINTS ZERO / HAND BACKS FACE OUTWARD / FINGERS DOWN":
        calibration.ready||o.demo?"MOVE YOUR WRISTS / KEEP TORSO FACING FORWARD":
        "ELBOWS BENT, THUMBS UP, PALMS FACING EACH OTHER. PRESS C.";
    rect(0,height-54,width-340,54,{.14f,.20f,.24f});
    text(20,height-34,note,white,1.25);
}
} // namespace

int main(int argc,char** argv) {
    try {
        Options o=options(argc,argv);
        glfwSetErrorCallback([](int,const char* description){ std::fprintf(stderr,"GLFW: %s\n",description); });
        if (!glfwInit()) return 1;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,2); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,1);
        glfwWindowHint(GLFW_SAMPLES,4);
        GLFWwindow* window=glfwCreateWindow(1280,900,"VIVE KIDA 7-DOF Wrist Teleop",nullptr,nullptr);
        if (!window) { glfwTerminate(); return 1; }
        glfwSetWindowSizeLimits(window,1060,850,GLFW_DONT_CARE,GLFW_DONT_CARE);
        glfwMakeContextCurrent(window); glfwSwapInterval(1);
        Input input; install_input(window,input);
        std::unique_ptr<Tracker> tracker;
        if (!o.simulated()) {
            tracker=std::make_unique<Tracker>();
            if (!tracker->start()) {
                std::fputs("Cannot start SDK server. Check dongle/USB access and stop any existing tracker server.\n",stderr);
                glfwDestroyWindow(window); glfwTerminate(); return 1;
            }
        }
        std::signal(SIGINT,interrupt_handler); std::signal(SIGTERM,interrupt_handler);
        Body body; body.height=o.height;
        kida::Calibration calibration;
        Vec3 root{0,0,body.shoulder_z()};
        std::array<kida::Arm,2> arms{{kida::Arm(0,root),kida::Arm(1,root)}};
        std::array<Pose,2> references{{arms[0].reference,arms[1].reference}};
        std::array<Pose,2> filtered=references;
        if (o.zero_pose) for (int i=0;i<2;++i) {
            arms[i].q.setZero(); arms[i].fk=arms[i].forward(arms[i].q);
            arms[i].position_error=arms[i].angle_error=0;
            filtered[i]=kida::pose(arms[i].fk.tcp);
        }
        std::array<Device,2> devices;
        std::string message;
        double last=glfwGetTime(),begin=last,message_until=0;
        unsigned long frames=0;
        std::puts(o.simulated()?"Model preview: no USB connection. Close window or press Esc to stop.":
            "Server started. Select IDs with L/R, bend elbows, thumbs up and palms facing each other, then press C.");
        std::puts("KIDA YAML geometry, 7 joints per arm; 6D numerical IK with joint-command bounds.\n"
                  "C captures the ready pose and maps it to KIDA home TCPs. Motion scale: 1 m -> scale m.");
        while (!glfwWindowShouldClose(window)&&!interrupted&&!(tracker&&tracker->stopped.load())) {
            glfwPollEvents();
            double now=glfwGetTime(),t=now-begin,dt=std::clamp(now-last,0.,.1); last=now;
            auto reset=[&] { calibration.ready=false; message="READY POSE + PRESS C TO CALIBRATE"; message_until=now+3; };
            if (input.swap) { std::swap(o.left,o.right); input.swap=false; reset(); }
            if (input.left||input.right) {
                auto& id=input.left?o.left:o.right;
                unsigned other=input.left?o.right:o.left;
                do { id=(id+1)%vive::GetNumTrackers(); } while (id==other);
                input.left=input.right=false; reset();
            }
            if (tracker) {
                auto all=tracker->snapshot(); devices={all[o.left],all[o.right]};
                if (input.echo_left||input.echo_right) {
                    std::string command="Echo "+std::to_string(input.echo_left?o.left:o.right);
                    tracker->SendCommand(command.c_str());
                }
            }
            input.echo_left=input.echo_right=false;
            if (input.calibrate) {
                input.calibrate=false;
                if (!o.simulated()) {
                    try {
                        if (!usable(devices[0])||!usable(devices[1]))
                            throw std::runtime_error("WAIT FOR BOTH WRISTS TO SHOW TRACKING.");
                        calibration.set(pose_of(devices[0]),pose_of(devices[1]),references,o.scale);
                        for (auto& a:arms) a.reset();
                        filtered=references;
                        message="CALIBRATED. MOVE YOUR WRISTS.";
                        std::printf("Calibrated IDs %u/%u: scale=%.3f\n",o.left,o.right,calibration.scale);
                    } catch (const std::exception& e) { message=e.what(); }
                    message_until=now+5;
                }
            }
            for (int i=0;i<2;++i) {
                Pose target=filtered[i];
                if (o.demo) {
                    auto q=arms[i].home;
                    for (int j=0;j<7;++j) q[j]+=(j<4?.20:.35)*std::sin(t*(.5+j*.07)+i*.6)*std::sin(std::min(1.,t)*pi/2);
                    q=q.cwiseMax(arms[i].low).cwiseMin(arms[i].high);
                    target=kida::pose(arms[i].forward(q).tcp);
                } else if (calibration.ready&&usable(devices[i])) {
                    target=calibration.map(pose_of(devices[i]),i);
                }
                double alpha=1-std::exp(-dt/.035);
                filtered[i].position=lerp(filtered[i].position,target.position,alpha);
                filtered[i].rotation=blend(filtered[i].rotation,target.rotation,alpha);
                if (!o.zero_pose) arms[i].solve(filtered[i]);
            }
            int width=0,height=0; glfwGetFramebufferSize(window,&width,&height);
            bool ending=o.seconds>0&&t>=o.seconds;
            if (width>0&&height>0) {
                scene(width,height,body,filtered,input,arms);
                panel(width,height,o,devices,calibration,arms,filtered,now<message_until?message:"");
                if (ending&&!o.screenshot.empty()) screenshot(o.screenshot,width,height);
                glfwSwapBuffers(window); ++frames;
            } else std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (ending) break;
        }
        tracker.reset();
        glfwDestroyWindow(window); glfwTerminate();
        std::printf("Closed normally; rendered %lu frames.\n",frames);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr,"Error: %s\n",e.what()); glfwTerminate(); return 1;
    }
}
