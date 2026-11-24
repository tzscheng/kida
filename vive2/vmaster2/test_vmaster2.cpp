// Tests execute the actual app's conversion/formatter without starting SDKs,
// ncurses, the logger or a robot. ZMQ checks use private in-process endpoints.
#define main vmaster2_application_main
#include "vmaster2.cpp"
#undef main
#include <cassert>
#include <sstream>

namespace {
void Same(const Mat3& a,const Mat3& b,double tolerance=1e-10) {
    for (int i=0;i<9;++i) assert(std::abs(a[i]-b[i])<tolerance);
}
vive::TrackerData Good(uint32_t id) {
    vive::TrackerData p{}; p.Id=id; p.State=vive::Ready; p.Flags=2;
    p.Rotation[3]=1; p.Location[0]=1; p.Location[1]=2; p.Location[2]=3;
    return p;
}
std::vector<std::string> Parts(const std::string& command) {
    std::vector<std::string> parts; std::istringstream input(command); std::string part;
    while (std::getline(input,part,',')) parts.push_back(part);
    return parts;
}
std::vector<double> Values(const std::string& part,const std::string& expected) {
    std::istringstream input(part); std::string word; input>>word; assert(word==expected);
    std::vector<double> out; double number;
    while (input>>number) out.push_back(number);
    return out;
}
void CheckClose(double actual,double expected) { assert(std::abs(actual-expected)<.00051); }

void TestInput() {
    ultimate::Trackers input;
    ultimate::Selection left{0,""},right{1,""};
    assert(!ultimate::usable(input.select(left)));
    input.Add(0,"WRIST-L"); input.Add(1,"WRIST-R"); input.Add(2,"SPARE");
    auto p=Good(0); input.Update(p); input.Update(Good(1)); input.Update(Good(2));
    auto l=input.select(left); auto r=input.select(right);
    assert(ultimate::usable(l)&&ultimate::usable(r));
    assert(left.serial=="WRIST-L"&&right.serial=="WRIST-R");
    assert(std::string(ultimate::health(l,l.received+std::chrono::milliseconds(301)))=="STALE DATA");
    for (uint32_t flags : {3u,2u|vive::TrackerFlag_RotationOnly,2u|vive::TrackerFlag_SetupRequired,0u}) {
        p.Flags=flags; input.Update(p); auto invalid=input.select(left);
        assert(!ultimate::usable(invalid)&&invalid.generation!=l.generation);
        p=Good(0); input.Update(p); auto recovered=input.select(left);
        assert(ultimate::usable(recovered)&&recovered.generation!=invalid.generation);
        l=recovered;
    }
    p.State=vive::Syncing; input.Update(p); assert(!ultimate::usable(input.select(left)));
    p=Good(0); p.Rotation[0]=std::numeric_limits<float>::quiet_NaN();
    input.Update(p); assert(!ultimate::usable(input.select(left)));
    p=Good(0); p.Rotation[3]=0; input.Update(p); assert(!ultimate::usable(input.select(left)));
    input.Remove(0); assert(!input.select(left).connected);
    input.Add(0,"DIFFERENT-TRACKER"); input.Update(Good(0));
    assert(!input.select(left).connected); // Never silently use the spare/new device.
    input.Add(3,"WRIST-L"); input.Update(Good(3));
    l=input.select(left); assert(l.id==3&&ultimate::usable(l));
    assert(l.generation!=r.generation);
    ultimate::Selection serial{0,"WRIST-R"}; assert(input.select(serial).id==1);
    input.ServerStop(); assert(input.stopped.load());
}

void TestTransformsAndClutch() {
    auto p=Good(0); p.Rotation[2]=float(std::sin(.37)*2); p.Rotation[3]=float(std::cos(.37)*2);
    auto transform=ultimate::pose_matrix(p); auto rotation=Mat3FromMat4(transform);
    assert(M(transform,0,3)==1&&M(transform,1,3)==2&&M(transform,2,3)==3);
    Same(Mat3Mul(rotation,Mat3Transpose(rotation)),Mat3Identity());
    Mat4 yaw=T_rot_z(M_PI/2); Mat4 rotated=MatMul(yaw,ultimate::pose_matrix(Good(0)));
    assert(std::abs(M(rotated,0,3)+2)<1e-12&&std::abs(M(rotated,1,3)-1)<1e-12);
    double raw[3]={5,6,7},offset[3]; Mat3 rotation_offset;
    const auto bias=Mat3FromMat4(T_rot_z(-.3));
    CaptureOrigin(raw,rotation,bias,offset,rotation_offset);
    for (int i=0;i<3;++i) assert(raw[i]+offset[i]==0);
    Same(Mat3Mul(rotation,rotation_offset),bias);
    auto increment=Mat3FromMat4(T_rot_z(.2));
    Same(Mat3Mul(Mat3Mul(increment,rotation),rotation_offset),Mat3Mul(increment,bias));
    // A new mounting angle on re-attach must preserve the last output.
    auto last=Mat3Mul(increment,bias);
    auto new_mount=Mat3FromMat4(T_rot_z(1.7));
    CaptureOrigin(raw,new_mount,last,offset,rotation_offset);
    Same(Mat3Mul(new_mount,rotation_offset),last);
}

void TestWire() {
    const double xyz[2][3]={{.31,.22,-.43},{.34,-.25,-.46}};
    const double rpy[2][3]={{.1,.2,.3},{-.4,-.5,-.6}};
    float q[40]; for (int i=0;i<40;++i) q[i]=i*.05f;
    int checked=0;
    for (int t : {0,1,2,5}) for (int g : {-1,0,1,2,3}) for (bool ee : {false,true}) for (bool hand : {false,true}) {
        int n=g<0?0:g==0?9:g==3?12:20;
        auto command=FormatTeleopCommand(t,g,n,xyz,rpy,q,.1,ee,hand);
        bool have_hand=t!=5&&g>=0;
        if (!ee&&!(hand&&have_hand)) { assert(command.empty()); ++checked; continue; }
        auto parts=Parts(command);
        assert(parts.size()==size_t(have_hand?(hand&&t==2?3:2):1));
        if (ee) {
            auto values=Values(parts[0],"task");
            assert(values.size()==size_t(t==2?12:t==5?7:6));
            for (int j=0;j<(t==2?2:1);++j) {
                int slot=t==1?1:j;
                for (int k=0;k<3;++k) { CheckClose(values[j*6+k],xyz[slot][k]); CheckClose(values[j*6+3+k],rpy[slot][k]); }
            }
            if (t==5) CheckClose(values[6],.1);
        } else assert(parts[0]=="none");
        if (have_hand) {
            if (!hand) assert(parts[1]==" none");
            else for (int j=0;j<(t==2?2:1);++j) {
                int slot=t==1?1:j; auto values=Values(parts[1+j],"joint");
                assert(values.size()==size_t(n));
                for (int k=0;k<n;++k) CheckClose(values[k],q[slot*20+k]);
            }
        }
        ++checked;
    }
    assert(checked==80);
}

void TestGloveLayout() {
    g_manus_have_left=g_manus_have_right=true;
    for (int i=0;i<20;++i) { g_manus_left_joints[i]=i*.03f; g_manus_right_joints[i]=-i*.02f; }
    for (int g=0;g<4;++g) {
        float expected[40]={},actual[40]={};
        if (g==0) { RetargetSide_h9(g_manus_left_joints,expected); RetargetSide_h9(g_manus_right_joints,expected+20); }
        if (g==1) { RetargetSide_dg5f('L',g_manus_left_joints,expected); RetargetSide_dg5f('R',g_manus_right_joints,expected+20); }
        if (g==2) { RetargetSide_dg5s('L',g_manus_left_joints,expected); RetargetSide_dg5s('R',g_manus_right_joints,expected+20); }
        if (g==3) { RetargetSide_h12(g_manus_left_joints,expected); RetargetSide_h12(g_manus_right_joints,expected+20); }
        RetargetAll(g,actual);
        for (int i=0;i<40;++i) assert(actual[i]==expected[i]);
    }
}

bool Parse(std::vector<std::string> args,Args& result) {
    std::vector<char*> pointers;
    for (auto& arg:args) pointers.push_back(arg.data());
    pointers.push_back(nullptr); optind=0;
    return ParseArgs(int(args.size()),pointers.data(),result);
}
void TestCli() {
    Args a; assert(Parse({"vmaster2","-t2","-g2","-b2","--left-id","2","--right-id","0","-n"},a));
    assert(a.t==2&&a.g==2&&a.n&&a.yaw_degrees==30&&a.left_id==2&&a.right_id==0);
    Args b; assert(Parse({"vmaster2","-t1","--right-serial","RIGHT","--yaw-deg","-90","--dry-run"},b));
    assert(b.right_serial=="RIGHT"&&b.yaw_degrees==-90&&b.dry_run);
    for (auto args : {std::vector<std::string>{"vmaster2","-t2","--right-id","0"},
        {"vmaster2","-t2","--left-id","0","--left-serial","LEFT"},
        {"vmaster2","-t2","--dry-run","-l"}, {"vmaster2","-t9"},
        {"vmaster2","-t2","--yaw-deg","nan"}, {"vmaster2","-t2","--seconds","0"}}) {
        Args bad; assert(!Parse(args,bad));
    }
}

void TestPrivateZmq() {
    g_slaveEndpoint="inproc://ultimate-vmaster2-test-slave";
    g_loggerEndpoint="inproc://ultimate-vmaster2-test-logger";
    assert(InitZmq());
    void* slave=zmq_socket(g_zmqCtx,ZMQ_PULL); void* logger=zmq_socket(g_zmqCtx,ZMQ_PULL);
    assert(slave&&logger);
    assert(zmq_bind(slave,g_slaveEndpoint.c_str())==0&&zmq_bind(logger,g_loggerEndpoint.c_str())==0);
    const int timeout=1000; zmq_setsockopt(slave,ZMQ_RCVTIMEO,&timeout,sizeof(timeout));
    zmq_setsockopt(logger,ZMQ_RCVTIMEO,&timeout,sizeof(timeout));
    for (const std::string command : {"init, home, home","home","rest, home, home"}) SendPush(g_slaveCtlSock,command.data(),command.size());
    char buffer[800];
    for (const std::string expected : {"init, home, home","home","rest, home, home"}) {
        int n=zmq_recv(slave,buffer,sizeof(buffer),0); assert(n>0&&std::string(buffer,size_t(n))==expected);
    }
    SendPush(g_loggerCtlSock,"log-on",6); SendPush(g_loggerCtlSock,"log-off",7);
    for (const std::string expected : {"log-on","log-off"}) {
        int n=zmq_recv(logger,buffer,sizeof(buffer),0); assert(n>0&&std::string(buffer,size_t(n))==expected);
    }
    g_dryRun=true; SendPush(g_slaveCtlSock,"not-sent",8); g_dryRun=false;
    const int short_timeout=10; zmq_setsockopt(slave,ZMQ_RCVTIMEO,&short_timeout,sizeof(short_timeout));
    assert(zmq_recv(slave,buffer,sizeof(buffer),0)<0&&zmq_errno()==EAGAIN);
    zmq_close(slave); zmq_close(logger); ShutdownZmq();
}
} // namespace

int main() {
    TestInput(); TestTransformsAndClutch(); TestWire(); TestGloveLayout(); TestCli(); TestPrivateZmq();
    std::puts("PASS: tracker health/serial pinning/relocalization, XYZ/quaternions, clutch continuity, 80 wire layouts, 4 glove maps, CLI and private inproc ZMQ. No device SDKs started or robot endpoints used.");
}
