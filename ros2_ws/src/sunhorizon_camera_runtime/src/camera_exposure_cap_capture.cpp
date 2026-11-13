// Exclusive, bounded, file-only A/B/A diagnostic. Only SDK property190 is written.
// The caller owns navigation-inactive/final-zero checks. No ROS or actuator API.
#include <libobsensor/ObSensor.hpp>
#include "sunhorizon_camera_runtime/exposure_cap_transaction.hpp"
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using sunhorizon_camera_runtime::ExposureCapDevice;
using sunhorizon_camera_runtime::ExposureCapRange;
using sunhorizon_camera_runtime::ExposureCapTransaction;
namespace {
constexpr const char * kSerial = "CP9W463000ZW";
constexpr const char * kJournal = "/data/navigation/camera-property-journals";
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) {interrupted = 1;}
void need(bool v, const char * text) {if (!v) throw std::runtime_error(text);}
std::string quoted(const std::string & s) {
  std::ostringstream o;o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') o << '\\' << c;
    else if (c < 32) {const char * h="0123456789abcdef";o << "\\u00" << h[c >> 4] << h[c & 15];}
    else o << c;
  }
  o << '"';return o.str();
}
std::ofstream file(const fs::path & p, bool binary = false) {
  std::ofstream out(p, binary ? std::ios::binary : std::ios::out);
  out.exceptions(std::ios::badbit | std::ios::failbit);return out;
}
struct Channel {
  const char * name;const char * frame_id;OBSensorType sensor;OBFrameType frame;OBFormat format;
  std::shared_ptr<ob::VideoStreamProfile> profile;
};
std::vector<uint8_t> structure(const std::shared_ptr<ob::Device> & d, OBPropertyID id, uint32_t size) {
  need(d->isPropertySupported(id, OB_PERMISSION_READ), "invariant structure unreadable");
  std::vector<uint8_t> b(size);auto actual=size;d->getStructuredData(id,b.data(),&actual);
  need(actual==size,"invariant structure size mismatch");return b;
}
struct Invariants {
  int target,laser,laser_control;bool depth_ae,ir_ae;OBRegionOfInterest roi{};OBHdrConfig hdr{};
};
Invariants snapshot(const std::shared_ptr<ob::Device> & d) {
  for (const auto id : {OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL,OB_PROP_IR_AUTO_EXPOSURE_BOOL,
    OB_PROP_IR_BRIGHTNESS_INT,OB_PROP_LASER_POWER_LEVEL_CONTROL_INT,OB_PROP_LASER_CONTROL_INT})
    need(d->isPropertySupported(id,OB_PERMISSION_READ),"invariant property unreadable");
  Invariants out{d->getIntProperty(OB_PROP_IR_BRIGHTNESS_INT),d->getIntProperty(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT),
    d->getIntProperty(OB_PROP_LASER_CONTROL_INT),d->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL),
    d->getBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL),{}, {}};
  const auto roi=structure(d,OB_STRUCT_DEPTH_AE_ROI,sizeof(out.roi));
  const auto hdr=structure(d,OB_STRUCT_DEPTH_HDR_CONFIG,sizeof(out.hdr));
  std::memcpy(&out.roi,roi.data(),sizeof(out.roi));std::memcpy(&out.hdr,hdr.data(),sizeof(out.hdr));return out;
}
void valid_controls(const Invariants & s) {
  need(s.depth_ae && s.ir_ae,"AE must remain on");need(!s.hdr.enable,"HDR must remain off");
}
std::vector<std::string> differences(const Invariants & a,const Invariants & b) {
  std::vector<std::string> out;
  if(a.target!=b.target)out.push_back("ir_brightness");
  if(a.laser!=b.laser)out.push_back("laser_power_level");
  if(a.laser_control!=b.laser_control)out.push_back("laser_control");
  if(a.depth_ae!=b.depth_ae)out.push_back("depth_auto_exposure");
  if(a.ir_ae!=b.ir_ae)out.push_back("ir_auto_exposure");
  if(a.roi.x0_left!=b.roi.x0_left || a.roi.y0_top!=b.roi.y0_top ||
    a.roi.x1_right!=b.roi.x1_right || a.roi.y1_bottom!=b.roi.y1_bottom)out.push_back("depth_ae_roi");
  // Compare all defined HDR fields, never ABI padding bytes.
  if(a.hdr.enable!=b.hdr.enable)out.push_back("hdr_enable");
  if(a.hdr.sequence_name!=b.hdr.sequence_name)out.push_back("hdr_sequence_name");
  if(a.hdr.exposure_1!=b.hdr.exposure_1)out.push_back("hdr_exposure_1");
  if(a.hdr.gain_1!=b.hdr.gain_1)out.push_back("hdr_gain_1");
  if(a.hdr.exposure_2!=b.hdr.exposure_2)out.push_back("hdr_exposure_2");
  if(a.hdr.gain_2!=b.hdr.gain_2)out.push_back("hdr_gain_2");
  return out;
}
void record_invariants(std::ofstream & out,const char * stage,const Invariants & s,
  const std::shared_ptr<ob::Device> & device,const Invariants * reference=nullptr) {
  out << "{\"stage\":" << quoted(stage) << ",\"steady_ns\":" <<
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count()
    << ",\"target\":" << s.target << ",\"laser_power_level\":" << s.laser
    << ",\"laser_control\":" << s.laser_control << ",\"depth_ae\":" << (s.depth_ae?"true":"false")
    << ",\"ir_ae\":" << (s.ir_ae?"true":"false") << ",\"roi_sdk\":[" << s.roi.x0_left << ',' << s.roi.y0_top << ',' << s.roi.x1_right << ',' << s.roi.y1_bottom
    << "],\"hdr\":{\"enable\":" << int(s.hdr.enable) << ",\"sequence_name\":" << int(s.hdr.sequence_name)
    << ",\"exposure_1\":" << s.hdr.exposure_1 << ",\"gain_1\":" << s.hdr.gain_1
    << ",\"exposure_2\":" << s.hdr.exposure_2 << ",\"gain_2\":" << s.hdr.gain_2 << '}';
  for(const auto & p: {std::make_pair("actual_cap",OB_PROP_IR_AE_MAX_EXPOSURE_INT),
    std::make_pair("depth_exposure_property",OB_PROP_DEPTH_EXPOSURE_INT),std::make_pair("ir_exposure_property",OB_PROP_IR_EXPOSURE_INT),
    std::make_pair("depth_gain_property",OB_PROP_DEPTH_GAIN_INT),std::make_pair("ir_gain_property",OB_PROP_IR_GAIN_INT)}) {
    try{const int value=device->getIntProperty(p.second);out << ',' << quoted(p.first) << ':' << value;}
    catch(const std::exception & e){out << ',' << quoted(std::string(p.first)+"_error") << ':' << quoted(e.what());}
  }
  out << ",\"changes_from_reference\":[";
  if(reference){const auto changes=differences(*reference,s);for(std::size_t i=0;i<changes.size();++i)out << (i?",":"") << quoted(changes[i]);}
  out << "]}\n";out.flush();
}
void unchanged(const Invariants & a,const std::shared_ptr<ob::Device> & d,
  std::ofstream & out,const char * stage) {
  const auto b=snapshot(d);record_invariants(out,stage,b,d,&a);valid_controls(b);
  need(differences(a,b).empty(),"non-cap running invariant changed; see invariants.ndjson; no compensating setter permitted");
}
void record_control(std::ofstream & out,const char * stage,const ExposureCapTransaction & transaction) {
  out << "{\"stage\":" << quoted(stage) << ",\"steady_ns\":" <<
    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  for(const auto & p:transaction.receipt())out << ',' << quoted(p.first) << ':' << quoted(p.second);
  out << "}\n";out.flush();
}
struct Running {
  ob::Pipeline & pipeline;bool started{false};
  void stop() {if(started){pipeline.stop();started=false;}}
  ~Running() {try {stop();}catch(...) {}}
};
int capture(const fs::path & output,int requested,bool restore_only) {
  const auto experiment_start=Clock::now();const auto deadline=experiment_start+std::chrono::seconds(15);
  ob::Context context;auto list=context.queryDeviceList();std::shared_ptr<ob::Device> device;unsigned matches=0;
  for(uint32_t i=0;i<list->getCount();++i)if(std::string(list->getSerialNumber(i))==kSerial){++matches;device=list->getDevice(i);}
  need(matches==1 && device->getDeviceInfo()->getSerialNumber()==std::string(kSerial),"exclusive belly serial mismatch");
  ExposureCapDevice access{
    [device]{return device->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT,OB_PERMISSION_READ);},
    [device]{return device->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT,OB_PERMISSION_WRITE);},
    [device]{return device->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL,OB_PERMISSION_READ);},
    [device]{return device->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT);},
    [device]{const auto r=device->getIntPropertyRange(OB_PROP_IR_AE_MAX_EXPOSURE_INT);return ExposureCapRange{r.min,r.max,r.step};},
    [device]{return device->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);},
    [device](int v){device->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT,v);}};
  // Constructor recovers PREPARED/APPLIED190 before any pipeline is created.
  need(fs::is_directory(kJournal),"fixed journal directory must already exist");
  ExposureCapTransaction transaction(kJournal,kSerial,std::move(access));
  auto control=file(output/"control.ndjson");record_control(control,"startup_recovered_before_pipeline",transaction);
  if(restore_only){
    transaction.begin(-1);record_control(control,"restore_only_complete",transaction);
    const int actual=device->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT);
    for(const auto & p:transaction.receipt())if(p.first=="exposure_cap_original" && std::stoi(p.second)>=0)
      need(actual==std::stoi(p.second),"restore-only original cap readback mismatch");
    need(device->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL),"restore-only AE no longer on");
    auto out=file(output/"complete.json");out << "{\"complete\":true,\"restore_only\":true,\"capture_started\":false,\"actual_cap\":" << actual << ",\"serial\":\"" << kSerial << "\"}\n";out.close();return 0;
  }
  auto invariant_log=file(output/"invariants.ndjson");
  const auto original=snapshot(device);record_invariants(invariant_log,"before_start",original,device);valid_controls(original);
  need(device->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT,OB_PERMISSION_READ) &&
    device->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT,OB_PERMISSION_WRITE),"cap must be explicitly read/write supported");
  const int original_cap=device->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT);
  const auto range=device->getIntPropertyRange(OB_PROP_IR_AE_MAX_EXPOSURE_INT);
  need(range.step>0 && requested>=range.min && requested<=range.max &&
    (int64_t(requested)-range.min)%range.step==0 && requested<original_cap,"requested cap must be valid and strictly lower");
  need(device->isGlobalTimestampSupported(),"global timestamps unsupported");
  // Same SDK host-clock mapping configuration as the authoritative runtime.
  // It does not set exposure/gain/laser/AE/ROI or any other property ID.
  device->enableGlobalTimestamp(true);
  ob::Pipeline pipeline(device);Running running{pipeline};
  std::array<Channel,3> channels{{
    {"depth","camera_belly_depth_optical_frame",OB_SENSOR_DEPTH,OB_FRAME_DEPTH,OB_FORMAT_Y16,{}},
    {"left","camera_belly_ir_left_optical_frame",OB_SENSOR_IR_LEFT,OB_FRAME_IR_LEFT,OB_FORMAT_Y8,{}},
    {"right","camera_belly_ir_right_optical_frame",OB_SENSOR_IR_RIGHT,OB_FRAME_IR_RIGHT,OB_FORMAT_Y8,{}}}};
  auto config=std::make_shared<ob::Config>();auto calibration=file(output/"calibration.ndjson");
  calibration << std::setprecision(17);
  for(auto & ch:channels){
    ch.profile=pipeline.getStreamProfileList(ch.sensor)->getVideoStreamProfile(640,480,ch.format,30);
    config->enableStream(ch.profile);const auto k=ch.profile->getIntrinsic();const auto d=ch.profile->getDistortion();
    need(k.width==640 && k.height==480,"unexpected raster");
    need(d.k1==0 && d.k2==0 && d.k3==0 && d.k4==0 && d.k5==0 && d.k6==0 && d.p1==0 && d.p2==0,"non-rectified profile");
    calibration << "{\"stream\":" << quoted(ch.name) << ",\"format\":" << ch.format
      << ",\"width\":640,\"height\":480,\"fps\":30,\"K\":[" << k.fx << ",0," << k.cx << ",0," << k.fy << ',' << k.cy << ",0,0,1]}\n";
  }
  const auto k=channels[0].profile->getIntrinsic();
  for(const auto & ch:channels){
    const auto ck=ch.profile->getIntrinsic();const auto e=channels[0].profile->getExtrinsicTo(ch.profile);
    need(std::abs(ck.fx-k.fx)<1.e-5 && std::abs(ck.fy-k.fy)<1.e-5 && std::abs(ck.cx-k.cx)<1.e-5 && std::abs(ck.cy-k.cy)<1.e-5,"IR/depth K differs");
    for(int i=0;i<9;++i)need(std::abs(e.rot[i]-(i%4==0?1.:0.))<1.e-6,"IR/depth rotation differs");
    need(std::abs(e.trans[1])<1.e-5 && std::abs(e.trans[2])<1.e-5,"non-horizontal stereo");
    need(ch.frame==OB_FRAME_IR_RIGHT?e.trans[0]<0.:std::abs(e.trans[0])<1.e-5,"unexpected native stereo translation");
    calibration << "{\"from\":\"depth\",\"to\":" << quoted(ch.name) << ",\"rotation_sdk\":[";
    for(int i=0;i<9;++i)calibration << (i?",":"") << e.rot[i];
    calibration << "],\"translation_mm\":[" << e.trans[0] << ',' << e.trans[1] << ',' << e.trans[2] << "]}\n";
  }
  calibration.close();
  auto manifest=file(output/"experiment.json");const auto info=device->getDeviceInfo();
  const auto & roi=original.roi;
  manifest << "{\"schema\":\"belly_ir_ae_cap_ABA_v2\",\"serial\":" << quoted(kSerial)
    << ",\"device\":" << quoted(info->getName()) << ",\"firmware\":" << quoted(info->getFirmwareVersion())
    << ",\"sdk\":\"" << ob::Version::getMajor() << '.' << ob::Version::getMinor() << '.' << ob::Version::getPatch()
    << "\",\"only_written_property\":190,\"original_cap\":" << original_cap << ",\"requested_cap\":" << requested
    << ",\"range\":[" << range.min << ',' << range.max << ',' << range.step << "],\"target\":" << original.target
    << ",\"laser_power\":" << original.laser << ",\"laser_control\":" << original.laser_control
    << ",\"roi_sdk\":[" << roi.x0_left << ',' << roi.y0_top << ',' << roi.x1_right << ',' << roi.y1_bottom
    << "],\"ae_enabled\":true,\"hdr_enabled\":false"
    << ",\"sdk_global_clock_mapping_enabled\":true,\"saved_frame_global_requirement\":\"SDK global_us>0; derived_header_ns=global_us*1000\""
    << ",\"exposure_units\":\"SDK units; metadata depth/IR usually1us, verified by actual frame values; no conversion\","
    << "\"header_semantics\":\"SDK global_us*1000 derived header, not a ROS published header\","
    << "\"quality_semantics\":\"no navigation classification or clearing; missing or saturated is not Free\"}\n";manifest.close();
  config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
  pipeline.enableFrameSync();auto frames=file(output/"frames.ndjson");
  unsigned saved[3]{},mismatched[3]{};uint64_t previous=0;
  try{
    need(Clock::now()+std::chrono::seconds(9)<deadline,"setup left less than9s before15s deadline");
    pipeline.start(config);running.started=true;
    const auto running_baseline=snapshot(device);
    record_invariants(invariant_log,"post_start_running_baseline",running_baseline,device,&original);valid_controls(running_baseline);
    need(Clock::now()+std::chrono::seconds(9)<deadline,"pipeline start left less than9s before15s deadline");
    for(unsigned phase=0;phase<3;++phase){
      need(!interrupted,"interrupted before phase");
      const char * label=phase==0?"A_baseline":phase==1?"B_cap":"A_restored";
      if(phase==1){transaction.begin(requested);record_control(control,"B_applied",transaction);}
      if(phase==2){transaction.restore();record_control(control,"A_restored_before_settle",transaction);}
      unchanged(running_baseline,device,invariant_log,(std::string(label)+"_begin").c_str());need(device->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT)==(phase==1?requested:original_cap),"phase cap readback mismatch");
      const auto start=Clock::now();
      record_control(control,label,transaction);
      fs::create_directory(output/label);
      while(Clock::now()-start<std::chrono::seconds(3)){
        need(!interrupted,"capture interrupted");need(Clock::now()<deadline,"15s acquisition deadline exceeded");
        const auto set=pipeline.waitForFrameset(100);if(!set)continue;
        if(Clock::now()-start<std::chrono::seconds(2))continue;
        std::array<std::shared_ptr<ob::Frame>,3> raw;
        for(unsigned i=0;i<3;++i)raw[i]=set->getFrame(channels[i].frame);
        need(raw[0] && raw[1] && raw[2],"incomplete frameset");
        const auto epoch=raw[0]->getTimeStampUs();
        if(epoch!=raw[1]->getTimeStampUs() || epoch!=raw[2]->getTimeStampUs()){++mismatched[phase];continue;}
        need(epoch>previous,"nonmonotonic exact device epoch");previous=epoch;
        for(unsigned i=0;i<3;++i){
          const auto & f=raw[i];const auto & ch=channels[i];const auto v=f->as<ob::VideoFrame>();
          const unsigned bytes=i==0?2:1;
          need(v->getWidth()==640 && v->getHeight()==480 && f->getFormat()==ch.format && f->getDataSize()==640*480*bytes,"unexpected raw layout");
          need(f->getGlobalTimeStampUs()>0,"global timestamp unavailable");
          need(f->hasMetadata(OB_FRAME_METADATA_TYPE_EXPOSURE) && f->hasMetadata(OB_FRAME_METADATA_TYPE_GAIN),"actual exposure/gain metadata unavailable");
          need(f->getMetadataValue(OB_FRAME_METADATA_TYPE_EXPOSURE)>0 &&
            (phase!=1 || f->getMetadataValue(OB_FRAME_METADATA_TYPE_EXPOSURE)<=requested),"actual B exposure exceeds requested cap or unit mismatch");
          const auto name=std::string(label)+"/"+std::to_string(epoch)+"-"+ch.name+".pgm";auto image=file(output/name,true);
          image << "P5\n640 480\n" << (bytes==2?65535:255) << "\n";
          if(bytes==2){
            const auto * p=f->getData();std::vector<uint8_t> b(640*480*2);
            for(unsigned j=0;j<640*480;++j){uint16_t value;std::memcpy(&value,p+j*2,sizeof(value));b[j*2]=uint8_t(value>>8);b[j*2+1]=uint8_t(value&255);}image.write(reinterpret_cast<const char *>(b.data()),b.size());
          }else image.write(reinterpret_cast<const char *>(f->getData()),f->getDataSize());
          image.close();
          frames << "{\"phase\":" << quoted(label) << ",\"set\":" << saved[phase] << ",\"stream\":" << quoted(ch.name)
            << ",\"file\":" << quoted(name) << ",\"device_us\":" << epoch << ",\"global_us\":" << f->getGlobalTimeStampUs()
            << ",\"derived_header_ns\":" << f->getGlobalTimeStampUs()*uint64_t(1000) << ",\"frame_id\":" << quoted(ch.frame_id)
            << ",\"system_us\":" << f->getSystemTimeStampUs() << ",\"frame_index\":" << f->getIndex()
            << ",\"exposure\":" << f->getMetadataValue(OB_FRAME_METADATA_TYPE_EXPOSURE)
            << ",\"gain\":" << f->getMetadataValue(OB_FRAME_METADATA_TYPE_GAIN);
          if(i==0){const auto d=f->as<ob::DepthFrame>();need(d->getPixelType()==OB_PIXEL_DEPTH,"depth frame is not DEPTH pixel type");frames << ",\"native_scale_mm\":" << d->getValueScale() << ",\"pixel_type\":" << int(d->getPixelType());}
          frames << "}\n";
        }
        ++saved[phase];
      }
      frames.flush();need(saved[phase]>=10,"phase lacks sufficient exact frames");unchanged(running_baseline,device,invariant_log,(std::string(label)+"_end").c_str());
    }
    unchanged(running_baseline,device,invariant_log,"before_stop");
    running.stop();transaction.restore();record_control(control,"stopped_restored_verified",transaction);
    const auto stopped=snapshot(device);record_invariants(invariant_log,"post_stop_lifecycle_observation",stopped,device,&running_baseline);valid_controls(stopped);
    need(stopped.target==running_baseline.target && stopped.laser==running_baseline.laser &&
      stopped.laser_control==running_baseline.laser_control,"post-stop target/laser invariant changed");
    need(device->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT)==original_cap,"post-stop original cap readback mismatch");
    const double elapsed=std::chrono::duration<double>(Clock::now()-experiment_start).count();need(elapsed<=15.,"capture exceeded15s including stop");
    auto complete=file(output/"complete.json");complete << "{\"complete\":true,\"serial\":" << quoted(kSerial)
      << ",\"property\":190,\"restored_readback\":" << device->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT)
      << ",\"saved_sets\":[" << saved[0] << ',' << saved[1] << ',' << saved[2] << "],\"nonexact_sets_rejected\":["
      << mismatched[0] << ',' << mismatched[1] << ',' << mismatched[2] << "],\"elapsed_s\":" << elapsed
      << ",\"capture_stopped\":true,\"all_saved_global_timestamps_valid\":true,\"other_controls_unchanged_during_ABA\":true,\"post_stop_lifecycle_recorded\":true}\n";complete.close();return 0;
  }catch(...){
    try{const auto failure=snapshot(device);record_invariants(invariant_log,"failure_before_stop",failure,device);}catch(...){}
    try{running.stop();}catch(...){}
    try{const auto stopped=snapshot(device);record_invariants(invariant_log,"failure_after_stop",stopped,device);}catch(...){}
    const bool restored=transaction.restore_noexcept();record_control(control,restored?"failure_restored":"failure_restore_pending",transaction);
    throw;
  }
}
} // namespace
int main(int argc,char ** argv){
  const bool restore_only=argc==6 && std::string(argv[5])=="--restore-only";
  if((argc!=7 && !restore_only) || std::string(argv[1])!="--serial" || std::string(argv[2])!=kSerial ||
    std::string(argv[3])!="--output" || (!restore_only && std::string(argv[5])!="--max-exposure")){
    std::cerr << "usage: camera_exposure_cap_capture --serial CP9W463000ZW --output NEW_ABSOLUTE_DIR (--max-exposure 500 | --restore-only)\n";return 2;}
  fs::path output=argv[4];bool output_created=false;
  try{
    int requested=-1;if(!restore_only){std::size_t end=0;requested=std::stoi(argv[6],&end);need(end==std::string(argv[6]).size() && requested>=0,"invalid requested exposure cap");}
    need(output.is_absolute() && fs::create_directory(output),"output must be a new absolute directory");
    output_created=true;
    std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
    return capture(output,requested,restore_only);
  }catch(const std::exception & e){
    std::cerr << "Exposure cap capture failed: " << e.what() << '\n';
    try{if(output_created){auto out=file(output/"failure.json");out << "{\"complete\":false,\"error\":" << quoted(e.what()) << ",\"resume_navigation\":false,\"required\":\"verify190 journal restored; use fresh output --restore-only before navigation\"}\n";out.close();}}catch(...){}
  }
  return 1;
}
