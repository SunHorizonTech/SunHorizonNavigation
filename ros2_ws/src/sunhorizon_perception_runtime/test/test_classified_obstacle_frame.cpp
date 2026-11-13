#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace g=sunhorizon_perception_runtime::ground_relative;
namespace {
std::size_t checks=0;
void check(bool value) {++checks;if(!value)throw std::runtime_error("classified obstacle check failed: "+std::to_string(checks));}
template<class F> void rejects(F&& f) {bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected);}
g::SourceFrame source() {
  g::SourceFrame result;
  result.identity={"belly","camera-epoch","measured-calibration","camera_optical","base_footprint",
    42,1000000000,1001000000,5000000000,300000000,900000000,4999000000,5000000000,-1};
  result.to_common.source_frame=result.identity.source_frame;
  result.to_common.common_frame=result.identity.common_frame;
  result.to_common.source_ns=result.to_common.common_ns=result.identity.source_ns;
  result.to_common.qualified=result.measurement_qualified=true;
  for(uint32_t y=0;y<5;++y)for(uint32_t x=0;x<5;++x)
    result.points.push_back({{(x+.5)*.1,(y+.5)*.1,-.9},g::ReturnKind::Measured});
  return result;
}
g::ClassifierConfig config() {
  g::ClassifierConfig c;
  c.ground_band_min_m=-.01;c.ground_band_max_m=.01;c.maximum_support_residual_m=.05;
  c.maximum_point_error_m=.1;c.maximum_transform_translation_error_m=.1;
  c.maximum_transform_angle_error_rad=.2;c.maximum_plane_offset_error_m=.1;
  c.maximum_plane_angle_error_rad=.2;c.maximum_normal_tilt_rad=.2;
  c.rotation_tolerance=c.normal_norm_tolerance=1.e-12;
  c.minimum_support_span_x_m=c.minimum_support_span_y_m=.1;
  c.minimum_support_double_area_m2=.005;
  c.minimum_samples_per_cell=1;c.minimum_training_cells=c.minimum_holdout_cells=3;
  c.continuous_support_radius_cells=1;c.maximum_points=1000;
  c.maximum_support_cells=100;c.maximum_support_checks=10000;return c;
}
g::GroundReference reference(const g::SourceFrame& frame) {
  g::GroundReference r;
  r.provenance_id="synthetic-independent-ground-roi";r.calibration_id="synthetic-ground-calibration";
  r.common_frame=frame.identity.common_frame;r.expected_source_id=frame.identity.source_id;
  r.expected_source_epoch=frame.identity.epoch;r.expected_source_calibration_id=frame.identity.calibration_id;
  r.valid_for_source_ns=frame.identity.source_ns;r.valid_for_common_ns=frame.to_common.common_ns;
  r.source_ns=frame.identity.source_ns;r.first_receipt_ros_ns=frame.identity.first_receipt_ros_ns;
  r.first_receipt_steady_ns=frame.identity.first_receipt_steady_ns;r.expiry_budget_ns=600000000;
  r.independently_qualified=r.source_time_transform_qualified=true;r.unit_normal={0.,0.,1.};
  r.anchor={0.,0.,-.9};r.support.resolution_m=.1;r.support.width=r.support.height=5;
  for(uint32_t y=0;y<5;++y)for(uint32_t x=0;x<5;++x)r.support.required_cells.push_back({x,y});
  return r;
}
void conservative(const g::ClassifiedObstacleFrame::ConstSharedPtr& frame,std::size_t measured) {
  check(frame->has_unknown());check(!frame->ground().safety_qualified);
  check(frame->clearing_indices().empty());check(frame->marking_indices().empty());
  check(frame->protected_hit_indices().size()==measured);
  for(const auto i:frame->protected_hit_indices())check(frame->classes()[i]==g::PointClass::Uncertain);
}
void run() {
  static_assert(std::is_same<g::ClassifiedObstacleFrame::ConstSharedPtr::element_type,const g::ClassifiedObstacleFrame>::value,
    "consumer ownership is immutable");
  auto blocked_source=source();blocked_source.transform_angle_error_bound_rad=.005;
  // Blocked measured endpoints cannot train/contradict support, become Ground,
  // or lose their full range-dependent protection error.
  blocked_source.points.push_back({{.05,.05,-.74},g::ReturnKind::Measured,true});
  blocked_source.points.push_back({{50.,50.,-.9},g::ReturnKind::Measured,true});
  const auto blocked=g::classify_obstacle_frame(blocked_source,config(),reference(blocked_source));
  check(blocked->ground().safety_qualified && blocked->ground().measured_points==27);
  check(blocked->clearing_indices().size()==25 && blocked->ground().candidate_ground_points==25);
  check(blocked->marking_indices().empty());
  check(blocked->protected_hit_indices()==std::vector<std::size_t>({25,26}) && blocked->has_unknown());
  check(blocked->classes()[25]==g::PointClass::Uncertain && blocked->classes()[26]==g::PointClass::Uncertain);
  check(blocked->ground().cells.front().samples==1);
  check(blocked->error_bounds_m().back()>=2.*std::hypot(50.,50.,.9)*std::sin(.005/2.));
  auto input=source();
  auto unavailable_transform=input;
  unavailable_transform.to_common.qualified=false;
  unavailable_transform.transform_diagnostic={5,"pcd_registration_original_tracking_evidence_expired"};
  const auto unavailable_frame=g::classify_obstacle_frame(unavailable_transform,config(),reference(unavailable_transform));
  conservative(unavailable_frame,25);
  check(unavailable_frame->ground().result==g::Qualification::InvalidTransform);
  check(unavailable_frame->ground().transform_diagnostic.provider_state==5);
  check(unavailable_frame->ground().transform_diagnostic.reason==unavailable_transform.transform_diagnostic.reason);
  check(unavailable_frame->source().transform_diagnostic.reason==unavailable_transform.transform_diagnostic.reason);
  auto no_reference=g::conservative_unqualified_frame(input);
  conservative(no_reference,25);check(no_reference->classification_complete());
  check(no_reference->ground().result==g::Qualification::AbsentReference);
  check(no_reference->source().identity.source_ns==input.identity.source_ns);
  check(no_reference->source().identity.first_receipt_ros_ns==input.identity.first_receipt_ros_ns);
  check(no_reference->source().identity.first_receipt_steady_ns==input.identity.first_receipt_steady_ns);
  check(no_reference->source().identity.sequence==42&&no_reference->source().identity.raw_source_ns==900000000);
  check(no_reference->source().identity.normalization_thread_cpu_ns==-1);
  check(std::all_of(no_reference->protected_hit_indices().begin(),no_reference->protected_hit_indices().end(),
    [&](auto i){return no_reference->source().points[i].position[2]<-.75;}));
  input.points[0].position[2]=100.;input.identity.source_ns=2;
  check(no_reference->source().points[0].position[2]==-.9);
  check(no_reference->source().identity.source_ns==1000000000);
  auto shared=g::classify_obstacle_frame(source(),g::ClassifierConfig{});
  check(shared->ground().result==g::Qualification::AbsentReference);
  check(shared->marking_indices()==no_reference->marking_indices());
  auto missing_calibration=source();missing_calibration.identity.calibration_id.clear();
  conservative(g::conservative_unqualified_frame(missing_calibration),25);

  input=source();input.points[1].kind=g::ReturnKind::Invalid;input.points[2].kind=g::ReturnKind::Self;
  input.points[3].kind=g::ReturnKind::Masked;
  input.points[4].position[0]=std::numeric_limits<double>::quiet_NaN();
  auto masked=g::conservative_unqualified_frame(input);conservative(masked,21);
  check(masked->classes()[1]==g::PointClass::Invalid&&masked->classes()[2]==g::PointClass::Self);
  check(masked->classes()[3]==g::PointClass::Masked&&masked->classes()[4]==g::PointClass::Invalid);
  input.points.clear();auto empty=g::conservative_unqualified_frame(input);
  check(!empty->classification_complete()&&empty->has_unknown()&&empty->clearing_indices().empty());
  for(const int fault:{0,1,2,3,4}) {
    auto bad=source();
    if(fault==0)bad.identity.sequence=0;
    if(fault==1)bad.identity.first_receipt_ros_ns=bad.identity.source_ns-1;
    if(fault==2)bad.identity.epoch.clear();
    if(fault==3)bad.identity.first_receipt_steady_ns=0;
    if(fault==4)bad.identity.expiry_budget_ns=0;
    rejects([&]{g::conservative_unqualified_frame(std::move(bad));});
  }
  input=source();input.points.resize(g::kMaximumClassifiedPoints+1);
  rejects([&]{g::conservative_unqualified_frame(std::move(input));});

  input=source();const auto c=config();auto ref=reference(input);
  auto ground=g::classify_obstacle_frame(input,c,ref);
  check(ground->ground().result==g::Qualification::Qualified&&ground->ground().safety_qualified);
  check(ground->ground().reference_geometry_valid&&ground->ground().support_valid);
  check(ground->classification_complete()&&!ground->has_unknown());
  check(ground->marking_indices().empty()&&ground->protected_hit_indices().empty());
  check(ground->clearing_indices().size()==25);
  // Boundary neighborhoods are clipped to the explicit domain. This accepts
  // actual measured edge returns, never an extrapolated endpoint outside it.
  check(ground->classes()[0]==g::PointClass::Ground&&ground->classes()[24]==g::PointClass::Ground);
  input.points.push_back({{.5,.25,-.9},g::ReturnKind::Measured});
  auto outside=g::classify_obstacle_frame(input,c,ref);
  check(outside->ground().safety_qualified&&outside->classes().back()==g::PointClass::Uncertain);
  check(outside->has_unknown()&&outside->marking_indices().empty());
  check(outside->protected_hit_indices()==std::vector<std::size_t>{25});
  input=source();input.points[12].position[2]=-.88;
  auto obstacle=g::classify_obstacle_frame(input,c,ref);
  check(obstacle->ground().safety_qualified&&obstacle->classes()[12]==g::PointClass::Obstacle);
  check(obstacle->marking_indices()==std::vector<std::size_t>{12});
  check(obstacle->protected_hit_indices()==obstacle->marking_indices());
  check(std::find(obstacle->clearing_indices().begin(),obstacle->clearing_indices().end(),12)==obstacle->clearing_indices().end());
  input=source();input.points[12].position[2]=-.91; // Ground band boundary is not rounded inward.
  auto uncertain=g::classify_obstacle_frame(input,c,ref);
  check(uncertain->classes()[12]==g::PointClass::Uncertain&&uncertain->has_unknown());

  input=source();input.point_error_bound_m=.004;input.points[12].position[2]=-.892;
  auto interval=g::classify_obstacle_frame(input,c,ref);
  check(interval->ground().safety_qualified&&interval->classes()[12]==g::PointClass::Uncertain);
  check(interval->marking_indices().empty());
  check(interval->protected_hit_indices()==std::vector<std::size_t>{12});
  // Rotation error uses 2*r*sin(theta/2), including the anchor lever arm.
  input=source();input.point_error_bound_m=.001;input.transform_translation_error_bound_m=.002;
  input.transform_angle_error_bound_rad=.01;ref=reference(input);
  ref.plane_offset_error_bound_m=.003;ref.plane_angle_error_bound_rad=.02;
  auto broad=c;broad.ground_band_min_m=-1.;broad.ground_band_max_m=1.;broad.maximum_support_residual_m=1.;
  auto bounded=g::classify_obstacle_frame(input,broad,ref);
  const auto& p=input.points[12].position;
  const auto expected=.001+.002+2.*std::hypot(p[0],p[1],p[2])*std::sin(.01/2.)+
    .003+2.*std::hypot(p[0],p[1],p[2]+.9)*std::sin(.02/2.);
  check(bounded->ground().safety_qualified&&bounded->error_bounds_m()[12]>=expected);
  check(bounded->error_bounds_m()[12]-expected<1.e-14);

  for(const int fault:{0,1,2,3,4,5,6,7,8,9,10,11}) {
    input=source();ref=reference(input);auto options=c;
    if(fault==0)ref.independently_qualified=false;
    if(fault==1)ref.source_time_transform_qualified=false;
    if(fault==2)ref.expected_source_epoch="old-epoch";
    if(fault==3)ref.valid_for_source_ns--;
    if(fault==4)ref.source_ns-=ref.expiry_budget_ns+1;
    if(fault==5)ref.unit_normal={0.,0.,2.};
    if(fault==6)input.to_common.rotation[0]=2.;
    if(fault==7)input.to_common.source_ns--;
    if(fault==8)input.point_error_bound_m=.2;
    if(fault==9)options.maximum_support_checks=1;
    if(fault==10)options.maximum_points=24;
    if(fault==11)input.measurement_qualified=false;
    auto failed=g::classify_obstacle_frame(input,options,ref);conservative(failed,25);
  }
  // A reference alive at ingress may expire while this same source frame is
  // still fresh. All original reference clocks must cover the whole source TTL.
  for(const bool observe:{false,true}) {
    input=source();ref=reference(input);
    ref.independently_qualified=!observe;
    ref.expiry_budget_ns=input.identity.expiry_budget_ns+
      (input.identity.first_receipt_ros_ns-ref.source_ns);
    auto exact=g::classify_obstacle_frame(input,c,ref,observe);
    check(exact->ground().result==(observe?g::Qualification::ObserveOnly:g::Qualification::Qualified));
    --ref.expiry_budget_ns;
    auto short_ros=g::classify_obstacle_frame(input,c,ref,observe);conservative(short_ros,25);
    check(short_ros->ground().result==g::Qualification::ReferenceExpired);
    ref=reference(input);ref.independently_qualified=!observe;
    ref.first_receipt_steady_ns-=100000000;
    ref.expiry_budget_ns=input.identity.expiry_budget_ns+100000000;
    auto exact_steady=g::classify_obstacle_frame(input,c,ref,observe);
    check(exact_steady->ground().result==(observe?g::Qualification::ObserveOnly:g::Qualification::Qualified));
    --ref.expiry_budget_ns;
    auto short_steady=g::classify_obstacle_frame(input,c,ref,observe);conservative(short_steady,25);
    check(short_steady->ground().result==g::Qualification::ReferenceExpired);
    // A one-nanosecond reference remainder at ingress was accepted by the old gate.
    ref=reference(input);ref.independently_qualified=!observe;
    ref.source_ns=input.identity.first_receipt_ros_ns-100000000;
    ref.expiry_budget_ns=100000001;
    auto near_expiry=g::classify_obstacle_frame(input,c,ref,observe);conservative(near_expiry,25);
    check(near_expiry->ground().result==g::Qualification::ReferenceExpired);
    // Source TTL itself must not exceed the reference budget, even with zero age.
    ref=reference(input);ref.independently_qualified=!observe;
    ref.source_ns=ref.first_receipt_ros_ns=input.identity.first_receipt_ros_ns;
    ref.expiry_budget_ns=input.identity.expiry_budget_ns-1;
    auto short_budget=g::classify_obstacle_frame(input,c,ref,observe);conservative(short_budget,25);
    check(short_budget->ground().result==g::Qualification::ReferenceExpired);
  }
  // Positive clocks close to INT64_MAX must be handled without deadline addition.
  input=source();const auto highest=std::numeric_limits<int64_t>::max();
  input.identity.source_ns=highest-11;input.identity.first_receipt_ros_ns=highest-10;
  input.identity.first_receipt_steady_ns=highest-10;
  input.to_common.source_ns=input.to_common.common_ns=input.identity.source_ns;
  ref=reference(input);ref.expiry_budget_ns=301000000;
  check(g::classify_obstacle_frame(input,c,ref)->ground().safety_qualified);
  ref.expiry_budget_ns=highest;input.identity.expiry_budget_ns=highest;
  auto overflow_counterexample=g::classify_obstacle_frame(input,c,ref);conservative(overflow_counterexample,25);
  check(overflow_counterexample->ground().result==g::Qualification::ReferenceExpired);

  input=source();ref=reference(input);ref.independently_qualified=false;
  auto geometry_only=g::classify_obstacle_frame(input,c,ref);
  check(geometry_only->ground().reference_geometry_valid&&geometry_only->ground().support_valid);
  check(!geometry_only->ground().safety_qualified);
  input.points[12].position[2]=-.5; // A box cannot pull the reference plane upwards.
  auto conflict=g::classify_obstacle_frame(input,c,reference(input));
  check(conflict->ground().safety_qualified&&conflict->has_unknown());
  check(conflict->ground().cells[12].failure==g::CellSupport::Failure::Contradictory);
  check(!conflict->ground().cells[12].passed&&conflict->classes()[12]==g::PointClass::Uncertain);
  check(conflict->clearing_indices().size()==16&&conflict->marking_indices().empty());
  check(conflict->protected_hit_indices().size()==9);
  check(conflict->reference()->anchor[2]==-.9&&conflict->reference()->unit_normal[2]==1.);
  input=source();input.points.erase(input.points.begin()+12);
  auto gap=g::classify_obstacle_frame(input,c,reference(input));
  check(gap->ground().safety_qualified&&gap->has_unknown());
  check(gap->ground().cells[12].failure==g::CellSupport::Failure::Missing);
  check(!gap->ground().cells[12].passed&&gap->clearing_indices().size()==16);
  check(gap->marking_indices().empty()&&gap->protected_hit_indices().size()==8);
  input=source();for(auto& point:input.points)point.position[2]=-.5;
  auto no_support=g::classify_obstacle_frame(input,c,reference(input));conservative(no_support,25);
  check(no_support->ground().result==g::Qualification::InsufficientSupportGeometry);
  input=source();ref=reference(input);ref.support.required_cells.erase(ref.support.required_cells.begin()+12);
  auto local_gap=g::classify_obstacle_frame(input,c,ref);
  check(local_gap->ground().safety_qualified&&local_gap->classes()[12]==g::PointClass::Uncertain);
  check(local_gap->clearing_indices().size()<25&&local_gap->has_unknown());
  ref=reference(input);ref.support.required_cells={{0,0},{1,0},{2,0},{3,0},{4,0}};
  auto line=g::classify_obstacle_frame(input,c,ref);conservative(line,25);
  check(line->ground().result==g::Qualification::InsufficientSupportGeometry);
  // A physical line can traverse a staircase of non-collinear GRID CENTERS.
  // Qualification must use measured support positions, not fabricated centers.
  input=source();input.points.clear();
  for(const auto x:{.02,.09,.15,.25,.32,.37,.45})
    input.points.push_back({{x,.37*x+.073,-.9},g::ReturnKind::Measured});
  ref=reference(input);ref.support.required_cells={{0,0},{0,1},{1,1},{2,1},{3,1},{3,2},{4,2}};
  auto physical_line=g::classify_obstacle_frame(input,c,ref);conservative(physical_line,7);
  check(physical_line->ground().result==g::Qualification::InsufficientSupportGeometry);
  input=source();
  ref=reference(input);ref.support.required_cells.push_back({0,0});
  conservative(g::classify_obstacle_frame(input,c,ref),25);
  rejects([&]{g::classify_obstacle_frame(source(),g::ClassifierConfig{},reference(source()));});
  auto invalid=c;invalid.ground_band_max_m=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{g::classify_obstacle_frame(source(),invalid,reference(source()));});
  invalid=c;invalid.maximum_transform_angle_error_rad=4.;
  rejects([&]{g::classify_obstacle_frame(source(),invalid,reference(source()));});

  input=source();for(auto& point:input.points)point.position[2]=-.9+.05*point.position[0];
  ref=reference(input);const auto length=std::hypot(.05,1.);ref.unit_normal={-.05/length,0.,1./length};
  auto slope=g::classify_obstacle_frame(input,c,ref);
  check(slope->ground().safety_qualified&&slope->clearing_indices().size()==25&&slope->marking_indices().empty());
  input=source();input.to_common.translation[2]=.2;ref=reference(input);ref.anchor[2]=-.7;
  auto translated=g::classify_obstacle_frame(input,c,ref);
  check(translated->ground().safety_qualified&&translated->clearing_indices().size()==25);
  // One real plane, two sensor acquisition poses within the same scan. The
  // original endpoints and source metadata stay intact; each nonzero offset
  // uses its exact-time transform rather than the scan header transform.
  input=source();auto later=input.to_common;later.source_ns=later.common_ns=input.identity.source_ns+50000000;
  later.translation[2]=.2;input.source_time_transforms.push_back(later);
  for(std::size_t i=1;i<input.points.size();i+=2) {
    input.points[i].position[2]-=.2;input.points[i].source_offset_ns=50000000;
  }
  const auto scan_points=input.points;
  auto motion=g::classify_obstacle_frame(input,c,reference(input));
  check(motion->ground().safety_qualified&&motion->clearing_indices().size()==25);
  for(std::size_t i=0;i<25;++i) {
    check(std::abs(motion->relative_heights_m()[i])<1.e-12);
    check(motion->source().points[i].position==scan_points[i].position);
    check(motion->source().points[i].source_offset_ns==scan_points[i].source_offset_ns);
  }
  auto missing=input;missing.source_time_transforms.clear();
  conservative(g::classify_obstacle_frame(missing,c,reference(missing)),25);
  missing=input;missing.source_time_transforms[0].common_ns+=1;
  conservative(g::classify_obstacle_frame(missing,c,reference(missing)),25);
  missing=input;missing.source_time_transforms.push_back(later);
  conservative(g::classify_obstacle_frame(missing,c,reference(missing)),25);
  missing=input;auto earlier=later;earlier.source_ns-=1;earlier.common_ns-=1;
  missing.source_time_transforms.push_back(earlier);
  conservative(g::classify_obstacle_frame(missing,c,reference(missing)),25);
  missing=input;missing.points[1].source_offset_ns=std::numeric_limits<int64_t>::max();
  conservative(g::classify_obstacle_frame(missing,c,reference(missing)),25);
  input=source();ref=reference(input);ref.unit_normal={1.,0.,0.};
  conservative(g::classify_obstacle_frame(input,c,ref),25);
}
void supported_segmentation() {
  // Synthetic detection thresholds, not robot calibration/error bounds.
  g::SegmentationConfig c;c.maximum_point_residual_m=.04;
  c.maximum_cell_median_residual_m=.03;c.maximum_cell_mad_m=.015;
  c.maximum_partition_median_difference_m=.02;c.maximum_normal_tilt_rad=.2;
  c.minimum_support_span_x_m=c.minimum_support_span_y_m=.1;c.minimum_support_double_area_m2=.005;
  c.minimum_samples_per_partition=1;c.minimum_supported_cells=6;c.continuous_support_radius_cells=1;
  c.maximum_points=4000;c.maximum_support_cells=100;c.maximum_support_checks=100000;
  auto input=source();input.points.clear();input.measurement_qualified=false;
  input.point_error_bound_m=input.transform_translation_error_bound_m=input.transform_angle_error_bound_rad=g::kUnspecified;
  for(unsigned y=0;y<5;++y)for(unsigned x=0;x<5;++x)for(unsigned sample=0;sample<16;++sample) {
    g::PointReturn p;p.position={(x+.5)*.1,(y+.5)*.1,-.9+(double(sample%3)-1.)*.001};
    p.kind=g::ReturnKind::Measured;p.sample_id=input.points.size();input.points.push_back(p);
  }
  auto ref=reference(input);ref.independently_qualified=ref.source_time_transform_qualified=false;
  ref.plane_offset_error_bound_m=ref.plane_angle_error_bound_rad=g::kUnspecified;
  const auto frame=g::classify_supported_ground_frame(input,c,ref);
  check(frame->ground().safety_qualified&&frame->ground().basis==g::GroundQualification::Basis::MeasuredSupport);
  check(frame->clearing_indices().size()==400&&frame->marking_indices().empty()&&!frame->has_unknown());
  check(!frame->source().measurement_qualified&&!frame->reference()->independently_qualified);
  check(std::isnan(frame->source().point_error_bound_m)&&std::isnan(frame->reference()->plane_offset_error_bound_m));
  for(const auto error:frame->error_bounds_m())check(std::isnan(error));
  for(const auto& cell:frame->ground().cells)check(cell.passed&&cell.training_samples&&cell.holdout_samples);
  auto shuffled=input;std::reverse(shuffled.points.begin(),shuffled.points.end());
  const auto stable=g::classify_supported_ground_frame(shuffled,c,ref);
  check(stable->clearing_indices().size()==400);
  for(std::size_t i=0;i<25;++i) {
    check(frame->ground().cells[i].training_samples==stable->ground().cells[i].training_samples);
    check(frame->ground().cells[i].holdout_samples==stable->ground().cells[i].holdout_samples);
  }
  // A visible 16cm box cannot move the fixed plane or become a ground inlier.
  auto box=input;for(std::size_t i=12*16;i<13*16;++i)box.points[i].position[2]+=.16;
  const auto boxed=g::classify_supported_ground_frame(box,c,ref);
  check(boxed->ground().safety_qualified&&boxed->has_unknown());
  check(boxed->reference()->anchor==ref.anchor);
  for(std::size_t i=12*16;i<13*16;++i) {
    check(boxed->classes()[i]!=g::PointClass::Ground);
    check(std::find(boxed->protected_hit_indices().begin(),boxed->protected_hit_indices().end(),i)!=boxed->protected_hit_indices().end());
  }
  auto all_box=input;for(auto& p:all_box.points)p.position[2]+=.16;
  conservative(g::classify_supported_ground_frame(all_box,c,ref),400);
  // A minority raised return cannot be swallowed by robust support statistics.
  auto leg=input;leg.points.push_back({{.25,.25,-.8},g::ReturnKind::Measured,false,0,10000});
  const auto raised=g::classify_supported_ground_frame(leg,c,ref);
  check(raised->ground().safety_qualified&&raised->classes().back()==g::PointClass::Obstacle);
  check(raised->protected_hit_indices()==std::vector<std::size_t>{400});
  // A dominant raised cluster in the same cell must stay marked without
  // turning its separately observed fixed-plane floor samples into obstacles.
  auto mixed=input;for(unsigned i=0;i<64;++i)
    mixed.points.push_back({{.25,.25,-.74},g::ReturnKind::Measured,false,0,20000+i});
  const auto mixture=g::classify_supported_ground_frame(mixed,c,ref);
  check(mixture->ground().safety_qualified&&mixture->clearing_indices().size()==400);
  check(mixture->ground().cells[12].samples==16&&mixture->marking_indices().size()==64);
  for(std::size_t i=12*16;i<13*16;++i)check(mixture->classes()[i]==g::PointClass::Ground);
  for(std::size_t i=400;i<464;++i)check(mixture->classes()[i]==g::PointClass::Obstacle);
  check(mixture->protected_hit_indices()==mixture->marking_indices());
  auto blocked=input;blocked.points.push_back({{.25,.25,-.9},g::ReturnKind::Measured,true,0,10001});
  blocked.points.push_back({{.25,.25,-.9},g::ReturnKind::Self});
  blocked.points.push_back({{.25,.25,-.9},g::ReturnKind::Masked});
  const auto masked=g::classify_supported_ground_frame(blocked,c,ref);
  check(masked->classes()[400]==g::PointClass::Uncertain&&masked->classes()[401]==g::PointClass::Self);
  check(masked->classes()[402]==g::PointClass::Masked&&masked->clearing_indices().size()==400&&masked->has_unknown());
  check(masked->ground().cells[12].samples==16&&masked->marking_indices().empty());
  check(masked->protected_hit_indices()==std::vector<std::size_t>{400});
  // Policy Self at a model edge keeps the mixed-depth original for auditing,
  // without importing it as a measured body hit or a ground/obstacle endpoint.
  auto edge_source=blocked;
  edge_source.points[401].ground_blocked=true;
  edge_source.points[401].position={50.,50.,50.};
  edge_source.points[401].sample_id=10002;
  const auto edge_qualified=g::classify_supported_ground_frame(edge_source,c,ref);
  const auto edge_unqualified=g::conservative_unqualified_frame(edge_source);
  for(const auto& edge_frame:{edge_qualified,edge_unqualified}) {
    check(edge_frame->classes()[401]==g::PointClass::Self);
    check(edge_frame->decisions()[401]==g::PointDecision::ModelProjectionEdge);
    check(edge_frame->source().self_support.empty());
    check(edge_frame->source().points[401].position==edge_source.points[401].position);
    for(const auto* indices:{&edge_frame->marking_indices(),&edge_frame->clearing_indices(),&edge_frame->protected_hit_indices()})
      check(std::find(indices->begin(),indices->end(),401)==indices->end());
  }
  check(edge_qualified->clearing_indices()==masked->clearing_indices());
  check(edge_qualified->ground().cells[12].samples==16);
  auto occluded_source=input;
  occluded_source.points.push_back({{.25,.25,-.9},g::ReturnKind::Invalid,true,0,10003,false,false,true});
  occluded_source.raster_geometry={640,480,1};
  occluded_source.body_occlusion={true,true,occluded_source.identity,640,480,.05,{10003}};
  const auto occluded_supported=g::classify_supported_ground_frame(occluded_source,c,ref);
  check(occluded_supported->ground().result==g::Qualification::Qualified);
  for(const auto& rejected:{occluded_supported,g::conservative_unqualified_frame(occluded_source)}) {
    check(rejected->classes().back()==g::PointClass::Invalid);
    check(rejected->decisions().back()==g::PointDecision::MeasurementBodyOccluded);
    check(rejected->source().self_support.empty());
    for(const auto* indices:{&rejected->marking_indices(),&rejected->clearing_indices(),&rejected->protected_hit_indices()})
      check(std::find(indices->begin(),indices->end(),400)==indices->end());
  }
  const auto reject_body_proof=[&](const g::SourceFrame& bad) {rejects([&]{g::conservative_unqualified_frame(bad);});};
  auto bad_body=occluded_source;bad_body.body_occlusion.complete=false;reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.body_occlusion.identity.epoch+="-wrong";reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.body_occlusion.identity.first_receipt_steady_ns+=1;reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.body_occlusion.identity.calibration_id+="-wrong";reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.body_occlusion.sample_ids={10004};reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.points.back().source_offset_ns=1;reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.points.back().position[0]=NAN;reject_body_proof(bad_body);
  bad_body=occluded_source;bad_body.points.back().measurement_unreliable=true;reject_body_proof(bad_body); // cannot rescue missing IR proof
  bad_body=occluded_source;bad_body.points.push_back(bad_body.points.back());
  bad_body.body_occlusion.sample_ids={10003,10004};reject_body_proof(bad_body); // duplicate point ID cannot consume another proof ID
  bad_body=occluded_source;bad_body.body_occlusion.sample_ids={10003,10003};reject_body_proof(bad_body);
  auto missing=input;for(std::size_t i=12*16;i<13*16;++i)missing.points[i].kind=g::ReturnKind::Invalid;
  const auto hole=g::classify_supported_ground_frame(missing,c,ref);
  check(hole->ground().safety_qualified&&hole->has_unknown());
  check(std::none_of(hole->ground().cells.begin(),hole->ground().cells.end(),
    [](const auto& cell){return cell.cell.x==2&&cell.cell.y==2;}));
  check(hole->classes()[11*16]==g::PointClass::Ground); // Real observations around a hole still segment.
  check(hole->classes()[12*16]==g::PointClass::Invalid&&hole->clearing_indices().size()==384);
  auto islands=input;islands.points.clear();
  for(std::size_t i=0;i<input.points.size();++i) {
    const auto cell=i/16;if(cell==0||cell==1||cell==5||cell==19||cell==23||cell==24)
      islands.points.push_back(input.points[i]);
  }
  conservative(g::classify_supported_ground_frame(islands,c,ref),96); // Two tiny components cannot pool geometry.
  auto separated=input;separated.points.clear();
  for(std::size_t i=0;i<input.points.size();++i)if((i/16)%5<2||(i/16)%5==4)
    separated.points.push_back(input.points[i]);
  const auto components=g::classify_supported_ground_frame(separated,c,ref);
  check(components->ground().safety_qualified&&components->clearing_indices().size()==160);
  check(components->marking_indices().empty()); // Isolated thin column remains Uncertain, never an obstacle.
  check(components->protected_hit_indices().size()==80);
  auto line=input;line.points.clear();auto line_ref=ref;line_ref.support.width=6;
  line_ref.support.required_cells.clear();
  for(uint32_t y=0;y<5;++y)for(uint32_t x=0;x<6;++x)line_ref.support.required_cells.push_back({x,y});
  for(unsigned x=0;x<6;++x)for(unsigned i=0;i<16;++i) {
    auto p=input.points[i];p.position={(x+.5)*.1,.05,-.9};p.sample_id=line.points.size();line.points.push_back(p);
  }
  conservative(g::classify_supported_ground_frame(line,c,line_ref),96); // Enough cells, zero area/span Y.
  auto outside=input;outside.points.push_back({{.51,.25,-.9},g::ReturnKind::Measured,false,0,10002});
  check(g::classify_supported_ground_frame(outside,c,ref)->classes().back()==g::PointClass::Uncertain);
  // The legacy reference rectangle cannot clip a real observed component.
  // Its origin/resolution still pin the same world lattice in both directions.
  for(const auto shift:{std::array<double,2>{5.,6.},std::array<double,2>{-.7,-.5}}) {
    auto moved=input;
    for(auto& point:moved.points){point.position[0]+=shift[0];point.position[1]+=shift[1];}
    const auto observed=g::classify_supported_ground_frame(moved,c,ref);
    check(observed->ground().safety_qualified&&observed->clearing_indices().size()==400);
    check(observed->reference()->anchor==ref.anchor&&observed->reference()->unit_normal==ref.unit_normal);
    check(std::abs(observed->reference()->support.origin_x_m-shift[0])<1.e-12);
    check(std::abs(observed->reference()->support.origin_y_m-shift[1])<1.e-12);
    check(observed->reference()->support.required_cells.size()==25);
    for(std::size_t i=0;i<25;++i) {
      check(observed->ground().cells[i].training_samples==frame->ground().cells[i].training_samples);
      check(observed->ground().cells[i].holdout_samples==frame->ground().cells[i].holdout_samples);
    }
  }
  // Exact/adjacent floating boundaries retain their original signed lattice
  // key. Grid rebasing must not run point_cell() a second time.
  auto boundaries=input;auto boundary_ref=ref;
  boundary_ref.support.origin_x_m=.3;boundary_ref.support.origin_y_m=-.1;
  for(std::size_t i=0;i<boundaries.points.size();++i) {
    const auto x=int((i/16)%5)-3,y=int((i/16)/5)-3;
    boundaries.points[i].position[0]=.3+(x+.5)*.1;
    boundaries.points[i].position[1]=-.1+(y+.5)*.1;
  }
  for(int y=-2;y<=1;++y)for(int x=-2;x<=1;++x)
    boundaries.points.push_back({{.3+x*.1,-.1+y*.1,-.9},g::ReturnKind::Measured,false,0,
      uint64_t(5000+boundaries.points.size())});
  const auto boundary_frame=g::classify_supported_ground_frame(boundaries,c,boundary_ref);
  check(boundary_frame->ground().safety_qualified&&boundary_frame->clearing_indices().size()==416);
  const auto& rebased=boundary_frame->reference()->support;
  for(const auto& point:boundaries.points) {
    const auto original_x=std::floor((point.position[0]-.3)/.1);
    const auto original_y=std::floor((point.position[1]+.1)/.1);
    const auto relative_x=original_x-std::round((rebased.origin_x_m-.3)/.1);
    const auto relative_y=original_y-std::round((rebased.origin_y_m+.1)/.1);
    check(std::any_of(boundary_frame->ground().cells.begin(),boundary_frame->ground().cells.end(),
      [&](const auto& cell){return cell.cell.x==relative_x&&cell.cell.y==relative_y;}));
  }
  auto no_rectangle=ref;no_rectangle.support.required_cells.clear();
  no_rectangle.support.width=no_rectangle.support.height=0;
  check(g::classify_supported_ground_frame(input,c,no_rectangle)->clearing_indices().size()==400);
  auto cell_budget=c;cell_budget.maximum_support_cells=24;
  const auto too_many_cells=g::classify_supported_ground_frame(input,cell_budget,ref);
  conservative(too_many_cells,400);check(too_many_cells->ground().result==g::Qualification::BudgetExceeded);
  auto huge_extent=input;huge_extent.points.push_back({{1.e10,.05,-.9},g::ReturnKind::Measured,false,0,10003});
  const auto too_wide=g::classify_supported_ground_frame(huge_extent,c,ref);
  conservative(too_wide,401);check(too_wide->ground().result==g::Qualification::BudgetExceeded);
  // The same plane observed at two nonzero acquisition transforms.
  auto moving=input;auto later=moving.to_common;later.source_ns+=50000000;later.common_ns=later.source_ns;
  later.translation[2]=.2;moving.source_time_transforms.push_back(later);
  for(std::size_t i=1;i<moving.points.size();i+=2){moving.points[i].source_offset_ns=50000000;moving.points[i].position[2]-=.2;}
  check(g::classify_supported_ground_frame(moving,c,ref)->clearing_indices().size()==400);
  // Failed point-time identity retains its precedence over a bad reference.
  auto missing_group=moving;missing_group.source_time_transforms.clear();
  auto bad_reference=ref;bad_reference.unit_normal={0.,0.,0.};
  check(g::classify_supported_ground_frame(missing_group,c,bad_reference)->ground().result==g::Qualification::InvalidTransform);
  auto overflow=moving;overflow.points[1].source_offset_ns=std::numeric_limits<int64_t>::max();
  check(g::classify_supported_ground_frame(overflow,c,ref)->ground().result==g::Qualification::InvalidTransform);
  // Repeated arbitrary-order groups must keep numerical heights and all cell
  // decisions identical to the same original returns under one rigid TF.
  auto same_pose=input;same_pose.source_time_transforms={later};
  same_pose.source_time_transforms.front().translation={0.,0.,0.};
  for(std::size_t i=0;i<same_pose.points.size();++i)if(i%3!=0)same_pose.points[i].source_offset_ns=50000000;
  const auto equivalent=g::classify_supported_ground_frame(same_pose,c,ref);
  check(equivalent->classes()==frame->classes());
  check(equivalent->marking_indices()==frame->marking_indices());
  check(equivalent->clearing_indices()==frame->clearing_indices());
  check(equivalent->protected_hit_indices()==frame->protected_hit_indices());
  check(equivalent->relative_heights_m()==frame->relative_heights_m());
  check(equivalent->seed_relative_heights_m()==frame->seed_relative_heights_m());
  check(equivalent->ground().support_checks==frame->ground().support_checks);
  moving.source_time_transforms.clear();conservative(g::classify_supported_ground_frame(moving,c,ref),400);
  auto bad=c;bad.maximum_cell_mad_m=-.01;rejects([&]{g::classify_supported_ground_frame(input,bad,ref);});
  bad=c;bad.maximum_support_checks=1;conservative(g::classify_supported_ground_frame(input,bad,ref),400);
  auto expired=ref;expired.expiry_budget_ns=1;conservative(g::classify_supported_ground_frame(input,c,expired),400);
  // Duplicate sample identities all occupy the same partition, so duplicated
  // returns cannot manufacture the missing independent holdout partition.
  auto one_partition=input;for(auto& p:one_partition.points)p.sample_id=0;
  conservative(g::classify_supported_ground_frame(one_partition,c,ref),400);
  // A measured offset is learned from training identities only; neither the
  // independent seed plane nor physical qualification fields are rewritten.
  auto biased=input;for(auto& p:biased.points)p.position[2]-=.035;
  const auto corrected=g::classify_supported_ground_frame(biased,c,ref);
  using FitResult=g::GroundQualification::MeasuredPlaneFit::Result;
  check(corrected->ground().measured_plane_fit.result==FitResult::Accepted);
  check(corrected->clearing_indices().size()==400&&corrected->reference()->anchor==ref.anchor);
  check(corrected->reference()->unit_normal==ref.unit_normal&&!corrected->source().measurement_qualified);
  check(corrected->seed_relative_heights_m().size()==400);
  for(std::size_t i=0;i<400;++i) {
    check(corrected->seed_relative_heights_m()[i]<-.03);
    check(std::abs(corrected->relative_heights_m()[i])<.003);
    check(std::isnan(corrected->error_bounds_m()[i]));
  }
  const auto holdout=[](uint64_t id){id+=0x9e3779b97f4a7c15ULL;
    id=(id^(id>>30))*0xbf58476d1ce4e5b9ULL;id=(id^(id>>27))*0x94d049bb133111ebULL;
    return ((id^(id>>31))&1U)!=0;};
  auto changed_holdout=biased;
  for(auto& p:changed_holdout.points)if(holdout(p.sample_id))p.position[2]+=.005;
  const auto independent=g::classify_supported_ground_frame(changed_holdout,c,ref);
  check(independent->ground().measured_plane_fit.result==FitResult::Accepted);
  for(std::size_t k=0;k<3;++k) {
    check(independent->ground().measured_plane_fit.unit_normal[k]==corrected->ground().measured_plane_fit.unit_normal[k]);
    check(independent->ground().measured_plane_fit.anchor[k]==corrected->ground().measured_plane_fit.anchor[k]);
  }
  auto failed_holdout=biased;
  for(auto& p:failed_holdout.points)if(holdout(p.sample_id))p.position[2]+=.035;
  const auto rejected_fit=g::classify_supported_ground_frame(failed_holdout,c,ref);
  check(rejected_fit->ground().measured_plane_fit.result==FitResult::HoldoutRejected);
  conservative(rejected_fit,400);
  // Correct previously out-of-seed negative measurements only inside an
  // actually observed seed cell. An unseen cell cannot acquire support.
  auto local=biased;local.points.push_back({{.25,.25,-.95},g::ReturnKind::Measured,false,0,50000});
  local.points.push_back({{2.,2.,-.95},g::ReturnKind::Measured,false,0,50001});
  const auto local_fit=g::classify_supported_ground_frame(local,c,ref);
  check(local_fit->classes()[400]==g::PointClass::Ground&&local_fit->classes()[401]==g::PointClass::Uncertain);
  check(!local_fit->source().points[401].ground_blocked);
  // Negative-side cells may acquire support only from their own independent
  // raster partitions AND a shared-edge path to the original qualified patch.
  // The original fit/seed/thresholds are unchanged; no synthetic plane samples.
  const auto append_patch=[](g::SourceFrame& f,unsigned start_x,unsigned start_y,
      unsigned width,unsigned height,double z,bool blocked=false,bool training_only=false) {
    const auto partition=[](uint64_t id){id+=0x9e3779b97f4a7c15ULL;
      id=(id^(id>>30))*0xbf58476d1ce4e5b9ULL;id=(id^(id>>27))*0x94d049bb133111ebULL;
      return ((id^(id>>31))&1U)!=0;};
    uint64_t identity=60000;
    for(unsigned y=start_y;y<start_y+height;++y)for(unsigned x=start_x;x<start_x+width;++x)
      for(unsigned k=0;k<16;++k) {
        while(training_only&&partition(identity))++identity;
        f.points.push_back({{(x+.5)*.1,(y+.5)*.1,z},g::ReturnKind::Measured,blocked,0,identity++});
      }
  };
  auto extension=biased;append_patch(extension,5,0,3,5,-.95);
  const auto expanded=g::classify_supported_ground_frame(extension,c,ref);
  check(expanded->ground().safety_qualified&&expanded->clearing_indices().size()==640);
  check(expanded->ground().measured_plane_fit.anchor==corrected->ground().measured_plane_fit.anchor);
  check(expanded->ground().measured_plane_fit.unit_normal==corrected->ground().measured_plane_fit.unit_normal);
  auto reordered_extension=extension;std::reverse(reordered_extension.points.begin(),reordered_extension.points.end());
  const auto reordered=g::classify_supported_ground_frame(reordered_extension,c,ref);
  for(std::size_t i=0;i<extension.points.size();++i)
    check(expanded->classes()[i]==reordered->classes()[extension.points.size()-1-i]);
  // Box endpoints sharing a newly supported floor cell remain Obstacle.
  auto mixed_extension=extension;const auto box_begin=mixed_extension.points.size();
  append_patch(mixed_extension,6,2,1,1,-.775);
  const auto with_box=g::classify_supported_ground_frame(mixed_extension,c,ref);
  check(with_box->clearing_indices().size()==640);
  for(std::size_t i=box_begin;i<mixed_extension.points.size();++i)check(with_box->classes()[i]==g::PointClass::Obstacle);
  // A whole missing cell column is not interpolated, even when an equally
  // flat, densely observed patch lies immediately beyond that hole.
  for(const auto start:{6U,10U}) {
    auto gap=biased;append_patch(gap,start,0,3,5,-.95);
    const auto stopped=g::classify_supported_ground_frame(gap,c,ref);
    check(stopped->clearing_indices().size()==400);
    for(std::size_t i=400;i<gap.points.size();++i)check(stopped->classes()[i]==g::PointClass::Uncertain);
  }
  auto diagonal=biased;append_patch(diagonal,5,5,3,3,-.95);
  check(g::classify_supported_ground_frame(diagonal,c,ref)->clearing_indices().size()==400);
  // Both cells individually fit the plane band, but a 27mm edge jump fails
  // the existing 20mm partition-agreement gate in each partition.
  auto step=biased;append_patch(step,5,0,3,5,-.962);
  check(g::classify_supported_ground_frame(step,c,ref)->clearing_indices().size()==400);
  auto only_training=biased;append_patch(only_training,5,0,3,5,-.95,false,true);
  check(g::classify_supported_ground_frame(only_training,c,ref)->clearing_indices().size()==400);
  auto blocked_extension=biased;append_patch(blocked_extension,5,0,3,5,-.95,true);
  check(g::classify_supported_ground_frame(blocked_extension,c,ref)->clearing_indices().size()==400);
  // Predicted body conflicts remain Uncertain even beside dense independent
  // floor support. They provide no obstacle vote or clearing endpoint.
  // Synthetic data is not a physical calibration or field acceptance claim.
  auto conflict=input;
  const auto selected=[](const g::PointReturn& p){return p.position[0]>.1&&p.position[0]<.2&&
    p.position[1]>.1&&p.position[1]<.3;};
  std::size_t conflict_count=0;
  for(auto& p:conflict.points)if(selected(p)){p.ground_blocked=p.model_conflict=true;++conflict_count;}
  check(conflict_count==32);
  const auto blocked_conflict=g::classify_supported_ground_frame(conflict,c,ref);
  check(blocked_conflict->clearing_indices().size()==368);
  for(std::size_t i=0;i<conflict.points.size();++i)if(selected(conflict.points[i])) {
    check(blocked_conflict->classes()[i]==g::PointClass::Uncertain);
    check(blocked_conflict->decisions()[i]==g::PointDecision::ModelBlocked);
  }
  auto ambiguous=conflict;for(auto& p:ambiguous.points)p.model_conflict=false;
  check(g::classify_supported_ground_frame(ambiguous,c,ref)->clearing_indices().size()==368);
  // Three, four and five centimetre raised conflict surfaces remain protected,
  // even if dense, connected and surrounded by observed floor.
  for(const auto height:{.03,.04,.05}) {
    auto low_obstacle=conflict;for(auto& p:low_obstacle.points)if(p.ground_blocked)p.position[2]+=height;
    const auto guarded=g::classify_supported_ground_frame(low_obstacle,c,ref);
    check(guarded->clearing_indices().size()==368);
    for(std::size_t i=0;i<low_obstacle.points.size();++i)if(low_obstacle.points[i].ground_blocked)
      check(guarded->classes()[i]==g::PointClass::Uncertain);
  }
  auto mixed_conflict=conflict;for(auto& p:mixed_conflict.points)if(p.ground_blocked){p.position[2]+=.03;break;}
  check(g::classify_supported_ground_frame(mixed_conflict,c,ref)->clearing_indices().size()==368);
  auto one_cell=conflict;for(auto& p:one_cell.points)if(p.position[1]>.2)p.ground_blocked=p.model_conflict=false;
  check(g::classify_supported_ground_frame(one_cell,c,ref)->clearing_indices().size()==384);
  auto conflict_partition=conflict;
  for(auto& p:conflict_partition.points)if(p.ground_blocked){while(holdout(p.sample_id))++p.sample_id;}
  check(g::classify_supported_ground_frame(conflict_partition,c,ref)->clearing_indices().size()==368);
  auto disconnected=input;const auto initial=disconnected.points.size();
  append_patch(disconnected,9,1,1,2,-.9,true);
  for(std::size_t i=initial;i<disconnected.points.size();++i)disconnected.points[i].model_conflict=true;
  check(g::classify_supported_ground_frame(disconnected,c,ref)->clearing_indices().size()==400);
  auto invalid_tf=conflict;invalid_tf.to_common.qualified=false;
  conservative(g::classify_supported_ground_frame(invalid_tf,c,ref),400);
  // A raised box, a drop below the fitted band and a disconnected box top
  // stay protected; negative seed height alone is never a Ground decision.
  for(const auto z:{-.775,-1.095}) {
    auto object=biased;append_patch(object,5,0,3,5,z);
    check(g::classify_supported_ground_frame(object,c,ref)->clearing_indices().size()==400);
  }
  auto no_anchor=biased;no_anchor.points.clear();append_patch(no_anchor,0,0,5,5,-.95);
  conservative(g::classify_supported_ground_frame(no_anchor,c,ref),400);
  // A valid near sloped patch must never extrapolate to a distant 16cm box
  // top or to an unobserved negative-side surface that happens to match it.
  for(const auto slope:{.19,-.19}) {
    auto sloped=input;sloped.points.clear();
    for(unsigned y=0;y<3;++y)for(unsigned x=0;x<3;++x)for(unsigned i=0;i<16;++i) {
      const double px=.025+x*.1,py=.025+y*.1;
      sloped.points.push_back({{px,py,-.9+slope*px-(slope>0?.02:-.02)},g::ReturnKind::Measured,
        false,0,uint64_t(sloped.points.size())});
    }
    const auto original=sloped.points.size();
    for(unsigned y=0;y<3;++y)for(unsigned x=0;x<3;++x)for(unsigned i=0;i<16;++i)
      sloped.points.push_back({{.85+x*.1,.05+y*.1,-.9+(slope>0?.16:-.16)},g::ReturnKind::Measured,
        false,0,uint64_t(sloped.points.size())});
    const auto distant=g::classify_supported_ground_frame(sloped,c,ref);
    check(distant->ground().measured_plane_fit.result==FitResult::Accepted);
    check(distant->clearing_indices().size()==original);
    for(std::size_t i=original;i<sloped.points.size();++i) {
      check(distant->classes()[i]!=g::PointClass::Ground);
      check(std::find(distant->protected_hit_indices().begin(),distant->protected_hit_indices().end(),i)!=distant->protected_hit_indices().end());
    }
  }
}

void unblocked_mixed_cell_ground_and_raised_returns() {
  g::SegmentationConfig c;c.maximum_point_residual_m=.04;
  c.maximum_cell_median_residual_m=.03;c.maximum_cell_mad_m=.015;
  c.maximum_partition_median_difference_m=.02;c.maximum_normal_tilt_rad=.2;
  c.minimum_support_span_x_m=c.minimum_support_span_y_m=.1;c.minimum_support_double_area_m2=.005;
  c.minimum_samples_per_partition=1;c.minimum_supported_cells=6;c.continuous_support_radius_cells=1;
  c.maximum_points=20000;c.maximum_support_cells=100;c.maximum_support_checks=4000000;
  auto input=source();input.points.clear();input.to_common.translation[2]=-1.9;
  input.raster_geometry={100,100,1};
  constexpr std::size_t floor_target=40*100+40,raised=45*100+45;
  for(unsigned v=0;v<100;++v)for(unsigned u=0;u<100;++u) {
    const auto id=uint64_t(v*100+u);const bool patch=u>=40&&u<60&&v>=40&&v<60;
    input.points.push_back({{(u+.5)*.005,(v+.5)*.005,1.+(patch&&id!=floor_target?.08:0.)},
      g::ReturnKind::Measured,false,0,id,false});
  }
  const auto ref=reference(input);auto no_raster=input;no_raster.raster_geometry={};
  const auto unproved=g::classify_supported_ground_frame(no_raster,c,ref);
  const auto repaired=g::classify_supported_ground_frame(input,c,ref);
  check(repaired->ground().safety_qualified);
  check(unproved->classes()[floor_target]==g::PointClass::Uncertain);
  check(repaired->classes()[floor_target]==g::PointClass::Uncertain);
  check(repaired->classes()[raised]==g::PointClass::Obstacle);
  check(repaired->decisions().size()==input.points.size());
  check(repaired->decisions()[raised]==g::PointDecision::ObstacleAnchored);
  check(repaired->decisions()[floor_target]==g::PointDecision::Unsupported);
  // A dense raised patch must not spend another eight anchor checks for every
  // point in the same cell. Keep the exact measured-support budget: the former
  // per-return traversal exhausts it and demotes the entire frame to Unknown.
  check(repaired->ground().support_checks==54741);
  auto tight=c;tight.maximum_support_checks=54741;
  const auto within_budget=g::classify_supported_ground_frame(input,tight,ref);
  check(within_budget->ground().safety_qualified);
  check(within_budget->classes()==repaired->classes());
  check(within_budget->protected_hit_indices()==repaired->protected_hit_indices());
  check(within_budget->clearing_indices()==repaired->clearing_indices());
  tight.maximum_support_checks=54740;
  const auto just_short=g::classify_supported_ground_frame(input,tight,ref);
  check(!just_short->ground().safety_qualified&&just_short->clearing_indices().empty());
  tight.maximum_support_checks=1;
  const auto failed=g::classify_supported_ground_frame(input,tight,ref);
  check(!failed->ground().safety_qualified&&failed->clearing_indices().empty());
  for(const auto decision:failed->decisions())check(decision==g::PointDecision::NotRecorded);

  check(repaired->ground().measured_plane_fit.anchor==unproved->ground().measured_plane_fit.anchor);
  check(repaired->ground().measured_plane_fit.unit_normal==unproved->ground().measured_plane_fit.unit_normal);
  // A 7x7 raised top cannot become Ground, even if its
  // surrounding cell is mixed and there are no CAD conflicts anywhere.
  for(double height:{.03,.05,.16}) {
    auto object=input;
    for(unsigned v=40;v<47;++v)for(unsigned u=40;u<47;++u)object.points[v*100+u].position[2]=1.+height;
    const auto guarded=g::classify_supported_ground_frame(object,c,ref);
    for(unsigned v=40;v<47;++v)for(unsigned u=40;u<47;++u)
      check(guarded->classes()[v*100+u]!=g::PointClass::Ground);
  }
  auto protected_input=input;protected_input.points[floor_target].ground_blocked=true;
  check(g::classify_supported_ground_frame(protected_input,c,ref)->classes()[floor_target]!=g::PointClass::Ground);
  auto restricted=c;restricted.maximum_support_checks=unproved->ground().support_checks+1;
  const auto stopped=g::classify_supported_ground_frame(input,restricted,ref);
  check(stopped->ground().support_checks<=restricted.maximum_support_checks);
  check(stopped->classes()[floor_target]!=g::PointClass::Ground);
}


void rejection_proof(g::SourceFrame& frame,uint64_t sample_id) {
  frame.raster_geometry={100,100,1};auto& q=frame.measurement_quality;const auto& id=frame.identity;
  q.present=q.exact_rectified_pair=true;q.algorithm="dual-ir-native-quality-v4";
  q.stereo_match_complete=true;q.stereo_match_reason="disabled_foreground_ambiguity";
  q.role=id.source_id;q.source_frame=id.source_frame;q.source_epoch=id.epoch;q.calibration_id=id.calibration_id;
  q.source_ns=id.source_ns;q.sequence=id.sequence;q.opened_serial="test-native-device";q.capture_epoch="test-capture";
  q.width=q.height=100;q.depth_pixel_type=0;q.device_us={1,1,1};q.global_us={1000000,1000000,1000000};q.sdk_depth_scale_m=.001;
  q.unreliable_sample_ids={sample_id};
}

void rejection_count_diagnostics_never_underflow() {
  g::MeasurementQualityProof q;check(g::measurement_rejection_counts_valid(q));
  q.component_sample_ids={1};check(!g::measurement_rejection_counts_valid(q));
  q.unreliable_sample_ids={1};check(g::measurement_rejection_counts_valid(q));
  q.stereo_mismatch_sample_ids={1};check(!g::measurement_rejection_counts_valid(q));
  q.unreliable_sample_ids={1,2};check(!g::measurement_rejection_counts_valid(q));
  q.stereo_mismatch_sample_ids={2};check(g::measurement_rejection_counts_valid(q));
}
void measurement_quality_never_marks_fits_or_clears() {
  rejection_count_diagnostics_never_underflow();
  auto input=source();for(std::size_t i=0;i<input.points.size();++i)input.points[i].sample_id=i;
  const auto c=config();auto ref=reference(input);
  const auto original=g::classify_obstacle_frame(input,c,ref);
  for(double height:{0.,.03,.05,.16}) {
    auto tested=input;g::PointReturn q{{.05,.05,-.9+height},g::ReturnKind::Measured,false,0,100,false,true};
    tested.points.push_back(q);rejection_proof(tested,q.sample_id);const auto result=g::classify_obstacle_frame(tested,c,ref);
    check(result->classes().back()==g::PointClass::Invalid);
    check(result->decisions().back()==g::PointDecision::MeasurementUnreliable);
    check(result->has_unknown());
    check(std::find(result->marking_indices().begin(),result->marking_indices().end(),25)==result->marking_indices().end());
    check(std::find(result->protected_hit_indices().begin(),result->protected_hit_indices().end(),25)==result->protected_hit_indices().end());
    for(int bad=0;bad<5;++bad){
      auto forged=tested;auto& proof=forged.measurement_quality;
      if(bad==0)proof.algorithm="dual-ir-native-quality-v3";
      if(bad==1)proof.stereo_match_checks=1;
      if(bad==2)proof.stereo_mismatch_sample_ids={q.sample_id};
      if(bad==3)proof.stereo_match_reason="complete";
      if(bad==4)proof.stereo_match_complete=false;
      rejects([&]{g::conservative_unqualified_frame(forged);});
    }
    const auto no_reference=g::conservative_unqualified_frame(tested);
    check(no_reference->classes().back()==g::PointClass::Invalid);
    check(no_reference->marking_indices().empty()&&no_reference->has_unknown());
    check(no_reference->protected_hit_indices().size()==25);
    check(std::find(result->clearing_indices().begin(),result->clearing_indices().end(),25)==result->clearing_indices().end());
    check(result->ground().cells.front().samples==original->ground().cells.front().samples);
    check(!result->source().points.back().ground_blocked&&!result->source().points.back().model_conflict);
    check(result->source().points.back().position==q.position);
    auto propagated=tested;auto& component=propagated.measurement_quality;
    component.component_sample_ids={q.sample_id};component.component_extension_complete=true;
    component.component_extension_reason="complete";component.component_checks=4;
    const auto inherited=g::classify_obstacle_frame(propagated,c,ref);
    check(inherited->classes().back()==g::PointClass::Invalid&&inherited->has_unknown());
    check(inherited->decisions().back()==g::PointDecision::SaturationDominatedComponent);
    check(inherited->clearing_indices()==result->clearing_indices()&&inherited->marking_indices()==result->marking_indices());
    for(int bad=0;bad<4;++bad){
      auto forged=propagated;auto& proof=forged.measurement_quality;
      if(bad==0)proof.component_extension_complete=false;
      if(bad==1)proof.component_sample_ids={q.sample_id+1};
      if(bad==2)proof.component_sample_ids.push_back(q.sample_id);
      if(bad==3)proof.component_checks=65537;
      rejects([&]{g::conservative_unqualified_frame(forged);});
    }
    auto stereo=tested;auto& mismatch_proof=stereo.measurement_quality;
    mismatch_proof.stereo_mismatch_sample_ids={q.sample_id};mismatch_proof.stereo_match_complete=true;
    mismatch_proof.stereo_match_reason="complete";mismatch_proof.stereo_match_checks=225;
    // Historical stereo reason4 cannot be relabelled as current v4 authority.
    rejects([&]{g::conservative_unqualified_frame(stereo);});
    for(int bad=0;bad<5;++bad){
      auto forged=stereo;auto& proof=forged.measurement_quality;
      if(bad==0)proof.stereo_match_complete=false;
      if(bad==1)proof.stereo_match_checks=224;
      if(bad==2)proof.stereo_mismatch_sample_ids={q.sample_id+1};
      if(bad==4)proof.stereo_match_checks=65537;
      if(bad==3){proof.component_sample_ids={q.sample_id};proof.component_extension_complete=true;proof.component_extension_reason="complete";proof.component_checks=4;}
      rejects([&]{g::conservative_unqualified_frame(forged);});
    }
    for(int mismatch=0;mismatch<12;++mismatch) {
      auto forged=tested;auto& proof=forged.measurement_quality;
      if(mismatch==0)proof.present=false;
      if(mismatch==1)proof.exact_rectified_pair=false;
      if(mismatch==2)++proof.source_ns;
      if(mismatch==3)++proof.sequence;
      if(mismatch==4)proof.source_epoch="other-epoch";
      if(mismatch==5)proof.unreliable_sample_ids.clear();
      if(mismatch==6)++proof.device_us[1];
      if(mismatch==7)proof.calibration_id="other-calibration";
      if(mismatch==8)proof.global_us={1000001,1000001,1000001};
      if(mismatch==11)proof.algorithm="dual-ir-5x5-saturation-component-v2";
      if(mismatch==10)proof.algorithm="dual-ir-5x5-saturation-v1";
      if(mismatch==9)proof.global_us.fill(std::numeric_limits<uint64_t>::max());
      rejects([&]{g::conservative_unqualified_frame(forged);});
    }
  }
  auto rejected_only=input;rejection_proof(rejected_only,0);
  rejected_only.measurement_quality.unreliable_sample_ids.clear();
  for(auto& p:rejected_only.points){p.measurement_unreliable=true;rejected_only.measurement_quality.unreliable_sample_ids.push_back(p.sample_id);}
  const auto empty_endpoints=g::classify_obstacle_frame(rejected_only,c,ref);
  check(!empty_endpoints->classification_complete()&&empty_endpoints->has_unknown());
  check(empty_endpoints->marking_indices().empty()&&empty_endpoints->clearing_indices().empty()&&empty_endpoints->protected_hit_indices().empty());
  for(const auto cls:empty_endpoints->classes())check(cls==g::PointClass::Invalid);
  g::SegmentationConfig sc;sc.maximum_point_residual_m=.04;sc.maximum_cell_median_residual_m=.03;sc.maximum_cell_mad_m=.015;
  sc.maximum_partition_median_difference_m=.02;sc.maximum_normal_tilt_rad=.2;
  sc.minimum_support_span_x_m=sc.minimum_support_span_y_m=.1;sc.minimum_support_double_area_m2=.005;
  sc.minimum_samples_per_partition=1;sc.minimum_supported_cells=6;sc.continuous_support_radius_cells=1;
  sc.maximum_points=4000;sc.maximum_support_cells=100;sc.maximum_support_checks=100000;
  input.points.clear();input.measurement_qualified=false;
  for(unsigned y=0;y<5;++y)for(unsigned x=0;x<5;++x)for(unsigned sample=0;sample<16;++sample)
    input.points.push_back({{(x+.5)*.1,(y+.5)*.1,-.9+(double(sample%3)-1.)*.001},g::ReturnKind::Measured,false,0,uint64_t(input.points.size()),false});
  ref=reference(input);ref.independently_qualified=ref.source_time_transform_qualified=false;
  const auto old=g::classify_supported_ground_frame(input,sc,ref);
  for(bool CADconflict:{false,true}) {
    auto tested=input;tested.points.push_back({{.05,.05,-.9},g::ReturnKind::Measured,CADconflict,0,1000,CADconflict,true});
    rejection_proof(tested,1000);
    const auto result=g::classify_supported_ground_frame(tested,sc,ref);
    check(result->classes().back()==g::PointClass::Invalid);
    check(result->decisions().back()==g::PointDecision::MeasurementUnreliable);
    check(result->marking_indices().empty()&&result->protected_hit_indices().empty()&&result->has_unknown());
    check(result->clearing_indices()==old->clearing_indices());
    check(result->ground().measured_plane_fit.anchor==old->ground().measured_plane_fit.anchor);
    check(result->ground().measured_plane_fit.unit_normal==old->ground().measured_plane_fit.unit_normal);
  }
}
}  // namespace
int main() {
  try {measurement_quality_never_marks_fits_or_clears();unblocked_mixed_cell_ground_and_raised_returns();run();supported_segmentation();std::cout<<"PASS "<<checks<<" classified obstacle checks\n";return 0;}
  catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
