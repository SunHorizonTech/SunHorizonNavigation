#include <gtest/gtest.h>
#include "sunhorizon_rgbd_adapter/depth_components.hpp"

using sunhorizon_rgbd_adapter::depth_component_mask;
using sunhorizon_rgbd_adapter::DepthImageView;

TEST(DepthComponents, FullResolutionConnectivityPreservesThinDiagonal) {
  // A four-pixel diagonal is connected in 8-neighborhood before stride sampling.
  std::vector<uint8_t> bytes(4 * 4 * 2, 0);
  for (int i = 0; i < 4; ++i) { bytes[(i*4+i)*2] = 0xe8; bytes[(i*4+i)*2+1] = 3; }
  auto r = depth_component_mask({bytes.data(),bytes.size(),4,4,8,false},.001,.2,8.,.05,4);
  ASSERT_TRUE(r.valid); EXPECT_EQ(r.removed_pixels,0U); EXPECT_EQ(r.valid_pixels,4U);
  for (int i=0;i<4;++i) EXPECT_EQ(r.keep[i*4+i],1);
}

TEST(DepthComponents, NeighborDifferenceDoesNotJoinDisjointDepthSurfaces) {
  std::vector<uint8_t> b{0xe8,3, 0x1a,4, 0x7c,4, 0,0}; // 1000,1050,1148,invalid
  auto r=depth_component_mask({b.data(),b.size(),4,1,8,false},.001,.2,8.,.05,2);
  ASSERT_TRUE(r.valid); EXPECT_EQ(r.keep,(std::vector<uint8_t>{1,1,0,0}));
  EXPECT_EQ(r.removed_components,1U); EXPECT_EQ(r.removed_pixels,1U);
}

TEST(DepthComponents, RejectedComponentIsCountedOnlyOnceAndInputIsUntouched) {
  std::vector<uint8_t> b{0xe8,3, 0xe8,3, 0,0, 0xd0,7, 0xd0,7, 0xd0,7};
  const auto original=b;
  auto r=depth_component_mask({b.data(),b.size(),6,1,12,false},.001,.2,8.,.05,3);
  ASSERT_TRUE(r.valid); EXPECT_EQ(r.removed_pixels,2U); EXPECT_EQ(r.removed_components,1U);
  EXPECT_EQ(r.keep,(std::vector<uint8_t>{0,0,0,1,1,1})); EXPECT_EQ(b,original);
}

TEST(DepthComponents, EndiannessPaddedRowsAndValidRange) {
  std::vector<uint8_t> b{3,0xe8, 0,0, 99,99, 3,0xe8, 0xff,0xff, 99,99};
  auto r=depth_component_mask({b.data(),b.size(),2,2,6,true},.001,.2,8.,.05,2);
  ASSERT_TRUE(r.valid); EXPECT_EQ(r.keep,(std::vector<uint8_t>{1,0,1,0}));
  EXPECT_EQ(r.valid_pixels,2U);
}

TEST(DepthComponents, BadLayoutAndParametersDoNotProduceMask) {
  std::vector<uint8_t> b(8);
  for (const auto& view : {DepthImageView{b.data(),7,2,2,4,false},
       DepthImageView{b.data(),8,2,2,3,false},DepthImageView{nullptr,8,2,2,4,false},
       DepthImageView{b.data(),8,4194305,1,8388610,false}}) {
    auto r=depth_component_mask(view,.001,.2,8.,.05,2);
    EXPECT_FALSE(r.valid); EXPECT_TRUE(r.keep.empty());
  }
  EXPECT_FALSE(depth_component_mask({b.data(),8,2,2,4,false},.001,.2,8.,0.,2).valid);
  EXPECT_FALSE(depth_component_mask({b.data(),8,2,2,4,false},.001,.2,8.,.05,0).valid);
}

#include "depth_components_legacy_reference.hpp"
#include <limits>

TEST(DepthComponents, ReusedWorkspaceMatchesFrozenLegacyAcrossRandomLayouts) {
  using namespace sunhorizon_rgbd_adapter;
  DepthComponentWorkspace workspace;
  uint32_t random = 0x53197a2d;
  const auto next = [&]() { random = random * 1664525u + 1013904223u; return random; };
  for (unsigned trial = 0; trial < 240; ++trial) {
    const uint32_t width = 1 + next() % 67, height = 1 + next() % 53;
    const uint32_t step = width * 2 + next() % 9;
    const bool big = next() % 2;
    std::vector<uint8_t> bytes(std::size_t(step) * height, 0x9a);
    for (uint32_t y = 0; y < height; ++y) for (uint32_t x = 0; x < width; ++x) {
      // Frequent components, invalids and exact range/neighbor boundaries.
      const uint16_t values[] = {0, 199, 200, 249, 250, 251, 950, 1000, 1050, 1051, 8000, 8001, 65535};
      const auto raw = values[next() % (sizeof(values) / sizeof(values[0]))];
      bytes[y*step+x*2] = big ? raw >> 8 : raw & 255;
      bytes[y*step+x*2+1] = big ? raw & 255 : raw >> 8;
    }
    const auto untouched = bytes;
    const DepthImageView view{bytes.data(), bytes.size(), width, height, step, big};
    const std::size_t minimum = 1 + next() % 200;
    const auto expected = legacy_depth_component_mask(view, .001, .2, 8., .05, minimum);
    const auto& actual = depth_component_mask(view, .001, .2, 8., .05, minimum, workspace);
    ASSERT_EQ(actual.valid, expected.valid); EXPECT_EQ(actual.reason, expected.reason);
    EXPECT_EQ(actual.keep, expected.keep); EXPECT_EQ(actual.valid_pixels, expected.valid_pixels);
    EXPECT_EQ(actual.removed_pixels, expected.removed_pixels);
    EXPECT_EQ(actual.removed_components, expected.removed_components);
    EXPECT_EQ(bytes, untouched);
    // Compatibility API uses independent ownership, not an alias of workspace.
    const auto owned = depth_component_mask(view, .001, .2, 8., .05, minimum);
    EXPECT_EQ(owned.keep, expected.keep);
  }
}

TEST(DepthComponents, WorkspaceRetainsCapacityButNeverPreviousFrameOnFailure) {
  using namespace sunhorizon_rgbd_adapter;
  DepthComponentWorkspace workspace;
  std::vector<uint8_t> large(640 * 480 * 2);
  for (std::size_t i = 0; i < large.size(); i += 2) { large[i] = 0xe8; large[i+1] = 3; }
  const DepthImageView view{large.data(), large.size(), 640, 480, 1280, false};
  ASSERT_TRUE(depth_component_mask(view, .001, .2, 8., .05, 200, workspace).valid);
  const auto* depth = workspace.depth.data(); const auto* queue = workspace.queue.data();
  const auto* keep = workspace.result.keep.data();
  for (const auto& bad : {DepthImageView{nullptr, large.size(), 640, 480, 1280, false},
                          DepthImageView{large.data(), 1, 640, 480, 1280, false},
                          DepthImageView{large.data(), large.size(), 4194305, 1, 8388610, false}}) {
    const auto& result = depth_component_mask(bad, .001, .2, 8., .05, 200, workspace);
    EXPECT_FALSE(result.valid); EXPECT_TRUE(result.keep.empty());
    EXPECT_EQ(result.valid_pixels, 0u); EXPECT_EQ(result.removed_pixels, 0u);
    EXPECT_EQ(result.removed_components, 0u);
    EXPECT_TRUE(workspace.depth.empty()); EXPECT_TRUE(workspace.queue.empty());
  }
  EXPECT_FALSE(depth_component_mask(view, std::numeric_limits<double>::quiet_NaN(),
      .2, 8., .05, 200, workspace).valid);
  ASSERT_TRUE(depth_component_mask(view, .001, .2, 8., .05, 200, workspace).valid);
  EXPECT_EQ(workspace.depth.data(), depth); EXPECT_EQ(workspace.queue.data(), queue);
  EXPECT_EQ(workspace.result.keep.data(), keep);
  std::vector<uint8_t> small{0,0,0xe8,3};
  const auto& result = depth_component_mask({small.data(),4,2,1,4,false},.001,.2,8.,.05,2,workspace);
  EXPECT_EQ(result.keep, (std::vector<uint8_t>{0,0})); EXPECT_EQ(result.valid_pixels,1u);
  EXPECT_EQ(result.removed_pixels,1u); EXPECT_EQ(result.removed_components,1u);
  EXPECT_EQ(workspace.depth.data(), depth); EXPECT_EQ(workspace.queue.data(), queue);
  EXPECT_EQ(workspace.result.keep.data(), keep);
}

TEST(DepthComponents, NearThresholdPreservesFarAndRangeCrossingComponents) {
  const std::vector<uint16_t> depth{400,400,0,2000,2000,0,590,610,0,400,400,400};
  std::vector<uint8_t> bytes;
  for (auto d : depth) { bytes.push_back(d & 255); bytes.push_back(d >> 8); }
  auto r = depth_component_mask({bytes.data(),bytes.size(),12,1,24,false},.001,.2,8.,.05,2,3,.6);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.keep,(std::vector<uint8_t>{0,0,0,1,1,0,1,1,0,1,1,1}));
  EXPECT_EQ(r.removed_pixels,2U);
}

TEST(DepthComponents, NearThresholdRejectsInvalidConfiguration) {
  std::vector<uint8_t> bytes{0x90,1};
  DepthImageView v{bytes.data(),bytes.size(),1,1,2,false};
  EXPECT_FALSE(depth_component_mask(v,.001,.2,8.,.05,65,64,.6).valid);
  EXPECT_FALSE(depth_component_mask(v,.001,.2,8.,.05,65,160,.1).valid);
  EXPECT_FALSE(depth_component_mask(v,.001,.2,8.,.05,65,160,9.).valid);
  EXPECT_FALSE(depth_component_mask(v,.001,.2,8.,.05,65,4194305,.6).valid);
}


#include "depth_components_c7_reference.hpp"

TEST(DepthComponents, WorkspaceAndValueNearModesMatchC7AcrossRandomLayouts) {
  using namespace sunhorizon_rgbd_adapter;
  DepthComponentWorkspace workspace;
  uint32_t random = 0xc7bf185u;
  const auto next = [&]() { random = random * 1664525u + 1013904223u; return random; };
  for (unsigned trial = 0; trial < 300; ++trial) {
    const uint32_t width = 1 + next() % 51, height = 1 + next() % 29;
    const uint32_t step = width * 2 + next() % 7;
    const bool big = (next() >> 8) & 1;
    std::vector<uint8_t> bytes(std::size_t(step) * height, 0xa5);
    for (uint32_t y=0; y<height; ++y) for (uint32_t x=0; x<width; ++x) {
      const uint16_t values[] = {0,199,200,400,550,590,599,600,601,610,650,1000,2000,8000,8001};
      const auto raw=values[(next() >> 8) % (sizeof(values)/sizeof(values[0]))];
      bytes[y*step+x*2]=big ? raw>>8 : raw&255;
      bytes[y*step+x*2+1]=big ? raw&255 : raw>>8;
    }
    const auto original=bytes;
    const DepthImageView view{bytes.data(),bytes.size(),width,height,step,big};
    const std::size_t minimum=trial%3 ? 1+next()%6 : 65;
    for (const std::size_t near : {std::size_t(0),minimum,std::size_t(160)}) {
      const auto expected=c7_depth_component_mask(view,.001,.2,8.,.05,minimum,near,.6);
      const auto& actual=depth_component_mask(view,.001,.2,8.,.05,minimum,workspace,near,.6);
      const auto owned=depth_component_mask(view,.001,.2,8.,.05,minimum,near,.6);
      for (const auto* result : {&actual,&owned}) {
        EXPECT_EQ(result->valid,expected.valid);EXPECT_EQ(result->reason,expected.reason);
        EXPECT_EQ(result->keep,expected.keep);EXPECT_EQ(result->valid_pixels,expected.valid_pixels);
        EXPECT_EQ(result->removed_pixels,expected.removed_pixels);
        EXPECT_EQ(result->removed_components,expected.removed_components);
      }
      EXPECT_EQ(bytes,original);
    }
  }
}

TEST(DepthComponents, NearErrorsResetWorkspaceAndDisabledRangeMatchesC7) {
  using namespace sunhorizon_rgbd_adapter;
  DepthComponentWorkspace workspace;
  std::vector<uint8_t> bytes{0x90,1,0x90,1,0x90,1};
  const DepthImageView view{bytes.data(),bytes.size(),3,1,6,false};
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  for (const double range : {.1,.2,.6,8.,9.,nan,inf}) {
    for (const std::size_t near : {std::size_t(0),std::size_t(1),std::size_t(3),kMaxDepthComponentPixels+1}) {
      ASSERT_TRUE(depth_component_mask(view,.001,.2,8.,.05,2,workspace).valid);
      const auto expected=c7_depth_component_mask(view,.001,.2,8.,.05,2,near,range);
      const auto& actual=depth_component_mask(view,.001,.2,8.,.05,2,workspace,near,range);
      const auto owned=depth_component_mask(view,.001,.2,8.,.05,2,near,range);
      EXPECT_EQ(actual.valid,expected.valid);EXPECT_EQ(actual.reason,expected.reason);
      EXPECT_EQ(actual.keep,expected.keep);EXPECT_EQ(owned.keep,expected.keep);
      EXPECT_EQ(owned.valid,expected.valid);EXPECT_EQ(owned.reason,expected.reason);
      if (!expected.valid) {
        EXPECT_TRUE(workspace.depth.empty());EXPECT_TRUE(workspace.queue.empty());
        EXPECT_EQ(actual.valid_pixels,0u);EXPECT_EQ(actual.removed_pixels,0u);
        EXPECT_EQ(actual.removed_components,0u);
      }
    }
  }
}
