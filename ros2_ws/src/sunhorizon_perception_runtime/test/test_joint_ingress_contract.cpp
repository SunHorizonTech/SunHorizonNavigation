#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "sunhorizon_perception_runtime/joint_ingress.hpp"

namespace {
using namespace sunhorizon_perception_runtime;

class JointIngressContractTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    model_options.urdf_xml = "<robot name=\"ingress_contract\"><link name=\"base_link\"/>";
    for (int index = 0; index < 13; ++index) {
      const auto joint = index == 12 ? std::string("waist_yaw") : "joint"+std::to_string(index);
      const auto link = "link"+std::to_string(index);
      model_options.expected_joint_names.push_back(joint);
      if (index != 12) model_options.target_frames.push_back(link);
      model_options.urdf_xml += "<link name=\""+link+"\"/><joint name=\""+joint+
          "\" type=\"continuous\"><parent link=\"base_link\"/><child link=\""+link+
          "\"/><axis xyz=\"0 0 1\"/></joint>";
    }
    model_options.urdf_xml += "</robot>";
    std::vector<std::string> source_names;
    for (int index = 0; index < 49; ++index)
      source_names.push_back(index == 12 ? std::string("waist_yaw") : "vendor"+std::to_string(index));
    const std::vector<std::string> selected(source_names.begin(), source_names.begin()+13);
    std::vector<int64_t> indices;
    for (int index = 0; index < 13; ++index) indices.push_back(index);
    parameters = {
        rclcpp::Parameter("source_contract_accepted", true),
        rclcpp::Parameter("source_owner_name", "hl_orin_core_msg"),
        rclcpp::Parameter("source_owner_namespace", "/"),
        rclcpp::Parameter("runtime_binding", "unique_current_endpoint_name_type_and_message_gid"),
        rclcpp::Parameter("expected_source_joint_names", source_names),
        rclcpp::Parameter("selected_source_joint_names", selected),
        rclcpp::Parameter("selected_source_joint_indices", indices),
        rclcpp::Parameter("output_joint_names", model_options.expected_joint_names),
        rclcpp::Parameter("position_unit", "rad"),
        rclcpp::Parameter("value_transform", "identity"),
        rclcpp::Parameter("source_receipt_path", "/data/navigation/source_receipt.json"),
        rclcpp::Parameter("source_receipt_sha256", std::string(64, 'a')),
        rclcpp::Parameter("source_evidence_sha256", std::string(64, 'b'))};
  }
  void TearDown() override {context->shutdown("joint ingress contract test complete");}

  rclcpp::NodeOptions node_options() {
    rclcpp::NodeOptions result;
    result.context(context);
    result.parameter_overrides(parameters);
    return result;
  }

  void replace(const rclcpp::Parameter& value) {
    for (auto& item : parameters) {
      if (item.get_name() == value.get_name()) {item = value; return;}
    }
    parameters.push_back(value);
  }

  std::shared_ptr<rclcpp::Context> context;
  JointKinematicsOptions model_options;
  std::vector<rclcpp::Parameter> parameters;
};

TEST_F(JointIngressContractTest, AcceptanceBooleanCannotAuthorizeAbsentSignedReceiptAssets) {
  auto model = std::make_shared<JointKinematics>(model_options);
  EXPECT_THROW(make_joint_ingress(model, node_options()), std::exception);
  EXPECT_EQ(model->evidence().source_stamp_ns, 0);
  EXPECT_EQ(model->lookup(100, [] {return JointKinematics::ClockSample{100, 100};}).status,
            JointKinematicsStatus::Missing);
}

TEST_F(JointIngressContractTest, OwnerAndReceiptContractFailClosedAtConstruction) {
  auto model = std::make_shared<JointKinematics>(model_options);
  replace(rclcpp::Parameter("source_owner_name", "another_robot"));
  EXPECT_THROW(make_joint_ingress(model, node_options()), std::runtime_error);
  replace(rclcpp::Parameter("source_owner_name", "hl_orin_core_msg"));
  replace(rclcpp::Parameter("source_receipt_sha256", "not_a_sha"));
  EXPECT_THROW(make_joint_ingress(model, node_options()), std::runtime_error);
}

TEST_F(JointIngressContractTest, CannotLoosenSourceAgeOrDisconnectOutputNamesFromModel) {
  auto model = std::make_shared<JointKinematics>(model_options);
  replace(rclcpp::Parameter("maximum_source_age_sec", 0.25001));
  EXPECT_THROW(make_joint_ingress(model, node_options()), std::runtime_error);
  replace(rclcpp::Parameter("maximum_source_age_sec", 0.25));
  auto wrong_names = model_options.expected_joint_names;
  wrong_names[0] = "wrong_joint";
  replace(rclcpp::Parameter("output_joint_names", wrong_names));
  EXPECT_THROW(make_joint_ingress(model, node_options()), std::runtime_error);
}

TEST_F(JointIngressContractTest, ReusesOriginalGateIndexAndWaistValidation) {
  auto model = std::make_shared<JointKinematics>(model_options);
  std::vector<int64_t> wrong_indices(13, 0);
  replace(rclcpp::Parameter("selected_source_joint_indices", wrong_indices));
  EXPECT_THROW(make_joint_ingress(model, node_options()), std::invalid_argument);
}
}  // namespace
