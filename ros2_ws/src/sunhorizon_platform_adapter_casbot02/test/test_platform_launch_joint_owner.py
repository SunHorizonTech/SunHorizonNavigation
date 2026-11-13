"""Exercise the actual launch function without requiring ROS or launching a process."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1] / 'launch/platform_evidence.launch.py'


class PlatformJointOwnerTest(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(SOURCE.read_text())
        function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                        and node.name == '_platform_nodes')
        self.mapping = dict(leg_joint_name_mapping_verified=True,
                            leg_joint_name_mapping_receipt_path='/original/mapping.json',
                            leg_joint_name_mapping_receipt_sha256='a'*64,
                            leg_joint_name_mapping_evidence_sha256='b'*64,
                            leg_joint_names=['leg'+str(i) for i in range(12)],
                            source_joint_names=['source'+str(i) for i in range(12)],
                            position_unit='radian', value_transform='identity')
        self.source = dict(selected_source_joint_names=[*self.mapping['source_joint_names'], 'waist'],
                           position_unit='radian', value_transform='identity',
                           source_topic='/joint_states', source_type='sensor_msgs/msg/JointState',
                           source_owner_name='hl_orin_core_msg', source_owner_namespace='/',
                           runtime_binding='original', expected_source_joint_names=['source'+str(i) for i in range(49)],
                           selected_source_joint_indices=list(range(13)), output_topic='/sunhorizon/platform/proof/live_joint_states',
                           receipt_path='/original/source.json', receipt_sha256='c'*64, evidence_sha256='d'*64)
        self.mapping_validator = Mock(return_value=self.mapping)
        self.source_validator = Mock(return_value=self.source)
        namespace = dict(Path=Path, os=SimpleNamespace(environ={}),
                         Node=lambda **kw: kw, Shutdown=lambda **kw: kw,
                         _load_mapping_contract=lambda: SimpleNamespace(launch_ros_mapping_parameters=self.mapping_validator),
                         _load_live_joint_source_contract=lambda: SimpleNamespace(validate_live_joint_state_receipt=self.source_validator))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'), namespace)
        self.launch = namespace['_platform_nodes']
        self.context = SimpleNamespace(perform_substitution=lambda value: value)

    def nodes(self, owner, source='/original/source.json'):
        return self.launch(self.context, '/original/platform.yaml', '/original/vendor.log',
                           '/original/mapping.json', source, owner)

    def test_legacy_platform_owner_keeps_four_processes_and_original_exit_contract(self):
        nodes = self.nodes('platform')
        self.assertEqual([node['name'] for node in nodes], ['sunhorizon_live_joint_state_bridge',
                         'sunhorizon_platform_adapter', 'sunhorizon_ethercat_evidence', 'sunhorizon_joint_log_evidence'])
        self.assertTrue(all(node['on_exit'] is nodes[0]['on_exit'] for node in nodes))
        pins = nodes[0]['parameters'][0]
        self.assertEqual(pins['source_receipt_sha256'], self.source['receipt_sha256'])
        self.assertEqual(pins['output_joint_names'], [*self.mapping['leg_joint_names'], 'waist_yaw'])

    def test_fused_owner_omits_only_bridge_after_same_original_receipt_validation(self):
        nodes = self.nodes('perception_runtime')
        self.assertEqual([node['name'] for node in nodes], ['sunhorizon_platform_adapter',
                         'sunhorizon_ethercat_evidence', 'sunhorizon_joint_log_evidence'])
        self.mapping_validator.assert_called_once_with('/original/mapping.json')
        self.source_validator.assert_called_once()
        self.assertTrue(all(node['on_exit'] is nodes[0]['on_exit'] for node in nodes))
        self.assertTrue(nodes[0]['parameters'][1]['joint_mapping_valid'])
        self.assertEqual(nodes[2]['parameters'][2]['leg_joint_name_mapping_receipt_sha256'], 'a'*64)

    def test_fused_owner_requires_original_receipts_and_never_bypasses_invalid_signature(self):
        with self.assertRaisesRegex(RuntimeError, 'requires_original_signed_receipts'):
            self.nodes('perception_runtime', '')
        self.mapping['leg_joint_name_mapping_verified'] = False
        with self.assertRaisesRegex(RuntimeError, 'requires_accepted_leg_joint_mapping'):
            self.nodes('perception_runtime')
        self.mapping['leg_joint_name_mapping_verified'] = True
        self.source_validator.side_effect = ValueError('original_signature_invalid')
        with self.assertRaisesRegex(ValueError, 'original_signature_invalid'):
            self.nodes('perception_runtime')

    def test_owner_and_source_semantics_cannot_be_guessed(self):
        with self.assertRaisesRegex(RuntimeError, 'invalid_live_joint_state_bridge_owner'):
            self.nodes('auto')
        self.source['value_transform'] = 'scale'
        with self.assertRaisesRegex(RuntimeError, 'semantics_mapping_mismatch'):
            self.nodes('perception_runtime')


if __name__ == '__main__':
    unittest.main()
