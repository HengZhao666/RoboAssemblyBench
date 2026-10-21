from toolkits.constraint_checking.integration.sequence_precheck import (
    AssemblySequencePrechecker,
)

ROBOTS = ['left', 'right']
OBJECTS = ['part_a', 'part_b']


def check(phases):
    return AssemblySequencePrechecker().check(
        phases=phases,
        robot_names=ROBOTS,
        object_names=OBJECTS,
    )


def test_valid_pick_carry_place_sequence():
    report = check(
        [
            {'name': 'approach', 'local_skill': {'name': 'move_above_part', 'robot': 'left', 'object': 'part_a'}},
            {'name': 'grasp', 'attach': [{'robot': 'left', 'object': 'part_a'}]},
            {'name': 'carry', 'local_skill': {'name': 'move_part_to_target', 'robot': 'left', 'object': 'part_a'}},
            {'name': 'place', 'lock': [{'object': 'part_a', 'target': 'slot'}]},
        ]
    )

    assert report.feasible
    assert report.errors == []
    assert report.final_holding['left'] is None


def test_free_object_fixture_lock_is_not_reported_as_release_without_attach():
    report = check(
        [
            {
                'name': 'initialize_fixture',
                'lock': [
                    {
                        'object': 'part_a',
                        'target': 'part_a_fixture_pickup',
                        'snap_free_object': True,
                    }
                ],
            }
        ]
    )

    assert report.feasible
    assert report.errors == []
    assert report.warnings == []
    assert report.actions[-1]['kind'] == 'fixture_lock'


def test_robot_cannot_attach_second_object_while_holding():
    report = check(
        [
            {'name': 'grasp_a', 'attach': [{'robot': 'left', 'object': 'part_a'}]},
            {'name': 'grasp_b', 'attach': [{'robot': 'left', 'object': 'part_b'}]},
        ]
    )

    assert not report.feasible
    assert {issue['code'] for issue in report.errors} == {'end_effector_occupied'}


def test_two_robots_cannot_grasp_same_object():
    report = check(
        [
            {'name': 'left_grasp', 'attach': [{'robot': 'left', 'object': 'part_a'}]},
            {'name': 'right_grasp', 'attach': [{'robot': 'right', 'object': 'part_a'}]},
        ]
    )

    assert not report.feasible
    assert 'double_grasp' in {issue['code'] for issue in report.errors}


def test_payload_motion_requires_prior_attach():
    report = check(
        [
            {
                'name': 'carry_without_grasp',
                'local_skill': {'name': 'move_part_to_target', 'robot': 'left', 'object': 'part_a'},
            }
        ]
    )

    assert not report.feasible
    assert 'payload_not_held' in {issue['code'] for issue in report.errors}


def test_release_by_non_owner_is_rejected():
    report = check(
        [
            {'name': 'grasp', 'attach': [{'robot': 'left', 'object': 'part_a'}]},
            {'name': 'bad_release', 'detach': [{'robot': 'right', 'object': 'part_a'}]},
        ]
    )

    assert not report.feasible
    assert 'release_by_non_owner' in {issue['code'] for issue in report.errors}


def test_unknown_references_are_reported():
    report = check([{'name': 'bad', 'attach': [{'robot': 'third', 'object': 'missing'}]}])

    assert not report.feasible
    codes = {issue['code'] for issue in report.errors}
    assert {'unknown_robot', 'unknown_object'}.issubset(codes)


def test_current_plumbers_block_recipe_passes_sequence_precheck():
    from toolkits.factory_dual_franka_assembly.task_specs import load_task_recipe

    recipe = load_task_recipe('fabrica_plumbers_block_ur5e_right_base_prepare')
    report = AssemblySequencePrechecker().check(
        phases=recipe['phases'],
        robot_names=[robot['name'] for robot in recipe['robots']],
        object_names=[item['name'] for item in recipe['objects']],
    )

    assert report.feasible, report.errors
    assert report.phase_count > 20


def shared_attach(robot, **changes):
    return dict({'robot': robot, 'object': 'part_a',
                 'attachment_mode': 'pure_physical_grasp',
                 'allow_shared_physical_grasp': True}, **changes)


def test_declared_shared_approach_and_selective_handover_preserve_receiver():
    report = check([
        {'name': 'first_grasp', 'attach': [shared_attach('left')]},
        {'name': 'approach_receiver', 'local_skill': {
            'name': 'move_above_part', 'robot': 'right', 'object': 'part_a'}},
        {'name': 'receiver_grasp', 'attach': [shared_attach('right')]},
        {'name': 'giver_release', 'detach': [{'robot': 'left', 'object': 'part_a'}]},
        {'name': 'receiver_carry', 'local_skill': {
            'name': 'move_part_to_target', 'robot': 'right', 'object': 'part_a'}},
    ])
    assert report.feasible, report.errors
    assert report.final_holding == {'left': None, 'right': 'part_a'}


def test_shared_grasp_requires_bilateral_permission_and_pure_mode():
    for first, second in [
        (shared_attach('left', allow_shared_physical_grasp=False), shared_attach('right')),
        (shared_attach('left'), shared_attach('right', allow_shared_physical_grasp=False)),
        (shared_attach('left', attachment_mode='fixed_joint'), shared_attach('right')),
        (shared_attach('left'), shared_attach('right', attachment_mode='fixed_joint')),
    ]:
        report = check([{'name': 'first', 'attach': [first]}, {'name': 'second', 'attach': [second]}])
        assert 'double_grasp' in {issue['code'] for issue in report.errors}


def test_future_shared_grasp_does_not_authorize_unheld_payload_motion():
    report = check([
        {'name': 'first', 'attach': [shared_attach('left')]},
        {'name': 'early_hold', 'local_skill': {'name': 'custom_hold', 'robot': 'right',
            'object': 'part_a', 'requires_held_object': True}},
        {'name': 'second', 'attach': [shared_attach('right')]},
    ])
    assert 'payload_not_held' in {issue['code'] for issue in report.errors}


def test_open_shared_hand_does_not_release_other_hand():
    report = check([
        {'name': 'first', 'attach': [shared_attach('left')]},
        {'name': 'second', 'attach': [shared_attach('right')]},
        {'name': 'open_first', 'gripper_commands': {'left': 'open'}},
        {'name': 'second_carry', 'local_skill': {'name': 'move_part_to_target',
            'robot': 'right', 'object': 'part_a'}},
    ])
    assert report.feasible, report.errors
    assert report.final_holding == {'left': None, 'right': 'part_a'}


def test_detach_one_hand_and_open_other_in_same_phase_releases_both():
    report = check([
        {'name': 'first', 'attach': [shared_attach('left')]},
        {'name': 'second', 'attach': [shared_attach('right')]},
        {'name': 'release_both', 'detach': [{'robot': 'left', 'object': 'part_a'}],
            'gripper_commands': {'right': 'open'}},
    ])
    assert report.feasible, report.errors
    assert report.final_holding == {'left': None, 'right': None}


def test_shared_grasp_does_not_authorize_occupied_hand_or_non_owner_release():
    report = check([
        {'name': 'first', 'attach': [shared_attach('left')]},
        {'name': 'second_payload', 'attach': [{'robot': 'right', 'object': 'part_b'}]},
        {'name': 'second_grasp', 'attach': [shared_attach('right')]},
        {'name': 'bad_release', 'detach': [{'robot': 'right', 'object': 'part_a'}]},
    ])
    assert {'end_effector_occupied', 'release_by_non_owner'}.issubset(
        {issue['code'] for issue in report.errors})


def test_generated_beam_sequence_passes_symbolic_shared_grasp_precheck():
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    cfg = build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_staged', seed=0,
        layout_seed=505, attach_runtime_cameras=False, domain_randomization_enabled=True)
    report = AssemblySequencePrechecker().check(phases=cfg.phase_specs,
        robot_names=cfg.robot_names, object_names=[obj.name for obj in cfg.objects])
    assert report.feasible, report.errors
    assert not report.warnings, report.warnings
    assert all(obj is None for obj in report.final_holding.values())
