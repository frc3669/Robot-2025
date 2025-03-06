import math

run_ave_max_index = 16
coral_elevator_in_to_rotations = 5/9*2.54
coral_angle_gear_ratio = 18

code_cycle_time = 0.02
max_current = 25
feedforward_current = 4
current_headroom = 4
max_accel = 9
max_m_per_sec_per_cycle = max_accel * code_cycle_time
current_to_accel_ratio = 10
motor_turns_per_wheel_turn = 6.12
wheel_diameter_m = 0.09906
motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*math.pi)
max_m_per_sec = 5

max_input_current = 40
braking_current = 15
max_forward_current = max_input_current - braking_current
max_reverse_current = max_input_current + braking_current

startingPosition = 0.5 + 4j
swerve_position_P = 0.04
swerve_heading_P = 2.5

swerve_autoalign_P = 0.2

#release_intake = -0.25
#L4_height = 72.5
#L3_height = 48.5
#L2_height = 32.5
#L1_height = 18.5