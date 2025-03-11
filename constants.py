import math

run_ave_max_index = 16
elevator_in_to_rotations = 2.54 # 9/9cm*2.54
angle_gear_ratio = 30
algae_angle_gear_ratio = 54

code_cycle_time = 0.02
max_current = 25
feedforward_current = 4
current_headroom = 4
max_accel = 10
max_m_per_sec_per_cycle = max_accel * code_cycle_time
current_to_accel_ratio = 9
motor_turns_per_wheel_turn = 6.75
wheel_diameter_m = 0.10081
motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter_m*math.pi)
max_m_per_sec = 4.5

swerve_position_P = 0.04
swerve_heading_P = 2.5

swerve_autoalign_P = 1.2

#release_intake = -0.25
#L4_height = 72.5
#L3_height = 48.5
#L2_height = 32.5
#L1_height = 18.5